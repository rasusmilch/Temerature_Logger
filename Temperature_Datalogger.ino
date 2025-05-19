/*
 * Copyright (c) 2019-2022,2024 Piotr Stolarz
 * OneWireNg: Arduino 1-wire service library
 *
 * Distributed under the 2-clause BSD License (the License)
 * see accompanying file LICENSE for details.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the License for more information.
 */

/**
 * Dallas family thermometers access example (Arduino).
 *
 * Required configuration:
 * - @c CONFIG_SEARCH_ENABLED for non single sensor setup,
 * - @c CONFIG_PWR_CTRL_ENABLED if @c PWR_CTRL_PIN is configured.
 */
#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"
#include "eewl.h"
#include "StaticSerialCommands.h"
#include <MD_MAX72xx.h>
#include <SPI.h>


#define OW_PIN    9

#define CLK_PIN   13  // or SCK
#define DATA_PIN  11  // or MOSI
#define CS_PIN    10  // or SS

#define VERSION_MAJOR 0
#define VERSION_MINOR 1
#define VERSION_PATCH 0



const char MSG_VER[] PROGMEM = "v";
const char firmwarePrefix[] PROGMEM = "FIRMWARE=";

// if default buffer size (64) is too small pass a buffer through constructor
char buffer[64];

// Define the number of devices we have in the chain and the hardware interface
// NOTE: These pin numbers will probably not work with your hardware and may
// need to be adapted
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 4

// SPI hardware interface
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

// Text parameters
#define CHAR_SPACING  1 // pixels between characters

#define DISP_MSG_SIZE 16
char disp_msg_buffer[DISP_MSG_SIZE];

/*
 * If defined: only one sensor device is allowed to be connected to the bus.
 * The library may be configured with 1-wire search activity disabled to
 * reduce its footprint.
 */
#define SINGLE_SENSOR

static Placeholder<OneWireNg_CurrentPlatform> ow;
DSTherm drv(ow);

struct coefficient {
    double a;  // Coefficient of x
    double b;  // Constant term
};

coefficient tempCoeff;

struct CalibrationData {
    float targetHigh, targetLow;
    float measuredHigh, measuredLow;
    uint8_t OWAddr[8];
    // Add more CalibrationCoefficient members if you have more ADC channels
};

struct CalibrationDataEEPROM {
    CalibrationData data;
    uint16_t checksum;
};

CalibrationDataEEPROM calibration_data_eeprom;

#define EEPROM_SIZE 1024UL  // Total EEPROM size for Arduino Nano
#define CALIBRATION_DATA_SIZE sizeof(calibration_data_eeprom)  // Size of data plus CRC
#define EEPROM_START_ADDR 1  // Start from address 1 to avoid address 0
#define EEPROM_BLOCK_NUM (EEPROM_SIZE / (CALIBRATION_DATA_SIZE + 1))  // Number of blocks that fit in the EEPROM

// Declare the EEWL object for unsigned long values with a CRC
EEWL eewl(calibration_data_eeprom, EEPROM_BLOCK_NUM, EEPROM_START_ADDR);

struct loggerStruct {
  uint32_t startMillis = 0;
  uint32_t previousMillis = 0;

  bool isRunning = false;
  bool isPaused = false;
};

// Global timer instance
loggerStruct logger;

void printText(uint8_t modStart, uint8_t modEnd, char *pMsg)
// Print the text string to the LED matrix modules specified.
// Message area is padded with blank columns after printing.
{
  uint8_t state = 0;
  uint8_t curLen;
  uint16_t showLen;
  uint8_t cBuf[8];
  int16_t col = ((modEnd + 1) * COL_SIZE) - 1;

  mx.control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  do // finite state machine to print the characters in the space available
  {
    switch (state) {
    case 0: // Load the next character from the font table
      // if we reached end of message, reset the message pointer
      if (*pMsg == '\0') {
        showLen = col - (modEnd * COL_SIZE); // padding characters
        state = 2;
        break;
      }

      // retrieve the next character form the font file
      showLen = mx.getChar(*pMsg++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
      curLen = 0;
      state++;
      // !! deliberately fall through to next state to start displaying

    case 1: // display the next part of the character
      mx.setColumn(col--, cBuf[curLen++]);

      // done with font character, now display the space between chars
      if (curLen == showLen) {
        showLen = CHAR_SPACING;
        state = 2;
      }
      break;

    case 2: // initialize state for displaying empty columns
      curLen = 0;
      state++;
      // fall through

    case 3: // display inter-character spacing or end of message padding (blank
            // columns)
      mx.setColumn(col--, 0);
      curLen++;
      if (curLen == showLen)
        state = 0;
      break;

    default:
      col = -1; // this definitely ends the do loop
    }
  } while (col >= (modStart * COL_SIZE));

  mx.control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

/**
 * @brief Converts a floating-point number to a string manually.
 * @param str Buffer to store the resulting string.
 * @param value Floating-point number to convert.
 * @param precision Number of decimal places.
 * @param maxLen Maximum length of the buffer.
 * @return Number of characters written (excluding the null terminator).
 */
size_t floatToStr(char *str, float value, int precision, size_t maxLen) {
  if (maxLen == 0)
    return 0; // Early exit if no space is provided

  char *start = str; // Remember the start of the buffer for length calculation
  char *end = str + maxLen - 1; // Calculate the end pointer of the buffer

  // Handle negative values
  if (value < 0) {
    if (str < end) {
      *str++ = '-';
    }
    value = -value;
  }

  long intPart = static_cast<long>(value); // Extract integer part
  float fractional = value - intPart;      // Extract fractional part

  // Convert integer part manually
  char *intEnd = str; // Pointer to start of the integer part
  do {
    if (intEnd < end) {
      *intEnd++ = '0' + intPart % 10; // Get last digit
    }
    intPart /= 10; // Remove last digit
  } while (intPart > 0 && intEnd < end);

  // Reverse the integer part
  for (char *p = str, *q = intEnd - 1; p < q; ++p, --q) {
    char temp = *p;
    *p = *q;
    *q = temp;
  }

  str = intEnd; // Adjust str to point right after the last digit of the
                // integer part

  // Process fractional part if precision is greater than 0
  if (precision > 0 && str < end) {
    *str++ = '.'; // Add decimal point
    while (precision-- > 0 && str < end) {
      fractional *= 10;
      int digit = static_cast<int>(fractional);
      *str++ = '0' + digit; // Convert to ASCII and append
      fractional -= digit;  // Remove the integer part
    }
  }
  if (str < end + 1) {
    *str = '\0'; // Null-terminate the string
  }

  // Calculate the length of the string excluding the null terminator
  return str - start;
}

/**
 * @brief Converts an int32_t to a string and stores it in a buffer.
 * @param value Integer number to convert.
 * @param buffer Buffer to store the resulting string.
 * @param bufferMaxLen Maximum length of the buffer.
 * @return Length of the resulting string, or -1 if the buffer is too small.
 */
int int32ToStr(int32_t value, char *buffer, size_t bufferMaxLen) {
  char temp[12]; // Temporary buffer for reversed digits
  char *p = temp;
  bool isNegative = false;
  size_t len = 0;

  if (value < 0) {
    isNegative = true;
    value = -value;
  }

  // Extract digits in reverse order
  do {
    if (len >= sizeof(temp) - 1)
      return -1; // Check temp buffer overflow
    *p++ = (value % 10) + '0';
    value /= 10;
    len++;
  } while (value > 0);

  if (isNegative) {
    if (len >= sizeof(temp) - 1)
      return -1; // Check temp buffer overflow
    *p++ = '-';
    len++;
  }

  if (len >= bufferMaxLen)
    return -1; // Check final buffer overflow

  // Reverse the digits into the buffer
  for (size_t i = 0; i < len; i++) {
    buffer[i] = temp[len - i - 1];
  }
  buffer[len] = '\0'; // Null-terminate the string

  return len; // Return the length of the resulting string
}

/**
 * @brief Copies a string from program memory to a buffer.
 *
 * This function takes a pointer to a string stored in program memory (PROGMEM)
 * and prints each character until it encounters a null terminator. This is
 * used to reduce RAM footprint by keeping static strings in flash memory.
 *
 * @param str Pointer to the program memory string.
 * @return None.
 */
void copyProgmem(const char *str, char *buffer, uint8_t bufferLen) {
  char ch;  // Temporary variable to store each character
  int bufferPtr = 0;

  while (bufferPtr < bufferLen && ch != '\0') {
    ch = pgm_read_byte(str);

    buffer[bufferPtr] = ch;
    bufferPtr++;
    str++;
  }
}

// Main function to create version string
bool createFirmwareString(const char* progmemPrefix, char* buffer, uint8_t bufferSize) {
  // Step 1: Copy prefix from PROGMEM
  copyProgmem(progmemPrefix, buffer, bufferSize);

  // Step 2: Find end of copied prefix
  uint8_t idx = strlen(buffer);

  // Helper to append integer
  auto appendInt = [&](int val) -> bool {
    char temp[10];
    int i = 0;

    if (val == 0) {
      if (idx >= bufferSize - 1) return false;
      buffer[idx++] = '0';
      return true;
    }

    while (val > 0 && i < sizeof(temp)) {
      temp[i++] = '0' + (val % 10);
      val /= 10;
    }

    while (i > 0) {
      if (idx >= bufferSize - 1) return false;
      buffer[idx++] = temp[--i];
    }

    return true;
  };

  // Step 3: Append major.minor.patch
  if (!appendInt(VERSION_MAJOR)) return false;
  if (idx >= bufferSize - 1) return false;
  buffer[idx++] = '.';
  if (!appendInt(VERSION_MINOR)) return false;
  if (idx >= bufferSize - 1) return false;
  buffer[idx++] = '.';
  if (!appendInt(VERSION_PATCH)) return false;

  if (idx >= bufferSize) return false;
  buffer[idx] = '\0';
  return true;
}

/**
 * @brief Calculate the coefficients for a quadratic equation passing through three points.
 * 
 * This function calculates the coefficients a, b, and c for the quadratic equation 
 * y = ax^2 + bx + c that passes through the three given points (x1, y1), (x2, y2), and (x3, y3).
 * 
 * @param coeff A reference to a CalibrationCoefficient structure to store the calculated coefficients.
 * @param x1 The x-coordinate of the first point.
 * @param y1 The y-coordinate of the first point.
 * @param x2 The x-coordinate of the second point.
 * @param y2 The y-coordinate of the second point.
 * @param x3 The x-coordinate of the third point.
 * @param y3 The y-coordinate of the third point.
 */
// void calculateCoefficients(coefficient& coeff, double x1, double y1, double x2, double y2, double x3, double y3) {
//     double denom = (x1 - x2) * (x1 - x3) * (x2 - x3); // Calculate the denominator for the coefficients
//     coeff.a = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / denom; // Calculate coefficient a
//     coeff.b = (x3 * x3 * (y1 - y2) + x2 * x2 * (y3 - y1) + x1 * x1 * (y2 - y3)) / denom; // Calculate coefficient b
//     coeff.c = (x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3) / denom; // Calculate coefficient c
// }

void calculateCoefficients(coefficient& coeff, double x1, double y1, double x2, double y2) {
    coeff.a = (y2 - y1) / (x2 - x1); // Calculate coefficient a
    coeff.b = y1 - coeff.a * x1; // Calculate coefficient b
}
/**
 * @brief Computes the CRC-16 checksum for a given array of data.
 *
 * This function calculates the CRC-16 checksum using a specified polynomial. The CRC-16 checksum
 * is used to detect alterations to raw data, and is widely used in protocols to ensure data integrity.
 * The polynomial used in this implementation is 0x8408, which is the reverse of the standard polynomial 0x1021
 * commonly used in CRC-16-CCITT.
 *
 * @param data Pointer to the data array over which the CRC is to be calculated.
 * @param size Size of the data array in bytes.
 * @return uint16_t The computed CRC-16 checksum value.
 */
uint16_t crc16(const uint8_t* data, uint16_t size) {
    uint16_t crc = 0xFFFF;  // Start with the mask 0xFFFF, which initializes the CRC register to all 1's

    // Process each byte of data in the array
    for (uint16_t i = 0; i < size; ++i) {
        crc ^= data[i];  // XOR the next data byte into the CRC register
        // Process each bit within the current byte
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                // If the LSB (Least Significant Bit) is set, apply the polynomial division
                crc = (crc >> 1) ^ 0x8408;  // 0x8408 is used here as the polynomial
            } else {
                // If the LSB is not set, just shift right
                crc >>= 1;
            }
        }
    }
    // After processing all data and bits, take the one's complement of the calculated CRC
    return ~crc;  // Return the final CRC-16 checksum
}

/**
 * @brief Saves the given calibration data structure to EEPROM and verifies the write operation.
 *
 * This function writes a calibration data structure to EEPROM, incorporating wear leveling via the EEWL library.
 * It computes a checksum for the entire data structure, writes the data to EEPROM, and then reads back the data 
 * to verify the integrity of the write operation using a memory comparison.
 *
 * @param calData The calibration data structure to be saved.
 * @return bool Returns true if the data is successfully written and verified, otherwise returns false.
 */
bool saveCalibrationValue(CalibrationDataEEPROM& cal) {
    CalibrationDataEEPROM read_data;   // Structure to hold data read back for verification

    cal.checksum = crc16(reinterpret_cast<byte*>(&cal.data), sizeof(cal.data));

    // Write the prepared data to EEPROM using the EEWL library which handles wear leveling
    eewl.put(cal);

    // Read back the data from EEPROM to verify the write operation was successful
    if (eewl.get(read_data) == 0) {  // Check if the read operation was successful
        Serial.println(F("Failed EEPROM read"));
        return false;  // Return false if reading from EEPROM failed
    }

    // Compare the written data with the data read back to ensure they are identical
    if (memcmp(&cal, &read_data, sizeof(CalibrationDataEEPROM)) != 0) {
        Serial.println(F("EEPROM MEMCMP FAILED!"));
        return false;  // Return false if the data does not match
    }

    return true;  // Return true indicating the write and verification were successful
}

/**
 * @brief Loads and verifies the calibration data from EEPROM.
 *
 * This function attempts to load a calibration data structure from EEPROM using the EEWL library. 
 * It checks the integrity of the data using a CRC checksum. If the checksum verifies correctly, 
 * it updates the passed reference variable with the loaded data.
 *
 * @param calData Reference to a CalibrationData variable where the loaded calibration data will be stored if successful.
 * @return bool Returns true if the calibration data is successfully read and verified, false otherwise.
 */
bool loadCalibrationValue(CalibrationDataEEPROM& cal) {
    CalibrationDataEEPROM read_data;  // Structure to hold the data read from EEPROM

    // Attempt to read calibration data from EEPROM
    if (eewl.get(read_data) == 0) {  // Check if data is valid
        Serial.println(F("EEPROM LOAD READ FAILED"));  // Log an error message if data reading fails
        return false;  // Return false indicating no valid data could be read
    }

    // Compute the CRC checksum of the read value for verification
    uint16_t checksum = crc16(reinterpret_cast<byte*>(&read_data.data), sizeof(read_data.data));
    if (read_data.checksum == checksum) {
        cal = read_data;  // If checksum matches, update the referenced variable with the read value
        return true;  // Return true indicating successful data verification and loading
    } else {
        Serial.println(F("EEPROM CRC FAILURE"));  // Log an error message if CRC check fails
        return false;  // Return false indicating a CRC error
    }
}

/* returns false if not supported */
static bool printId(const OneWireNg::Id& id)
{
    const char *name = DSTherm::getFamilyName(id);

    Serial.print(id[0], HEX);
    for (size_t i = 1; i < sizeof(OneWireNg::Id); i++) {
        Serial.print(':');
        Serial.print(id[i], HEX);
    }
    if (name) {
        Serial.print(" -> ");
        Serial.print(name);
    }
    Serial.println();

    return (name != NULL);
}

static void printScratchpad(const DSTherm::Scratchpad& scrpd)
{
    const uint8_t *scrpd_raw = scrpd.getRaw();

    Serial.print("  Scratchpad:");
    for (size_t i = 0; i < DSTherm::Scratchpad::LENGTH; i++) {
        Serial.print(!i ? ' ' : ':');
        Serial.print(scrpd_raw[i], HEX);
    }

    Serial.print("; Th:");
    Serial.print(scrpd.getTh());

    Serial.print("; Tl:");
    Serial.print(scrpd.getTl());

    Serial.print("; Resolution:");
    Serial.print(9 + (int)(scrpd.getResolution() - DSTherm::RES_9_BIT));

    long temp = scrpd.getTemp2();
    Serial.print("; Temp:");
    Serial.print((float)temp / 16);
    Serial.print(" C");

    Serial.println();
}

/**
 * @brief Prints a string from program memory to the serial port.
 *
 * This function takes a pointer to a string stored in program memory (PROGMEM)
 * and prints each character until it encounters a null terminator. This is
 * used to reduce RAM footprint by keeping static strings in flash memory.
 *
 * @param str Pointer to the program memory string.
 * @return None.
 */
void printProgmem(Stream& stream, const char *str) {
  char ch;  // Temporary variable to store each character
  while ((ch = pgm_read_byte(str)) != '\0') {
    stream.print(ch);
    str++;  // Move to the next character
  }
}


bool getSensorAddress(uint8_t *address) {
    static Placeholder<DSTherm::Scratchpad> scrpd;

    OneWireNg::ErrorCode ec = drv.readScratchpadSingle(scrpd);
    if (ec == OneWireNg::EC_SUCCESS) {
        const OneWireNg::Id& id = scrpd->getId();  // Reference avoids copy

        for (uint8_t i = 0; i < sizeof(OneWireNg::Id); i++) {
            address[i] = id[i];
        }

        return true;
    } else if (ec == OneWireNg::EC_CRC_ERROR) {
        Serial.println("  CRC error.");
    }

    return false;
}


float getRawTemp() {
    long rawTemp;
    float convertedToC = 999.0;

    drv.convertTempAll(DSTherm::MAX_CONV_TIME, false);
    /*
     * Scratchpad placeholder is static to allow reuse of the associated
     * sensor id while reissuing readScratchpadSingle() calls.
     * Note, due to its storage class the placeholder is zero initialized.
     */
    static Placeholder<DSTherm::Scratchpad> scrpd;

    OneWireNg::ErrorCode ec = drv.readScratchpadSingle(scrpd);
    if (ec == OneWireNg::EC_SUCCESS) {
        rawTemp = scrpd->getTemp2();

        convertedToC = (float)rawTemp / 16.0;
    } else if (ec == OneWireNg::EC_CRC_ERROR) {
        Serial.println("  CRC error.");
    }

    return convertedToC;
}

float getCorrectedTemp(float rawTemp) {
    float correctedTemp;

    correctedTemp = tempCoeff.a * rawTemp + tempCoeff.b;

    return correctedTemp;
}

void printCalValues() {
    Serial.print(F("Sensor address: "));
    printSensorAddress(calibration_data_eeprom.data.OWAddr);
    Serial.println();
    Serial.print(F("High - T: "));
    Serial.print(calibration_data_eeprom.data.targetHigh);
    Serial.print(F(", M: "));
    Serial.println(calibration_data_eeprom.data.measuredHigh);

    Serial.print(F("Low - T: "));
    Serial.print(calibration_data_eeprom.data.targetLow);
    Serial.print(F(", M: "));
    Serial.println(calibration_data_eeprom.data.measuredLow);

}

/**
 * @brief Prints a string from program memory to the serial port with a newline.
 * @param str Pointer to the program memory string.
 * @return None.
 */
void printProgmemLn(Stream& stream, const char *str) {
  printProgmem(stream, str);
  stream.println();
}


void loggerStart() {
  logger.isRunning = true;

  logger.previousMillis = millis();
}

void loggerStop() {
  logger.isRunning = false;
}

float boilingPointFromAdjustedInHg(float adjustedPressure_inHg) {
    // Constants
    const float P0 = 29.9213;      // Standard pressure in inHg (sea-level)
    const float T0 = 373.12;       // Boiling point at P0 in Kelvin (99.97 °C)
    const float L = 40650.0;       // Latent heat of vaporization [J/mol]
    const float R = 8.314;         // Ideal gas constant [J/mol·K]

    // Avoid log(0) or negative pressure
    if (adjustedPressure_inHg <= 0.0) {
        return -999.9;  // Error value
    }

    // Clausius–Clapeyron formula using adjusted pressure
    float inv_T = (1.0 / T0) - (R / L) * log(adjustedPressure_inHg / P0);
    float T = 1.0 / inv_T;

    return T - 273.15;  // Convert K to °C
}

float stationToSeaLevelPressure(float stationPressure_inHg, float elevation_m) {
    // Approximate conversion using barometric formula
    // Constants
    const float tempLapseRate = 0.0065;   // Temperature lapse rate [K/m]
    const float seaLevelTempK = 288.15;   // Standard temperature at sea level [K]
    const float gravity = 9.80665;         // Gravity [m/s^2]
    const float molarMass = 0.0289644;     // Molar mass of air [kg/mol]
    const float R = 8.31447;                // Universal gas constant [J/(mol·K)]

    float exponent = (gravity * molarMass) / (R * tempLapseRate);
    float adjustedPressure = stationPressure_inHg * pow(
        1 - (tempLapseRate * elevation_m / seaLevelTempK), 
        -exponent
    );

    return adjustedPressure;
}



void updateLogger() {
    float rawTemp;
    float correctedTemp;
    char chars;

    rawTemp = getRawTemp();

    correctedTemp = getCorrectedTemp(rawTemp);

    Serial.print(F("Raw = "));
    Serial.print(rawTemp);
    Serial.print(F("C, Corrected = "));
    Serial.print(correctedTemp);
    Serial.println(F("C"));

    chars = floatToStr(buffer, correctedTemp, 1, sizeof(buffer));

    if (chars < sizeof(buffer) + 2) {
        buffer[chars] = 'C';
        chars++;
        buffer[chars] = '\0';
    }

    printText(0, MAX_DEVICES - 1, buffer);
    // Serial.println(buffer);
}

bool loggerUpdate() {
  if (!logger.isRunning) {
    return false;
  }

  uint32_t currentMillis = millis();
  uint32_t elapsed = currentMillis - logger.previousMillis;

  if (elapsed >= 1000UL) {
    logger.previousMillis = currentMillis;

    updateLogger();
  }

  return false;
}

void cmd_help(SerialCommands& sender, Args& args) {
  sender.listCommands();
}

void cmd_probeFirmware(SerialCommands& sender, Args& args) {
  if (createFirmwareString(firmwarePrefix, buffer, sizeof(buffer))) {
    sender.getSerial().println(buffer);
  } else {
    // Handle error: buffer too small
  }
}

void cmd_printCal(SerialCommands& sender, Args& args) {
  
  printCalValues();
  printCoeff();
  
}

void cmd_startLogger(SerialCommands& sender, Args& args) {
  loggerStart();
  sender.getSerial().println(F("Started logger"));
}

void cmd_stopLogger(SerialCommands& sender, Args& args) {
  loggerStop();
  sender.getSerial().println(F("Stopped logger"));
}

void printSensorAddress(const uint8_t *address) {
    Serial.print(address[0], HEX);
    for (size_t i = 1; i < 8; i++) {
        Serial.print(':');
        Serial.print(address[i], HEX);
    }
}
bool updateCal() {

    uint8_t id[8];
    bool result = false;

    result = getSensorAddress(id);

    if (result == false) {
        Serial.print(F("Error getting sensor address. Address: "));
        printSensorAddress(id);
        Serial.println();
    }

    for (uint8_t i = 0; i < 8; i++) {
        calibration_data_eeprom.data.OWAddr[i] = id[i];
    }

    // Attempt to save the updated calibration value to EEPROM
    if (saveCalibrationValue(calibration_data_eeprom)) {
        Serial.println(F("Calibration data SAVED!")); // Indicate calibration saved successfully
    } else {
        Serial.println(F("Calibration data FAILED")); // Indicate calibration save failed
        return false; // Return 0 if saving to EEPROM failed
    }

    calculateCoefficients(tempCoeff, calibration_data_eeprom.data.measuredLow, calibration_data_eeprom.data.targetLow, 
        calibration_data_eeprom.data.measuredHigh, calibration_data_eeprom.data.targetHigh);

    printCalValues();
    printCoeff();

    return true;

}

void cmd_setCalHighAdj(SerialCommands& sender, Args& args) {
    auto inHg = args[0].getFloat();

    float targetBoiling = boilingPointFromAdjustedInHg(inHg);

    Serial.print(F("Calculated boiling point from ADJUSTED PRESSURE = "));
    Serial.print(targetBoiling);
    Serial.print(F("C at "));
    Serial.print(targetBoiling);
    Serial.println(F(" inHg"));

    float measured = getRawTemp();

    if (measured < 999) {

        calibration_data_eeprom.data.measuredHigh = measured;
        calibration_data_eeprom.data.targetHigh = targetBoiling;

        updateCal();

        sender.getSerial().println("OK");
    } else {
        Serial.println(F("ERROR"));
    }
}

void cmd_setCalHighStation(SerialCommands& sender, Args& args) {
    auto inHg = args[0].getFloat();
    auto elevation = args[1].getFloat();

    float adjustedPressure = stationToSeaLevelPressure(inHg, elevation);
    float targetBoiling = boilingPointFromAdjustedInHg(adjustedPressure);

    Serial.print(F("Adjusting pressure to calculated sea level value: "));
    Serial.print(adjustedPressure);
    Serial.println(F("inHg"));
    Serial.print(F("Calculated boiling point from ADJUSTED PRESSURE: "));
    Serial.print(targetBoiling);
    Serial.print(F("C at "));
    Serial.print(targetBoiling);
    Serial.println(F(" inHg"));

    float measured = getRawTemp();

    if (measured < 999) {

        calibration_data_eeprom.data.measuredHigh = measured;
        calibration_data_eeprom.data.targetHigh = targetBoiling;

        updateCal();

        sender.getSerial().println("OK");
    } else {
        Serial.println(F("ERROR"));
    }
}

void cmd_setCalLow(SerialCommands& sender, Args& args) {
    float target = 0.0;

    float measured = getRawTemp();

    if (measured < 999) {

        calibration_data_eeprom.data.measuredLow = measured;
        calibration_data_eeprom.data.targetLow = target;

        updateCal();

        sender.getSerial().println("OK");
    } else {
        Serial.println(F("ERROR"));
    }
}

/*
COMMAND macro is used to create Command object.
It takes the following arguments:
    COMMAND(function, command)
    COMMAND(function, command, subcommands)
    COMMAND(function, command, subcommands, description)
    COMMAND(function, command, arguments..., subcommands, description)

ARG macro is used to specify argument type, range (if type is numeric) and name.
It takes the following arguments:
    ARG(type)
    ARG(type, name)
    ARG(type, min, max)
    ARG(type, min, max, name)
*/

Command commands[] {
  COMMAND(cmd_help, "help", NULL, "list commands"),

  COMMAND(cmd_setCalHighAdj, "setCalBPAdj", ArgType::Float, NULL, "Set cal boiling H2O at {adjusted pressure inHg}"),
  COMMAND(cmd_setCalHighStation, "setCalBPStation", ArgType::Float, ArgType::Float, NULL, "Set cal boiling H2O at {station pressure, inHg} {elevation, m}"),
  COMMAND(cmd_setCalLow, "setCalIce", NULL, "Set cal ice bath"),
  COMMAND(cmd_printCal, "printCal", NULL, "Print calibration data"),
  COMMAND(cmd_probeFirmware, "probeFirmware", NULL, "Probe command to return firmware version"),
  COMMAND(cmd_startLogger, "startLogger", NULL, "Start timer"),
  COMMAND(cmd_stopLogger, "stopLogger", NULL, "Stop timer"),

};

//SerialCommands serialCommands(Serial, commands, sizeof(commands) / sizeof(Command));

// 10 millisecond timeout for blocking readSerial command
SerialCommands serialCommands(Serial, commands, sizeof(commands) / sizeof(Command), buffer, sizeof(buffer), 10);

void printCoeff() {
    Serial.print(F("Calculated coeff - a: "));
    Serial.print(tempCoeff.a);
    Serial.print(F(", b: "));
    Serial.println(tempCoeff.b);

}

bool compareSensorAddress(uint8_t *address1, uint8_t *address2) {

    for (uint8_t i = 0; i < 8; i++) {
        if (address1[i] != address2[i]) {
            return false;
        }
    }

    return true;
}

bool loadCalDefaults() {
    calibration_data_eeprom.data.targetHigh = 100.0;
    calibration_data_eeprom.data.measuredHigh = 100.0;
    calibration_data_eeprom.data.targetLow = 0.0;
    calibration_data_eeprom.data.measuredLow = 0.0;

    updateCal();
}

void setup()
{
    bool result;

    new (&ow) OneWireNg_CurrentPlatform(OW_PIN, false);
    DSTherm drv(ow);

    Serial.begin(115200);

    mx.begin();

    if (createFirmwareString(MSG_VER, buffer, sizeof(buffer))) {
        Serial.println(buffer);
        printText(0, MAX_DEVICES - 1, buffer);
    } else {
        // Handle error: buffer too small
    }

    // Initialize EEPROM wear leveling
    eewl.begin();

    result = loadCalibrationValue(calibration_data_eeprom);

    if (result != true) {
        
        Serial.println(F("FAILED to load calibration values!"));
        loadCalDefaults();
    }

    uint8_t id[8];

    result = getSensorAddress(id);

    if (result != true) {
        Serial.println(F("Unable to get sensor ID"));
    } else {
        Serial.print(F("Found sensor at: "));
        printSensorAddress(id);
        Serial.println();
    }
    
    result = compareSensorAddress(id, calibration_data_eeprom.data.OWAddr);

    if (result != true) {
        Serial.println(F("Sensor addresses DO NOT MATCH!"));
        Serial.print(F("EEPROM = "));
        printSensorAddress(calibration_data_eeprom.data.OWAddr);
        Serial.print(F("  Present = "));
        printSensorAddress(id);
        Serial.println();
    }
    

    calculateCoefficients(tempCoeff, calibration_data_eeprom.data.measuredLow, calibration_data_eeprom.data.targetLow, 
        calibration_data_eeprom.data.measuredHigh, calibration_data_eeprom.data.targetHigh);


    Serial.println(F("EEPROM Values"));
    printCalValues();

    Serial.println();
    printCoeff();

    Serial.println();

    // customize delimiter, termination and quotation characters
    serialCommands.setDelimiterChars<' ', ','>();
    serialCommands.setTerminationChars<'\n', '\r', ';'>();
    serialCommands.setQuotationChars<'"', '|', '\''>();

    if (createFirmwareString(MSG_VER, buffer, sizeof(buffer))) {
        Serial.println(buffer);
    } else {
        // Handle error: buffer too small
    }

    serialCommands.listCommands();

    // drv.writeScratchpadAll(0, 0, COMMON_RES);


}

void loop()
{
    serialCommands.readSerial();

    loggerUpdate();
}
