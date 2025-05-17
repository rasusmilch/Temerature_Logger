import argparse
import serial
import time
import csv
import signal
import sys
import re
from datetime import datetime

# Globals for cleanup
ser = None
txt_file = None
csv_file = None
csv_writer = None
start_time = None  # Time when the first valid line is logged

def generate_timestamped_filenames(base_name):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{base_name}_{timestamp}.txt", f"{base_name}_{timestamp}.csv"

def open_serial_port(port, baudrate):
    print(f"Opening serial port {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(3)
    ser.reset_input_buffer()
    return ser

def open_output_files(base_name):
    global txt_file, csv_file, csv_writer
    txt_filename, csv_filename = generate_timestamped_filenames(base_name)

    txt_file = open(txt_filename, "w")
    csv_file = open(csv_filename, "w", newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Timestamp", "Elapsed Time (s)", "Raw Temp (°C)", "Corrected Temp (°C)"])
    print(f"Logging to {txt_filename} and {csv_filename}")
    return txt_filename, csv_filename

def parse_and_log_line(line):
    global start_time
    txt_file.write(line + "\n")
    txt_file.flush()

    match = re.search(r"Raw\s*=\s*([-+]?\d*\.\d+|\d+)C,\s*Corrected\s*=\s*([-+]?\d*\.\d+|\d+)C", line)
    if match:
        now = datetime.now()
        if start_time is None:
            start_time = now
        elapsed = (now - start_time).total_seconds()

        timestamp_str = now.strftime("%Y-%m-%d %H:%M:%S")
        raw_temp = float(match.group(1))
        corrected_temp = float(match.group(2))
        csv_writer.writerow([timestamp_str, int(round(elapsed)), raw_temp, corrected_temp])
        csv_file.flush()

def handle_signal(sig, frame):
    print("\nReceived Ctrl+C, sending stopLogger and exiting...")
    try:
        if ser and ser.is_open:
            ser.write(b"stopLogger;")
            time.sleep(1)
    except Exception as e:
        print(f"Error sending stopLogger: {e}")
    finally:
        close_all()
        sys.exit(0)

def close_all():
    global txt_file, csv_file, ser
    if txt_file:
        txt_file.close()
    if csv_file:
        csv_file.close()
    if ser and ser.is_open:
        ser.close()

def start_logging(args):
    global ser
    ser = open_serial_port(args.port, args.baudrate)
    open_output_files(args.output)

    signal.signal(signal.SIGINT, handle_signal)

    ser.write(b"startLogger;")
    print("Logging started. Press Ctrl+C to stop.")

    while True:
        if ser.in_waiting:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print(line)
                    parse_and_log_line(line)
            except Exception as e:
                print(f"Error reading/parsing line: {e}")

def parse_args():
    parser = argparse.ArgumentParser(description="Serial temperature logger")
    parser.add_argument("-p", "--port", required=True, help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("-b", "--baudrate", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("-o", "--output", required=True, help="Base name for output files (no extension)")
    return parser.parse_args()

def main():
    args = parse_args()
    start_logging(args)

if __name__ == "__main__":
    main()
