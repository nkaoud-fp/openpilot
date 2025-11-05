import argparse
import time
import os
import re
from collections import defaultdict

import cereal.messaging as messaging
from opendbc.can.parser import CANParser
import opendbc

LOG_FILENAME = "unknown_signals.log"

#
## On your Comma device's terminal
## python3 /data/monitor_unknown_signals.py toyota_nodsu_pt_generated
#
#  The script will print any detected signal changes directly to your terminal screen.
#
#  At the same time, it will save a complete record to the unknown_signals.log file in the same directory. You can view this file later with cat /data/unknown_signals.log.
#
#  To stop the script from running, simply press Ctrl + C.
#
#  The script is designed to automatically find the correct DBC file using Openpilot's library, regardless of which folder you run the script from.
#
#

def find_unknown_signals(dbc_name, dbc_file_path):
    """
    Finds all UNKNOWN_ signals by manually parsing the DBC file text.
    This is more reliable than relying on CANParser to load all signals implicitly.
    """
    messages_to_monitor = []
    unknown_signals_map = defaultdict(list)
    
    # Regular expressions to find message names (BO_) and unknown signals (SG_ UNKNOWN_)
    msg_regex = re.compile(r"^BO_ \d+ (\w+):")
    sig_regex = re.compile(r"^\s+SG_ (UNKNOWN_\w+) :")

    current_msg_name = None
    
    print(f"Reading DBC file: {dbc_file_path}")
    with open(dbc_file_path, 'r') as f:
        for line in f:
            msg_match = msg_regex.match(line)
            if msg_match:
                current_msg_name = msg_match.group(1)
                continue

            if current_msg_name:
                sig_match = sig_regex.match(line)
                if sig_match:
                    sig_name = sig_match.group(1)
                    unknown_signals_map[current_msg_name].append(sig_name)

    # Now, create the list of messages that need to be monitored
    for msg_name in unknown_signals_map:
        # Add the message to our list to be monitored, using a default frequency.
        messages_to_monitor.append((msg_name, 100))
        
    return messages_to_monitor, unknown_signals_map


# --- MODIFIED FUNCTION ---
def check_for_toggles(can_parser, unknown_signals_map, last_values, log_file_handle, log_time=None):
    """Checks the latest CAN values for any signals that have changed and logs them."""
    for msg_name in can_parser.vl:
        for sig_name in unknown_signals_map.get(msg_name, []):
            # THE FIX: Before accessing the signal, check if the CANParser library actually loaded it.
            # This prevents a KeyError if our manual parser finds a signal that the library failed to parse.
            if sig_name in can_parser.vl[msg_name]:
                value_key = f"{msg_name}:{sig_name}"

                new_value = can_parser.vl[msg_name][sig_name]
                last_value = last_values.get(value_key)

                if new_value != last_value:
                    timestamp = f"{log_time:.2f}s" if log_time is not None else time.strftime('%Y-%m-%d %H:%M:%S')
                    old_val_str = f"'{last_value}'" if last_value is not None else "None"

                    output_line = f"[{timestamp}] {msg_name} | '{sig_name}' toggled: {old_val_str} -> '{new_value}'"

                    # Print to screen
                    print(output_line)

                    # Write to the log file
                    log_file_handle.write(output_line + "\n")

                    last_values[value_key] = new_value
# --- END OF MODIFIED FUNCTION ---

def monitor_live_bus(dbc_name, messages_to_monitor, unknown_signals_map, log_file_handle):
    """Monitors the live CAN bus for signal changes."""
    can_parser = CANParser(dbc_name, messages_to_monitor, 0)
    can_sock = messaging.sub_sock('can', timeout=100)
    last_values = {}

    print("\nMonitoring for value changes on live bus. Press Ctrl+C to exit.")
    while True:
        can_msgs = messaging.drain_sock_raw(can_sock)
        can_parser.update_strings(can_msgs)
        check_for_toggles(can_parser, unknown_signals_map, last_values, log_file_handle)
        time.sleep(1.2) # Set from user profile

def parse_log_file(dbc_name, messages_to_monitor, unknown_signals_map, file_path, log_file_handle):
    """Parses a log file and prints/logs signal changes."""
    can_parser = CANParser(dbc_name, messages_to_monitor, 0)
    last_values = {}

    print(f"\nParsing log file: {file_path}")
    try:
        lr = messaging.log_reader(file_path)
        for msg in lr:
            if msg.which() == 'can':
                can_parser.update_strings([msg.as_builder().to_bytes()])
                check_for_toggles(can_parser, unknown_signals_map, last_values, log_file_handle, msg.logMonoTime / 1e9)
        print("Log file parsing complete.")
    except Exception as e:
        print(f"Error parsing log file: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Monitor UNKNOWN signals from a live bus or log file and save output to unknown_signals.log.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("dbc_file", help="Name of the DBC file to use (e.g., 'toyota_nodsu_pt_generated').")
    parser.add_argument("-f", "--file", help="Path to an rlog or qlog file to parse instead of live monitoring.")
    args = parser.parse_args()

    dbc_name = args.dbc_file.replace('.dbc', '')

    # Find the DBC file path within the installed opendbc library
    opendbc_dir = os.path.dirname(opendbc.__file__)
    dbc_file_path = os.path.join(opendbc_dir, f"{dbc_name}.dbc")

    if not os.path.isfile(dbc_file_path):
        print(f"Error: DBC file not found for '{dbc_name}'.")
        print(f"Attempted path: {dbc_file_path}")
        return

    messages_to_monitor, unknown_signals_map = find_unknown_signals(dbc_name, dbc_file_path)
    if not messages_to_monitor:
        print("No signals with the prefix 'UNKNOWN_' found in the DBC file. Exiting.")
        return

    print(f"Found {sum(len(s) for s in unknown_signals_map.values())} UNKNOWN signals to monitor.")

    # Delete old log file if it exists
    if os.path.exists(LOG_FILENAME):
        os.remove(LOG_FILENAME)

    log_file_handle = None
    try:
        # Open the new log file in append mode
        log_file_handle = open(LOG_FILENAME, 'a', encoding='utf-8')
        print(f"Output will be saved to: {LOG_FILENAME}")

        if args.file:
            parse_log_file(dbc_name, messages_to_monitor, unknown_signals_map, args.file, log_file_handle)
        else:
            monitor_live_bus(dbc_name, messages_to_monitor, unknown_signals_map, log_file_handle)

    except KeyboardInterrupt:
        print("\nMonitoring stopped by user. Exiting.")
    finally:
        if log_file_handle:
            log_file_handle.close()
            print(f"Log file saved to {LOG_FILENAME}")


if __name__ == "__main__":
    main()