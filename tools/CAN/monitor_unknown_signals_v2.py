import time
import os
import re
from collections import defaultdict

import cereal.messaging as messaging
from opendbc.can.parser import CANParser
import opendbc

LOG_FILENAME = "unknown_signals.log"
DISCOVERED_SIGNALS_FILENAME = "discovered_signals.txt"


class bcolors:
    """A class for terminal color codes."""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

def find_signals_to_monitor(dbc_file_path, monitor_all=False):
    """Finds all signals to monitor by manually parsing the DBC file text."""
    messages_to_monitor = []
    signals_map = defaultdict(list)
    
    msg_regex = re.compile(r"^BO_ \d+ (\w+):")
    sig_regex_pattern = r"^\s+SG_ (UNKNOWN_\w+) :" if not monitor_all else r"^\s+SG_ (\w+) :"
    sig_regex = re.compile(sig_regex_pattern)
    
    search_type = "signals prefixed with 'UNKNOWN_'" if not monitor_all else "ALL signals in the DBC"
    print(f"Searching for {search_type}...")

    with open(dbc_file_path, 'r') as f:
        current_msg_name = None
        for line in f:
            msg_match = msg_regex.match(line)
            if msg_match:
                current_msg_name = msg_match.group(1)
                continue
            if current_msg_name:
                sig_match = sig_regex.match(line)
                if sig_match:
                    signals_map[current_msg_name].append(sig_match.group(1))

    for msg_name in signals_map:
        messages_to_monitor.append((msg_name, 100))
        
    return messages_to_monitor, signals_map

def get_toggles(can_parser, signals_map, last_values):
    """Checks the latest CAN values and returns a dictionary of signals that have changed."""
    changes = {}
    for msg_name in can_parser.vl:
        for sig_name in signals_map.get(msg_name, []):
            if sig_name in can_parser.vl[msg_name]:
                value_key = f"{msg_name}:{sig_name}"
                new_value = can_parser.vl[msg_name][sig_name]
                last_value = last_values.get(value_key)

                if new_value != last_value:
                    changes[value_key] = (last_value, new_value)
    return changes

def format_and_log_changes(changes, log_file_handle, log_time=None):
    """Formats the output for detected changes and logs them."""
    if not changes:
        return 0
        
    for signal_key, (last_value, new_value) in changes.items():
        msg_name, sig_name = signal_key.split(':')
        timestamp = f"{log_time:.2f}s" if log_time is not None else time.strftime('%H:%M:%S')
        old_val_str = f"'{last_value}'" if last_value is not None else "None"

        try:
            formatted_new_val = f"'{new_value}' (hex: {new_value:#04x}, bin: {bin(new_value)})"
        except (TypeError, ValueError):
            formatted_new_val = f"'{new_value}'"
        
        output_line_log = f"[{timestamp}] {msg_name} | '{sig_name}' toggled: {old_val_str} -> {formatted_new_val}"
        output_line_term = (f"[{timestamp}] {bcolors.OKBLUE}{msg_name}{bcolors.ENDC} | "
                            f"'{bcolors.OKCYAN}{sig_name}{bcolors.ENDC}' toggled: "
                            f"{bcolors.WARNING}{old_val_str}{bcolors.ENDC} -> "
                            f"{bcolors.OKGREEN}{formatted_new_val}{bcolors.ENDC}")

        print(output_line_term)
        log_file_handle.write(output_line_log + "\n")
    return len(changes)

def monitor_continuous(dbc_name, messages_to_monitor, signals_map, log_file_handle, delay):
    """Monitors the live CAN bus continuously."""
    can_parser = CANParser(dbc_name, messages_to_monitor, 0)
    can_sock = messaging.sub_sock('can', timeout=100)
    last_values = {}

    print(f"\nMonitoring for value changes on live bus (delay: {delay}s). Press Ctrl+C to exit.")
    while True:
        can_msgs = messaging.drain_sock_raw(can_sock)
        can_parser.update_strings(can_msgs)
        changes = get_toggles(can_parser, signals_map, last_values)
        if changes:
            format_and_log_changes(changes, log_file_handle)
            for signal_key, (last_val, new_val) in changes.items():
                last_values[signal_key] = new_val
        time.sleep(delay)

def monitor_interactive(dbc_name, messages_to_monitor, signals_map, log_file_handle):
    """Monitors the live CAN bus interactively."""
    can_parser = CANParser(dbc_name, messages_to_monitor, 0)
    can_sock = messaging.sub_sock('can', timeout=100)
    
    print("\n✅ Interactive mode ready.")
    while True:
        try:
            input("   Press Enter, perform ONE action, then press Enter again...")
            can_msgs = messaging.drain_sock_raw(can_sock)
            can_parser.update_strings(can_msgs)
            
            last_values = {f"{msg}:{sig}": can_parser.vl[msg][sig] for msg in can_parser.vl for sig in signals_map.get(msg, []) if sig in can_parser.vl[msg]}

            input("   Action performed. Press Enter to see changes...")
            
            can_msgs = messaging.drain_sock_raw(can_sock)
            can_parser.update_strings(can_msgs)
            
            print("\n--- 🔬 Signal Changes Detected ---")
            changes = get_toggles(can_parser, signals_map, last_values)
            if not format_and_log_changes(changes, log_file_handle):
                print("   No changes detected for monitored signals.")
            print("---------------------------------\n")

        except (KeyboardInterrupt, EOFError):
            break

def save_and_rename_signal(signal_key, change):
    """Asks user to rename a discovered signal and saves it to a file."""
    print(f"\n{bcolors.BOLD}Now naming signal: {bcolors.OKCYAN}{signal_key}{bcolors.ENDC}")
    
    try:
        new_name_input = input("Enter a new name (or press Enter to skip): ").strip()
    except (KeyboardInterrupt, EOFError):
        return None

    if not new_name_input:
        print("   Skipped.")
        return None

    msg_name, old_sig_name = signal_key.split(':')
    new_signal_key = f"{msg_name}:{new_name_input}"
    
    entry = (
        f"--- Signal Discovered: {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n"
        f"Original Name: {signal_key}\n"
        f"New Name:      {new_signal_key}\n"
        f"Consistently Changed: {change}\n"
        f"--------------------------------------------------\n"
    )

    with open(DISCOVERED_SIGNALS_FILENAME, 'a', encoding='utf-8') as f:
        f.write(entry)
    
    print(f"{bcolors.OKGREEN}   Saved to {DISCOVERED_SIGNALS_FILENAME}{bcolors.ENDC}")
    return new_name_input

# --- NEW, INTEGRATED SEQUENCE ANALYSIS FUNCTION ---
def analyze_sequence_of_last_action(message_batch, consistent_signal_keys, last_values, dbc_name, messages_to_monitor):
    """Processes a batch of messages to find the timing of known consistent signals."""
    print(f"\n--- ⏱️  Sequence Analysis on {len(consistent_signal_keys)} Consistent Signal(s) ---")
    
    detected_changes = []
    changed_signals = set()
    parser_for_batch = CANParser(dbc_name, messages_to_monitor, 0)
    
    for msg_bytes in message_batch:
        msg = messaging.Event.from_bytes(msg_bytes)
        timestamp = msg.logMonoTime
        parser_for_batch.update_strings([msg_bytes])
        
        for msg_name in parser_for_batch.vl:
            for sig_name in parser_for_batch.vl[msg_name]:
                signal_key = f"{msg_name}:{sig_name}"
                # Only analyze signals we already know are consistent
                if signal_key not in consistent_signal_keys or signal_key in changed_signals:
                    continue

                new_val = parser_for_batch.vl[msg_name][sig_name]
                last_val = last_values.get(signal_key)
                
                if new_val != last_val:
                    change_str = f"'{last_val}' -> '{new_val}'"
                    detected_changes.append((timestamp, signal_key, change_str))
                    changed_signals.add(signal_key)
    
    if not detected_changes:
        print("   Could not determine sequence. No changes found in the last action's message batch.")
        return
        
    detected_changes.sort(key=lambda x: x[0])
    start_time = detected_changes[0][0]
    
    for ts, key, change in detected_changes:
        relative_time = (ts - start_time) / 1e9  # Convert nanoseconds to seconds
        print(f"  T+{relative_time:8.4f}s | {bcolors.OKCYAN}{key}{bcolors.ENDC} changed {bcolors.OKGREEN}{change}{bcolors.ENDC}")

# --- HEAVILY MODIFIED GUIDED DISCOVERY FUNCTION ---
def monitor_guided_discovery(dbc_name, messages_to_monitor, signals_map, log_file_handle):
    """Guides the user through tests to find, analyze sequence, name, and save signals."""
    can_parser = CANParser(dbc_name, messages_to_monitor, 0)
    can_sock = messaging.sub_sock('can', timeout=100)
    
    print("\n🕵️  Welcome to Guided Discovery Mode.")
    while True:
        try:
            reps_str = input("   How many times do you want to test this action? (3-5 is recommended): ")
            num_reps = int(reps_str)
            if num_reps > 1: break
            print("   Please enter a number greater than 1.")
        except ValueError:
            print("   Invalid input. Please enter a number.")

    all_run_changes = defaultdict(list)
    last_run_message_batch = []
    last_run_initial_state = {}

    for i in range(num_reps):
        try:
            print(f"\n--- ⚡️ Run {i + 1} of {num_reps} ---")
            input("   Press Enter, then perform the action...")
            can_msgs_before = messaging.drain_sock_raw(can_sock)
            can_parser.update_strings(can_msgs_before)
            last_run_initial_state = {f"{msg}:{sig}": can_parser.vl[msg][sig] for msg in can_parser.vl for sig in signals_map.get(msg, []) if sig in can_parser.vl[msg]}

            input("   Action complete. Press Enter to capture...")
            last_run_message_batch = messaging.drain_sock_raw(can_sock)
            can_parser.update_strings(last_run_message_batch)
            
            changes = get_toggles(can_parser, signals_map, last_run_initial_state)
            if not changes:
                print("   No changes detected in this run.")
            else:
                for signal_key, (last_val, new_val) in changes.items():
                    change_str = f"'{last_val}' -> '{new_val}'"
                    print(f"   Detected change for {signal_key}: {change_str}")
                    all_run_changes[signal_key].append(change_str)
        except (KeyboardInterrupt, EOFError):
            print("\nGuided discovery cancelled.")
            return

    # --- Analysis Phase ---
    print(f"\n--- 📊 Consistency Analysis ---")
    consistent_signals, inconsistent_signals = [], []
    for signal_key, changes_list in all_run_changes.items():
        if len(changes_list) == num_reps and len(set(changes_list)) == 1:
            consistent_signals.append((signal_key, changes_list[0]))
        else:
            inconsistent_signals.append((signal_key, changes_list))

    # --- Automatic Sequence Analysis Phase ---
    if consistent_signals:
        consistent_keys = {key for key, change in consistent_signals}
        analyze_sequence_of_last_action(last_run_message_batch, consistent_keys, last_run_initial_state, dbc_name, messages_to_monitor)
    
    # --- Save & Rename Phase ---
    renamed_signals = {}
    if consistent_signals:
        print(f"\n--- ✍️  Name Your Discovered Signal(s) ---")
        for signal_key, change in consistent_signals:
            new_name = save_and_rename_signal(signal_key, change)
            if new_name:
                renamed_signals[signal_key] = new_name
    
    # --- Final Report Phase ---
    print(f"\n--- 📝 Final Report ---")
    print(f"{bcolors.OKGREEN}{bcolors.BOLD}✅ Consistent Signals Found:{bcolors.ENDC}")
    if not consistent_signals:
        print("   None. Try increasing repetitions or checking a different action.")
    else:
        for signal_key, change in consistent_signals:
            rename_info = f" (Renamed to: {bcolors.OKCYAN}{renamed_signals[signal_key]}{bcolors.ENDC})" if signal_key in renamed_signals else ""
            print(f"   - {signal_key}{rename_info} consistently changed {change}")

    print(f"\n{bcolors.WARNING}{bcolors.BOLD}⚠️ Inconsistent Signals (Noise):{bcolors.ENDC}")
    if not inconsistent_signals:
        print("   None.")
    else:
        for signal_key, changes in inconsistent_signals:
            print(f"   - {signal_key}: {changes}")
    print("\n--------------------------\n")

def get_user_settings():
    """Prompt the user to select monitoring settings."""
    print(f"{bcolors.HEADER}--- CAN Signal Monitor Setup ---{bcolors.ENDC}")
    
    default_dbc = 'toyota_nodsu_pt_generated'
    prompt = f"Enter the DBC file name [default: {default_dbc}]: "
    dbc_file = input(prompt) or default_dbc

    mode = '0'
    while mode not in ['1', '2', '3']:
        mode = input("Select mode: (1) Continuous Log (2) Interactive Action (3) Guided Discovery? [1/2/3]: ")

    all_signals_choice = '0'
    while all_signals_choice not in ['1', '2']:
        all_signals_choice = input("Monitor: (1) UNKNOWN_ signals only or (2) ALL signals? [1/2]: ")
        
    delay = 1.2
    if mode == '1':
        try:
            delay_input = input(f"Enter logging delay in seconds (default is {delay}): ")
            if delay_input:
                delay = float(delay_input)
        except ValueError:
            print(f"Invalid number, using default delay of {delay}s.")

    settings = {
        "dbc_file": dbc_file,
        "mode": mode,
        "monitor_all": all_signals_choice == '2',
        "delay": delay
    }
    print(f"{bcolors.HEADER}----------------------------------{bcolors.ENDC}\n")
    return settings

def main():
    settings = get_user_settings()
    
    dbc_name = settings["dbc_file"].replace('.dbc', '')

    try:
        opendbc_dir = os.path.dirname(opendbc.__file__)
        dbc_file_path = os.path.join(opendbc_dir, "dbc", f"{dbc_name}.dbc")
        if not os.path.isfile(dbc_file_path):
            dbc_file_path = os.path.join(opendbc_dir, f"{dbc_name}.dbc")
        if not os.path.isfile(dbc_file_path):
            raise FileNotFoundError
    except (FileNotFoundError, AttributeError):
        print(f"{bcolors.FAIL}Error: DBC file not found for '{dbc_name}'.{bcolors.ENDC}")
        return

    messages_to_monitor, signals_map = find_signals_to_monitor(dbc_file_path, settings["monitor_all"])
    
    if not messages_to_monitor:
        print("No signals found to monitor based on your criteria. Exiting.")
        return

    print(f"{bcolors.OKGREEN}Found {sum(len(s) for s in signals_map.values())} signals to monitor across {len(signals_map)} messages.{bcolors.ENDC}")

    if os.path.exists(LOG_FILENAME):
        os.remove(LOG_FILENAME)

    try:
        with open(LOG_FILENAME, 'a', encoding='utf-8') as log_file_handle:
            print(f"Logging to: {os.path.join(os.getcwd(), LOG_FILENAME)}")
            
            mode = settings["mode"]
            if mode == '3':
                monitor_guided_discovery(dbc_name, messages_to_monitor, signals_map, log_file_handle)
            elif mode == '2':
                monitor_interactive(dbc_name, messages_to_monitor, signals_map, log_file_handle)
            else:
                monitor_continuous(dbc_name, messages_to_monitor, signals_map, log_file_handle, settings["delay"])

    except KeyboardInterrupt:
        print("\nMonitoring stopped by user.")
    finally:
        print(f"Log file saved to {LOG_FILENAME}")
        if os.path.exists(DISCOVERED_SIGNALS_FILENAME):
            print(f"Discoveries saved to {DISCOVERED_SIGNALS_FILENAME}")

if __name__ == "__main__":
    main()