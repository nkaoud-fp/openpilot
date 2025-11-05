import time
import os
import re
import sys
from contextlib import contextmanager
from collections import defaultdict

import cereal.messaging as messaging
from cereal import log
from opendbc.can.parser import CANParser


LOG_FILENAME = "unknown_signals.log"
DISCOVERED_SIGNALS_FILENAME = "discovered_signals.txt"


@contextmanager
def suppress_stdout_stderr():
    """
    A context manager that redirects stdout and stderr to devnull
    to suppress C++ library logs.
    """
    # Save original file descriptors
    original_stdout_fd = sys.stdout.fileno()
    saved_stdout_fd = os.dup(original_stdout_fd)
    original_stderr_fd = sys.stderr.fileno()
    saved_stderr_fd = os.dup(original_stderr_fd)
    
    try:
        # Open devnull
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        
        # Redirect stdout and stderr
        os.dup2(devnull_fd, original_stdout_fd)
        os.dup2(devnull_fd, original_stderr_fd)
        os.close(devnull_fd)
        
        yield
    finally:
        # Restore stdout and stderr
        os.dup2(saved_stdout_fd, original_stdout_fd)
        os.dup2(saved_stderr_fd, original_stderr_fd)
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)


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

def find_signals_to_monitor(dbc_file_path, monitor_all=False, selected_messages=None):
    """
    Finds signals to monitor by parsing the DBC file.
    Can be filtered to a specific list of messages.
    """
    messages_to_monitor = []
    signals_map = defaultdict(list)
    
    msg_regex = re.compile(r"^BO_ \d+ (\w+):")
    sig_regex_pattern = r"^\s+SG_ (UNKNOWN_\w+) :" if not monitor_all else r"^\s+SG_ (\w+) :"
    sig_regex = re.compile(sig_regex_pattern)
    
    search_type = "signals prefixed with 'UNKNOWN_'" if not monitor_all else "ALL signals"
    if selected_messages:
        print(f"Searching for {search_type} in messages: {', '.join(selected_messages)}...")
    else:
        print(f"Searching for {search_type} in ALL messages...")

    with open(dbc_file_path, 'r') as f:
        current_msg_name = None
        for line in f:
            msg_match = msg_regex.match(line)
            if msg_match:
                current_msg_name = msg_match.group(1)
                if selected_messages and current_msg_name not in selected_messages:
                    current_msg_name = None
                continue

            if current_msg_name:
                sig_match = sig_regex.match(line)
                if sig_match:
                    signals_map[current_msg_name].append(sig_match.group(1))

    for msg_name in signals_map:
        messages_to_monitor.append((msg_name, 100))
        
    return messages_to_monitor, signals_map

def get_message_list_from_dbc(dbc_file_path):
    """Parses a DBC file and returns a list of all message names."""
    msg_regex = re.compile(r"^BO_ \d+ (\w+):")
    messages = []
    try:
        with open(dbc_file_path, 'r') as f:
            for line in f:
                msg_match = msg_regex.match(line)
                if msg_match:
                    messages.append(msg_match.group(1))
    except FileNotFoundError:
        return []
    return sorted(messages)


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
        with suppress_stdout_stderr():
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
    
    print("\n? Interactive mode ready.")
    while True:
        try:
            input("   Press Enter, perform ONE action, then press Enter again...")
            with suppress_stdout_stderr():
                can_msgs = messaging.drain_sock_raw(can_sock)
                can_parser.update_strings(can_msgs)
            
            last_values = {f"{msg}:{sig}": can_parser.vl[msg][sig] for msg in can_parser.vl for sig in signals_map.get(msg, []) if sig in can_parser.vl[msg]}

            input("   Action performed. Press Enter to see changes...")
            
            with suppress_stdout_stderr():
                can_msgs = messaging.drain_sock_raw(can_sock)
                can_parser.update_strings(can_msgs)
            
            print("\n--- ?? Signal Changes Detected ---")
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

def analyze_sequence_of_last_action(message_batch, consistent_signal_keys, last_values, dbc_name, messages_to_monitor):
    """Processes a batch of messages to find the timing of known consistent signals."""
    print(f"\n--- ??  Sequence Analysis on {len(consistent_signal_keys)} Consistent Signal(s) ---")
    
    detected_changes = []
    changed_signals = set()
    parser_for_batch = CANParser(dbc_name, messages_to_monitor, 0)
    
    for msg in message_batch:
        timestamp = msg.logMonoTime
        msg_bytes = msg.as_builder().to_bytes()
        with suppress_stdout_stderr():
            parser_for_batch.update_strings([msg_bytes])
        
        for msg_name in parser_for_batch.vl:
            for sig_name in parser_for_batch.vl[msg_name]:
                signal_key = f"{msg_name}:{sig_name}"
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
        relative_time = (ts - start_time) / 1e9
        print(f"  T+{relative_time:8.4f}s | {bcolors.OKCYAN}{key}{bcolors.ENDC} changed {bcolors.OKGREEN}{change}{bcolors.ENDC}")

def monitor_guided_discovery(dbc_name, messages_to_monitor, signals_map, log_file_handle):
    """Guides the user through tests to find, analyze sequence, name, and save signals."""
    can_parser = CANParser(dbc_name, messages_to_monitor, 0)
    can_sock = messaging.sub_sock('can', timeout=100)
    
    print("\n???  Welcome to Guided Discovery Mode.")
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
            print(f"\n--- ?? Run {i + 1} of {num_reps} ---")
            input("   Press Enter, then perform the action...")
            with suppress_stdout_stderr():
                can_msgs_before = messaging.drain_sock_raw(can_sock)
                can_parser.update_strings(can_msgs_before)
            last_run_initial_state = {f"{msg}:{sig}": can_parser.vl[msg][sig] for msg in can_parser.vl for sig in signals_map.get(msg, []) if sig in can_parser.vl[msg]}

            input("   Action complete. Press Enter to capture...")
            with suppress_stdout_stderr():
                last_run_message_batch = messaging.recv_sock(can_sock, wait=False)

            if last_run_message_batch is None:
                last_run_message_batch = []
            elif not isinstance(last_run_message_batch, list):
                last_run_message_batch = [last_run_message_batch]
            
            changes_this_run = {}
            change_counts_this_run = defaultdict(int)

            if last_run_message_batch:
                current_run_state = {k: v for k, v in last_run_initial_state.items() if v is not None}
                temp_parser = CANParser(dbc_name, messages_to_monitor, 0)
                
                for msg in last_run_message_batch:
                    msg_bytes = msg.as_builder().to_bytes()
                    with suppress_stdout_stderr():
                        temp_parser.update_strings([msg_bytes])
                    toggles_in_step = get_toggles(temp_parser, signals_map, current_run_state)

                    if toggles_in_step:
                        for key, (last_val, new_val) in toggles_in_step.items():
                            change_counts_this_run[key] += 1
                            if key not in changes_this_run:
                                changes_this_run[key] = (last_run_initial_state.get(key), new_val)
                    
                    for msg_name, sigs in temp_parser.vl.items():
                        for sig_name, val in sigs.items():
                            current_run_state[f"{msg_name}:{sig_name}"] = val

            filtered_changes = {
                key: val for key, val in changes_this_run.items() 
                if change_counts_this_run.get(key, 0) == 1
            }

            if not filtered_changes:
                print("   No single-change signals detected in this run.")
            else:
                for signal_key, (last_val, new_val) in filtered_changes.items():
                    change_str = f"'{last_val}' -> '{new_val}'"
                    print(f"   Detected change for {signal_key}: {change_str}")
                    all_run_changes[signal_key].append(change_str)
        except (KeyboardInterrupt, EOFError):
            print("\nGuided discovery cancelled.")
            return

    # --- Analysis Phase ---
    print(f"\n--- ?? Consistency Analysis ---")
    consistent_signals, inconsistent_signals = [], []
    for signal_key, changes_list in all_run_changes.items():
        if len(changes_list) == num_reps and len(set(changes_list)) == 1:
            consistent_signals.append((signal_key, changes_list[0]))
        else:
            inconsistent_signals.append((signal_key, changes_list))

    # --- Automatic Sequence Analysis Phase ---
    first_actor = None
    if consistent_signals:
        consistent_keys = {key for key, change in consistent_signals}
        first_actor = analyze_sequence_of_last_action(last_run_message_batch, consistent_keys, last_run_initial_state, dbc_name, messages_to_monitor)
    
    # --- Save & Rename Phase ---
    renamed_signals = {}
    if consistent_signals:
        print(f"\n--- ??  Name Your Discovered Signal(s) ---")
        for signal_key, change in consistent_signals:
            new_name = save_and_rename_signal(signal_key, change)
            if new_name:
                renamed_signals[signal_key] = new_name
    
    # --- Final Report Phase ---
    print(f"\n--- ?? Final Report ---")
    print(f"{bcolors.OKGREEN}{bcolors.BOLD}? Consistent Signals Found:{bcolors.ENDC}")
    if not consistent_signals:
        print("   None. Try increasing repetitions or checking a different action.")
    else:
        for signal_key, change in consistent_signals:
            rename_info = f" (Renamed to: {bcolors.OKCYAN}{renamed_signals[signal_key]}{bcolors.ENDC})" if signal_key in renamed_signals else ""
            first_actor_highlight = ""
            if signal_key == first_actor:
                first_actor_highlight = f" {bcolors.OKGREEN}(First to Change ??){bcolors.ENDC}"
            print(f"   - {signal_key}{rename_info} consistently changed {change}{first_actor_highlight}")

    print(f"\n{bcolors.WARNING}{bcolors.BOLD}?? Inconsistent Signals (Noise):{bcolors.ENDC}")
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
    dbc_name = dbc_file.replace('.dbc', '')

    dbc_lib_choice = '0'
    #while dbc_lib_choice not in ['1', '2']:
    #    dbc_lib_choice = input("Select DBC library: (1) opendbc or (2) chffr-panda? [1]: ") or '1'
    #dbc_library = "opendbc" if dbc_lib_choice == '1' else "chffr-panda"
    dbc_library = "opendbc"
    try:
        if dbc_library == "opendbc":
            import opendbc
            base_dir = os.path.dirname(opendbc.__file__)
            possible_paths = [
                os.path.join(base_dir, "dbc", f"{dbc_name}.dbc"),
                os.path.join(base_dir, f"{dbc_name}.dbc"),
            ]
        elif dbc_library == "chffr-panda":
            import panda
            base_dir = os.path.dirname(panda.__file__)
            possible_paths = [
                os.path.join(base_dir, "dbc", f"{dbc_name}.dbc"),
                os.path.join(base_dir, "dbc", "car", f"{dbc_name}.dbc"),
            ]
        
        dbc_file_path = None
        for path in possible_paths:
            if os.path.isfile(path):
                dbc_file_path = path
                break
        
        if dbc_file_path is None:
            raise FileNotFoundError

    except (FileNotFoundError, AttributeError, ImportError):
        print(f"{bcolors.FAIL}Error: DBC file for '{dbc_name}' not found in the '{dbc_library}' library.{bcolors.ENDC}")
        return None
    
    available_messages = get_message_list_from_dbc(dbc_file_path)
    if not available_messages:
        print(f"{bcolors.WARNING}Could not find any messages in {dbc_name}.dbc.{bcolors.ENDC}")
        return None

    print(f"\n--- Found Messages in {dbc_name}.dbc ---")
    for i, msg in enumerate(available_messages, 1):
        print(f"  {i:2d}. {msg}")
    print("---------------------------------------")

    prompt = "Enter message numbers or names to monitor (comma-separated), or press Enter for ALL: "
    selected_messages_str = input(prompt).strip()
    final_selected_messages = set()
    invalid_selection = []

    if selected_messages_str:
        user_inputs = [s.strip() for s in selected_messages_str.split(',')]
        for item in user_inputs:
            if not item: continue
            try:
                msg_idx = int(item) - 1
                if 0 <= msg_idx < len(available_messages):
                    final_selected_messages.add(available_messages[msg_idx])
                else:
                    invalid_selection.append(item)
            except ValueError:
                if item in available_messages:
                    final_selected_messages.add(item)
                else:
                    invalid_selection.append(item)

    if invalid_selection:
        print(f"{bcolors.WARNING}Warning: The following messages were not found and will be ignored: {', '.join(invalid_selection)}{bcolors.ENDC}")

    selected_messages = list(final_selected_messages)

    mode = '0'
    while mode not in ['1', '2', '3']:
        mode = input("\nSelect mode: (1) Continuous Log (2) Interactive Action (3) Guided Discovery? [1/2/3]: ")

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
        "dbc_name": dbc_name,
        "dbc_file_path": dbc_file_path,
        "selected_messages": selected_messages if selected_messages else None,
        "mode": mode,
        "monitor_all": all_signals_choice == '2',
        "delay": delay
    }
    print(f"{bcolors.HEADER}----------------------------------{bcolors.ENDC}\n")
    return settings

def main():
    settings = get_user_settings()
    if settings is None:
        return

    messages_to_monitor, signals_map = find_signals_to_monitor(
        settings["dbc_file_path"],
        settings["monitor_all"],
        settings["selected_messages"]
    )
    
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
            dbc_name = settings["dbc_name"]
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