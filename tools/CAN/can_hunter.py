#!/usr/bin/env python3
import os
import time
import json
from collections import defaultdict

import cereal.messaging as messaging

# --- Configuration ---
EXCLUSION_LIST_PATH = "/data/can_exclusion_list.json"
# Using the 1.2 second delay from your saved preferences for the live monitoring refresh rate.
LIVE_MONITOR_REFRESH_RATE = 1.2

def print_header(title):
    """Prints a formatted header to the console."""
    print("\n" + "="*60)
    print(f"| {title.center(56)} |")
    print("="*60)

def load_exclusion_list(filepath):
    """Loads the exclusion list from a JSON file."""
    if not os.path.exists(filepath):
        return set()
    try:
        with open(filepath, 'r') as f:
            return set(json.load(f))
    except (json.JSONDecodeError, IOError) as e:
        print(f"⚠️ Warning: Could not load exclusion list: {e}")
        return set()

def save_exclusion_list(filepath, id_set):
    """Saves the exclusion list to a JSON file."""
    try:
        with open(filepath, 'w') as f:
            json.dump(sorted(list(id_set)), f, indent=2)
    except IOError as e:
        print(f"❌ Error: Could not save exclusion list: {e}")

def generate_dbc_file(messages, filename):
    """Creates a skeleton DBC file for a dictionary of messages."""
    if not messages:
        print("No messages to generate a DBC for.")
        return

    print_header(f"Generating DBC file: {filename}")
    try:
        with open(filename, 'w') as f:
            f.write('VERSION ""\n\nBS_:\n\nBU_: XXX\n\n')
            
            sorted_ids = sorted(messages.keys())
            for msg_id in sorted_ids:
                max_dlc = messages[msg_id]['max_dlc']
                # Ensure DLC is at least 1 to avoid empty messages
                max_dlc = max(1, max_dlc)

                f.write(f"BO_ {msg_id} UNKNOWN_MSG_{msg_id}: {max_dlc} XXX\n")
                for i in range(max_dlc):
                    start_bit = (i * 8) + 7
                    f.write(f' SG_ BYTE_{i} : {start_bit}|8@0+ (1,0) [0|255] "" XXX\n')
                f.write("\n")
                
        print(f"✅ Successfully created '{filename}'.")
    except Exception as e:
        print(f"❌ Error writing DBC file: {e}")

def main():
    """Main execution workflow for the CAN Hunter script."""
    # (a) Listen to all messages without a reference DBC
    can_sock = messaging.sub_sock('can')
    
    # Load the persistent exclusion list
    exclusion_list = load_exclusion_list(EXCLUSION_LIST_PATH)
    print(f"Loaded {len(exclusion_list)} message IDs from the existing exclusion list.")
    
    candidate_msgs = {}
    
    # Main workflow loop (steps b, c, d)
    while True:
        # (b) Capture baseline messages for the exclusion list
        print_header("Step 1: Capture Baseline")
        print("Do NOT perform the target activity now.")
        print("Let the script capture routine messages (e.g., drive normally or let the car sit in accessory mode).")
        
        # Drain any old messages
        messaging.drain_sock(can_sock)
        
        baseline_capture_start_time = time.monotonic()
        newly_captured_baseline_ids = set()
        
        input("Press Enter to START capturing baseline...")
        print("\nCapturing baseline... Press Enter to STOP.")

        while True:
            try:
                # A simple non-blocking check for user input could be added here for more complex scripts,
                # but for this workflow, blocking with input() is sufficient. Cereal buffers the messages.
                break
            except KeyboardInterrupt:
                break # Allow Ctrl+C to stop this stage

        # After user stops, process all buffered messages
        can_msgs = messaging.drain_sock(can_sock)
        for msg in can_msgs:
            for can_msg in msg.can:
                newly_captured_baseline_ids.add(can_msg.address)

        if newly_captured_baseline_ids:
            original_count = len(exclusion_list)
            exclusion_list.update(newly_captured_baseline_ids)
            new_count = len(exclusion_list)
            print(f"\nCaptured {len(newly_captured_baseline_ids)} new message IDs.")
            print(f"Exclusion list updated from {original_count} to {new_count} IDs.")
            save_exclusion_list(EXCLUSION_LIST_PATH, exclusion_list)
        else:
            print("\nNo new baseline messages were captured in this session.")

        # (c) Capture the activity and filter out the exclusion list
        print_header("Step 2: Capture Activity")
        print("Now, get ready to perform the specific activity you want to capture.")
        
        input("Press Enter to START capturing the activity...")
        print("\nCapturing activity... Perform the action now. Press Enter to STOP.")
        
        candidate_msgs = defaultdict(lambda: {'count': 0, 'max_dlc': 0, 'data_samples': []})
        
        # Drain socket before starting
        messaging.drain_sock(can_sock)
        
        while True:
            try:
                # As before, input() will block, and we'll process the buffer after.
                break
            except KeyboardInterrupt:
                break

        activity_msgs = messaging.drain_sock(can_sock)
        for msg in activity_msgs:
            for can_msg in msg.can:
                if can_msg.address not in exclusion_list:
                    candidate_msgs[can_msg.address]['count'] += 1
                    msg_len = len(can_msg.dat)
                    if msg_len > candidate_msgs[can_msg.address]['max_dlc']:
                        candidate_msgs[can_msg.address]['max_dlc'] = msg_len
                    # Store up to 5 unique data samples
                    if can_msg.dat not in candidate_msgs[can_msg.address]['data_samples'] and len(candidate_msgs[can_msg.address]['data_samples']) < 5:
                        candidate_msgs[can_msg.address]['data_samples'].append(can_msg.dat)
                        
        print_header("Step 3: Review Filtered Messages")
        if not candidate_msgs:
            print("No new messages were detected during the activity.")
        else:
            print(f"Found {len(candidate_msgs)} potential message(s) not in the exclusion list:")
            sorted_ids = sorted(candidate_msgs.keys())
            for msg_id in sorted_ids:
                data = candidate_msgs[msg_id]
                print(f"  - ID: {msg_id:<5} (0x{msg_id:X:<4}) | Count: {data['count']:<6} | Max Length: {data['max_dlc']} bytes")

        # (d) Ask user for suspected messages
        print_header("Step 4: Identify Suspects")
        user_input = input("Enter suspected message IDs (comma-separated), or 'n' to repeat the capture process: ").lower()

        if user_input in ('n', 'no'):
            print("\nRepeating the process to refine the exclusion list...")
            continue
        
        try:
            suspected_ids = [int(x.strip()) for x in user_input.split(',')]
            # Validate user input
            valid_ids = [i for i in suspected_ids if i in candidate_msgs]
            invalid_ids = [i for i in suspected_ids if i not in candidate_msgs]
            
            if invalid_ids:
                print(f"⚠️ Warning: The following IDs were not in the candidate list and will be ignored: {invalid_ids}")
            
            if valid_ids:
                suspected_ids = valid_ids
                break # Exit the main workflow loop
            else:
                print("No valid IDs entered. Repeating the process.")
        except ValueError:
            print("Invalid input. Please enter numbers or 'n'. Repeating the process.")
    
    # (e) Live monitor the suspected messages
    print_header("Step 5: Live Monitoring Suspected IDs")
    print(f"Showing live data for IDs: {suspected_ids}. Press Ctrl+C to stop and generate DBC.")
    
    try:
        last_msgs = {}
        while True:
            can_msgs = messaging.drain_sock(can_sock)
            for msg in can_msgs:
                for can_msg in msg.can:
                    if can_msg.address in suspected_ids:
                        last_msgs[can_msg.address] = can_msg.dat.hex()

            os.system('clear')
            print("--- Live Monitoring --- (Press Ctrl+C to Stop)")
            print(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
            for msg_id in sorted(suspected_ids):
                hex_data = last_msgs.get(msg_id, "...")
                print(f"ID: {msg_id:<5} (0x{msg_id:X:<4}) | Data: {hex_data}")
            
            time.sleep(LIVE_MONITOR_REFRESH_RATE)
            
    except KeyboardInterrupt:
        print("\n\nLive monitoring stopped by user.")

    # (f) Create a DBC file
    print_header("Step 6: Generate DBC File")
    default_filename = f"/data/discovered_{'_'.join(map(str, sorted(suspected_ids)))}.dbc"
    filename_input = input(f"Enter a filename for the new DBC [{default_filename}]: ")
    
    final_filename = filename_input.strip() if filename_input.strip() else default_filename
    
    # Generate DBC for ALL candidates found, not just suspected ones, as requested.
    generate_dbc_file(candidate_msgs, final_filename)
    print("\nProcess complete.")

if __name__ == "__main__":
    main()