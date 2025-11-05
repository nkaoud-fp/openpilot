#!/usr/bin/env python3
import os
import re
import time
from collections import defaultdict

import cereal.messaging as messaging
import opendbc

#
## On your Comma device's terminal, simply run the script:
##   python3 /data/dbc_discovery.py
#
#  The script will then prompt you to choose a discovery mode.
#

def get_message_ids_from_file(dbc_file_path):
    """Parses a DBC file and returns a set of all defined message IDs."""
    if not os.path.exists(dbc_file_path):
        return set()
    
    known_ids = set()
    msg_id_regex = re.compile(r"^BO_ (\d+)")
    with open(dbc_file_path, 'r') as f:
        for line in f:
            match = msg_id_regex.match(line)
            if match:
                known_ids.add(int(match.group(1)))
    return known_ids

def parse_dbc_into_blocks(dbc_file_path):
    """Parses an existing DBC file into a dictionary of {id: block_text}."""
    if not os.path.exists(dbc_file_path):
        return {}

    blocks = {}
    current_block = []
    current_id = None
    msg_id_regex = re.compile(r"^BO_ (\d+)")

    with open(dbc_file_path, 'r') as f:
        for line in f:
            match = msg_id_regex.match(line)
            if match:
                if current_id is not None:
                    blocks[current_id] = "".join(current_block)
                
                current_id = int(match.group(1))
                current_block = [line]
            elif current_id is not None:
                current_block.append(line)
    
    if current_id is not None:
        blocks[current_id] = "".join(current_block)
        
    return blocks

def update_dbc_file(newly_discovered_msgs, output_filename):
    """Creates or intelligently updates a DBC file with newly discovered messages."""
    print(f"\nUpdating DBC file: {output_filename}")
    
    existing_blocks = parse_dbc_into_blocks(output_filename)
    
    new_block_count = 0
    for msg_id, data in newly_discovered_msgs.items():
        if msg_id not in existing_blocks:
            max_dlc = data['max_dlc']
            block_lines = [f"BO_ {msg_id} UNKNOWN_MSG_{msg_id}: {max_dlc} XXX\n"]
            for i in range(max_dlc):
                start_bit = (i * 8) + 7
                block_lines.append(f' SG_ BYTE_{i} : {start_bit}|8@0+ (1,0) [0|255] "" XXX\n')
            existing_blocks[msg_id] = "".join(block_lines)
            new_block_count += 1

    if new_block_count == 0:
        print("No new messages to add. File is already up-to-date.")
        return

    print(f"Adding {new_block_count} new message definitions...")

    try:
        with open(output_filename, 'w') as f:
            f.write('VERSION ""\n\nBS_:\n\nBU_: XXX\n\n')
            
            sorted_ids = sorted(existing_blocks.keys())
            for msg_id in sorted_ids:
                f.write(existing_blocks[msg_id] + "\n")
                
        print(f"✅ Successfully updated '{output_filename}'.")
    except Exception as e:
        print(f"❌ Error writing DBC file: {e}")


def main():
    source_ids = set()
    output_filename = "/data/new_discovered_messages.dbc"

    # --- Interactive Menu ---
    while True:
        print("\n--- DBC Discovery Mode ---")
        print("1: Use a reference DBC (find unknown messages)")
        print("2: Start from scratch (discover all messages)")
        print("3: Exit")
        choice = input("Enter your choice [1, 2, or 3]: ")

        if choice == '1':
            # --- MODIFIED SECTION: Added default value for the prompt ---
            default_dbc = "toyota_nodsu_pt_generated"
            prompt = f"Enter the source DBC file [{default_dbc}]: "
            source_name_input = input(prompt)

            if not source_name_input.strip():
                source_name_input = default_dbc
            
            source_name = source_name_input.lower().replace('.dbc', '')
            
            opendbc_dir = os.path.dirname(opendbc.__file__)
            source_dbc_path = os.path.join(opendbc_dir, f"{source_name}.dbc")
            
            print(f"Loading reference DBC: {source_dbc_path}")
            source_ids = get_message_ids_from_file(source_dbc_path)

            if not source_ids:
                print(f"❌ Error: Source DBC '{source_name}' not found or is empty. Please try again.")
                continue
            else:
                break

        elif choice == '2':
            print("🚀 Starting in 'scratch' mode. All messages will be treated as new.")
            break

        elif choice == '3':
            print("Exiting.")
            return
        
        else:
            print("Invalid choice. Please enter 1, 2, or 3.")

    # Prompt for output file
    output_prompt = f"Enter the output DBC file path [{output_filename}]: "
    user_output_path = input(output_prompt)
    if user_output_path.strip():
        output_filename = user_output_path.strip()

    print(f"\nTarget DBC file: {output_filename}")
    existing_generated_ids = get_message_ids_from_file(output_filename)
    
    print(f"Loaded {len(source_ids)} IDs from source and {len(existing_generated_ids)} IDs from target.")
    print("\nMonitoring CAN bus. Press Ctrl+C to stop and update file.")

    can_sock = messaging.sub_sock('can')
    newly_discovered_msgs = defaultdict(lambda: {'count': 0, 'max_dlc': 0})
    
    try:
        while True:
            can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)
            for msg in can_msgs:
                for can_msg in msg.can:
                    msg_id = can_msg.address
                    if msg_id not in source_ids and msg_id not in existing_generated_ids:
                        if msg_id not in newly_discovered_msgs:
                            print(f"🕵️‍♂️ New Message ID Found: {msg_id} (0x{msg_id:X})")
                        
                        newly_discovered_msgs[msg_id]['count'] += 1
                        msg_len = len(can_msg.dat)
                        if msg_len > newly_discovered_msgs[msg_id]['max_dlc']:
                           newly_discovered_msgs[msg_id]['max_dlc'] = msg_len
            
            time.sleep(1.2)

    except KeyboardInterrupt:
        print("\n\n--- Monitoring Stopped by User ---")
        if not newly_discovered_msgs:
            print("✅ No new message IDs were discovered during this session.")
        else:
            print(f"Discovered {len(newly_discovered_msgs)} new message IDs this session. Summary:")
            sorted_ids = sorted(newly_discovered_msgs.keys())
            for msg_id in sorted_ids:
                data = newly_discovered_msgs[msg_id]
                print(f"  - ID: {msg_id:<5} (0x{msg_id:X:<4}) | Count: {data['count']:<6} | Max Length: {data['max_dlc']} bytes")
        
        update_dbc_file(newly_discovered_msgs, output_filename)
        print("----------------------------------")

if __name__ == "__main__":
    main()