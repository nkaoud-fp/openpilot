#!/usr/bin/env python3
import os
import re
import time
from collections import defaultdict

import cereal.messaging as messaging
# On a comma device, opendbc is usually available.
# If not, run: pip install opendbc-python
try:
    import opendbc
except ImportError:
    print("Warning: opendbc-python not found. Reference DBC mode will be limited.")
    opendbc = None

#
## HOW TO RUN:
## On your Comma device's terminal, simply run the script:
##   python3 /data/dbc_maker.py
#
#  The script will guide you through discovery and then automatically
#  analyze the structure of new messages when you stop it.
#

def get_message_ids_from_file(dbc_file_path):
    """Parses a DBC file and returns a set of all defined message IDs."""
    if not os.path.exists(dbc_file_path):
        return set()
    
    known_ids = set()
    msg_id_regex = re.compile(r"^BO_ (\d+)")
    try:
        with open(dbc_file_path, 'r', errors='ignore') as f:
            for line in f:
                match = msg_id_regex.match(line)
                if match:
                    known_ids.add(int(match.group(1)))
    except Exception as e:
        print(f"?? Error reading DBC {dbc_file_path}: {e}")
    return known_ids

def parse_dbc_into_blocks(dbc_file_path):
    """Parses an existing DBC file into a dictionary of {id: block_text}."""
    if not os.path.exists(dbc_file_path):
        return {}

    blocks = {}
    current_block = []
    current_id = None
    msg_id_regex = re.compile(r"^BO_ (\d+)")

    try:
        with open(dbc_file_path, 'r', errors='ignore') as f:
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
    except Exception as e:
        print(f"?? Error parsing DBC {dbc_file_path}: {e}")
    return blocks

def to_motorola_bit(bit_index):
    """Converts a standard bit index (0-63) to a Motorola CAN bit number for DBC."""
    byte_num = bit_index // 8
    bit_in_byte = bit_index % 8
    return byte_num * 8 + (7 - bit_in_byte)

def analyze_discovered_messages(discovered_data):
    """Analyzes captured payloads to find changing bits and guess signal structure."""
    print("\n?? Analyzing captured data for changing signals...")
    discovered_signals = defaultdict(list)
    message_info = {}

    for addr, data in discovered_data.items():
        payloads = data['payloads']
        if len(payloads) < 2:
            continue
        
        max_dlc = max(len(p) for p in payloads)
        message_info[addr] = {'max_dlc': max_dlc}
        max_bits = max_dlc * 8
        
        first_msg_int = int.from_bytes(payloads[0].ljust(max_dlc, b'\x00'), 'big')
        changing_bits_mask = 0
        for dat_bytes in payloads[1:]:
            dat_int = int.from_bytes(dat_bytes.ljust(max_dlc, b'\x00'), 'big')
            changing_bits_mask |= (first_msg_int ^ dat_int)

        in_signal = False
        start_lsb = 0
        for i in range(max_bits + 1):
            is_set = (changing_bits_mask >> i) & 1
            if is_set and not in_signal:
                in_signal = True
                start_lsb = i
            elif not is_set and in_signal:
                in_signal = False
                end_msb = i - 1
                length = end_msb - start_lsb + 1
                start_bit_motorola = to_motorola_bit(end_msb)
                discovered_signals[addr].append({
                    'start_bit': start_bit_motorola,
                    'length': length,
                    'name': f'SIG_{start_bit_motorola}'
                })
    
    print(f"? Analysis complete. Found potential signals in {len(discovered_signals)} messages.")
    return discovered_signals, message_info

def update_dbc_with_guessed_signals(discovered_signals, msg_info, output_filename):
    """Creates or intelligently updates a DBC file with guessed signal structures."""
    print(f"\nUpdating DBC file: {output_filename}")
    
    existing_blocks = parse_dbc_into_blocks(output_filename)
    new_block_count = 0

    for msg_id, signals in discovered_signals.items():
        if msg_id not in existing_blocks:
            max_dlc = msg_info[msg_id]['max_dlc']
            block_lines = [f"BO_ {msg_id} MSG_{msg_id}: {max_dlc} XXX\n"]
            
            sorted_signals = sorted(signals, key=lambda x: x['start_bit'])
            for sig in sorted_signals:
                block_lines.append(f" SG_ {sig['name']} : {sig['start_bit']}|{sig['length']}@0- (1,0) [0|0] \"\" XXX\n")
            
            existing_blocks[msg_id] = "".join(block_lines)
            new_block_count += 1

    if new_block_count == 0:
        print("No new messages to add. File is already up-to-date.")
        return

    print(f"Adding definitions for {new_block_count} new messages with guessed signals...")

    try:
        with open(output_filename, 'w') as f:
            f.write('VERSION ""\n\nBS_:\n\nBU_: XXX\n\n')
            for msg_id in sorted(existing_blocks.keys()):
                f.write(existing_blocks[msg_id] + "\n")
        print(f"? Successfully updated '{output_filename}'.")
    except Exception as e:
        print(f"? Error writing DBC file: {e}")

def main():
    source_ids = set()
    output_filename = "/data/analyzed_messages.dbc"

    # --- Interactive Menu ---
    while True:
        print("\n--- DBC Live Analyzer ---")
        print("1: Use a reference DBC (find and analyze unknown messages)")
        print("2: Start from scratch (discover and analyze all messages)")
        print("3: Exit")
        choice = input("Enter your choice [1, 2, or 3]: ")

        if choice == '1':
            if opendbc is None:
                print("? opendbc-python library not found. Cannot use reference mode.")
                continue
            
            default_dbc = "toyota_nodsu_pt_generated"
            source_name = input(f"Enter the source DBC name from opendbc [{default_dbc}]: ") or default_dbc
            
            opendbc_dir = os.path.dirname(opendbc.__file__)
            source_dbc_path = os.path.join(opendbc_dir, f"{source_name.lower().replace('.dbc', '')}.dbc")
            
            print(f"Loading reference DBC: {source_dbc_path}")
            source_ids = get_message_ids_from_file(source_dbc_path)

            if not source_ids:
                print(f"? Error: Source DBC '{source_name}' not found or is empty. Please try again.")
                continue
            break
        elif choice == '2':
            print("?? Starting in 'scratch' mode. All messages will be analyzed.")
            break
        elif choice == '3':
            print("Exiting.")
            return
        else:
            print("Invalid choice.")

    user_output_path = input(f"Enter the output DBC file path [{output_filename}]: ")
    if user_output_path.strip():
        output_filename = user_output_path.strip()

    print(f"\nTarget DBC file will be: {output_filename}")
    existing_generated_ids = get_message_ids_from_file(output_filename)
    
    all_known_ids = source_ids.union(existing_generated_ids)
    print(f"Monitoring CAN bus. Ignoring {len(all_known_ids)} known message IDs.")
    print("Press Ctrl+C to stop, analyze, and update the file.")

    can_sock = messaging.sub_sock('can')
    # This will now store all payloads for analysis
    newly_discovered_data = defaultdict(lambda: {'payloads': []})
    
    try:
        while True:
            can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)
            for msg in can_msgs:
                for can_msg in msg.can:
                    msg_id = can_msg.address
                    if msg_id not in all_known_ids:
                        if msg_id not in newly_discovered_data:
                            print(f"?????? New Message ID Found: {msg_id} (0x{msg_id:X}). Collecting data...")
                        
                        newly_discovered_data[msg_id]['payloads'].append(can_msg.dat)
            time.sleep(1.0) # Using 1.0 second delay, can be adjusted.

    except KeyboardInterrupt:
        print("\n\n--- Monitoring Stopped by User ---")
        if not newly_discovered_data:
            print("? No new message IDs were discovered during this session.")
            return

        discovered_signals, msg_info = analyze_discovered_messages(newly_discovered_data)
        
        if not discovered_signals:
            print("? No changing signals found in the new messages. DBC file will not be updated.")
            return

        print("\n--- Analysis Summary ---")
        for msg_id in sorted(discovered_signals.keys()):
            print(f"\n  ID: {msg_id} (0x{msg_id:X}) | Found {len(discovered_signals[msg_id])} potential signal(s):")
            for sig in sorted(discovered_signals[msg_id], key=lambda x: x['start_bit']):
                 print(f"    - {sig['name']} (start_bit: {sig['start_bit']}, length: {sig['length']})")
        
        update_dbc_with_guessed_signals(discovered_signals, msg_info, output_filename)
        print("--------------------------\n?? Reminder: The generated DBC is a best-guess. Use Cabana to verify!")

if __name__ == "__main__":
    main()