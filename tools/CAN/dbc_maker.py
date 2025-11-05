#!/usr/bin/env python3
import os
import re
import time
from collections import defaultdict

import cereal.messaging as messaging
try:
    import opendbc
except ImportError:
    print("Warning: opendbc-python not found. Reference DBC mode will be limited.")
    opendbc = None

#
## HOW TO RUN:
##   python3 /data/dbc_maker.py
#
#  The script provides a real-time dashboard of discovered message structures.
#  Perform actions in your car and watch the structures evolve on screen.
#

# Using the 1.2 second delay from your saved preferences for the live analysis refresh rate.
LIVE_REFRESH_RATE = 1.2

def print_header(title):
    os.system('clear')
    print("="*70)
    print(f"| {title.center(66)} |")
    print("="*70)

def get_message_ids_from_file(dbc_file_path):
    if not os.path.exists(dbc_file_path): return set()
    known_ids = set()
    msg_id_regex = re.compile(r"^BO_ (\d+)")
    try:
        with open(dbc_file_path, 'r', errors='ignore') as f:
            for line in f:
                match = msg_id_regex.match(line)
                if match: known_ids.add(int(match.group(1)))
    except Exception as e:
        print(f"⚠️ Error reading DBC {dbc_file_path}: {e}")
    return known_ids

def parse_dbc_into_blocks(dbc_file_path):
    if not os.path.exists(dbc_file_path): return {}
    blocks = {}
    current_block, current_id = [], None
    msg_id_regex = re.compile(r"^BO_ (\d+)")
    try:
        with open(dbc_file_path, 'r', errors='ignore') as f:
            for line in f:
                match = msg_id_regex.match(line)
                if match:
                    if current_id is not None: blocks[current_id] = "".join(current_block)
                    current_id = int(match.group(1))
                    current_block = [line]
                elif current_id is not None:
                    current_block.append(line)
        if current_id is not None: blocks[current_id] = "".join(current_block)
    except Exception as e:
        print(f"⚠️ Error parsing DBC {dbc_file_path}: {e}")
    return blocks

def to_motorola_bit(bit_index):
    byte_num, bit_in_byte = bit_index // 8, bit_index % 8
    return byte_num * 8 + (7 - bit_in_byte)

def get_signals_from_mask(mask, max_dlc):
    """Calculates the list of guessed signals from a bitmask."""
    signals = []
    max_bits = max_dlc * 8
    in_signal = False
    start_lsb = 0
    for i in range(max_bits + 1):
        is_set = (mask >> i) & 1
        if is_set and not in_signal:
            in_signal = True
            start_lsb = i
        elif not is_set and in_signal:
            in_signal = False
            end_msb = i - 1
            length = end_msb - start_lsb + 1
            start_bit_motorola = to_motorola_bit(end_msb)
            signals.append({'start_bit': start_bit_motorola, 'length': length, 'name': f'SIG_{start_bit_motorola}'})
    return signals

def update_dbc_with_final_state(final_state, output_filename):
    """Intelligently updates a DBC file with the final discovered structures."""
    print(f"\nUpdating DBC file: {output_filename}")
    existing_blocks = parse_dbc_into_blocks(output_filename)
    new_block_count = 0

    for msg_id, data in final_state.items():
        if msg_id not in existing_blocks and data['signals']:
            max_dlc = data['max_dlc']
            block_lines = [f"BO_ {msg_id} MSG_{msg_id}: {max_dlc} XXX\n"]
            for sig in sorted(data['signals'], key=lambda x: x['start_bit']):
                block_lines.append(f" SG_ {sig['name']} : {sig['start_bit']}|{sig['length']}@0- (1,0) [0|0] \"\" XXX\n")
            existing_blocks[msg_id] = "".join(block_lines)
            new_block_count += 1

    if new_block_count == 0:
        print("No new message structures to add. File is up-to-date.")
        return

    print(f"Adding definitions for {new_block_count} new messages...")
    try:
        with open(output_filename, 'w') as f:
            f.write('VERSION ""\n\nBS_:\n\nBU_: XXX\n\n')
            for msg_id in sorted(existing_blocks.keys()):
                f.write(existing_blocks[msg_id] + "\n")
        print(f"✅ Successfully updated '{output_filename}'.")
    except Exception as e:
        print(f"❌ Error writing DBC file: {e}")

def main():
    source_ids = set()
    output_filename = "/data/live_analyzed.dbc"

    # --- Interactive Menu ---
    # ... [Menu code from dbc_analyzer.py is unchanged] ...
    while True:
        print("\n--- DBC Live Analyzer ---")
        print("1: Use a reference DBC (find and analyze unknown messages)")
        print("2: Start from scratch (discover and analyze all messages)")
        print("3: Exit")
        choice = input("Enter your choice [1, 2, or 3]: ")

        if choice == '1':
            if opendbc is None:
                print("❌ opendbc-python library not found. Cannot use reference mode.")
                continue
            default_dbc = "toyota_nodsu_pt_generated"
            source_name = input(f"Enter the source DBC name from opendbc [{default_dbc}]: ") or default_dbc
            opendbc_dir = os.path.dirname(opendbc.__file__)
            source_dbc_path = os.path.join(opendbc_dir, f"{source_name.lower().replace('.dbc', '')}.dbc")
            print(f"Loading reference DBC: {source_dbc_path}")
            source_ids = get_message_ids_from_file(source_dbc_path)

            if not source_ids:
                print(f"❌ Error: Source DBC '{source_name}' not found or is empty. Please try again.")
                continue
            break
        elif choice == '2':
            print("🚀 Starting in 'scratch' mode. All messages will be analyzed.")
            break
        elif choice == '3':
            print("Exiting.")
            return
        else:
            print("Invalid choice.")

    user_output_path = input(f"Enter the output DBC file path [{output_filename}]: ")
    if user_output_path.strip(): output_filename = user_output_path.strip()

    existing_generated_ids = get_message_ids_from_file(output_filename)
    all_known_ids = source_ids.union(existing_generated_ids)

    print_header("Live CAN Analysis")
    print(f"Target DBC: {output_filename}")
    print(f"Ignoring {len(all_known_ids)} known message IDs.")
    print("Press Ctrl+C to stop and save the final DBC.")
    print("\nStarting live analysis...")
    time.sleep(2)

    can_sock = messaging.sub_sock('can')
    live_state = defaultdict(lambda: {'payloads': [], 'last_mask': 0, 'signals': [], 'max_dlc': 0})
    last_update_time = 0

    try:
        while True:
            can_msgs = messaging.drain_sock(can_sock)
            for msg in can_msgs:
                for can_msg in msg.can:
                    msg_id = can_msg.address
                    if msg_id not in all_known_ids:
                        live_state[msg_id]['payloads'].append(can_msg.dat)

            current_time = time.monotonic()
            if current_time - last_update_time < LIVE_REFRESH_RATE:
                continue
            last_update_time = current_time

            structure_changed = False
            for msg_id, data in live_state.items():
                if len(data['payloads']) < 2: continue

                # Calculate current changing bits mask
                max_dlc = max(len(p) for p in data['payloads'])
                first_msg_int = int.from_bytes(data['payloads'][0].ljust(max_dlc, b'\x00'), 'big')
                current_mask = 0
                for p in data['payloads'][1:]:
                    current_mask |= (first_msg_int ^ int.from_bytes(p.ljust(max_dlc, b'\x00'), 'big'))

                if current_mask != data['last_mask']:
                    structure_changed = True
                    data['last_mask'] = current_mask
                    data['max_dlc'] = max_dlc
                    data['signals'] = get_signals_from_mask(current_mask, max_dlc)
                    data['last_change_time'] = current_time

            if structure_changed:
                print_header("Live CAN Analysis")
                for msg_id in sorted(live_state.keys()):
                    data = live_state[msg_id]
                    if not data['signals']: continue

                    change_indicator = ""
                    if 'last_change_time' in data and current_time - data['last_change_time'] < (LIVE_REFRESH_RATE + 0.5):
                       change_indicator = "    <-- STRUCTURE EVOLVED!"

                    print(f"\n--- ID: {msg_id} (0x{msg_id:X}) ---{change_indicator}")
                    for sig in sorted(data['signals'], key=lambda x: x['start_bit']):
                        print(f"  SG_ {sig['name']:<12} : {sig['start_bit']:>3}|{sig['length']<2}")

    except KeyboardInterrupt:
        print("\n\n--- Monitoring Stopped by User ---")
        if not live_state:
            print("✅ No new messages were discovered.")
            return
        
        update_dbc_with_final_state(live_state, output_filename)
        print("------------------------------------")

if __name__ == "__main__":
    main()