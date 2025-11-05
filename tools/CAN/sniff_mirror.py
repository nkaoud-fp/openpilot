#!/usr/bin/env python3
import os
import re
import time
from collections import defaultdict
from datetime import datetime # Added for timestamps

import cereal.messaging as messaging
try:
    import opendbc
except ImportError:
    print("Warning: opendbc-python not found. Reference DBC mode will be limited.")
    opendbc = None

# --- Configuration for our specific mirror commands ---
GATEWAY_ID    = 0x750
MIRROR_FOLD_R = b"\xA5\x04\x30\x21\x00\x08\x00\x00"
MIRROR_FOLD_L = b"\xA6\x04\x30\x21\x00\x08\x00\x00"
# --- End Configuration ---

LIVE_REFRESH_RATE = 1.2

# ... [All functions from your script like print_header, get_message_ids_from_file, etc. remain unchanged] ...
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

    # --- This menu is simplified for our purpose ---
    print("\n--- Live Mirror Sniffer (based on dbc_discovery.py) ---")
    print("This script will perform DBC discovery AND sniff for mirror commands.")
    print("Press Ctrl+C to stop.\n")
    time.sleep(2)
    
    all_known_ids = set() # Start from scratch for simplicity

    print_header("Live CAN Analysis & Mirror Sniffing")
    print("Press the 'Fold All Mirrors' button to see the commands...")
    time.sleep(2)

    can_sock = messaging.sub_sock('can')
    live_state = defaultdict(lambda: {'payloads': [], 'last_mask': 0, 'signals': [], 'max_dlc': 0})
    last_update_time = 0

    try:
        while True:
            # Using drain_sock as it's proven to work in your script
            can_msgs = messaging.drain_sock(can_sock) 
            for msg in can_msgs:
                for can_msg in msg.can:
                    
                    # --- Start of our custom mirror sniffer logic ---
                    if can_msg.address == GATEWAY_ID:
                        if can_msg.dat == MIRROR_FOLD_R:
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            print(f"\n[{timestamp}] >>> FOLD RIGHT MIRROR Command Captured! <<<\n")
                        elif can_msg.dat == MIRROR_FOLD_L:
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                            print(f"\n[{timestamp}] >>> FOLD LEFT MIRROR Command Captured! <<<\n")
                    # --- End of our custom logic ---
                    
                    # Original dbc_discovery logic continues below
                    msg_id = can_msg.address
                    if msg_id not in all_known_ids:
                        live_state[msg_id]['payloads'].append(can_msg.dat)

            current_time = time.monotonic()
            if current_time - last_update_time < LIVE_REFRESH_RATE:
                continue
            last_update_time = current_time

            # ... [The rest of the dbc_discovery.py analysis logic remains unchanged] ...
            structure_changed = False
            for msg_id, data in live_state.items():
                if len(data['payloads']) < 2: continue
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
                print_header("Live CAN Analysis & Mirror Sniffing")
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