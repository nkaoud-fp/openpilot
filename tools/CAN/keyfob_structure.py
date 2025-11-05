#!/usr/bin/env python3
import os
import time
from collections import defaultdict

try:
    import cereal.messaging as messaging
except ImportError:
    print("Error: cereal library not found. Make sure you are running this on a comma device.")
    exit()

# Using the 1.2 second delay from your saved preferences for the live analysis refresh rate.
LIVE_REFRESH_RATE = 1.2

def print_header(title):
    """Clears the screen and prints a formatted header."""
    os.system('clear')
    print("=" * 70)
    print(f"| {title.center(66)} |")
    print("=" * 70)

def to_motorola_bit(bit_index):
    """Converts a standard little-endian bit index to big-endian (Motorola) format."""
    byte_num, bit_in_byte = bit_index // 8, bit_index % 8
    return byte_num * 8 + (7 - bit_in_byte)

def get_signals_from_mask(mask, max_dlc):
    """Calculates the list of guessed signals from a bitmask of changing bits."""
    signals = []
    max_bits = max_dlc * 8
    in_signal = False
    start_lsb = 0
    # Add a sentinel bit at the end to close any open signals
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
    return sorted(signals, key=lambda x: x['start_bit'])

def main():
    """Main function to capture and analyze a single CAN message ID."""
    
    # --- Get User Input ---
    target_id_str = input("Enter the Message ID to analyze (e.g., 343 or 0x157): ").strip()
    if not target_id_str:
        print("Error: No ID provided.")
        return
        
    try:
        # Handle decimal or hex (0x...) input
        target_id = int(target_id_str, 0)
    except ValueError:
        print(f"Error: Invalid Message ID '{target_id_str}'")
        return

    # --- Initialization ---
    title = f"Live Analysis for ID: {target_id} (0x{target_id:X})"
    print_header(title)
    print("Starting capture...")
    print("Perform actions in the car to change the message's data.")
    print("Press Ctrl+C to stop and show the final DBC structure.")
    time.sleep(2)

    can_sock = messaging.sub_sock('can')
    # Use a set to store unique payloads efficiently
    payloads = set()
    last_mask = 0
    last_update_time = 0
    
    try:
        while True:
            can_msgs = messaging.drain_sock(can_sock)
            for msg in can_msgs:
                for can_msg in msg.can:
                    if can_msg.address == target_id:
                        payloads.add(can_msg.dat)
            
            current_time = time.monotonic()
            if current_time - last_update_time < LIVE_REFRESH_RATE:
                continue
            last_update_time = current_time
            
            if len(payloads) < 2:
                continue

            # --- Analyze and Display ---
            payload_list = list(payloads)
            max_dlc = max(len(p) for p in payload_list) if payload_list else 0
            
            # Pad all payloads to the max length for correct XORing
            padded_payloads_int = [int.from_bytes(p.ljust(max_dlc, b'\x00'), 'big') for p in payload_list]
            
            first_msg_int = padded_payloads_int[0]
            current_mask = 0
            for p_int in padded_payloads_int[1:]:
                current_mask |= (first_msg_int ^ p_int)

            # Update display only if the structure has changed
            if current_mask != last_mask:
                last_mask = current_mask
                signals = get_signals_from_mask(current_mask, max_dlc)
                
                print_header(title)
                print(f"Unique payloads seen: {len(payloads)}")
                print(f"Max data length (DLC): {max_dlc} bytes")
                print("-" * 70)
                print(f"BO_ {target_id} MSG_{target_id}: {max_dlc} XXX")
                for sig in signals:
                    print(f"  SG_ {sig['name']:<12} : {sig['start_bit']:>2}|{sig['length']<2}@0- (1,0) [0|0] \"\" XXX")
                print("-" * 70)

    except KeyboardInterrupt:
        print("\n\n--- Capture Stopped by User ---")
        if last_mask == 0:
            print("No changing bits were detected.")
            return

        signals = get_signals_from_mask(last_mask, max_dlc)
        print("Final discovered DBC structure:\n")
        print("```dbc")
        print(f"BO_ {target_id} MSG_{target_id}: {max_dlc} XXX")
        for sig in signals:
            print(f"  SG_ {sig['name']:<12} : {sig['start_bit']:>2}|{sig['length']<2}@0- (1,0) [0|0] \"\" XXX")
        print("```")
        print("---------------------------------")

if __name__ == "__main__":
    main()