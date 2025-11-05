#!/usr/bin/env python3
import os
import time
import cereal.messaging as messaging
import struct
from datetime import datetime

# --- Configuration ---
# How many messages to capture to determine the baseline
SAMPLE_SIZE = 100

def main():
  os.system('clear')
  print("="*60)
  print("             ✅ Live Bitmask Analyzer ✅")
  print("="*60)

  # --- User Input for Target ID ---
  target_id_str = input("\nEnter the busy CAN ID to analyze (e.g., 394 or 0x18a): ")
  try:
    target_id = int(target_id_str, 0)
  except ValueError:
    print(f"\n❌ Invalid ID: '{target_id_str}'. Exiting.")
    return

  can_sock = messaging.sub_sock('can')

  # --- Phase 1: Learning ---
  print(f"\n--- Phase 1: Learning Baseline for ID {hex(target_id)} ---")
  print(f"Collecting {SAMPLE_SIZE} messages. Please wait and do not perform any actions...")
  
  sample_messages = []
  while len(sample_messages) < SAMPLE_SIZE:
    can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)
    for msg in can_msgs:
      for can_msg in msg.can:
        if can_msg.address == target_id and len(can_msg.dat) == 8:
          sample_messages.append(can_msg.dat)
          print(f"\rCollected {len(sample_messages)}/{SAMPLE_SIZE} messages...", end="")
  
  print("\nSample collection complete.")
  print("\n--- Phase 2: Analyzing Bitmask ---")

  # --- Phase 2: Analysis ---
  # Convert byte strings to 64-bit integers
  sample_ints = [struct.unpack('>Q', msg)[0] for msg in sample_messages]

  # Calculate masks
  bits_always_one = 0xFFFFFFFFFFFFFFFF
  bits_always_zero = 0xFFFFFFFFFFFFFFFF
  for msg_int in sample_ints:
    bits_always_one &= msg_int
    bits_always_zero &= ~msg_int

  static_mask = bits_always_one | bits_always_zero
  changing_mask = ~static_mask
  static_bits_values = bits_always_one

  print(f"Analysis complete.")
  print(f"  - Changing Bits (Noise): {bin(changing_mask)}")
  print(f"  - Stable Bits (Signal):  {bin(static_mask)}")
  print("-" * 60)

  # --- Phase 3: Monitoring ---
  print("\n--- Phase 3: Monitoring for changes in STABLE bits ---")
  print(">>> You may now perform an action in the car. <<<")

  try:
    while True:
      can_msgs = messaging.drain_sock(can_sock, wait_for_one=True)
      for msg in can_msgs:
        for can_msg in msg.can:
          if can_msg.address == target_id and len(can_msg.dat) == 8:
            msg_int = struct.unpack('>Q', can_msg.dat)[0]
            
            # Check if the stable bits in this message have changed from the baseline
            if (msg_int & static_mask) != static_bits_values:
              timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
              changed_bits = (msg_int ^ static_bits_values) & static_mask
              
              print("\n" + "="*60)
              print(f"🚨 STABLE BIT CHANGED! @ {timestamp} 🚨")
              print(f"  - Full Data:      {can_msg.dat.hex()}")
              print(f"  - Changed Bits:   {bin(changed_bits)}")
              print("="*60 + "\n")

              # The baseline is now invalid, so we update it with the new information
              # This is an adaptive learning step
              print("Adapting to new baseline...")
              sample_ints.append(msg_int)
              bits_always_one &= msg_int
              bits_always_zero &= ~msg_int
              static_mask = bits_always_one | bits_always_zero
              changing_mask = ~static_mask
              static_bits_values = bits_always_one
              print(f"New Changing Bits: {bin(changing_mask)}")
              print("Monitoring again...")

  except KeyboardInterrupt:
    print("\n\n" + "="*60)
    print("Analyzer stopped by user.")
    print("="*60)

if __name__ == "__main__":
    main()