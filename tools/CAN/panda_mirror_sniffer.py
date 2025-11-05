#!/usr/bin/env python3
import time
from datetime import datetime
from panda import Panda

# --- Configuration ---
GATEWAY_ID    = 0x750 # Correct hexadecimal value
MIRROR_FOLD_R = b"\xA5\x04\x30\x21\x00\x08\x00\x00"
MIRROR_FOLD_L = b"\xA6\x04\x30\x21\x00\x08\x00\x00"

def main():
  print("="*60)
  print("         ✅ Direct Panda Hardware Sniffer ✅")
  print("="*60)
  print("\nThis script connects directly to the Panda, bypassing all")
  print("other software. This is the final test.")
  print("\n>>> Please press the 'Fold All Mirrors' button in your car. <<<")
  print("\nPress Ctrl+C to stop sniffing.")
  print("-" * 60)

  try:
    # Connect directly to the Panda hardware
    with Panda() as p:
      while True:
        # can_recv() gets all messages buffered in the panda
        can_data = p.can_recv()

        if can_data:
          timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
          for address, _, data, bus in can_data:
            # Check if the address matches our gateway ID
            if address == GATEWAY_ID:

              if data == MIRROR_FOLD_R:
                print(f"[{timestamp}] >>> FOLD RIGHT MIRROR Command Captured! <<<")
              
              elif data == MIRROR_FOLD_L:
                print(f"[{timestamp}] >>> FOLD LEFT MIRROR Command Captured! <<<")
        
        # Small sleep to prevent pegging the CPU
        time.sleep(0.01)

  except Exception as e:
    print(f"\nAn error occurred: {e}")
    if "Panda not found" in str(e):
      print("Please ensure the comma device is connected and the car is on.")

if __name__ == "__main__":
    main()