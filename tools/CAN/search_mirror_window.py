#!/usr/bin/env python3
import sys
import time

# --- This script must be run on a comma device ---
try:
  from panda import Panda
except ImportError:
  print("\n" + "="*50)
  print("This script can only be run on a comma device.")
  print("Please transfer it to your device and run it there.")
  print("="*50 + "\n")
  exit()

# --- Full Command Strings (for verification) ---
LOCK_CMD = b'\x02\x10\x03\x00\x00\x00\x00\x00'
WINDOW_CLOSE_FR = b"\x91\x04\x30\x01\x05\x20\x00\x00"
WINDOW_OPEN_FR  = b"\x91\x04\x30\x01\x05\x10\x00\x00"
MIRROR_FOLD_R   = b"\xA5\x04\x30\x21\x00\x08\x00\x00"
MIRROR_UNFOLD_R = b"\xA5\x04\x30\x21\x00\x04\x00\x00"

# --- Data Payloads Only (for building broadcast commands) ---
WINDOW_CLOSE_DATA = b"\x04\x30\x01\x05\x20\x00\x00"
WINDOW_OPEN_DATA  = b"\x04\x30\x01\x05\x10\x00\x00"
MIRROR_FOLD_DATA   = b"\x04\x30\x21\x00\x08\x00\x00"
MIRROR_UNFOLD_DATA = b"\x04\x30\x21\x00\x04\x00\x00"

GATEWAY_ID = 0x750 # The diagnostic gateway address

def send_gateway_command(full_command_data, verbose=True):
  """
  Sends a command via the diagnostic gateway, including the LOCK prerequisite.
  """
  try:
    with Panda() as panda:
      panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
      panda.can_send(GATEWAY_ID, LOCK_CMD, 0)
      panda.send_heartbeat()
      time.sleep(0.2)
      panda.can_send(GATEWAY_ID, full_command_data, 0)
      panda.send_heartbeat()
      if verbose:
        print(f"  -> Sent command {full_command_data.hex()} to gateway {hex(GATEWAY_ID)}.")
      return True
  except Exception as e:
    if verbose:
      print(f"  -> Panda connection error: {e}")
    return False

def main():
  """
  Main function to run the broadcast byte discovery tool.
  """
  print("\n" + "="*60)
  print("       CAN Broadcast Byte Finder for Mirrors and Windows")
  print("="*60)
  print("\n⚠️  DISCLAIMER ⚠️")
  print("Use with the ignition ON (accessory mode) but the ENGINE OFF.")
  print("-" * 60)

  ids_to_test = [0x00, 0xFF, 0x80]

  while True:
    choice = input("\nWhat would you like to test?\n1. Windows\n2. Mirrors\nEnter choice (1 or 2): ")
    if choice == '1':
      test_name, verify_cmd, action_data, reverse_data, action_name, reverse_name, prep_instructions = \
      "Windows", WINDOW_CLOSE_FR, WINDOW_CLOSE_DATA, WINDOW_OPEN_DATA, "CLOSE", "OPEN", "Please open the front-right window."
      break
    elif choice == '2':
      test_name, verify_cmd, action_data, reverse_data, action_name, reverse_name, prep_instructions = \
      "Mirrors", MIRROR_FOLD_R, MIRROR_FOLD_DATA, MIRROR_UNFOLD_DATA, "FOLD", "UNFOLD", "Please ensure the right mirror is unfolded."
      break
    else:
      print("Invalid choice.")

  print(f"\nGreat! We will now test the '{test_name}' function.")
  print(prep_instructions)

  # --- Step 1: Verify Known Individual Command ---
  print("\n" + "="*60 + "\n         Step 1: Verify Known Individual Command\n" + "="*60)
  input(f"Press Enter to send the '{action_name}' command to the Front Right {test_name[:-1]}...")
  if send_gateway_command(verify_cmd):
      time.sleep(2)
      if input(f"Did the Front Right {test_name[:-1]} {action_name.lower()}? (y/n): ").lower() != 'y':
          print("\n❌ Verification failed. Exiting."); return
      print("\n✅ Great! The known gateway command is working.")
  else:
      print("\n❌ Failed to send CAN message. Exiting."); return

  # --- Step 2: Search for Broadcast Byte ---
  print("\n" + "="*60 + "\n         Step 2: Search for Broadcast Byte\n" + "="*60)
  found_id = None
  for test_id in ids_to_test:
    print(f"\n--- Testing likely broadcast byte: {hex(test_id)} ---")
    broadcast_command = bytes([test_id]) + action_data
    try:
      input(f"Press Enter to send the '{action_name} ALL' command...")
    except KeyboardInterrupt:
      print("\nExiting script."); return
    if send_gateway_command(broadcast_command):
      time.sleep(2)
      response = input(f"Did all {test_name.lower()} {action_name.lower()}? (y/n/q to quit): ").lower()
      if response == 'y':
        print(f"\n🎉 Success! The broadcast byte appears to be {hex(test_id)}.")
        found_id = test_id
        break
      elif response == 'q':
        print("\nQuitting test."); return

  # --- Step 2.5: Focused Mirror Scan (if applicable) ---
  if found_id is None and choice == '2':
      print("\n" + "-"*60 + "\nThe initial scan failed. Starting a focused scan for mirror addresses.")
      if input("Scan from 0xA0 to 0xAF (excluding known IDs)? (y/n): ").lower() == 'y':
          print("\n" + "="*60 + "\n         STARTING FOCUSED MIRROR SCAN\n" + "="*60)
          print(">>> PRESS CTRL+C <<< the moment the mirrors fold.")
          print("="*60 + "\n"); time.sleep(4)
          last_tested_id = None
          try:
              exclusions = {0xA5, 0xA6}
              focused_range = [i for i in range(0xA0, 0xB0) if i not in exclusions]
              for test_id in focused_range:
                  last_tested_id = test_id
                  sys.stdout.write(f"\r--> Focused Scan - Testing Byte: {hex(last_tested_id)}  "); sys.stdout.flush()
                  broadcast_command = bytes([last_tested_id]) + action_data
                  send_gateway_command(broadcast_command, verbose=False)
                  time.sleep(1.2)
          except KeyboardInterrupt:
              print(f"\n\nScan stopped! Last tested byte was {hex(last_tested_id)}.")
              if input(f"Did the mirrors fold with this byte? (y/n): ").lower() == 'y':
                  print(f"\n🎉 Success! Focused scan found the byte: {hex(last_tested_id)}.")
                  found_id = last_tested_id

  # --- Step 3: Brute-force scan if no byte was found ---
  if found_id is None:
    print("\n" + "-"*60 + "\nNo broadcast byte found yet.")
    if input("Start a full brute-force scan from 0x00 to 0xFF? (y/n): ").lower() == 'y':
      print("\n" + "="*60 + "\n         STARTING FULL BRUTE-FORCE SCAN\n" + "="*60)
      print(">>> PRESS CTRL+C <<< the moment the action happens to stop the scan.")
      print("="*60 + "\n"); time.sleep(5)
      last_tested_id = None
      try:
        for test_id in range(256):
          last_tested_id = test_id
          sys.stdout.write(f"\r--> Last Byte Tested: {hex(last_tested_id)}  "); sys.stdout.flush()
          broadcast_command = bytes([last_tested_id]) + action_data
          send_gateway_command(broadcast_command, verbose=False)
          time.sleep(1.2)
      except KeyboardInterrupt:
        print(f"\n\nScan stopped! Last tested byte was {hex(last_tested_id)}.")
        if input(f"Did the action succeed with this byte? (y/n): ").lower() == 'y':
          print(f"\n🎉 Success! Brute-force found the byte: {hex(last_tested_id)}.")
          found_id = last_tested_id

  # --- Final confirmation ---
  if found_id is not None:
    if input(f"\nTest the '{reverse_name} ALL' command to confirm? (y/n): ").lower() == 'y':
      reverse_broadcast_command = bytes([found_id]) + reverse_data
      print(f"\nSending the '{reverse_name} ALL' command...")
      send_gateway_command(reverse_broadcast_command)
      print("\nTest complete!")
  else:
    print("\n" + "-"*60 + "\nTest finished. No working broadcast byte was found.")

if __name__ == "__main__":
  main()