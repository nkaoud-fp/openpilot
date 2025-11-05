import time
from panda import Panda

# Define the full command strings
GATEWAY_ID    = 0x750
LOCK_CMD      = b'\x02\x10\x03\x00\x00\x00\x00\x00'
MIRROR_FOLD_R = b"\xA5\x04\x30\x21\x00\x08\x00\x00"
MIRROR_FOLD_L = b"\xA6\x04\x30\x21\x00\x08\x00\x00"

# 1. Create a list of messages using the correct 4-item tuple format.
# Each message is a tuple: (address, None, data, bus)
messages_to_send = [
  (GATEWAY_ID, None, LOCK_CMD, 0),
  (GATEWAY_ID, None, MIRROR_FOLD_R, 0),
  (GATEWAY_ID, None, MIRROR_FOLD_L, 0),
]

try:
  with Panda() as panda:
    print("Setting safety mode...")
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

    print(f"Sending {len(messages_to_send)} commands at once...")
    # 2. Send the entire list in one call
    panda.can_send_many(messages_to_send)
    panda.send_heartbeat()

    print("\nCommands sent successfully!")

except Exception as e:
  print(f"An error occurred: {e}")