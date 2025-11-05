#!/usr/bin/env python3
import sys
import threading
import re

#python can_stream.py | python stream_filter.py

# --- Global variables for state management ---
filter_string = ""
is_paused = False
pattern_lock = threading.Lock()

# ANSI color codes for highlighting
HIGHLIGHT = "\033[91m"  # Bright Red
RESET = "\033[0m"

def print_help():
  """Prints the help message to the console."""
  help_text = """
--- Interactive CAN Sniffer Help ---
Enter a filter pattern and press Enter.

- ID Filter: Type a number (decimal or hex).
  Example: 1872  or  0x750

- Data Filter: Type any other text string.
  Example: a50430

- AND Filter: Require multiple conditions (space-separated).
  Example: 1872 a504

- OR Filter: Match any condition separated by '|'.
  Example: 1872 | 1880

- NOT Filter: Exclude a condition by prefixing it with '!'.
  Example: !452

--- Commands ---
pause, resume, clear, help, Ctrl+C (exit)
"""
  print(help_text, file=sys.stderr)

def update_thread():
  """A thread that waits for user input to update the filter or run commands."""
  global filter_string, is_paused
  while True:
    try:
      new_input = input()
      with pattern_lock:
        cmd = new_input.lower().strip()
        if cmd == 'pause':
          is_paused = True
          print("--- Output Paused ---", file=sys.stderr)
        elif cmd == 'resume':
          is_paused = False
          print("--- Output Resumed ---", file=sys.stderr)
        elif cmd == 'clear':
          filter_string = ""
          print("--- Filters Cleared ---", file=sys.stderr)
        elif cmd == 'help':
          print_help()
        else:
          filter_string = new_input
          if new_input.strip():
            print(f"--- Filter updated to: '{new_input}' ---", file=sys.stderr)
          else:
            print("--- Filters Cleared ---", file=sys.stderr)
    except (EOFError, KeyboardInterrupt):
      break

def is_id_pattern(p):
  """Checks if a string pattern is a valid number (hex or dec)."""
  if p.startswith('!'):
    p = p[1:]
  try:
    int(p, 0)
    return True
  except ValueError:
    return False

def main():
  print_help()
  updater = threading.Thread(target=update_thread, daemon=True)
  updater.start()

  # Regex to extract the decimal ID from a log line
  id_regex = re.compile(r'\((\d+)\)')

  try:
    for line in sys.stdin:
      with pattern_lock:
        paused = is_paused
        current_filter = filter_string

      if paused or not current_filter.strip():
        # If paused or the filter is empty, just print the raw line
        if not paused:
            print(line, end='')
        continue
      
      # Extract the decimal ID from the line
      match = id_regex.search(line)
      if not match:
        continue
      line_id = int(match.group(1))

      # --- Advanced Filtering Logic ---
      or_groups = current_filter.split('|')
      line_matches = False
      for group in or_groups:
        patterns = group.strip().split()
        if not patterns:
          continue

        # Separate patterns into ID filters and data filters
        id_filters = {p for p in patterns if is_id_pattern(p)}
        data_filters = {p for p in patterns if not is_id_pattern(p)}
        
        # Check if the line's ID matches all ID filters
        id_match = True
        for p in id_filters:
          negate = p.startswith('!')
          p_val = int(p[1:] if negate else p, 0)
          if (line_id == p_val) == negate: # XOR logic for NOT
            id_match = False
            break
        if not id_match:
          continue

        # Check if the line contains all data filters
        data_match = True
        for p in data_filters:
          negate = p.startswith('!')
          p_val = p[1:] if negate else p
          if (p_val in line) == negate: # XOR logic for NOT
            data_match = False
            break
        if not data_match:
          continue
        
        # If both ID and data filters for this group pass, the line is a match
        line_matches = True
        break
      # --- End of Filtering Logic ---

      if line_matches:
        # --- Highlighting Logic ---
        highlighted_line = line
        all_patterns = [p[1:] if p.startswith('!') else p for p in re.split(r'\s|\|', current_filter) if p]
        for p in set(all_patterns):
            highlighted_line = highlighted_line.replace(p, f"{HIGHLIGHT}{p}{RESET}")
        print(highlighted_line, end='')

  except KeyboardInterrupt:
    print("\n--- Sniffer stopped. ---", file=sys.stderr)

if __name__ == "__main__":
    main()