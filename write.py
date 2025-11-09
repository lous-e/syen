"""
collect_digits / write.py
-------------------------
Utility to collect a single labeled writing segment from the ESP32 via
serial and save it as a CSV under the `imu_digits/` directory.

Configure the `PORT` and `BAUD` variables below for your system.
"""

import serial, time, os, csv, re
from pathlib import Path

# ====== set your port here ======
PORT = "COM9"          # e.g., "COM5" on Windows; "/dev/ttyUSB0" on Linux
BAUD = 115200
OUT  = Path("imu_digits")
OUT.mkdir(exist_ok=True)

start_re = re.compile(r"# START,([0-9]),(\d+)")
stop_re  = re.compile(r"# STOP,([0-9]),(\d+)")
header   = ["t_ms","ax","ay","az","gx","gy","gz","state","label"]

def read_line(ser):
  try:
    return ser.readline().decode('utf-8', errors='ignore').strip()
  except:
    return ""

def wait_for_segment(ser, target_label):
  """Wait one WRITING segment for the given label; return rows list."""
  buf = []
  in_seg = False
  while True:
    line = read_line(ser)
    if not line:
      time.sleep(0.002)
      continue

    # markers
    if line.startswith("# START"):
      m = start_re.match(line)
      if m and m.group(1) == target_label:
        in_seg = True
        buf = []
        print(f"[START] label={target_label}")
      continue

    if line.startswith("# STOP"):
      m = stop_re.match(line)
      if m and m.group(1) == target_label and in_seg:
        print(f"[STOP]  label={target_label}  samples={len(buf)}")
        return buf
      continue

    if line.startswith("#"):
      # comment / log lines
      continue

    # data row
    parts = line.split(",")
    if len(parts) == 9:
      if in_seg:
        buf.append(parts)

def main():
  print("Opening serial ...")
  ser = serial.Serial(PORT, BAUD, timeout=0.05)
  time.sleep(1.0)  # let the board reset/settle

  print("Ready. Type a digit (0-9) to record one sample, or 'q' to quit.")
  print("Tip: write each digit in ~1–2 seconds, then return to rest for ~1 s.")

  while True:
    lab = input("\nEnter digit label [0-9] (or q): ").strip()
    if lab.lower() == 'q':
      break
    if len(lab) != 1 or lab < '0' or lab > '9':
      print("Please enter a single digit 0–9.")
      continue

    # send label to ESP32
    ser.write(lab.encode('utf-8'))
    print(f"Label set to {lab}. Begin writing the digit now...")

    # capture exactly ONE WRITING segment for this label
    rows = wait_for_segment(ser, lab)
    if not rows:
      print("No data captured; try again.")
      continue

    # save
    ts = int(time.time()*1000)
    fname = OUT / f"digit_{lab}_{ts}.csv"
    with open(fname, "w", newline="") as f:
      w = csv.writer(f)
      w.writerow(header)
      w.writerows(rows)

    print(f"[SAVED] {fname}  ({len(rows)} samples)")

  ser.close()
  print("Done.")

if __name__ == "__main__":
  main()
