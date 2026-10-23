# Dextrain with Python

Read the Dextrain Manipulandum from Python, without Unity and without `DAQMANIPULANDUM.dll`.

The script talks to the NI acquisition card directly through the `nidaqmx` library.
It reads the 5 finger channels continuously at up to 1000 Hz per channel.
The old DLL could only do about 30 Hz.

## What you need

- Windows
- The **NI-DAQmx** driver from National Instruments: https://www.ni.com/en/support/downloads/drivers/download.ni-daq-mx.html
- Python 3.8 or newer
- The Dextrain plugged in by USB. It must show up as `Dev1` in NI MAX.

Install the Python package:

```
pip install -r requirements.txt
```

## Quick start

```
python daq_continuous.py
```

1. **Don't touch the device for the first second.** The script measures the rest position (the "zero") during that time.
2. You then see the 5 forces update live in the terminal. `TAP` shows up when a tap is detected.
3. Press **Ctrl+C** to stop. The script then prints some stats (sample rate, delays, errors).

## Options

| Option | What it does | Default |
|---|---|---|
| `--csv FILE` | Save every sample to a CSV file | no file |
| `--duration SEC` | Stop after this many seconds | runs until Ctrl+C |
| `--rate HZ` | Samples per second, per channel (max 1000) | 1000 |
| `--calibration FILE` | Use your device's calibration file (see below) | Unity default values |
| `--zero-seconds SEC` | How long to measure the zero at start | 1 |
| `--raw-only` | Show raw voltages only: no zero, no calibration, no tap detection | off |
| `--display-hz HZ` | How often the terminal updates. Keep it low, a fast display slows down reading. | 5 |

Example: record 60 seconds with your calibration file.

```
python daq_continuous.py --duration 60 --csv session1.csv --calibration calib.txt
```

## Calibration file

This is the same calibration file the Unity app uses. It is a text file with tab-separated values:

- line 4: `g1` gains for the 5 channels
- line 5: `g2` gains
- line 6: `c2` offsets

The first column of each line is a label and is skipped.

Without a calibration file, the script uses the Unity defaults (`g1 = 1.6`, `g2 = 0.85`, `c2 = 0.45`).
The forces will then be roughly right, but not exact for your device.

## CSV format

```
t_perf_s, t_wall_s, ai4, ai3, ai2, ai1, ai0
```

- `t_perf_s`: high-precision timer in seconds. Use it to measure time between samples.
- `t_wall_s`: Unix time in seconds. Use it to line up with other recordings.
- `ai4` to `ai0`: **raw voltages** (V), before zero and calibration.

Raw values are saved on purpose, so you can redo the zero or the calibration later.

## How it works

- The NI task is created once and runs continuously, clocked by the card itself.
  The old DLL created and destroyed a new task on every read, which made it slow.
- One thread only reads data from the card. Another thread writes the CSV.
  The main thread only updates the display.
- Zero, calibration and tap detection follow the same math as the Unity script.
  The raw voltages were checked against the old DLL and match within 0.0006 V.

## Known limits

- The card (NI USB-6000) has a single converter shared by all channels: 10000 samples/s in total, so 2000 Hz per channel at most for 5 channels.
  The script caps it at 1000 Hz.
- A few reads arrive late, sometimes by 20 ms or more.
  This looks like a limit of the card's USB link, not a software issue.
  No data is lost: late samples are simply delivered together.
