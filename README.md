# Eyeball Controller

A 6-servo control system for an animatronic eye/eyelid rig built around an
Arduino Nano. Two files make up the whole project:

| File | What it is | Runs on |
|---|---|---|
| `eyeball_controller.pde` | A Processing GUI for live-tuning servos, recording motion sequences, and tuning lifelike random movement | Your computer |
| `servo_live_receiver.ino` | Firmware that drives the servos - talks live to the GUI, and falls back to running on its own when nothing's connected | Arduino Nano |

They're designed to be used together: you tune everything live from the
GUI, then use its **Export Sketch** button to bake your settings into a
new copy of the firmware that keeps running the same behavior standalone,
with no computer attached.

## Hardware

- Arduino Nano (tested wiring assumes a NOYITO Nano I/O expansion shield,
  but any Nano wired the same way works)
- 6 hobby servos, by default on pins **3, 5, 6, 9, 10, 11**
  (pin assignment is adjustable per servo in the GUI)
- Optional PIR motion sensor, by default on pin **2**

By default, the servos on **pins 6 and 9** are treated as eyelid ("Blink")
servos; the rest pan like eyeballs. All of this - pins, min/max travel,
center position, active/inactive, and blink behavior - is adjustable per
servo from the GUI.

## Software requirements

- **Processing** 3 or 4, with the Serial library (included by default)
- **Arduino IDE**, with the built-in `Servo` library (no extra libraries
  needed)

## Getting started

1. Open `servo_live_receiver.ino` in the Arduino IDE and upload it to the
   Nano. This is the one you flash first - it starts out as a plain live
   receiver with no autonomous behavior baked in, so it's safe to upload
   before you've tuned anything.
2. Open `eyeball_controller.pde` in Processing and run it.
3. Click **Connect**, pick the Nano's serial port from the list that
   appears, and wait a couple of seconds for the handshake - all 6 servos
   will move to 90°.
4. Tune away. Drag sliders, flip Active/Blink per servo, adjust pins and
   travel limits - everything updates the physical servos live.
5. When you're happy, click **Export Sketch**. This writes
   `export/export_live_receiver.ino` with your current settings and mode
   baked in as its offline fallback behavior. Upload that file to the
   Nano to replace what's running - it's still a full live receiver for
   this GUI, and after a few seconds with no GUI commands it runs the
   baked-in behavior on its own.

Whenever you change anything you care about, **Export Sketch again and
re-upload** - the previous export is deleted automatically each time, so
there's never more than one sitting in the `export/` folder to confuse
you with an out-of-date copy.

## The GUI (`eyeball_controller.pde`)

### Per-servo controls
For each of the 6 servos: pin number, min/max/center travel, an
Active toggle (inactive servos ignore all commands and stay put), and a
Blink toggle (marks it as an eyelid servo rather than a pan/eyeball servo
for Random mode).

Click **Save Settings** to write these to `eye_servo_params.txt` next to
the sketch; they're loaded back automatically every time you launch the
GUI, so you only need to click Save when you actually change something.

### Random mode
Tunes lifelike autonomous motion:
- **Pan speed / Pan hold** - how fast the eyes move, and how long they
  typically pause between moves
- **Blink gap** - typical time between single blinks (no double-blinks)
- **Squint gap / Squint hold** - how often the eyelids close about
  halfway and hold there, and for how long. While squinting, the eyes
  also sweep slowly back and forth near the edges of their travel,
  pausing briefly at each limit, instead of their usual random panning.

Click **Play** to preview all of this live over serial, no export needed.

### Sequence mode
- **Record Frame** saves the current pose of the active servos, along
  with whatever Move time/Hold time the sliders currently show - each
  frame keeps its own speed from the moment it was recorded.
- Frames are written to `eye_sequence.txt` as you go, and reloaded
  automatically on launch.
- **Random Blink** can be toggled on while you're posing frames, so the
  eyelids blink naturally in the background without disturbing the pose
  you're setting up. It pauses automatically whenever Play is actually
  running a saved sequence.
- **Play** plays the saved frames back on a loop, using each frame's own
  recorded timing.
- **Clear** wipes the saved sequence (also clears the file on disk).

### PIR motion sensor (optional)
Set the pin and click **Enable PIR** to have the Nano report live motion
status back to the GUI. When enabled, both Random and Sequence autonomous
playback wait for motion before animating.

## The firmware (`servo_live_receiver.ino`)

### Serial protocol
One command per line, newline-terminated, used by the GUI for live
control:

```
S:<index>:<degrees>     Move servo <index> (0-5) to <degrees> (0-180)
ACTIVE:<index>:<0|1>    Mark servo <index> active/inactive
PIRPIN:<pin>            Set which digital pin the PIR sensor is on
PIRENABLE:<0|1>         Enable/disable PIR reporting
```

When PIR reporting is enabled, the Nano sends a line back whenever the
sensor's state changes:

```
PIR:0   no motion
PIR:1   motion detected
```

### Live receiver + autonomous fallback
While it's hearing commands from the GUI, it behaves purely as a live
receiver. A few seconds after the commands stop, it switches to running
whatever was baked in by the last Export Sketch click - a looped
sequence of saved frames, or lifelike random movement - and switches
straight back to live control the moment a new command arrives. The GUI
sends a small heartbeat every second while connected, so a live tuning
session never accidentally triggers the autonomous fallback just because
you paused to look at something.

## Files this project creates at runtime

| File | Created by | Purpose |
|---|---|---|
| `eye_servo_params.txt` | Save Settings | Per-servo min/max/center/active/blink + PIR settings |
| `eye_sequence.txt` | Record Frame | Saved sequence frames, each with its own move/hold time |
| `export/export_live_receiver.ino` | Export Sketch | Firmware with your current settings/mode baked in, ready to flash |

None of these need to be committed to version control if you'd rather
keep your personal rig tuning out of the repo - add them to
`.gitignore` if you prefer:

```
eye_servo_params.txt
eye_sequence.txt
export/
```

## Notes

- Both files track a shared build tag in their header comments
  (e.g. `Build: 2026-09-26-k`) so it's easy to confirm you're running a
  matching, up-to-date pair.
- Saved-sequence files from before per-frame timing was added still load
  fine; frames missing timing data just fall back to the sliders'
  current values.
