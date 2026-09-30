/*
  servo_live_receiver.ino

  Live-tuning + standalone firmware for eyeball_controller.pde.

  While the GUI is connected and sending commands, this behaves as a live
  receiver: drag sliders, watch servos move immediately. After
  IDLE_TIMEOUT_MS with no commands, it falls back to running on its own
  using whatever AUTO_MODE/settings were baked in below by the last
  "Export Sketch" click - so you can unplug the laptop and it keeps
  animating (saved-frame playback, or lifelike random movement).

  This file starts out with AUTO_MODE = MODE_NONE (pure live receiver, no
  autonomous fallback), so it's safe to flash before you've tuned anything.
  Every time you change settings in the GUI (min/max/center/active/blink,
  PIR, timing, or the saved frames), click Export Sketch again and
  re-upload the result - it produces a new copy of this exact file with
  your latest values baked in, meant to replace this one.

  Protocol (one command per line, newline-terminated), used for live
  tuning from eyeball_controller.pde:
    S:<index>:<degrees>     Move servo <index> (0-5) to <degrees> (0-180)
    ACTIVE:<index>:<0|1>    Mark servo <index> active/inactive.
                            Inactive servos ignore S: commands and stay put.
    PIRPIN:<pin>            Set which digital pin the PIR sensor is on.
    PIRENABLE:<0|1>         Enable/disable PIR reporting.
  When PIR reporting is enabled, this sketch sends a line back to the GUI
  every time the sensor's state changes:
    PIR:0 / PIR:1
*/

#include <Servo.h>

// ====================== BAKED-IN SETTINGS (filled in by Export Sketch) ======================
#define MODE_NONE     0
#define MODE_SEQUENCE 1
#define MODE_RANDOM   2
const uint8_t AUTO_MODE = MODE_NONE;

const uint8_t NUM_SERVOS = 6;
const unsigned long IDLE_TIMEOUT_MS = 3000; // ms with no GUI command before auto playback resumes
const unsigned int STARTUP_HOLD_MS = 2000;

const bool    PIR_ENABLED      = false;
const uint8_t PIR_PIN_DEFAULT  = 2;
const unsigned long PIR_IDLE_MS = 5000; // RANDOM mode: ms to keep animating after last motion

const uint8_t PIN[NUM_SERVOS]      = {3, 5, 6, 9, 10, 11};
const int     MIN_V[NUM_SERVOS]    = {0, 0, 0, 0, 0, 0};
const int     MAX_V[NUM_SERVOS]    = {180, 180, 180, 180, 180, 180};
const int     CENTER[NUM_SERVOS]   = {90, 90, 90, 90, 90, 90};
const bool    ACTIVE0[NUM_SERVOS]  = {true, true, true, true, true, true}; // starting ACTIVE state
const bool    IS_BLINK[NUM_SERVOS] = {false, false, false, false, false, false};

// ---- SEQUENCE mode data (only used when AUTO_MODE == MODE_SEQUENCE) ----
const unsigned int MOVE_MS = 900; // used only for the return-to-center move after a full loop
const uint8_t NUM_KEYFRAMES = 0;
const int KEYFRAMES[1][NUM_SERVOS] = { {0, 0, 0, 0, 0, 0} }; // placeholder, unused when NUM_KEYFRAMES==0
const unsigned int KEYFRAME_MOVE_MS[1] = { 700 }; // placeholder, unused
const unsigned int KEYFRAME_HOLD_MS[1] = { 400 }; // placeholder, unused

// ---- RANDOM mode tuning (only used when AUTO_MODE == MODE_RANDOM) ----
const unsigned int PAN_SPEED_MS   = 900;
const unsigned int PAN_HOLD_MS    = 1500;
const unsigned int BLINK_GAP_MS   = 3500;
const unsigned int BLINK_CLOSE_MS = 70;
const unsigned int BLINK_HOLD_MS  = 30;
const unsigned int BLINK_OPEN_MS  = 120;
const unsigned int SQUINT_CLOSE_MS = 280;   // slower than a blink
const unsigned int SQUINT_OPEN_MS  = 280;
const unsigned int SQUINT_HOLD_MS = 900;      // "a few seconds" - jittered +/-30% at runtime
const unsigned long SQUINT_GAP_MS = 20000;    // "occasionally" - jittered +/-30% at runtime
const float SQUINT_CLOSED_FRAC = 0.5;         // halfway between each servo's MIN_V and MAX_V
const float SQUINT_PAN_AMP = 0.45;         // ~90% full swing (0.45 amplitude each side of center)
const unsigned int SQUINT_PAN_LEG_MS = 1400;   // time to sweep from one limit to the other
const unsigned int SQUINT_PAN_PAUSE_MS = 450;  // pause held at each limit before reversing

// ====================== LIVE RECEIVER STATE ======================
Servo servos[NUM_SERVOS];
bool active[NUM_SERVOS];
String line = "";

unsigned long lastCommandAt = 0;
bool autoRunning = false;

uint8_t pirPin = PIR_PIN_DEFAULT;
bool pirEnabled = PIR_ENABLED;
bool pirPinReady = false;
int lastPirState = -1;

int cur[NUM_SERVOS]; // current commanded position (used by live S: and by SEQUENCE moves)

// ---- SEQUENCE FSM ----
enum SeqState { SEQ_WAIT_MOTION, SEQ_MOVING, SEQ_HOLDING, SEQ_RETURN };
SeqState seqState = SEQ_WAIT_MOTION;
uint8_t seqIndex = 0;
int seqFrom[NUM_SERVOS];
unsigned long seqT0 = 0;

// ---- RANDOM FSM ----
float gaze = 0.5, gazeFrom = 0.5, gazeTo = 0.5;
bool panMoving = false;
unsigned long panStart = 0, nextPanAt = 0;
unsigned int panDur = 100;
enum BlinkState { B_IDLE, B_CLOSING, B_HOLD, B_OPENING };
BlinkState blinkState = B_IDLE;
unsigned long blinkT0 = 0, nextBlinkAt = 0, nextSquintAt = 0;
bool isSquint = false;
unsigned int curHoldMs = BLINK_HOLD_MS; // hold duration in progress (blink or squint)
float squintPanCenter = 0.5;
unsigned long squintPanT0 = 0;
bool randAnimating = true;
unsigned long lastMotionAt = 0;

bool startupDone = false;
unsigned long startupT0 = 0;

float ease1(float t) { return t < 0.5 ? 4 * t * t * t : 1 - pow(-2 * t + 2, 3) / 2; }
float ease2(float t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A0));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    active[i] = ACTIVE0[i];
    servos[i].write(90);          // pre-set so it snaps to 90 the instant it's attached
    servos[i].attach(PIN[i]);
    cur[i] = 90;
  }
  startupT0 = millis();
  lastCommandAt = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleLine(line);
      line = "";
      lastCommandAt = millis();
    } else if (c != '\r') {
      line += c;
    }
  }
  updatePir();

  // Non-blocking startup hold: loop() (and Serial reads above) runs from
  // the very first iteration, so a GUI connecting right at boot can't
  // have its initial commands lost while the Nano is stuck in delay().
  if (!startupDone) {
    if (millis() - startupT0 >= STARTUP_HOLD_MS) {
      startupDone = true;
      for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        if (active[i]) { servos[i].write(CENTER[i]); cur[i] = CENTER[i]; }
      }
      lastCommandAt = millis(); // don't let the startup hold count toward the idle timeout
    }
    return;
  }

  bool shouldAuto = (AUTO_MODE != MODE_NONE) && ((millis() - lastCommandAt) >= IDLE_TIMEOUT_MS);
  if (shouldAuto) {
    if (!autoRunning) { autoRunning = true; autoResume(); }
    autoUpdate();
  } else {
    autoRunning = false;
  }
}

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;

  int firstColon = s.indexOf(':');
  if (firstColon < 0) return;
  String cmd = s.substring(0, firstColon);
  String rest = s.substring(firstColon + 1);

  if (cmd == "PIRPIN") {
    int pin = rest.toInt();
    if (pin != pirPin) {
      pirPin = (uint8_t) pin;
      pirPinReady = false; // re-arm pinMode() for the new pin
      lastPirState = -1;   // force a fresh report on the new pin
    }
    return;
  } else if (cmd == "PIRENABLE") {
    pirEnabled = (rest.toInt() != 0);
    if (!pirEnabled) lastPirState = -1; // re-report fresh next time it's enabled
    return;
  }

  int secondColon = rest.indexOf(':');
  if (secondColon < 0) return;
  int idx = rest.substring(0, secondColon).toInt();
  int val = rest.substring(secondColon + 1).toInt();
  if (idx < 0 || idx >= NUM_SERVOS) return;

  if (cmd == "S") {
    if (active[idx]) {
      val = constrain(val, 0, 180);
      servos[idx].write(val);
      cur[idx] = val;
    }
  } else if (cmd == "ACTIVE") {
    active[idx] = (val != 0);
  }
}

void updatePir() {
  if (!pirEnabled) return;
  if (!pirPinReady) {
    pinMode(pirPin, INPUT);
    pirPinReady = true;
  }
  int state = digitalRead(pirPin);
  if (state != lastPirState) {
    lastPirState = state;
    Serial.print("PIR:");
    Serial.println(state == HIGH ? 1 : 0);
  }
}

// ====================== AUTONOMOUS PLAYBACK ======================
// Runs once IDLE_TIMEOUT_MS has passed with no GUI command. Any command
// from the GUI (including just reconnecting) immediately hands control
// back to the live receiver above; autoResume() re-primes the FSM the
// next time it goes idle so playback restarts cleanly rather than from
// wherever it happened to be interrupted.
void autoResume() {
  unsigned long now = millis();
  if (AUTO_MODE == MODE_SEQUENCE) {
    seqIndex = 0;
    for (int i = 0; i < NUM_SERVOS; i++) seqFrom[i] = cur[i];
    seqState = pirEnabled ? SEQ_WAIT_MOTION : SEQ_MOVING;
    seqT0 = now;
  } else if (AUTO_MODE == MODE_RANDOM) {
    gazeFrom = gaze; gazeTo = gaze; panMoving = false;
    nextPanAt = now + 300;
    blinkState = B_IDLE;
    nextBlinkAt = now + 800;
    nextSquintAt = now + (unsigned long)random((long)(SQUINT_GAP_MS * 0.7), (long)(SQUINT_GAP_MS * 1.3) + 1);
    lastMotionAt = now;
    randAnimating = !pirEnabled;
  }
}

void autoUpdate() {
  unsigned long now = millis();
  if (AUTO_MODE == MODE_SEQUENCE) {
    updateSequence(now);
  } else if (AUTO_MODE == MODE_RANDOM) {
    if (pirEnabled) {
      if (digitalRead(pirPin) == HIGH) lastMotionAt = now;
      randAnimating = (now - lastMotionAt) < PIR_IDLE_MS;
    }
    if (isSquint && blinkState == B_HOLD) updateSquintPan(now);
    else updateRandomPan(now);
    updateRandomBlink(now);
  }
}

// ---- SEQUENCE mode: plays NUM_KEYFRAMES on a loop, non-blocking so the
//      serial line stays responsive the whole time ----
void seqMoveStep(unsigned long now, const int target[NUM_SERVOS], unsigned int durMs) {
  float t = durMs == 0 ? 1.0 : (float)(now - seqT0) / (float)durMs;
  if (t >= 1.0) t = 1.0;
  float e = ease1(t);
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!active[i] || target[i] < 0) continue;
    int v = seqFrom[i] + (int)round((target[i] - seqFrom[i]) * e);
    if (v != cur[i]) { servos[i].write(v); cur[i] = v; }
  }
}

void updateSequence(unsigned long now) {
  switch (seqState) {
    case SEQ_WAIT_MOTION:
      if (!pirEnabled || digitalRead(pirPin) == HIGH) {
        for (int i = 0; i < NUM_SERVOS; i++) seqFrom[i] = cur[i];
        seqT0 = now;
        seqState = SEQ_MOVING;
      }
      break;
    case SEQ_MOVING:
      if (NUM_KEYFRAMES == 0) { seqState = SEQ_RETURN; break; }
      seqMoveStep(now, KEYFRAMES[seqIndex], KEYFRAME_MOVE_MS[seqIndex]);
      if (now - seqT0 >= KEYFRAME_MOVE_MS[seqIndex]) {
        seqT0 = now;
        seqState = SEQ_HOLDING;
      }
      break;
    case SEQ_HOLDING:
      if (now - seqT0 >= KEYFRAME_HOLD_MS[seqIndex]) {
        seqIndex++;
        for (int i = 0; i < NUM_SERVOS; i++) seqFrom[i] = cur[i];
        seqT0 = now;
        if (seqIndex >= NUM_KEYFRAMES) {
          seqIndex = 0;
          seqState = pirEnabled ? SEQ_RETURN : SEQ_MOVING;
        } else {
          seqState = SEQ_MOVING;
        }
      }
      break;
    case SEQ_RETURN: {
      int centerPose[NUM_SERVOS];
      for (int i = 0; i < NUM_SERVOS; i++) centerPose[i] = active[i] ? CENTER[i] : -1;
      seqMoveStep(now, centerPose, MOVE_MS);
      if (now - seqT0 >= MOVE_MS) {
        seqState = SEQ_WAIT_MOTION;
      }
      break;
    }
    default: break;
  }
}

// ---- RANDOM mode: shared "gaze" pan across all pan servos, shared blink
//      state across all blink servos - same model as the live GUI preview ----
// While the lids are held at squint depth, sweep the eyes slowly back and forth
// between the two limits, pausing at each limit before reversing. Runs at its own
// fixed pace (SQUINT_PAN_LEG_MS/PAUSE_MS), independent of the squint hold length -
// it just gets cut off wherever it is when the eyelids reopen.
void updateSquintPan(unsigned long now) {
  unsigned long legMs = SQUINT_PAN_LEG_MS;
  unsigned long pauseMs = SQUINT_PAN_PAUSE_MS;
  unsigned long cycle = 2UL * (legMs + pauseMs);
  unsigned long elapsed = (now - squintPanT0) % cycle;
  float lo = constrain(squintPanCenter - SQUINT_PAN_AMP, 0.0, 1.0);
  float hi = constrain(squintPanCenter + SQUINT_PAN_AMP, 0.0, 1.0);
  float g;
  if (elapsed < legMs) {
    g = lo + (hi - lo) * ease2((float)elapsed / legMs);
  } else if (elapsed < legMs + pauseMs) {
    g = hi;
  } else if (elapsed < 2UL * legMs + pauseMs) {
    g = hi - (hi - lo) * ease2((float)(elapsed - legMs - pauseMs) / legMs);
  } else {
    g = lo;
  }
  gaze = g;
  writeRandomPan(gaze);
}

void writeRandomPan(float g) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!active[i] || IS_BLINK[i]) continue;
    int v = MIN_V[i] + (int)round((MAX_V[i] - MIN_V[i]) * g);
    servos[i].write(v);
    cur[i] = v;
  }
}

void writeRandomLids(float closedFrac) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!active[i] || !IS_BLINK[i]) continue;
    int v = MIN_V[i] + (int)round((MAX_V[i] - MIN_V[i]) * closedFrac);
    servos[i].write(v);
    cur[i] = v;
  }
}

void startRandomPan(unsigned long now) {
  float target;
  if (random(100) < 30) {
    target = gaze + (random(-100, 101) / 1000.0);
  } else {
    target = (random(0, 1001) / 1000.0 + random(0, 1001) / 1000.0) * 0.5;
  }
  target = constrain(target, 0.0, 1.0);
  gazeFrom = gaze;
  gazeTo = target;
  float amp = fabs(gazeTo - gazeFrom);
  panDur = (unsigned int)(PAN_SPEED_MS * (0.3 + 0.7 * amp));
  panStart = now;
  panMoving = true;
  if (amp > 0.5 && random(100) < 12) startRandomBlink(now);
}

void updateRandomPan(unsigned long now) {
  if (panMoving) {
    float t = (float)(now - panStart) / (float)panDur;
    if (t >= 1.0) {
      t = 1.0;
      panMoving = false;
      nextPanAt = now + random(PAN_HOLD_MS / 2, PAN_HOLD_MS * 2 + 1);
    }
    gaze = gazeFrom + (gazeTo - gazeFrom) * ease2(t);
    writeRandomPan(gaze);
  } else if (randAnimating && (long)(now - nextPanAt) >= 0) {
    startRandomPan(now);
  }
}

void startRandomBlink(unsigned long now) {
  if (blinkState == B_IDLE) {
    blinkState = B_CLOSING;
    blinkT0 = now;
    isSquint = false;
    curHoldMs = BLINK_HOLD_MS;
  }
}

void startRandomSquint(unsigned long now) {
  if (blinkState == B_IDLE) {
    blinkState = B_CLOSING;
    blinkT0 = now;
    isSquint = true;
    curHoldMs = random((long)(SQUINT_HOLD_MS * 0.7), (long)(SQUINT_HOLD_MS * 1.3) + 1);
  }
}

// Single blinks only (no double-blink chance) - always waits the full gap.
void scheduleNextRandomBlink(unsigned long now) {
  nextBlinkAt = now + random(BLINK_GAP_MS, BLINK_GAP_MS * 2 + 1);
}

void scheduleNextRandomSquint(unsigned long now) {
  nextSquintAt = now + (unsigned long)random((long)(SQUINT_GAP_MS * 0.7), (long)(SQUINT_GAP_MS * 1.3) + 1);
}

void updateRandomBlink(unsigned long now) {
  float t;
  unsigned int closeMs = isSquint ? SQUINT_CLOSE_MS : BLINK_CLOSE_MS;
  unsigned int openMs  = isSquint ? SQUINT_OPEN_MS  : BLINK_OPEN_MS;
  float target = isSquint ? SQUINT_CLOSED_FRAC : 1.0;
  switch (blinkState) {
    case B_IDLE:
      if (randAnimating && (long)(now - nextSquintAt) >= 0) startRandomSquint(now);
      else if (randAnimating && (long)(now - nextBlinkAt) >= 0) startRandomBlink(now);
      break;
    case B_CLOSING:
      t = (float)(now - blinkT0) / (float)closeMs;
      if (t >= 1.0) {
        writeRandomLids(target); blinkState = B_HOLD; blinkT0 = now;
        if (isSquint) {
          squintPanCenter = constrain(gaze, SQUINT_PAN_AMP, 1.0 - SQUINT_PAN_AMP);
          squintPanT0 = now;
          panMoving = false; // don't let a normal saccade fight the sweep
        }
      }
      else writeRandomLids(target * ease2(t));
      break;
    case B_HOLD:
      if (now - blinkT0 >= curHoldMs) {
        blinkState = B_OPENING; blinkT0 = now;
        if (isSquint) nextPanAt = now + random(200, 600); // resume panning shortly
      }
      break;
    case B_OPENING:
      t = (float)(now - blinkT0) / (float)openMs;
      if (t >= 1.0) {
        writeRandomLids(0.0);
        blinkState = B_IDLE;
        if (isSquint) scheduleNextRandomSquint(now); else scheduleNextRandomBlink(now);
      } else {
        writeRandomLids(target * (1.0 - ease2(t)));
      }
      break;
  }
}
