/*
  eyeball_controller.pde
  Build: 2026-09-26-k  (each recorded frame keeps its own move/hold speed)

  A 6-servo control GUI for an animatronic eye/eyelid rig on an Arduino
  Nano (tested layout: NOYITO Nano I/O expansion shield).

  What it does:
   - Connects to the Nano over serial and drives 6 servos live via sliders.
   - Lets you set, per servo: min travel, max travel, center position,
     active/inactive (all 6 start INACTIVE - turn on the ones you're using),
     and whether it behaves like an eyelid ("Blink") or an eyeball/pan servo
     for random-motion export.
   - All 6 servos move to 90 degrees the moment you connect (matching
     servo_live_receiver.ino, which also snaps to 90 on its own boot),
     regardless of active state. "Go To Center" sends every ACTIVE servo
     to its saved center position.
   - Optional PIR motion sensor support: set the pin (defaults to pin 2),
     click Enable PIR, and servo_live_receiver.ino will report live motion
     status back to the GUI. When enabled, both exported sketches wait for
     motion before animating.
   - Settings (min/max/center/active/blink) auto-save to eye_servo_params.txt
     whenever you click Save Settings, and auto-load from that file every
     time you launch the sketch - so click Save Settings before closing if
     you've changed anything, and it'll be there next time. Load Settings
     is still there if you want to revert mid-session.
   - Two modes:
       RANDOM   - tune pan speed/hold and blink timing. The Play button
                  runs a live lifelike random-motion preview (eyes pan,
                  eyelids blink once at a time, occasionally squinting
                  halfway shut for a few seconds) right in the GUI, no
                  export needed to see it work.
       SEQUENCE - "Record Frame" saves the current pose of the active
                  (selected) servos as a frame; frames are written to
                  eye_sequence.txt next to the sketch as you go, and
                  reloaded automatically on launch. The Play button plays
                  those saved frames back live over serial on a loop.
   - Export Sketch writes export/export_live_receiver.ino: the same live
     receiver, with your current mode (saved frames or random movement),
     each servo's min/max/center travel, and the PIR pin/enable setting
     baked in as its offline fallback. Upload it to replace whatever
     firmware is currently on the Nano - it still works as a live receiver
     for this GUI, and after a few seconds with no GUI commands it runs
     the baked-in behavior on its own. Whenever you change settings,
     Export Sketch again and re-upload - the old exported file is deleted
     automatically each time so there's never more than one to mix up.

  Requires: Processing 3 or 4, with the Serial library (built in).
  Pair with: servo_live_receiver.ino (upload that to the Nano first,
  before using this GUI - it's what the sliders talk to).

  This does not claim to be a byte-for-byte reference implementation of
  any particular commercial product; it was written from scratch, with
  the Slider/Button widget pattern and general workflow (live control,
  save/load settings, export a standalone sketch) inspired by the
  Servo16.pde example the user provided as a style reference.
  // lines 170, 241, 247
*/

import processing.serial.*;
import java.util.ArrayList;
import java.io.File;

// ====================== CONSTANTS ======================
final int NUM_SERVOS = 6;
final int WIN_W = 900;
final int WIN_H = 760;

final color BG            = color(238);
final color PANEL         = color(250);
final color TEXT_COL      = color(30);
final color SLIDER_TRACK  = color(190, 205, 220);
final color SLIDER_LIMIT  = color(255, 150, 90);
final color HANDLE_COL    = color(90, 140, 200);
final color HANDLE_INACT  = color(180);
final color BTN_COL       = color(210, 225, 240);
final color BTN_HOVER     = color(160, 195, 230);
final color BTN_ON        = color(120, 210, 120);
final color BTN_OFF       = color(230, 120, 120);
final color BTN_DANGER    = color(230, 140, 120);
final color BTN_PRIMARY   = color(110, 175, 235);

// ====================== SERVO DATA ======================
String[]  servoLabel   = {"Eyeball Left", "Eyeball Right", "EyeLid Left", "EyeLid Right", "Servo5", "Servo6"};
int[]     servoPin     = {3, 5, 6, 9, 10, 11};
int[]     servoMin     = new int[NUM_SERVOS];
int[]     servoMax     = new int[NUM_SERVOS];
int[]     servoCenter  = new int[NUM_SERVOS];
boolean[] servoActive  = new boolean[NUM_SERVOS];
boolean[] servoBlink   = new boolean[NUM_SERVOS]; // true = eyelid-style for random export
Slider[]  servoSlider  = new Slider[NUM_SERVOS];

// ====================== SERIAL ======================
Serial arduinoPort;
boolean connected = false;
String[] portList;
boolean showPortList = false;
String statusMsg = "Not connected.";
int statusMsgTime = 0;
boolean startupPending = false;
int connectedAt = 0;
final int STARTUP_SEND_DELAY_MS = 2000; // let the Nano finish its own boot before we send anything
int lastHeartbeatAt = 0;
final int HEARTBEAT_INTERVAL_MS = 1000; // keeps the Nano's idle clock from elapsing just because you paused

// ====================== PIR ======================
int pirPin = 2;
boolean pirEnabled = false;
boolean pirDetected = false;
int pirDetectedTime = 0;
String serialBuffer = "";

// ====================== MODE ======================
final String MODE_RANDOM   = "RANDOM";
final String MODE_SEQUENCE = "SEQUENCE";
String mode = MODE_RANDOM;

// ---- Random-mode tuning (values are "typical"; the exported sketch
//      randomizes each move somewhat around them, same idea as the
//      earlier animatronic_eyes.ino) ----
Slider panSpeedSlider;    // ms for a full-range eye saccade
Slider panHoldSlider;     // ms typically spent fixated between moves
Slider blinkGapSlider;    // ms typically between blinks
Slider squintGapSlider;   // ms typically between squints
Slider squintHoldSlider;  // ms typically held at squint depth

// ---- Random-mode live preview state (mirrors the exported sketch's logic) ----
boolean randomPreviewing = false;
float rGaze = 0.5, rGazeFrom = 0.5, rGazeTo = 0.5;
boolean rPanMoving = false;
int rPanStart = 0, rNextPanAt = 0, rPanDur = 100;
final int R_IDLE = 0, R_CLOSING = 1, R_HOLD = 2, R_OPENING = 3;
int rBlinkState = R_IDLE;
int rBlinkT0 = 0, rNextBlinkAt = 0, rNextSquintAt = 0;
boolean rIsSquint = false;
int rHoldMs = 30; // current hold duration in progress (blink or squint - set when it starts)
final int R_BLINK_CLOSE_MS = 70, R_BLINK_HOLD_MS = 30, R_BLINK_OPEN_MS = 120;
final int R_SQUINT_CLOSE_MS = 280, R_SQUINT_OPEN_MS = 280;   // slower than a blink
final float R_SQUINT_CLOSED_FRAC = 0.5; // halfway between each servo's MIN_V and MAX_V
final float R_SQUINT_PAN_AMP = 0.45; // how far the eyes sweep from center while squinting (0.45 = ~90% full swing)
final int R_SQUINT_PAN_LEG_MS = 1400;   // time to sweep from one limit to the other (slow, deliberate)
final int R_SQUINT_PAN_PAUSE_MS = 450;  // pause held at each limit before reversing
float rSquintPanCenter = 0.5;
int rSquintPanT0 = 0;

// ---- Sequence-mode data ----
class Keyframe {
  int[] pos = new int[NUM_SERVOS];
  int moveMs = 700; // captured from the sliders at record time, so each frame keeps its own speed
  int holdMs = 400;
}
ArrayList<Keyframe> sequence = new ArrayList<Keyframe>();
Slider seqMoveSlider;   // ms to move between keyframes
Slider seqHoldSlider;   // ms to hold at each keyframe
boolean previewing = false;
int previewIndex = 0;
int previewPhaseStart = 0;
boolean previewMoving = true;
int[] previewFrom = new int[NUM_SERVOS];

// ====================== LAYOUT ======================
int rowTop = 150;
int rowH   = 78;
int sliderX = 260;
int sliderW = 340;

// ====================== BUTTONS ======================
Button connectButton, refreshPortsButton;
Button saveSettingsButton, loadSettingsButton;
Button goToCenterButton;
Button pirPinUpButton, pirPinDownButton, pirEnableButton;
Button modeRandomButton, modeSequenceButton;
Button exportButton;
Button[] activeButton  = new Button[NUM_SERVOS];
Button[] blinkButton   = new Button[NUM_SERVOS];
Button[] minButton     = new Button[NUM_SERVOS];
Button[] maxButton     = new Button[NUM_SERVOS];
Button[] centerButton  = new Button[NUM_SERVOS];
Button[] resetButton   = new Button[NUM_SERVOS];
Button[] pinUpButton   = new Button[NUM_SERVOS];
Button[] pinDownButton = new Button[NUM_SERVOS];

Button recordFrameButton, clearSeqButton, playButton, seqBlinkButton;
boolean seqBlinkEnabled = false;
int seqBlinkState = 0; // reuses R_IDLE/R_CLOSING/R_HOLD/R_OPENING once declared below
int seqBlinkT0 = 0, seqNextBlinkAt = 0;

int draggingSlider = -1;

void setup() {
  size(900, 870);
  surface.setTitle("Animatronic Eyes Controller - build 2026-09-26-k");
  textFont(createFont("Arial", 14));

  for (int i = 0; i < NUM_SERVOS; i++) {
    servoMin[i] = 0;
    servoMax[i] = 180;
    servoCenter[i] = 90;
    servoActive[i] = false;
    servoBlink[i] = (servoPin[i] == 6 || servoPin[i] == 9);
    int y = rowTop + i * rowH;
    servoSlider[i] = new Slider(sliderX-70, y + 10, sliderW, 22, 0, 180); //Move servo Sliders
    servoSlider[i].value = 90;
    servoSlider[i].loLimit = 0;
    servoSlider[i].hiLimit = 180;

  activeButton[i]  = new Button(20, y - 5, 70, 26, "Active"); // Modified: y + 6 - 20 = y - 14
  blinkButton[i]   = new Button(20, y + 24, 70, 26, "Blink");  // Modified: y + 36 - 20 = y + 16
    pinDownButton[i] = new Button(140, y + 20, 22, 22, "-");
   pinUpButton[i]   = new Button(196, y + 20, 22, 22, "+");

    int bx = sliderX + sliderW + 15;
    minButton[i]    = new Button(bx-20, y, 62, 24, "Set Min");                 // move buttons
    maxButton[i]    = new Button(bx-20, y + 27, 62, 24, "Set Max");
    centerButton[i] = new Button(bx-20 + 68, y, 62, 24, "Center");
    resetButton[i]  = new Button(bx-20 + 68, y + 27, 62, 24, "Reset");
  }

  connectButton      = new Button(510, 15, 110, 30, "Connect");
  refreshPortsButton = new Button(630, 15, 110, 30, "Refresh Ports");
  saveSettingsButton = new Button(510, 55, 110, 28, "Save Settings");
  loadSettingsButton = new Button(630, 55, 110, 28, "Load Settings");
  goToCenterButton   = new Button(510, 95, 220, 28, "Go To Center (active)");

  pirPinDownButton = new Button(240, 80, 22, 22, "-");
  pirPinUpButton   = new Button(266, 80, 22, 22, "+");
  pirEnableButton  = new Button(296, 80, 100, 26, "Enable PIR");

  modeRandomButton   = new Button(20, 630, 160, 32, "Random Movement");   //Modify Random buttons
  modeSequenceButton = new Button(190, 630, 160, 32, "Record Sequence");
  exportButton       = new Button(360, 630, 160, 32, "Export Sketch");

  panSpeedSlider = new Slider(150, 720, 200, 20, 50, 500);               // Pan Slider
  panSpeedSlider.value = 150;
  panHoldSlider  = new Slider(400, 720, 200, 20, 200, 4000);
  panHoldSlider.value = 1200;
  blinkGapSlider = new Slider(650, 720, 200, 20, 1000, 8000);
  blinkGapSlider.value = 3500;
  squintGapSlider  = new Slider(150, 795, 200, 20, 3000, 60000);
  squintGapSlider.value = 20000;
  squintHoldSlider = new Slider(400, 795, 200, 20, 200, 4000);
  squintHoldSlider.value = 900;

  seqMoveSlider = new Slider(150, 720, 200, 20, 50, 3000);
  seqMoveSlider.value = 700;
  seqHoldSlider = new Slider(400, 720, 200, 20, 0, 3000);
  seqHoldSlider.value = 400;

  recordFrameButton = new Button(150, 785, 150, 30, "Record Frame");
  clearSeqButton    = new Button(310, 785, 90, 30, "Clear");
  seqBlinkButton    = new Button(410, 785, 180, 30, "Random Blink: Off");
  playButton        = new Button(530, 630, 110, 32, "Play");

  portList = Serial.list();
  loadSequenceFromFile();
  loadSettings();
}

void draw() {
  background(BG);
  drawHeader();
  for (int i = 0; i < NUM_SERVOS; i++) drawServoRow(i);
  drawModePanel();
  drawStatus();

  if (mode == MODE_SEQUENCE && previewing) updatePreview();
  if (mode == MODE_RANDOM && randomPreviewing) updateRandomPreview();
  if (mode == MODE_SEQUENCE && seqBlinkEnabled && !previewing) updateSeqBlink(millis());

  if (startupPending && millis() - connectedAt >= STARTUP_SEND_DELAY_MS) {
    startupPending = false;
    sendStartupCommands();
  }

  if (connected && !startupPending && millis() - lastHeartbeatAt >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatAt = millis();
    sendCmd("PING"); // any received line resets the Nano's idle clock, keeping it in live mode
  }

  drawPortListOverlay(); // always last, so nothing else can paint over it
}

// The servo rows' background panels are drawn after drawHeader() and would
// otherwise cover this dropdown (it used to render at y=130, right where
// the first servo row's panel starts at y=132) - so it's called separately,
// at the very end of draw(), to guarantee it's on top.
void drawPortListOverlay() {
  if (!showPortList) return;

  if (portList.length == 0) {
    fill(255);
    stroke(180, 60, 60);
    rect(510, 130, 300, 40);
    noStroke();
    fill(160, 30, 30);
    textSize(11);
    text("No serial ports found. Check the USB cable/drivers,", 516, 145);
    text("then click Connect again to re-check.", 516, 160);
    return;
  }

  for (int i = 0; i < portList.length; i++) {
    float y = 130 + i * 24;
    fill(255);
    stroke(150);
    rect(510, y, 230, 22);
    fill(TEXT_COL);
    noStroke();
    textSize(12);
    text(portList[i], 516, y + 15);
  }
}

// ---------------------------------------------------------------
void drawHeader() {
  fill(TEXT_COL);
  textSize(20);
  text("Animatronic Eyes with Eyelids Controller", 20, 30);
  textSize(12);
  fill(90);
  text("Upload servo_live_receiver.ino to the Nano first, then Connect.", 20, 48);

  connectButton.label = connected ? "Disconnect" : "Connect";
  connectButton.baseColor = connected ? BTN_OFF : BTN_PRIMARY;
  connectButton.display();
  refreshPortsButton.display();
  saveSettingsButton.display();
  loadSettingsButton.display();
  goToCenterButton.display();

  // ---- PIR controls ----
  fill(TEXT_COL);
  textSize(12);
  text("PIR pin", 240, 125);
  textAlign(CENTER, CENTER);
  text(pirPin, 290, 122);
  textAlign(LEFT, BASELINE);
  pirPinDownButton.display();
  pirPinUpButton.display();
  pirEnableButton.baseColor = pirEnabled ? BTN_ON : BTN_COL;
  pirEnableButton.display();

  if (pirEnabled) {
    boolean recentMotion = pirDetected && (millis() - pirDetectedTime < 1500);
    fill(recentMotion ? color(30, 140, 30) : color(120));
    textSize(14);
    text(recentMotion ? "Motion: YES" : "Motion: no", 320, 125);
  } else {
    fill(150);
    textSize(14);
    text("(live once connected)", 320, 125);
  }

  // Port list dropdown (if open) is drawn last, in drawPortListOverlay(),
  // so the servo-row panels below can't paint over it.
}

void drawServoRow(int i) {
  int y = rowTop + i * rowH;
  fill(PANEL);
  stroke(210);
  rect(10, y-18, WIN_W - 20, rowH +5, 10);
  noStroke();

  fill(TEXT_COL);
  textSize(15);
  text(servoLabel[i], 750, y + 40 < rowTop ? y + 46 : y + 22); //  text(servoLabel[i], 20, y +10 < rowTop ? y + 16 : y - 8); Move servo labels

  // pin number with +/- steppers
  fill(TEXT_COL);
  textSize(13);
  textAlign(CENTER, CENTER);
  text("pin " + servoPin[i], 120, y + 20);   //moved pin# text
  textAlign(LEFT, BASELINE);

  activeButton[i].baseColor = servoActive[i] ? BTN_ON : BTN_OFF;
  activeButton[i].display();
  blinkButton[i].baseColor = servoBlink[i] ? color(120, 170, 230) : BTN_COL;
  blinkButton[i].display();
//  pinDownButton[i].display();
//  pinUpButton[i].display();

  // slider + limit markers
  servoSlider[i].display(servoActive[i]);
  drawLimitMarker(servoSlider[i], servoMin[i], color(220, 90, 60));
  drawLimitMarker(servoSlider[i], servoMax[i], color(60, 130, 220));
  drawCenterMarker(servoSlider[i], servoCenter[i]);

  fill(80);
  textSize(11);
  text("min " + servoMin[i] + "  max " + servoMax[i] + "  ctr " + servoCenter[i],
       sliderX, servoSlider[i].y + servoSlider[i].h + 14);

  minButton[i].display();
  maxButton[i].display();
  centerButton[i].display();
  resetButton[i].display();
}

void drawLimitMarker(Slider s, int val, color c) {
  float x = map(val, s.min, s.max, s.x, s.x + s.w);
  stroke(c);
  strokeWeight(3);
  line(x, s.y - 4, x, s.y + s.h + 4);
  noStroke();
}

void drawCenterMarker(Slider s, int val) {
  float x = map(val, s.min, s.max, s.x, s.x + s.w);
  stroke(255, 200, 0);
  strokeWeight(2);
  line(x, s.y - 8, x, s.y - 2);
  noStroke();
}

void drawModePanel() {
  modeRandomButton.baseColor   = (mode == MODE_RANDOM)   ? BTN_ON : BTN_COL;
  modeSequenceButton.baseColor = (mode == MODE_SEQUENCE) ? BTN_ON : BTN_COL;
  modeRandomButton.display();
  modeSequenceButton.display();

 fill(PANEL);
 stroke(210);
  rect(10, 688, WIN_W - 20, 170, 6);              //Move bottom panel
  noStroke();

  if (mode == MODE_RANDOM) {
    fill(TEXT_COL);
    textSize(12);
    text("Pan speed (ms)", 150, 710);
    text("Pan hold (ms)", 400, 710);
    text("Blink gap (ms)", 650, 710);
    panSpeedSlider.display(true);
    panHoldSlider.display(true);
    blinkGapSlider.display(true);
    text("Squint gap (ms)", 150, 785);
    text("Squint hold (ms)", 400, 785);
    squintGapSlider.display(true);
    squintHoldSlider.display(true);
    fill(90);
    textSize(11);
    text("\"Blink\" servos above blink/squint together; others pan together using their Min/Max/Center.",
         20, 830);
    text(randomPreviewing ? "Playing lifelike random movement live... click Play to stop."
                           : "Click Play to preview lifelike random eye/eyelid movement live over serial.",
         20, 846);
  } else {
    fill(TEXT_COL);
    textSize(12);
    text("Move time (ms)", 150, 710);
    text("Hold time (ms)", 400, 710);
    seqMoveSlider.display(true);
    seqHoldSlider.display(true);

    recordFrameButton.display();
    clearSeqButton.display();
    seqBlinkButton.label = seqBlinkEnabled ? "Random Blink: On" : "Random Blink: Off";
    seqBlinkButton.baseColor = seqBlinkEnabled ? BTN_ON : BTN_COL;
    seqBlinkButton.display();

    fill(90);
    textSize(11);
    text("Pose the sliders, click Record Frame to save the pose. Each frame keeps the",
         20, 818);
    text("Move/Hold time shown above at the moment you recorded it. Saved frames: " + sequence.size(),
         20, 834);
    text(previewing ? "Playing saved frames... (loops)" : "Play button plays back the saved frames live over serial.",
         20, 850);
  }

  boolean playingNow = (mode == MODE_SEQUENCE) ? previewing : randomPreviewing;
  playButton.label = playingNow ? "Stop" : "Play";
  playButton.baseColor = playingNow ? BTN_OFF : BTN_PRIMARY;
  playButton.display();

  exportButton.display();
}

void drawStatus() {
  if (millis() - statusMsgTime < 4000) {
    fill(30, 120, 30);
    textSize(12);
    text(statusMsg, 20, 68);
  }
}

// ====================== INTERACTION ======================
void mousePressed() {
  // port list
  if (showPortList) {
    for (int i = 0; i < portList.length; i++) {
      float y = 130 + i * 24;
      if (mouseX > 510 && mouseX < 740 && mouseY > y && mouseY < y + 22) {
        openPort(portList[i]);
        showPortList = false;
        return;
      }
    }
  }

  if (connectButton.isOver()) {
    if (connected) {
      closePort();
    } else {
      portList = Serial.list(); // always fresh, in case the Nano was plugged in after launch
      showPortList = !showPortList;
    }
    return;
  }
  if (refreshPortsButton.isOver()) { portList = Serial.list(); return; }
  if (saveSettingsButton.isOver()) { saveSettings(); return; }
  if (loadSettingsButton.isOver()) { loadSettings(); return; }
  if (goToCenterButton.isOver())   { goToCenter(); return; }
  if (pirPinDownButton.isOver()) {
    pirPin = constrain(pirPin - 1, 2, 13);
    sendCmd("PIRPIN:" + pirPin);
    return;
  }
  if (pirPinUpButton.isOver()) {
    pirPin = constrain(pirPin + 1, 2, 13);
    sendCmd("PIRPIN:" + pirPin);
    return;
  }
  if (pirEnableButton.isOver()) {
    pirEnabled = !pirEnabled;
    sendCmd("PIRPIN:" + pirPin);
    sendCmd("PIRENABLE:" + (pirEnabled ? 1 : 0));
    return;
  }
  if (modeRandomButton.isOver())   { mode = MODE_RANDOM; previewing = false; return; }
  if (modeSequenceButton.isOver()) { mode = MODE_SEQUENCE; randomPreviewing = false; return; }
  if (exportButton.isOver())       { exportSketch(); return; }
  if (playButton.isOver())         { togglePlay(); return; }

  if (mode == MODE_SEQUENCE) {
    if (recordFrameButton.isOver()) { recordFrame(); return; }
    if (clearSeqButton.isOver())    { clearSequence(); return; }
    if (seqBlinkButton.isOver())    { toggleSeqBlink(); return; }
    if (seqMoveSlider.isOver()) { draggingSlider = -100; return; }
    if (seqHoldSlider.isOver()) { draggingSlider = -101; return; }
  } else {
    if (panSpeedSlider.isOver())  { draggingSlider = -200; return; }
    if (panHoldSlider.isOver())   { draggingSlider = -201; return; }
    if (blinkGapSlider.isOver())  { draggingSlider = -202; return; }
    if (squintGapSlider.isOver())  { draggingSlider = -203; return; }
    if (squintHoldSlider.isOver()) { draggingSlider = -204; return; }
  }

  for (int i = 0; i < NUM_SERVOS; i++) {
    if (activeButton[i].isOver()) {
      servoActive[i] = !servoActive[i];
      sendCmd("ACTIVE:" + i + ":" + (servoActive[i] ? 1 : 0));
      return;
    }
    if (blinkButton[i].isOver()) { servoBlink[i] = !servoBlink[i]; return; }
    if (pinUpButton[i].isOver())   { servoPin[i] = constrain(servoPin[i] + 1, 2, 13); return; }
    if (pinDownButton[i].isOver()) { servoPin[i] = constrain(servoPin[i] - 1, 2, 13); return; }
    if (minButton[i].isOver()) {
      servoMin[i] = round(servoSlider[i].value);
      if (servoMin[i] > servoMax[i]) servoMax[i] = servoMin[i];
      servoSlider[i].loLimit = servoMin[i];
      return;
    }
    if (maxButton[i].isOver()) {
      servoMax[i] = round(servoSlider[i].value);
      if (servoMax[i] < servoMin[i]) servoMin[i] = servoMax[i];
      servoSlider[i].hiLimit = servoMax[i];
      return;
    }
    if (centerButton[i].isOver()) {
      servoCenter[i] = round(servoSlider[i].value);
      return;
    }
    if (resetButton[i].isOver()) {
      servoMin[i] = 0; servoMax[i] = 180;
      servoSlider[i].loLimit = 0; servoSlider[i].hiLimit = 180;
      return;
    }
    if (servoSlider[i].isOver() && servoActive[i]) { draggingSlider = i; return; }
  }
}

void mouseDragged() {
  if (draggingSlider == -100) { seqMoveSlider.updatePosition(mouseX); return; }
  if (draggingSlider == -101) { seqHoldSlider.updatePosition(mouseX); return; }
  if (draggingSlider == -200) { panSpeedSlider.updatePosition(mouseX); return; }
  if (draggingSlider == -201) { panHoldSlider.updatePosition(mouseX); return; }
  if (draggingSlider == -202) { blinkGapSlider.updatePosition(mouseX); return; }
  if (draggingSlider == -203) { squintGapSlider.updatePosition(mouseX); return; }
  if (draggingSlider == -204) { squintHoldSlider.updatePosition(mouseX); return; }
  if (draggingSlider >= 0) {
    int i = draggingSlider;
    servoSlider[i].updatePosition(mouseX);
    sendCmd("S:" + i + ":" + round(servoSlider[i].value));
  }
}

void mouseReleased() { draggingSlider = -1; }

// ====================== SERIAL ======================
void openPort(String name) {
  try {
    arduinoPort = new Serial(this, name, 9600);
    connected = true;
    startupPending = true;
    connectedAt = millis();
    lastHeartbeatAt = millis();
    setStatus("Connecting to " + name + " ... (waiting for the Nano to finish booting)");
  } catch (Exception e) {
    setStatus("Could not open " + name + ": " + e.getMessage());
    connected = false;
  }
}

// Sent once, STARTUP_SEND_DELAY_MS after openPort() - non-blocking, so the
// GUI keeps redrawing and responding to input while it waits, instead of
// freezing on a delay() call.
void sendStartupCommands() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoSlider[i].value = 90;
    sendCmd("S:" + i + ":90"); // all servos to 90 at start, per spec
    sendCmd("ACTIVE:" + i + ":" + (servoActive[i] ? 1 : 0));
  }
  sendCmd("PIRPIN:" + pirPin);
  sendCmd("PIRENABLE:" + (pirEnabled ? 1 : 0));
  setStatus("Connected. All servos sent to 90.");
}

void closePort() {
  if (arduinoPort != null) arduinoPort.stop();
  connected = false;
  startupPending = false;
  setStatus("Disconnected.");
}

void sendCmd(String s) {
  if (connected && arduinoPort != null) arduinoPort.write(s + "\n");
}

void setStatus(String s) {
  statusMsg = s;
  statusMsgTime = millis();
}

void goToCenter() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!servoActive[i]) continue;
    servoSlider[i].value = servoCenter[i];
    sendCmd("S:" + i + ":" + servoCenter[i]);
  }
  setStatus("Active servos sent to their center position.");
}

// Reads PIR status lines ("PIR:0" / "PIR:1") sent back by servo_live_receiver.ino
void serialEvent(Serial p) {
  while (p.available() > 0) {
    char c = (char) p.read();
    if (c == '\n') {
      String line = serialBuffer.trim();
      serialBuffer = "";
      if (line.startsWith("PIR:")) {
        boolean on = line.substring(4).trim().equals("1");
        if (on) { pirDetected = true; pirDetectedTime = millis(); }
        else pirDetected = false;
      }
    } else if (c != '\r') {
      serialBuffer += c;
    }
  }
}

// ====================== SETTINGS FILE ======================
void saveSettings() {
  String[] lines = new String[NUM_SERVOS + 1];
  lines[0] = "label,pin,min,max,center,active,blink";
  for (int i = 0; i < NUM_SERVOS; i++) {
    lines[i + 1] = servoLabel[i] + "," + servoPin[i] + "," + servoMin[i] + "," +
                   servoMax[i] + "," + servoCenter[i] + "," + servoActive[i] + "," + servoBlink[i];
  }
  saveStrings(sketchPath("eye_servo_params.txt"), lines);
  setStatus("Settings saved to eye_servo_params.txt");
}

void loadSettings() {
  String path = sketchPath("eye_servo_params.txt");
  File f = new File(path);
  if (!f.exists()) { setStatus("No eye_servo_params.txt found next to this sketch."); return; }
  String[] lines = loadStrings(path);
  int row = 0;
  for (int li = 1; li < lines.length && row < NUM_SERVOS; li++) {
    String[] parts = split(lines[li], ',');
    if (parts.length < 7) continue;
    servoLabel[row]  = parts[0];
    servoPin[row]    = int(parts[1]);
    servoMin[row]    = int(parts[2]);
    servoMax[row]    = int(parts[3]);
    servoCenter[row] = int(parts[4]);
    servoActive[row] = boolean(parts[5]);
    servoBlink[row]  = boolean(parts[6]);
    servoSlider[row].loLimit = servoMin[row];
    servoSlider[row].hiLimit = servoMax[row];
    row++;
  }
  setStatus("Settings loaded from eye_servo_params.txt");
}

// ====================== SEQUENCE MODE ======================
// "Record Frame" saves the current pose of the active (selected) servos as
// a frame, and immediately persists it to eye_sequence.txt. The Play
// button (see togglePlay(), shared with Random mode) plays those saved
// frames back live over serial.
void recordFrame() {
  Keyframe k = new Keyframe();
  for (int i = 0; i < NUM_SERVOS; i++) {
    k.pos[i] = servoActive[i] ? round(servoSlider[i].value) : -1; // -1 = not selected, skip
  }
  k.moveMs = round(seqMoveSlider.value);
  k.holdMs = round(seqHoldSlider.value);
  sequence.add(k);
  saveSequenceToFile();
  setStatus("Frame " + sequence.size() + " recorded and saved.");
}

void clearSequence() {
  sequence.clear();
  previewing = false;
  saveSequenceToFile();
  setStatus("Saved frames cleared.");
}

// A simple, standalone blink cycle you can leave running in Sequence mode
// while posing frames, so the "Blink" servos keep blinking naturally in
// the background - independent of the Random-mode preview above (no
// squinting here, just blinking), and paused automatically whenever a
// saved-frame playback (Play button) is actually running, since that
// already drives every active servo itself.
void toggleSeqBlink() {
  seqBlinkEnabled = !seqBlinkEnabled;
  if (seqBlinkEnabled) {
    seqBlinkState = R_IDLE;
    seqNextBlinkAt = millis() + 500;
    setStatus("Random blinking on while posing frames.");
  } else {
    seqBlinkState = R_IDLE;
    writeLiveLids(0.0); // don't leave a blink servo stuck mid-blink
    setStatus("Random blinking off.");
  }
}

void updateSeqBlink(int now) {
  float t;
  switch (seqBlinkState) {
    case R_IDLE:
      if (now - seqNextBlinkAt >= 0) { seqBlinkState = R_CLOSING; seqBlinkT0 = now; }
      break;
    case R_CLOSING:
      t = (now - seqBlinkT0) / (float) R_BLINK_CLOSE_MS;
      if (t >= 1.0) { writeLiveLids(1.0); seqBlinkState = R_HOLD; seqBlinkT0 = now; }
      else writeLiveLids(easeInOutCubic(t));
      break;
    case R_HOLD:
      if (now - seqBlinkT0 >= R_BLINK_HOLD_MS) { seqBlinkState = R_OPENING; seqBlinkT0 = now; }
      break;
    case R_OPENING:
      t = (now - seqBlinkT0) / (float) R_BLINK_OPEN_MS;
      if (t >= 1.0) {
        writeLiveLids(0.0);
        seqBlinkState = R_IDLE;
        float gap = blinkGapSlider.value;
        seqNextBlinkAt = now + round(random(gap, gap * 2));
      } else {
        writeLiveLids(1.0 - easeInOutCubic(t));
      }
      break;
  }
}

// Shared Play/Stop button: plays saved frames in Sequence mode, or runs a
// live lifelike random-movement preview in Random mode.
void togglePlay() {
  if (mode == MODE_SEQUENCE) {
    if (sequence.size() == 0) { setStatus("No saved frames to play."); return; }
    previewing = !previewing;
    if (previewing) {
      previewIndex = 0;
      previewMoving = true;
      previewPhaseStart = millis();
      for (int i = 0; i < NUM_SERVOS; i++) previewFrom[i] = round(servoSlider[i].value);
      setStatus("Playing saved frames.");
    } else {
      setStatus("Playback stopped.");
    }
  } else {
    randomPreviewing = !randomPreviewing;
    if (randomPreviewing) {
      int now = millis();
      rGaze = 0.5; rGazeFrom = 0.5; rGazeTo = 0.5; rPanMoving = false;
      rNextPanAt = now + 300;
      rBlinkState = R_IDLE;
      rNextBlinkAt = now + 800;
      rNextSquintAt = now + round(random(squintGapSlider.value * 0.7, squintGapSlider.value * 1.3));
      setStatus("Playing lifelike random movement.");
    } else {
      setStatus("Random movement stopped.");
    }
  }
}

// Persist the recorded sequence to a text file next to the sketch, so
// frames survive between sessions ("saved frames"). Each row is the
// servo positions followed by that frame's own move/hold time in ms.
void saveSequenceToFile() {
  String path = sketchPath("eye_sequence.txt");
  if (sequence.size() == 0) {
    saveStrings(path, new String[]{ "servos=" + NUM_SERVOS });
    return;
  }
  String[] lines = new String[sequence.size() + 1];
  lines[0] = "servos=" + NUM_SERVOS;
  for (int k = 0; k < sequence.size(); k++) {
    Keyframe kf = sequence.get(k);
    String row = "";
    for (int i = 0; i < NUM_SERVOS; i++) row += kf.pos[i] + ",";
    row += kf.moveMs + "," + kf.holdMs;
    lines[k + 1] = row;
  }
  saveStrings(path, lines);
}

void loadSequenceFromFile() {
  String path = sketchPath("eye_sequence.txt");
  File f = new File(path);
  if (!f.exists()) return;
  String[] lines = loadStrings(path);
  sequence.clear();
  for (int li = 1; li < lines.length; li++) {
    if (lines[li] == null || lines[li].trim().length() == 0) continue;
    String[] parts = split(lines[li], ',');
    if (parts.length < NUM_SERVOS) continue;
    Keyframe k = new Keyframe();
    for (int i = 0; i < NUM_SERVOS; i++) k.pos[i] = int(parts[i]);
    if (parts.length >= NUM_SERVOS + 2) {
      // newer file format: per-frame move/hold time included
      k.moveMs = int(parts[NUM_SERVOS]);
      k.holdMs = int(parts[NUM_SERVOS + 1]);
    } else {
      // older file saved before per-frame timing existed - fall back to
      // whatever the sliders currently read, so it still plays reasonably
      k.moveMs = round(seqMoveSlider.value);
      k.holdMs = round(seqHoldSlider.value);
    }
    sequence.add(k);
  }
}

void updatePreview() {
  int now = millis();
  Keyframe target = sequence.get(previewIndex);
  int moveMs = target.moveMs;
  int holdMs = target.holdMs;

  if (previewMoving) {
    float t = constrain((now - previewPhaseStart) / (float) max(1, moveMs), 0, 1);
    float e = easeInOutCubic(t);
    for (int i = 0; i < NUM_SERVOS; i++) {
      if (target.pos[i] < 0) continue;
      int v = round(lerp(previewFrom[i], target.pos[i], e));
      servoSlider[i].value = v;
      sendCmd("S:" + i + ":" + v);
    }
    if (t >= 1.0) { previewMoving = false; previewPhaseStart = now; }
  } else {
    if (now - previewPhaseStart >= holdMs) {
      for (int i = 0; i < NUM_SERVOS; i++) previewFrom[i] = round(servoSlider[i].value);
      previewIndex = (previewIndex + 1) % sequence.size();
      previewMoving = true;
      previewPhaseStart = now;
    }
  }
}

float easeInOutCubic(float t) {
  return t < 0.5 ? 4 * t * t * t : 1 - pow(-2 * t + 2, 3) / 2;
}

// ====================== RANDOM MODE LIVE PREVIEW ======================
// Mirrors the pan/blink state machine baked into the exported Random
// sketch (see buildMergedSketch's RANDOM branch below), but drives the sliders and serial
// output live so Play works as a preview without exporting first.
void updateRandomPreview() {
  int now = millis();
  if (rIsSquint && rBlinkState == R_HOLD) updateSquintPan(now);
  else updateRandomPan(now);
  updateRandomBlink(now);
}

void startRandomPan(int now) {
  float target;
  if (random(100) < 30) {
    target = rGaze + random(-0.1, 0.1); // small nudge
  } else {
    target = (random(0, 1) + random(0, 1)) * 0.5; // biased to center
  }
  target = constrain(target, 0.0, 1.0);
  rGazeFrom = rGaze;
  rGazeTo = target;
  float amp = abs(rGazeTo - rGazeFrom);
  rPanDur = (int) (panSpeedSlider.value * (0.3 + 0.7 * amp));
  rPanStart = now;
  rPanMoving = true;
  if (amp > 0.5 && random(100) < 12) startRandomBlink(now);
}

void updateRandomPan(int now) {
  if (rPanMoving) {
    float t = (now - rPanStart) / (float) max(1, rPanDur);
    if (t >= 1.0) {
      t = 1.0;
      rPanMoving = false;
      rNextPanAt = now + round(random(panHoldSlider.value / 2, panHoldSlider.value * 2));
    }
    rGaze = rGazeFrom + (rGazeTo - rGazeFrom) * easeInOutCubic(t);
    writeLivePan(rGaze);
  } else if (now - rNextPanAt >= 0) {
    startRandomPan(now);
  }
}

// While the eyelids are held at squint depth, sweep the eyes slowly back
// and forth between the two limits around wherever they were looking when
// the squint began, pausing at each limit before reversing. Runs at its
// own fixed pace (R_SQUINT_PAN_LEG_MS/PAUSE_MS), independent of how long
// the squint hold itself lasts, and just gets cut off wherever it is when
// the eyelids reopen.
void updateSquintPan(int now) {
  int legMs = R_SQUINT_PAN_LEG_MS;
  int pauseMs = R_SQUINT_PAN_PAUSE_MS;
  int cycle = 2 * (legMs + pauseMs);
  int elapsed = (now - rSquintPanT0) % cycle;
  float lo = constrain(rSquintPanCenter - R_SQUINT_PAN_AMP, 0.0, 1.0);
  float hi = constrain(rSquintPanCenter + R_SQUINT_PAN_AMP, 0.0, 1.0);
  float g;
  if (elapsed < legMs) {
    g = lerp(lo, hi, easeInOutCubic(elapsed / (float) legMs));
  } else if (elapsed < legMs + pauseMs) {
    g = hi;
  } else if (elapsed < 2 * legMs + pauseMs) {
    g = lerp(hi, lo, easeInOutCubic((elapsed - legMs - pauseMs) / (float) legMs));
  } else {
    g = lo;
  }
  rGaze = g;
  writeLivePan(rGaze);
}

void writeLivePan(float g) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!servoActive[i] || servoBlink[i]) continue;
    int v = round(lerp(servoMin[i], servoMax[i], g));
    servoSlider[i].value = v;
    sendCmd("S:" + i + ":" + v);
  }
}

void startRandomBlink(int now) {
  if (rBlinkState == R_IDLE) {
    rBlinkState = R_CLOSING;
    rBlinkT0 = now;
    rIsSquint = false;
    rHoldMs = R_BLINK_HOLD_MS;
  }
}

void startRandomSquint(int now) {
  if (rBlinkState == R_IDLE) {
    rBlinkState = R_CLOSING;
    rBlinkT0 = now;
    rIsSquint = true;
    rHoldMs = round(random(squintHoldSlider.value * 0.7, squintHoldSlider.value * 1.3));
  }
}

// Single blinks only (no double-blink chance) - always waits the full gap.
void scheduleNextRandomBlink(int now) {
  float gap = blinkGapSlider.value;
  rNextBlinkAt = now + round(random(gap, gap * 2)); // was gap/2 floor - doubled
}

void scheduleNextRandomSquint(int now) {
  rNextSquintAt = now + round(random(squintGapSlider.value * 0.7, squintGapSlider.value * 1.3));
}

void updateRandomBlink(int now) {
  float t;
  float closeMs = rIsSquint ? R_SQUINT_CLOSE_MS : R_BLINK_CLOSE_MS;
  float openMs  = rIsSquint ? R_SQUINT_OPEN_MS  : R_BLINK_OPEN_MS;
  float target  = rIsSquint ? R_SQUINT_CLOSED_FRAC : 1.0;
  switch (rBlinkState) {
    case R_IDLE:
      if (now - rNextSquintAt >= 0) startRandomSquint(now);
      else if (now - rNextBlinkAt >= 0) startRandomBlink(now);
      break;
    case R_CLOSING:
      t = (now - rBlinkT0) / closeMs;
      if (t >= 1.0) {
        writeLiveLids(target);
        rBlinkState = R_HOLD;
        rBlinkT0 = now;
        if (rIsSquint) {
          rSquintPanCenter = constrain(rGaze, R_SQUINT_PAN_AMP, 1.0 - R_SQUINT_PAN_AMP);
          rSquintPanT0 = now;
          rPanMoving = false; // don't let a normal saccade fight the sweep
        }
      } else {
        writeLiveLids(target * easeInOutCubic(t));
      }
      break;
    case R_HOLD:
      if (now - rBlinkT0 >= rHoldMs) {
        rBlinkState = R_OPENING;
        rBlinkT0 = now;
        if (rIsSquint) rNextPanAt = now + round(random(200, 600)); // resume panning shortly
      }
      break;
    case R_OPENING:
      t = (now - rBlinkT0) / openMs;
      if (t >= 1.0) {
        writeLiveLids(0.0);
        rBlinkState = R_IDLE;
        if (rIsSquint) scheduleNextRandomSquint(now); else scheduleNextRandomBlink(now);
      } else {
        writeLiveLids(target * (1.0 - easeInOutCubic(t)));
      }
      break;
  }
}

void writeLiveLids(float closedFrac) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!servoActive[i] || !servoBlink[i]) continue;
    int v = round(lerp(servoMin[i], servoMax[i], closedFrac));
    servoSlider[i].value = v;
    sendCmd("S:" + i + ":" + v);
  }
}

// ====================== EXPORT ======================
void exportSketch() {
  ArrayList<String> L = new ArrayList<String>();
  if (mode == MODE_SEQUENCE) {
    if (sequence.size() == 0) { setStatus("Record at least one frame before exporting."); return; }
    buildMergedSketch(L, true);
  } else {
    buildMergedSketch(L, false);
  }
  deleteExportFolder();
  writeExport(L, "export_live_receiver.ino");
}

// Wipes the whole export/ folder (if it exists) before writing a fresh
// export_live_receiver.ino, so there's never a stale file - or a stale
// folder - left over from a previous export to cause confusion.
void deleteExportFolder() {
  File dir = new File(sketchPath("export"));
  if (dir.exists()) deleteRecursive(dir);
}

void deleteRecursive(File f) {
  if (f.isDirectory()) {
    File[] kids = f.listFiles();
    if (kids != null) for (File k : kids) deleteRecursive(k);
  }
  f.delete();
}

void writeExport(ArrayList<String> L, String filename) {
  String path = sketchPath("export/" + filename);
  saveStrings(path, L.toArray(new String[0]));
  setStatus("Exported export/" + filename + " - upload it to the Nano to replace what's currently running.");
}

// Produces one merged live-receiver + autonomous-fallback sketch: it keeps
// working as a live receiver for the GUI, and after IDLE_TIMEOUT_MS with no
// GUI command it falls back to running on its own with the settings baked
// in here (a saved-frame sequence, or lifelike random movement), including
// each servo's min/max travel and the current PIR settings.
void buildMergedSketch(ArrayList<String> L, boolean sequenceMode) {
  L.add("// Auto-generated by eyeball_controller.pde");
  L.add("// Baked-in AUTO_MODE: " + (sequenceMode ? "SEQUENCE (plays back saved frames)" : "RANDOM (lifelike eye/eyelid movement)"));
  L.add("// Also still a live receiver: eyeball_controller.pde can reconnect and");
  L.add("// drive it via serial at any time; after going idle it resumes running");
  L.add("// on its own using the settings below. Re-export after any change.");
  L.add("#include <Servo.h>");
  L.add("");
  L.add("// ====================== BAKED-IN SETTINGS (filled in by Export Sketch) ======================");
  L.add("#define MODE_NONE     0");
  L.add("#define MODE_SEQUENCE 1");
  L.add("#define MODE_RANDOM   2");
  L.add("const uint8_t AUTO_MODE = " + (sequenceMode ? "MODE_SEQUENCE" : "MODE_RANDOM") + ";");
  L.add("");
  L.add("const uint8_t NUM_SERVOS = " + NUM_SERVOS + ";");
  L.add("const unsigned long IDLE_TIMEOUT_MS = 3000; // ms with no GUI command before auto playback resumes");
  L.add("const unsigned int STARTUP_HOLD_MS = 2000;");
  L.add("");
  appendPirConsts(L);
  L.add("");
  appendPerServoArrays(L);
  L.add("");
  L.add("// ---- SEQUENCE mode data (only used when AUTO_MODE == MODE_SEQUENCE) ----");
  L.add("const unsigned int MOVE_MS = " + round(seqMoveSlider.value) + "; // used only for the return-to-center move after a full loop");
  if (sequenceMode) {
    L.add("const uint8_t NUM_KEYFRAMES = " + sequence.size() + ";");
    L.add("const int KEYFRAMES[NUM_KEYFRAMES][NUM_SERVOS] = {");
    for (int k = 0; k < sequence.size(); k++) {
      Keyframe kf = sequence.get(k);
      String row = "  {";
      for (int i = 0; i < NUM_SERVOS; i++) row += kf.pos[i] + (i < NUM_SERVOS - 1 ? ", " : "");
      row += "}" + (k < sequence.size() - 1 ? "," : "");
      L.add(row);
    }
    L.add("};");
    L.add("// Each frame's own move/hold time, exactly as it was when recorded.");
    L.add("const unsigned int KEYFRAME_MOVE_MS[NUM_KEYFRAMES] = {");
    String moveRow = "  ";
    for (int k = 0; k < sequence.size(); k++) moveRow += sequence.get(k).moveMs + (k < sequence.size() - 1 ? ", " : "");
    L.add(moveRow);
    L.add("};");
    L.add("const unsigned int KEYFRAME_HOLD_MS[NUM_KEYFRAMES] = {");
    String holdRow = "  ";
    for (int k = 0; k < sequence.size(); k++) holdRow += sequence.get(k).holdMs + (k < sequence.size() - 1 ? ", " : "");
    L.add(holdRow);
    L.add("};");
  } else {
    L.add("const uint8_t NUM_KEYFRAMES = 0;");
    L.add("const int KEYFRAMES[1][NUM_SERVOS] = { {0, 0, 0, 0, 0, 0} }; // placeholder, unused");
    L.add("const unsigned int KEYFRAME_MOVE_MS[1] = { 700 }; // placeholder, unused");
    L.add("const unsigned int KEYFRAME_HOLD_MS[1] = { 400 }; // placeholder, unused");
  }
  L.add("");
  L.add("// ---- RANDOM mode tuning (only used when AUTO_MODE == MODE_RANDOM) ----");
  L.add("const unsigned int PAN_SPEED_MS   = " + round(panSpeedSlider.value) + ";");
  L.add("const unsigned int PAN_HOLD_MS    = " + round(panHoldSlider.value)  + ";");
  L.add("const unsigned int BLINK_GAP_MS   = " + round(blinkGapSlider.value) + ";");
  L.add("const unsigned int BLINK_CLOSE_MS = 70;");
  L.add("const unsigned int BLINK_HOLD_MS  = 30;");
  L.add("const unsigned int BLINK_OPEN_MS  = 120;");
  L.add("const unsigned int SQUINT_CLOSE_MS = 280;");   // slower than a blink
  L.add("const unsigned int SQUINT_OPEN_MS  = 280;");
  L.add("const unsigned int SQUINT_HOLD_MS = " + round(squintHoldSlider.value) + ";"); // jittered +/-30% at runtime
  L.add("const unsigned long SQUINT_GAP_MS = " + round(squintGapSlider.value) + ";");  // jittered +/-30% at runtime
  L.add("const float SQUINT_CLOSED_FRAC = 0.5;");          // halfway between each servo's MIN_V and MAX_V
  L.add("const float SQUINT_PAN_AMP = 0.45;");              // ~90% full swing (0.45 amplitude each side of center)
  L.add("const unsigned int SQUINT_PAN_LEG_MS = 1400;");     // time to sweep from one limit to the other
  L.add("const unsigned int SQUINT_PAN_PAUSE_MS = 450;");    // pause held at each limit before reversing
  L.add("");
  appendMergedFirmwareBody(L);
}

// The live-receiver + autonomous-playback engine itself. This is identical
// on every export (and matches the servo_live_receiver.ino template) - only
// the constants block above changes between exports.
void appendMergedFirmwareBody(ArrayList<String> L) {
  String[] body = {
"// ====================== LIVE RECEIVER STATE ======================",
"Servo servos[NUM_SERVOS];",
"bool active[NUM_SERVOS];",
"String line = \"\";",
"",
"unsigned long lastCommandAt = 0;",
"bool autoRunning = false;",
"",
"uint8_t pirPin = PIR_PIN_DEFAULT;",
"bool pirEnabled = PIR_ENABLED;",
"bool pirPinReady = false;",
"int lastPirState = -1;",
"",
"int cur[NUM_SERVOS]; // current commanded position (used by live S: and by SEQUENCE moves)",
"",
"bool startupDone = false;",
"unsigned long startupT0 = 0;",
"",
"// ---- SEQUENCE FSM ----",
"enum SeqState { SEQ_WAIT_MOTION, SEQ_MOVING, SEQ_HOLDING, SEQ_RETURN };",
"SeqState seqState = SEQ_WAIT_MOTION;",
"uint8_t seqIndex = 0;",
"int seqFrom[NUM_SERVOS];",
"unsigned long seqT0 = 0;",
"",
"// ---- RANDOM FSM ----",
"float gaze = 0.5, gazeFrom = 0.5, gazeTo = 0.5;",
"bool panMoving = false;",
"unsigned long panStart = 0, nextPanAt = 0;",
"unsigned int panDur = 100;",
"enum BlinkState { B_IDLE, B_CLOSING, B_HOLD, B_OPENING };",
"BlinkState blinkState = B_IDLE;",
"unsigned long blinkT0 = 0, nextBlinkAt = 0, nextSquintAt = 0;",
"bool isSquint = false;",
"unsigned int curHoldMs = BLINK_HOLD_MS; // hold duration in progress (blink or squint)",
"float squintPanCenter = 0.5;",
"unsigned long squintPanT0 = 0;",
"bool randAnimating = true;",
"unsigned long lastMotionAt = 0;",
"",
"float ease1(float t) { return t < 0.5 ? 4 * t * t * t : 1 - pow(-2 * t + 2, 3) / 2; }",
"float ease2(float t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }",
"",
"void setup() {",
"  Serial.begin(9600);",
"  randomSeed(analogRead(A0));",
"  for (uint8_t i = 0; i < NUM_SERVOS; i++) {",
"    active[i] = ACTIVE0[i];",
"    servos[i].write(90);          // pre-set so it snaps to 90 the instant it's attached",
"    servos[i].attach(PIN[i]);",
"    cur[i] = 90;",
"  }",
"  startupT0 = millis();",
"  lastCommandAt = millis();",
"}",
"",
"void loop() {",
"  while (Serial.available()) {",
"    char c = Serial.read();",
"    if (c == '\\n') {",
"      handleLine(line);",
"      line = \"\";",
"      lastCommandAt = millis();",
"    } else if (c != '\\r') {",
"      line += c;",
"    }",
"  }",
"  updatePir();",
"",
"  // Non-blocking startup hold: loop() (and the Serial reads above) runs",
"  // from the very first iteration, so a GUI connecting right at boot",
"  // can't have its initial commands lost while stuck in delay().",
"  if (!startupDone) {",
"    if (millis() - startupT0 >= STARTUP_HOLD_MS) {",
"      startupDone = true;",
"      for (uint8_t i = 0; i < NUM_SERVOS; i++) {",
"        if (active[i]) { servos[i].write(CENTER[i]); cur[i] = CENTER[i]; }",
"      }",
"      lastCommandAt = millis(); // don't let the startup hold count toward the idle timeout",
"    }",
"    return;",
"  }",
"",
"  bool shouldAuto = (AUTO_MODE != MODE_NONE) && ((millis() - lastCommandAt) >= IDLE_TIMEOUT_MS);",
"  if (shouldAuto) {",
"    if (!autoRunning) { autoRunning = true; autoResume(); }",
"    autoUpdate();",
"  } else {",
"    autoRunning = false;",
"  }",
"}",
"",
"void handleLine(String s) {",
"  s.trim();",
"  if (s.length() == 0) return;",
"",
"  int firstColon = s.indexOf(':');",
"  if (firstColon < 0) return;",
"  String cmd = s.substring(0, firstColon);",
"  String rest = s.substring(firstColon + 1);",
"",
"  if (cmd == \"PIRPIN\") {",
"    int pin = rest.toInt();",
"    if (pin != pirPin) {",
"      pirPin = (uint8_t) pin;",
"      pirPinReady = false; // re-arm pinMode() for the new pin",
"      lastPirState = -1;   // force a fresh report on the new pin",
"    }",
"    return;",
"  } else if (cmd == \"PIRENABLE\") {",
"    pirEnabled = (rest.toInt() != 0);",
"    if (!pirEnabled) lastPirState = -1; // re-report fresh next time it's enabled",
"    return;",
"  }",
"",
"  int secondColon = rest.indexOf(':');",
"  if (secondColon < 0) return;",
"  int idx = rest.substring(0, secondColon).toInt();",
"  int val = rest.substring(secondColon + 1).toInt();",
"  if (idx < 0 || idx >= NUM_SERVOS) return;",
"",
"  if (cmd == \"S\") {",
"    if (active[idx]) {",
"      val = constrain(val, 0, 180);",
"      servos[idx].write(val);",
"      cur[idx] = val;",
"    }",
"  } else if (cmd == \"ACTIVE\") {",
"    active[idx] = (val != 0);",
"  }",
"}",
"",
"void updatePir() {",
"  if (!pirEnabled) return;",
"  if (!pirPinReady) {",
"    pinMode(pirPin, INPUT);",
"    pirPinReady = true;",
"  }",
"  int state = digitalRead(pirPin);",
"  if (state != lastPirState) {",
"    lastPirState = state;",
"    Serial.print(\"PIR:\");",
"    Serial.println(state == HIGH ? 1 : 0);",
"  }",
"}",
"",
"// ====================== AUTONOMOUS PLAYBACK ======================",
"void autoResume() {",
"  unsigned long now = millis();",
"  if (AUTO_MODE == MODE_SEQUENCE) {",
"    seqIndex = 0;",
"    for (int i = 0; i < NUM_SERVOS; i++) seqFrom[i] = cur[i];",
"    seqState = pirEnabled ? SEQ_WAIT_MOTION : SEQ_MOVING;",
"    seqT0 = now;",
"  } else if (AUTO_MODE == MODE_RANDOM) {",
"    gazeFrom = gaze; gazeTo = gaze; panMoving = false;",
"    nextPanAt = now + 300;",
"    blinkState = B_IDLE;",
"    nextBlinkAt = now + 800;",
"    nextSquintAt = now + (unsigned long)random((long)(SQUINT_GAP_MS * 0.7), (long)(SQUINT_GAP_MS * 1.3) + 1);",
"    lastMotionAt = now;",
"    randAnimating = !pirEnabled;",
"  }",
"}",
"",
"void autoUpdate() {",
"  unsigned long now = millis();",
"  if (AUTO_MODE == MODE_SEQUENCE) {",
"    updateSequence(now);",
"  } else if (AUTO_MODE == MODE_RANDOM) {",
"    if (pirEnabled) {",
"      if (digitalRead(pirPin) == HIGH) lastMotionAt = now;",
"      randAnimating = (now - lastMotionAt) < PIR_IDLE_MS;",
"    }",
"    if (isSquint && blinkState == B_HOLD) updateSquintPan(now);",
"    else updateRandomPan(now);",
"    updateRandomBlink(now);",
"  }",
"}",
"",
"void seqMoveStep(unsigned long now, const int target[NUM_SERVOS], unsigned int durMs) {",
"  float t = durMs == 0 ? 1.0 : (float)(now - seqT0) / (float)durMs;",
"  if (t >= 1.0) t = 1.0;",
"  float e = ease1(t);",
"  for (int i = 0; i < NUM_SERVOS; i++) {",
"    if (!active[i] || target[i] < 0) continue;",
"    int v = seqFrom[i] + (int)round((target[i] - seqFrom[i]) * e);",
"    if (v != cur[i]) { servos[i].write(v); cur[i] = v; }",
"  }",
"}",
"",
"void updateSequence(unsigned long now) {",
"  switch (seqState) {",
"    case SEQ_WAIT_MOTION:",
"      if (!pirEnabled || digitalRead(pirPin) == HIGH) {",
"        for (int i = 0; i < NUM_SERVOS; i++) seqFrom[i] = cur[i];",
"        seqT0 = now;",
"        seqState = SEQ_MOVING;",
"      }",
"      break;",
"    case SEQ_MOVING:",
"      if (NUM_KEYFRAMES == 0) { seqState = SEQ_RETURN; break; }",
"      seqMoveStep(now, KEYFRAMES[seqIndex], KEYFRAME_MOVE_MS[seqIndex]);",
"      if (now - seqT0 >= KEYFRAME_MOVE_MS[seqIndex]) {",
"        seqT0 = now;",
"        seqState = SEQ_HOLDING;",
"      }",
"      break;",
"    case SEQ_HOLDING:",
"      if (now - seqT0 >= KEYFRAME_HOLD_MS[seqIndex]) {",
"        seqIndex++;",
"        for (int i = 0; i < NUM_SERVOS; i++) seqFrom[i] = cur[i];",
"        seqT0 = now;",
"        if (seqIndex >= NUM_KEYFRAMES) {",
"          seqIndex = 0;",
"          seqState = pirEnabled ? SEQ_RETURN : SEQ_MOVING;",
"        } else {",
"          seqState = SEQ_MOVING;",
"        }",
"      }",
"      break;",
"    case SEQ_RETURN: {",
"      int centerPose[NUM_SERVOS];",
"      for (int i = 0; i < NUM_SERVOS; i++) centerPose[i] = active[i] ? CENTER[i] : -1;",
"      seqMoveStep(now, centerPose, MOVE_MS);",
"      if (now - seqT0 >= MOVE_MS) {",
"        seqState = SEQ_WAIT_MOTION;",
"      }",
"      break;",
"    }",
"    default: break;",
"  }",
"}",
"",
"// While the lids are held at squint depth, sweep the eyes slowly back and forth",
"// between the two limits, pausing at each limit before reversing. Runs at its own",
"// fixed pace (SQUINT_PAN_LEG_MS/PAUSE_MS), independent of the squint hold length -",
"// it just gets cut off wherever it is when the eyelids reopen.",
"void updateSquintPan(unsigned long now) {",
"  unsigned long legMs = SQUINT_PAN_LEG_MS;",
"  unsigned long pauseMs = SQUINT_PAN_PAUSE_MS;",
"  unsigned long cycle = 2UL * (legMs + pauseMs);",
"  unsigned long elapsed = (now - squintPanT0) % cycle;",
"  float lo = constrain(squintPanCenter - SQUINT_PAN_AMP, 0.0, 1.0);",
"  float hi = constrain(squintPanCenter + SQUINT_PAN_AMP, 0.0, 1.0);",
"  float g;",
"  if (elapsed < legMs) {",
"    g = lo + (hi - lo) * ease2((float)elapsed / legMs);",
"  } else if (elapsed < legMs + pauseMs) {",
"    g = hi;",
"  } else if (elapsed < 2UL * legMs + pauseMs) {",
"    g = hi - (hi - lo) * ease2((float)(elapsed - legMs - pauseMs) / legMs);",
"  } else {",
"    g = lo;",
"  }",
"  gaze = g;",
"  writeRandomPan(gaze);",
"}",
"",
"void writeRandomPan(float g) {",
"  for (int i = 0; i < NUM_SERVOS; i++) {",
"    if (!active[i] || IS_BLINK[i]) continue;",
"    int v = MIN_V[i] + (int)round((MAX_V[i] - MIN_V[i]) * g);",
"    servos[i].write(v);",
"    cur[i] = v;",
"  }",
"}",
"",
"void writeRandomLids(float closedFrac) {",
"  for (int i = 0; i < NUM_SERVOS; i++) {",
"    if (!active[i] || !IS_BLINK[i]) continue;",
"    int v = MIN_V[i] + (int)round((MAX_V[i] - MIN_V[i]) * closedFrac);",
"    servos[i].write(v);",
"    cur[i] = v;",
"  }",
"}",
"",
"void startRandomPan(unsigned long now) {",
"  float target;",
"  if (random(100) < 30) {",
"    target = gaze + (random(-100, 101) / 1000.0);",
"  } else {",
"    target = (random(0, 1001) / 1000.0 + random(0, 1001) / 1000.0) * 0.5;",
"  }",
"  target = constrain(target, 0.0, 1.0);",
"  gazeFrom = gaze;",
"  gazeTo = target;",
"  float amp = fabs(gazeTo - gazeFrom);",
"  panDur = (unsigned int)(PAN_SPEED_MS * (0.3 + 0.7 * amp));",
"  panStart = now;",
"  panMoving = true;",
"  if (amp > 0.5 && random(100) < 12) startRandomBlink(now);",
"}",
"",
"void updateRandomPan(unsigned long now) {",
"  if (panMoving) {",
"    float t = (float)(now - panStart) / (float)panDur;",
"    if (t >= 1.0) {",
"      t = 1.0;",
"      panMoving = false;",
"      nextPanAt = now + random(PAN_HOLD_MS / 2, PAN_HOLD_MS * 2 + 1);",
"    }",
"    gaze = gazeFrom + (gazeTo - gazeFrom) * ease2(t);",
"    writeRandomPan(gaze);",
"  } else if (randAnimating && (long)(now - nextPanAt) >= 0) {",
"    startRandomPan(now);",
"  }",
"}",
"",
"void startRandomBlink(unsigned long now) {",
"  if (blinkState == B_IDLE) {",
"    blinkState = B_CLOSING;",
"    blinkT0 = now;",
"    isSquint = false;",
"    curHoldMs = BLINK_HOLD_MS;",
"  }",
"}",
"",
"void startRandomSquint(unsigned long now) {",
"  if (blinkState == B_IDLE) {",
"    blinkState = B_CLOSING;",
"    blinkT0 = now;",
"    isSquint = true;",
"    curHoldMs = random((long)(SQUINT_HOLD_MS * 0.7), (long)(SQUINT_HOLD_MS * 1.3) + 1);",
"  }",
"}",
"",
"// Single blinks only (no double-blink chance) - always waits the full gap.",
"void scheduleNextRandomBlink(unsigned long now) {",
"  nextBlinkAt = now + random(BLINK_GAP_MS, BLINK_GAP_MS * 2 + 1);",
"}",
"",
"void scheduleNextRandomSquint(unsigned long now) {",
"  nextSquintAt = now + (unsigned long)random((long)(SQUINT_GAP_MS * 0.7), (long)(SQUINT_GAP_MS * 1.3) + 1);",
"}",
"",
"void updateRandomBlink(unsigned long now) {",
"  float t;",
"  unsigned int closeMs = isSquint ? SQUINT_CLOSE_MS : BLINK_CLOSE_MS;",
"  unsigned int openMs  = isSquint ? SQUINT_OPEN_MS  : BLINK_OPEN_MS;",
"  float target = isSquint ? SQUINT_CLOSED_FRAC : 1.0;",
"  switch (blinkState) {",
"    case B_IDLE:",
"      if (randAnimating && (long)(now - nextSquintAt) >= 0) startRandomSquint(now);",
"      else if (randAnimating && (long)(now - nextBlinkAt) >= 0) startRandomBlink(now);",
"      break;",
"    case B_CLOSING:",
"      t = (float)(now - blinkT0) / (float)closeMs;",
"      if (t >= 1.0) {",
"        writeRandomLids(target); blinkState = B_HOLD; blinkT0 = now;",
"        if (isSquint) {",
"          squintPanCenter = constrain(gaze, SQUINT_PAN_AMP, 1.0 - SQUINT_PAN_AMP);",
"          squintPanT0 = now;",
"          panMoving = false; // don't let a normal saccade fight the sweep",
"        }",
"      }",
"      else writeRandomLids(target * ease2(t));",
"      break;",
"    case B_HOLD:",
"      if (now - blinkT0 >= curHoldMs) {",
"        blinkState = B_OPENING; blinkT0 = now;",
"        if (isSquint) nextPanAt = now + random(200, 600); // resume panning shortly",
"      }",
"      break;",
"    case B_OPENING:",
"      t = (float)(now - blinkT0) / (float)openMs;",
"      if (t >= 1.0) {",
"        writeRandomLids(0.0);",
"        blinkState = B_IDLE;",
"        if (isSquint) scheduleNextRandomSquint(now); else scheduleNextRandomBlink(now);",
"      } else {",
"        writeRandomLids(target * (1.0 - ease2(t)));",
"      }",
"      break;",
"  }",
"}"
  };
  for (String ln : body) L.add(ln);
}

void appendPirConsts(ArrayList<String> L) {
  L.add("const bool    PIR_ENABLED     = " + (pirEnabled ? "true" : "false") + ";");
  L.add("const uint8_t PIR_PIN_DEFAULT = " + pirPin + ";");
  L.add("const unsigned long PIR_IDLE_MS = 5000; // RANDOM mode: ms to keep animating after last motion");
}

void appendPerServoArrays(ArrayList<String> L) {
  L.add(arrLine("const uint8_t PIN[NUM_SERVOS]", intArr(servoPin)));
  L.add(arrLine("const int MIN_V[NUM_SERVOS]", intArr(servoMin)));
  L.add(arrLine("const int MAX_V[NUM_SERVOS]", intArr(servoMax)));
  L.add(arrLine("const int CENTER[NUM_SERVOS]", intArr(servoCenter)));
  L.add(arrLine("const bool ACTIVE0[NUM_SERVOS]", boolArr(servoActive)));
  L.add(arrLine("const bool IS_BLINK[NUM_SERVOS]", boolArr(servoBlink)));
}

String arrLine(String decl, String vals) { return decl + " = {" + vals + "};"; }

String intArr(int[] a) {
  String s = "";
  for (int i = 0; i < a.length; i++) s += a[i] + (i < a.length - 1 ? ", " : "");
  return s;
}

String boolArr(boolean[] a) {
  String s = "";
  for (int i = 0; i < a.length; i++) s += (a[i] ? "true" : "false") + (i < a.length - 1 ? ", " : "");
  return s;
}

// ====================== WIDGETS ======================
class Slider {
  float x, y, w, h, min, max, value, loLimit, hiLimit;
  Slider(float x, float y, float w, float h, float min, float max) {
    this.x = x; this.y = y; this.w = w; this.h = h;
    this.min = min; this.max = max;
    this.value = (min + max) / 2;
    this.loLimit = min; this.hiLimit = max;
  }
  void display(boolean active) {
    noStroke();
    fill(SLIDER_TRACK);
    rect(x, y, w, h, h / 2);
    float hx = map(value, min, max, x, x + w);
    fill(active ? HANDLE_COL : HANDLE_INACT);
    ellipse(hx, y + h / 2, h * 1.2, h * 1.2);
    fill(30);
    textAlign(CENTER, CENTER);
    textSize(12);
    text(round(value), hx, y + h / 2);
    textAlign(LEFT, BASELINE);
  }
  boolean isOver() {
    return mouseX >= x - 10 && mouseX <= x + w + 10 && mouseY >= y - 10 && mouseY <= y + h + 10;
  }
  void updatePosition(float mx) {
    float v = map(constrain(mx, x, x + w), x, x + w, min, max);
    value = constrain(v, loLimit, hiLimit);
  }
}

class Button {
  float x, y, w, h;
  String label;
  color baseColor = BTN_COL;
  Button(float x, float y, float w, float h, String label) {
    this.x = x; this.y = y; this.w = w; this.h = h; this.label = label;
  }
  void display() {
    stroke(120);
    strokeWeight(1);
    fill(isOver() ? lerpColor(baseColor, color(255), 0.25) : baseColor);
    rect(x, y, w, h, 5);
    noStroke();
    fill(20);
    textAlign(CENTER, CENTER);
    textSize(12);
    text(label, x + w / 2, y + h / 2);
    textAlign(LEFT, BASELINE);
  }
  boolean isOver() {
    return mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h;
  }
}
