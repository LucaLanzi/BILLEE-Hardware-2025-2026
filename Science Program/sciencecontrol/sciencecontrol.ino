#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Basicmicro.h>

/*
  IMPORTANT ARDUINO QUIRK:
  The Arduino IDE auto-generates function prototypes near the top of the file.
  If any function signature uses a struct/enum type that is defined later, compilation fails.
  So: ALL structs/enums used in function signatures MUST be defined right after #includes.

  ALSO IMPORTANT:
  Your RoboClaw example uses Serial1 and Serial2. That will NOT compile on an Arduino Uno R3
  (it only has Serial). This Drill submenu therefore requires a board with Serial1/Serial2
  (e.g., Mega 2560, Due, etc.) OR you must rewrite it to use SoftwareSerial.
*/

// ========================= TYPES (PUT THESE FIRST) =========================
struct ServoState {
  uint8_t channel;
  float currentDeg;
  float targetDeg;
  float speedDegPerSec;
  bool moving;

  uint16_t minUs;
  uint16_t maxUs;
  uint16_t centerUs;

  unsigned long lastUpdateMs;
};

struct Relay {
  const char* label;
  uint8_t pin;
  bool state;
};

enum class MenuState {
  MAIN,
  SERVO_MENU,
  SERVO_1,
  SERVO_2,
  SERVO_3,
  PUMPS_UV_MENU,
  PUMPS_UV_CONTROL,
  ACTUATOR_MENU,
  DRILL_MENU
};

enum class ActDir { STOP, FWD, REV };

struct ActuatorState {
  ActDir dir;
  uint8_t speed; // 0..255
};

// ---- Drill / RoboClaw sequence state machine types ----
enum DrillSeqState {
  DRILL_STEP1,
  DRILL_STEP1_STOP,
  DRILL_STEP2,
  DRILL_STEP2_STOP,
  DRILL_STEP3,
  DRILL_STEP3_STOP,
  DRILL_STEP4,
  DRILL_STEP4_STOP,
  DRILL_STEP5,
  DRILL_STEP5_STOP,
  DRILL_DONE,
  DRILL_ABORTED
};

// ========================= GLOBALS / CONFIG =========================

// ---- PCA9685 ----
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
static const uint16_t SERVO_FREQ = 50;
ServoState servos[3];

// ---- Relays (Pumps and UV) ----
// NOTE: You requested stirrer on pin 0. On Uno, D0 is Serial RX and WILL conflict with Serial.
Relay relays[] = {
  {"pump1",   13, false},
  {"pump2",   12, false},
  {"pump3",   11, false},
  {"stirrer",  0, false},  // WARNING: conflicts with Serial RX on Uno
  {"uv_led",   9, false},
};
static const uint8_t RELAY_COUNT = sizeof(relays) / sizeof(relays[0]);

// ---- Linear Actuator (H-Bridge IN1..IN4) ----
#define ACT_IN1 4
#define ACT_IN2 5
#define ACT_IN3 6   // PWM on Uno
#define ACT_IN4 7
ActuatorState actuator = { ActDir::STOP, 150 };

// ---- Menu state ----
MenuState menuState = MenuState::MAIN;
int selectedRelay = -1;

// ========================= DRILL / ROBOCLAW CONFIG =========================
// NOTE: You provided both addresses as 0x81. Typically two RoboClaws should have different addresses.
// Kept exactly as provided.
const uint8_t  ROBOCLAW1_ADDR = 0x81;
const uint8_t  ROBOCLAW2_ADDR = 0x82;
const uint32_t ROBOCLAW_BAUD  = 9600;

Basicmicro roboclaw1(&Serial1, 10000);
Basicmicro roboclaw2(&Serial2, 10000);

volatile bool drillStopRequested = false;
volatile bool drillControlMode   = false;
bool drillAutoEnabled            = true;   // auto sequence runs when true
DrillSeqState drillSeqState      = DRILL_STEP1;
unsigned long drillStateStartMs  = 0;

// ========================= GENERIC HELPERS =========================
static uint16_t usToTicks(uint16_t us, uint16_t freq) {
  uint32_t ticks = (uint32_t)us * (uint32_t)freq * 4096UL;
  ticks = (ticks + 500000UL) / 1000000UL;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

static uint16_t degToUs(const ServoState &s, float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  float t = deg / 180.0f;
  return (uint16_t)(s.minUs + t * (float)(s.maxUs - s.minUs));
}

static void writeServoDeg(uint8_t idx, float deg) {
  ServoState &s = servos[idx];
  s.currentDeg = deg;
  uint16_t us = degToUs(s, deg);
  uint16_t ticks = usToTicks(us, SERVO_FREQ);
  pca.setPWM(s.channel, 0, ticks);
}

static void applyRelay(uint8_t i) {
  digitalWrite(relays[i].pin, relays[i].state ? HIGH : LOW);
}

static void actuatorApply() {
  switch (actuator.dir) {
    case ActDir::STOP:
      digitalWrite(ACT_IN1, LOW);
      digitalWrite(ACT_IN2, LOW);
      break;
    case ActDir::FWD:
      digitalWrite(ACT_IN1, HIGH);
      digitalWrite(ACT_IN2, LOW);
      break;
    case ActDir::REV:
      digitalWrite(ACT_IN1, LOW);
      digitalWrite(ACT_IN2, HIGH);
      break;
  }
  analogWrite(ACT_IN3, actuator.speed);
  digitalWrite(ACT_IN4, (actuator.dir == ActDir::STOP) ? LOW : HIGH);
}

static const __FlashStringHelper* actDirToStr(ActDir d) {
  switch (d) {
    case ActDir::STOP: return F("STOP");
    case ActDir::FWD:  return F("FWD");
    case ActDir::REV:  return F("REV");
  }
  return F("?");
}

// Non-blocking line reader from Serial (main UI)
static String readLineNonBlocking() {
  static String buf;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = buf;
      buf = "";
      out.trim();
      return out;
    } else {
      buf += c;
      if (buf.length() > 200) {
        buf = "";
        Serial.println(F("ERR: line too long"));
        return "";
      }
    }
  }
  return "";
}

static bool isInteger(const String& s) {
  if (s.length() == 0) return false;
  int i = 0;
  if (s[0] == '-' || s[0] == '+') i = 1;
  for (; i < (int)s.length(); i++) if (!isDigit(s[i])) return false;
  return true;
}

// ========================= DRILL HELPERS (ported from your working sketch) =========================
static int16_t drillClampDuty(long v) {
  if (v < -32768) return -32768;
  if (v >  32767) return  32767;
  return (int16_t)v;
}

static void drillStopAll() {
  roboclaw1.DutyM1M2(ROBOCLAW1_ADDR, 0, 0);
  roboclaw2.DutyM1(ROBOCLAW2_ADDR, 0);
}

static void drillSetM1(int16_t d) { roboclaw1.DutyM1(ROBOCLAW1_ADDR, d); }
static void drillSetM2(int16_t d) { roboclaw1.DutyM2(ROBOCLAW1_ADDR, d); }
static void drillSetM3(int16_t d) { roboclaw2.DutyM1(ROBOCLAW2_ADDR, d); } // third motor on roboclaw2 M1

static void drillEnterState(DrillSeqState s) {
  drillSeqState = s;
  drillStateStartMs = millis();

  switch (s) {
    case DRILL_STEP1:      Serial.println(F("DRILL SEQ: STEP1 m1,m2=-13000 for 42.2s")); break;
    case DRILL_STEP1_STOP: Serial.println(F("DRILL SEQ: STEP1 stop")); break;
    case DRILL_STEP2:      Serial.println(F("DRILL SEQ: STEP2 m1,m3=-13000 for 21s")); break;
    case DRILL_STEP2_STOP: Serial.println(F("DRILL SEQ: STEP2 stop")); break;
    case DRILL_STEP3:      Serial.println(F("DRILL SEQ: STEP3 m1=-13000 m3=-6500 for 39s")); break;
    case DRILL_STEP3_STOP: Serial.println(F("DRILL SEQ: STEP3 stop")); break;
    case DRILL_STEP4:      Serial.println(F("DRILL SEQ: STEP4 m2=13000 for 17.5s")); break;
    case DRILL_STEP4_STOP: Serial.println(F("DRILL SEQ: STEP4 stop")); break;
    case DRILL_STEP5:      Serial.println(F("DRILL SEQ: STEP5 m1,m2=13000 for 26.7s")); break;
    case DRILL_STEP5_STOP: Serial.println(F("DRILL SEQ: STEP5 stop")); break;
    case DRILL_DONE:       Serial.println(F("DRILL SEQ: DONE")); break;
    case DRILL_ABORTED:    Serial.println(F("DRILL SEQ: ABORTED")); break;
  }
}

static void drillPrintHelp() {
  Serial.println();
  Serial.println(F("=== DRILL COMMANDS ==="));
  Serial.println(F("Mode control:"));
  Serial.println(F("  start                  - start/restart auto sequence (enables auto)"));
  Serial.println(F("  pause                  - pause auto sequence (does NOT stop motors)"));
  Serial.println(F("  resume                 - resume auto sequence"));
  Serial.println(F("  stop                   - IMMEDIATELY stop ALL drill motors + abort sequence"));
  Serial.println(F("  control                - enter manual motor control mode"));
  Serial.println(F("  exit                   - leave manual control mode (auto can run if enabled)"));
  Serial.println(F("  status                 - show drill state"));
  Serial.println(F("  help                   - show this list"));
  Serial.println();
  Serial.println(F("Manual motor commands (ONLY when in control mode):"));
  Serial.println(F("  m1 <duty>               - set M1 on RoboClaw1"));
  Serial.println(F("  m2 <duty>               - set M2 on RoboClaw1"));
  Serial.println(F("  m3 <duty>               - set M1 on RoboClaw2 (3rd motor)"));
  Serial.println(F("  both <d1> <d2>          - set M1 and M2 on RoboClaw1"));
  Serial.println(F("  all <d1> <d2> <d3>      - set M1,M2 on RC1 and M1 on RC2"));
  Serial.println(F("Duty range: -32768..32767 (negative = reverse)"));
  Serial.println(F("Back: type 'b' to return to main menu."));
  Serial.println();
}

static void drillPrintStatus() {
  Serial.print(F("DRILL: auto="));
  Serial.print(drillAutoEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" controlMode="));
  Serial.print(drillControlMode ? F("YES") : F("NO"));
  Serial.print(F(" stopRequested="));
  Serial.println(drillStopRequested ? F("YES") : F("NO"));

  Serial.print(F("DRILL: seqState="));
  Serial.println((int)drillSeqState);
}

static void drillHandleCommandLine(const String &rawLine) {
  String line = rawLine;
  line.trim();
  if (line.length() == 0) return;

  String lower = line;
  lower.toLowerCase();

  if (lower == "help" || lower == "?") { drillPrintHelp(); return; }

  if (lower == "stop") {
    drillStopRequested = true;
    drillAutoEnabled = false;
    drillStopAll();
    drillEnterState(DRILL_ABORTED);
    Serial.println(F("OK: ALL drill motors stopped. Sequence aborted."));
    return;
  }

  if (lower == "start") {
    drillStopRequested = false;
    drillAutoEnabled = true;
    drillControlMode = false;
    drillStopAll();
    drillEnterState(DRILL_STEP1);
    Serial.println(F("OK: Drill auto sequence started."));
    return;
  }

  if (lower == "pause") {
    drillAutoEnabled = false;
    Serial.println(F("OK: Drill auto sequence paused."));
    return;
  }

  if (lower == "resume") {
    if (!drillStopRequested && drillSeqState != DRILL_ABORTED && drillSeqState != DRILL_DONE) {
      drillAutoEnabled = true;
      Serial.println(F("OK: Drill auto sequence resumed."));
    } else {
      Serial.println(F("NOTE: sequence is done/aborted; use 'start' to restart."));
    }
    return;
  }

  if (lower == "control") {
    drillControlMode = true;
    drillAutoEnabled = false; // pause auto while manually driving
    Serial.println(F("Entered DRILL CONTROL mode. Type motor commands, or 'exit' to leave."));
    return;
  }

  if (lower == "exit") {
    if (drillControlMode) {
      drillControlMode = false;
      Serial.println(F("Exited DRILL CONTROL mode. Use 'resume' to continue auto, or 'start' to restart."));
    } else {
      Serial.println(F("Not in control mode."));
    }
    return;
  }

  if (lower == "status") { drillPrintStatus(); return; }

  // If not in control mode, ignore motor commands (but still allow stop/start/control/help/status/etc.)
  if (!drillControlMode) {
    Serial.println(F("NOTE: type 'control' to manually drive drill motors (or 'start' for auto)."));
    return;
  }

  // Tokenize: cmd and args
  int sp = line.indexOf(' ');
  if (sp < 0) { Serial.println(F("ERR: missing args. Type 'help'.")); return; }
  String cmd = line.substring(0, sp);
  String args = line.substring(sp + 1);
  cmd.toLowerCase();
  args.trim();

  if (cmd == "m1") {
    int16_t d = drillClampDuty(args.toInt());
    drillSetM1(d);
    Serial.print(F("OK: M1=")); Serial.println(d);
    return;
  }
  if (cmd == "m2") {
    int16_t d = drillClampDuty(args.toInt());
    drillSetM2(d);
    Serial.print(F("OK: M2=")); Serial.println(d);
    return;
  }
  if (cmd == "m3") {
    int16_t d = drillClampDuty(args.toInt());
    drillSetM3(d);
    Serial.print(F("OK: M3=")); Serial.println(d);
    return;
  }
  if (cmd == "both") {
    int sp2 = args.indexOf(' ');
    if (sp2 < 0) { Serial.println(F("ERR: both needs 2 numbers. Example: both 8000 8000")); return; }
    int16_t d1 = drillClampDuty(args.substring(0, sp2).toInt());
    int16_t d2 = drillClampDuty(args.substring(sp2 + 1).toInt());
    roboclaw1.DutyM1M2(ROBOCLAW1_ADDR, d1, d2);
    Serial.print(F("OK: M1=")); Serial.print(d1); Serial.print(F(" M2=")); Serial.println(d2);
    return;
  }
  if (cmd == "all") {
    int sp1 = args.indexOf(' ');
    if (sp1 < 0) { Serial.println(F("ERR: all needs 3 numbers. Example: all 8000 8000 8000")); return; }
    int sp2 = args.indexOf(' ', sp1 + 1);
    if (sp2 < 0) { Serial.println(F("ERR: all needs 3 numbers. Example: all 8000 8000 8000")); return; }

    int16_t d1 = drillClampDuty(args.substring(0, sp1).toInt());
    int16_t d2 = drillClampDuty(args.substring(sp1 + 1, sp2).toInt());
    int16_t d3 = drillClampDuty(args.substring(sp2 + 1).toInt());

    roboclaw1.DutyM1M2(ROBOCLAW1_ADDR, d1, d2);
    drillSetM3(d3);

    Serial.print(F("OK: M1=")); Serial.print(d1);
    Serial.print(F(" M2=")); Serial.print(d2);
    Serial.print(F(" M3=")); Serial.println(d3);
    return;
  }

  Serial.println(F("ERR: unknown command. Type 'help'."));
}

static void drillUpdateSequence() {
  if (!drillAutoEnabled) return;
  if (drillControlMode)  return;

  if (drillStopRequested) {
    if (drillSeqState != DRILL_ABORTED) {
      drillEnterState(DRILL_ABORTED);
      drillStopAll();
    }
    return;
  }

  unsigned long now = millis();

  switch (drillSeqState) {
    case DRILL_STEP1:
      // NOTE: kept the *exact timing + commands* from your code (even if comments differ)
      drillSetM1(13000); drillSetM2(13000);
      if (now - drillStateStartMs >= 42200UL) { drillSetM1(0); drillSetM2(0); drillEnterState(DRILL_STEP1_STOP); }
      break;

    case DRILL_STEP1_STOP:
      drillEnterState(DRILL_STEP2);
      break;

    case DRILL_STEP2:
      drillSetM1(13000); drillSetM3(-13000);
      if (now - drillStateStartMs >= 21000UL) { drillSetM1(0); drillSetM3(0); drillEnterState(DRILL_STEP2_STOP); }
      break;

    case DRILL_STEP2_STOP:
      drillEnterState(DRILL_STEP3);
      break;

    case DRILL_STEP3:
      drillSetM1(-13000); drillSetM3(-6500);
      if (now - drillStateStartMs >= 39000UL) { drillSetM1(0); drillSetM3(0); drillEnterState(DRILL_STEP3_STOP); }
      break;

    case DRILL_STEP3_STOP:
      drillEnterState(DRILL_STEP4);
      break;

    case DRILL_STEP4:
      drillSetM2(-13000);
      if (now - drillStateStartMs >= 17500UL) { drillSetM2(0); drillEnterState(DRILL_STEP4_STOP); }
      break;

    case DRILL_STEP4_STOP:
      drillEnterState(DRILL_STEP5);
      break;

    case DRILL_STEP5:
      drillSetM1(-13000); drillSetM2(-13000);
      if (now - drillStateStartMs >= 26700UL) { drillSetM1(0); drillSetM2(0); drillEnterState(DRILL_STEP5_STOP); }
      break;

    case DRILL_STEP5_STOP:
      drillEnterState(DRILL_DONE);
      break;

    case DRILL_DONE:
    case DRILL_ABORTED:
      // nothing
      break;
  }
}

// ========================= MENU PRINTING =========================
static void printMainMenu() {
  Serial.println();
  Serial.println(F("=== MAIN MENU ==="));
  Serial.println(F("1) Servos"));
  Serial.println(F("2) Pumps and UV"));
  Serial.println(F("3) Linear Actuator"));
  Serial.println(F("4) Drill"));
  Serial.println(F("h) Help (show this menu)"));
  Serial.println(F("Type option and press Enter:"));
}

static void printServoSelectMenu() {
  Serial.println();
  Serial.println(F("=== SERVO MENU ==="));
  Serial.println(F("1) Servo 1"));
  Serial.println(F("2) Servo 2"));
  Serial.println(F("3) Servo 3"));
  Serial.println(F("b) Back to Main"));
  Serial.println(F("Select servo:"));
}

static void printServoControlMenu(uint8_t idx) {
  Serial.println();
  Serial.print(F("=== SERVO "));
  Serial.print(idx + 1);
  Serial.println(F(" CONTROL ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  f              -> move forward (toward 180 deg)"));
  Serial.println(F("  r              -> move reverse/backward (toward 0 deg)"));
  Serial.println(F("  stop           -> stop movement"));
  Serial.println(F("  goto <deg>     -> go to specific angle (0-180)"));
  Serial.println(F("  speed <deg/s>  -> set speed (e.g., speed 45)"));
  Serial.println(F("  center         -> send center pulse (centerUs)"));
  Serial.println(F("  calib min <us> -> set min pulse width (e.g., 500)"));
  Serial.println(F("  calib max <us> -> set max pulse width (e.g., 2500)"));
  Serial.println(F("  calib center <us> -> set center pulse (e.g., 1500)"));
  Serial.println(F("  status         -> print current settings"));
  Serial.println(F("  b              -> back"));
  Serial.println(F("Enter command:"));
}

static void printPumpsUvMenu() {
  Serial.println();
  Serial.println(F("=== PUMPS AND UV ==="));
  Serial.println(F("Select item by number or name:"));
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    Serial.print(i + 1);
    Serial.print(F(") "));
    Serial.print(relays[i].label);
    Serial.print(F("  [pin D"));
    Serial.print(relays[i].pin);
    Serial.print(F("] state="));
    Serial.println(relays[i].state ? F("ON") : F("OFF"));
  }
  Serial.println(F("b) Back to Main"));
  Serial.println(F("Type name (e.g., pump1) or number (e.g., 1):"));
}

static void printPumpsUvControlMenu() {
  Serial.println();
  Serial.print(F("=== CONTROL: "));
  Serial.print(relays[selectedRelay].label);
  Serial.println(F(" ==="));
  Serial.println(F("Type: on | off | toggle | status | b"));
}

static void printActuatorMenu() {
  Serial.println();
  Serial.println(F("=== LINEAR ACTUATOR ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  f            -> extend (forward)"));
  Serial.println(F("  r            -> retract (reverse)"));
  Serial.println(F("  stop         -> stop"));
  Serial.println(F("  speed <0-255>-> set PWM speed"));
  Serial.println(F("  status       -> show state"));
  Serial.println(F("  pins         -> show pin mapping"));
  Serial.println(F("  b            -> back"));
  Serial.println(F("Enter command:"));
}

static void printActuatorStatus() {
  Serial.print(F("Actuator dir="));
  Serial.print(actDirToStr(actuator.dir));
  Serial.print(F(" speed="));
  Serial.println(actuator.speed);
}

static void printDrillMenu() {
  Serial.println();
  Serial.println(F("=== DRILL ==="));
  Serial.println(F("Type 'help' to see commands."));
  Serial.println(F("Common: start | pause | resume | stop | control | exit | status | b"));
  Serial.println(F("Enter command:"));
}

// ========================= SERVO UPDATE =========================
static void updateServos() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < 3; i++) {
    ServoState &s = servos[i];
    if (!s.moving) continue;

    unsigned long dtMs = now - s.lastUpdateMs;
    if (dtMs < 20) continue;
    s.lastUpdateMs = now;

    float dt = dtMs / 1000.0f;
    float step = s.speedDegPerSec * dt;
    if (step <= 0) step = 0.1f;

    float diff = s.targetDeg - s.currentDeg;
    if (abs(diff) <= step) {
      s.currentDeg = s.targetDeg;
      s.moving = false;
      writeServoDeg(i, s.currentDeg);
    } else {
      s.currentDeg += (diff > 0 ? step : -step);
      writeServoDeg(i, s.currentDeg);
    }
  }
}

// ========================= COMMAND HANDLERS =========================
static void handleMain(const String& cmd) {
  if (cmd == "1" || cmd.equalsIgnoreCase("servos")) {
    menuState = MenuState::SERVO_MENU;
    printServoSelectMenu();
  } else if (cmd == "2" || cmd.equalsIgnoreCase("pumps") || cmd.equalsIgnoreCase("uv") || cmd.equalsIgnoreCase("pumps and uv")) {
    menuState = MenuState::PUMPS_UV_MENU;
    printPumpsUvMenu();
  } else if (cmd == "3" || cmd.equalsIgnoreCase("actuator") || cmd.equalsIgnoreCase("linear")) {
    menuState = MenuState::ACTUATOR_MENU;
    printActuatorMenu();
    printActuatorStatus();
  } else if (cmd == "4" || cmd.equalsIgnoreCase("drill")) {
    menuState = MenuState::DRILL_MENU;
    printDrillMenu();
    drillPrintStatus();
  } else if (cmd == "h" || cmd.equalsIgnoreCase("help")) {
    printMainMenu();
  } else if (cmd.length() > 0) {
    Serial.println(F("Unknown option. Type 'h' for help."));
  }
}

static void handleServoSelect(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (cmd == "1") { menuState = MenuState::SERVO_1; printServoControlMenu(0); return; }
  if (cmd == "2") { menuState = MenuState::SERVO_2; printServoControlMenu(1); return; }
  if (cmd == "3") { menuState = MenuState::SERVO_3; printServoControlMenu(2); return; }
  if (cmd.length() > 0) Serial.println(F("Pick 1, 2, 3 or b."));
}

static void printServoStatus(uint8_t idx) {
  ServoState &s = servos[idx];
  Serial.print(F("Servo ")); Serial.print(idx + 1);
  Serial.print(F(" ch=")); Serial.print(s.channel);
  Serial.print(F(" current=")); Serial.print(s.currentDeg, 1);
  Serial.print(F(" target=")); Serial.print(s.targetDeg, 1);
  Serial.print(F(" moving=")); Serial.print(s.moving ? F("yes") : F("no"));
  Serial.print(F(" speed=")); Serial.print(s.speedDegPerSec, 1); Serial.println(F(" deg/s"));
  Serial.print(F(" calib minUs=")); Serial.print(s.minUs);
  Serial.print(F(" maxUs=")); Serial.print(s.maxUs);
  Serial.print(F(" centerUs=")); Serial.println(s.centerUs);
}

static void servoGoTo(uint8_t idx, float deg) {
  ServoState &s = servos[idx];
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  s.targetDeg = deg;
  s.moving = true;
  s.lastUpdateMs = millis();
  Serial.print(F("Moving servo ")); Serial.print(idx + 1);
  Serial.print(F(" to ")); Serial.print(deg, 1); Serial.println(F(" deg"));
}

static void handleServoControl(uint8_t idx, const String& cmd) {
  if (cmd == "b") { menuState = MenuState::SERVO_MENU; printServoSelectMenu(); return; }
  if (cmd.equalsIgnoreCase("f")) { servoGoTo(idx, 180.0f); return; }
  if (cmd.equalsIgnoreCase("r")) { servoGoTo(idx, 0.0f); return; }

  if (cmd.equalsIgnoreCase("stop")) { servos[idx].moving = false; Serial.println(F("Stopped.")); return; }
  if (cmd.equalsIgnoreCase("status")) { printServoStatus(idx); return; }

  if (cmd.equalsIgnoreCase("center")) {
    servos[idx].moving = false;
    servos[idx].currentDeg = 90.0f;
    servos[idx].targetDeg = 90.0f;
    uint16_t ticks = usToTicks(servos[idx].centerUs, SERVO_FREQ);
    pca.setPWM(servos[idx].channel, 0, ticks);
    Serial.println(F("Sent center pulse."));
    return;
  }

  if (cmd.startsWith("goto ")) {
    String arg = cmd.substring(5); arg.trim();
    servoGoTo(idx, arg.toFloat());
    return;
  }

  if (cmd.startsWith("speed ")) {
    String arg = cmd.substring(6); arg.trim();
    float spd = arg.toFloat();
    if (spd <= 0) spd = 1;
    servos[idx].speedDegPerSec = spd;
    Serial.print(F("Speed set to ")); Serial.print(spd, 1); Serial.println(F(" deg/s"));
    return;
  }

  if (cmd.startsWith("calib ")) {
    String rest = cmd.substring(6); rest.trim();
    int sp = rest.indexOf(' ');
    if (sp < 0) { Serial.println(F("Usage: calib min <us> | calib max <us> | calib center <us>")); return; }

    String which = rest.substring(0, sp);
    String valS  = rest.substring(sp + 1);
    which.trim(); valS.trim();

    int us = valS.toInt();
    if (us < 100) us = 100;
    if (us > 3000) us = 3000;

    if (which.equalsIgnoreCase("min")) { servos[idx].minUs = (uint16_t)us; Serial.println(F("Updated minUs.")); }
    else if (which.equalsIgnoreCase("max")) { servos[idx].maxUs = (uint16_t)us; Serial.println(F("Updated maxUs.")); }
    else if (which.equalsIgnoreCase("center")) { servos[idx].centerUs = (uint16_t)us; Serial.println(F("Updated centerUs.")); }
    else { Serial.println(F("Unknown calib field. Use min/max/center.")); }
    return;
  }

  if (cmd.length() > 0) Serial.println(F("Unknown servo command. Type 'status' or 'b'."));
}

static int findRelayByName(const String& name) {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (name.equalsIgnoreCase(relays[i].label)) return i;
  }
  return -1;
}

static void handlePumpsUvMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }

  int idx = -1;
  if (isInteger(cmd)) {
    int n = cmd.toInt();
    if (n >= 1 && n <= (int)RELAY_COUNT) idx = n - 1;
  } else {
    idx = findRelayByName(cmd);
  }

  if (idx >= 0) {
    selectedRelay = idx;
    menuState = MenuState::PUMPS_UV_CONTROL;
    printPumpsUvControlMenu();
  } else if (cmd.length() > 0) {
    Serial.println(F("Unknown selection. Type a name or number."));
  }
}

static void handlePumpsUvControl(const String& cmd) {
  if (selectedRelay < 0 || selectedRelay >= (int)RELAY_COUNT) {
    menuState = MenuState::PUMPS_UV_MENU;
    printPumpsUvMenu();
    return;
  }

  if (cmd == "b") { menuState = MenuState::PUMPS_UV_MENU; printPumpsUvMenu(); return; }

  if (cmd.equalsIgnoreCase("on"))      { relays[selectedRelay].state = true;  applyRelay(selectedRelay); Serial.println(F("ON.")); return; }
  if (cmd.equalsIgnoreCase("off"))     { relays[selectedRelay].state = false; applyRelay(selectedRelay); Serial.println(F("OFF.")); return; }
  if (cmd.equalsIgnoreCase("toggle"))  { relays[selectedRelay].state = !relays[selectedRelay].state; applyRelay(selectedRelay); Serial.println(F("Toggled.")); return; }

  if (cmd.equalsIgnoreCase("status")) {
    Serial.print(F("State: "));
    Serial.println(relays[selectedRelay].state ? F("ON") : F("OFF"));
    return;
  }

  if (cmd.length() > 0) Serial.println(F("Type: on | off | toggle | status | b"));
}

static void handleActuatorMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }

  if (cmd.equalsIgnoreCase("f")) {
    actuator.dir = ActDir::FWD;
    actuatorApply();
    Serial.println(F("Actuator forward (extend)."));
    printActuatorStatus();
    return;
  }

  if (cmd.equalsIgnoreCase("r")) {
    actuator.dir = ActDir::REV;
    actuatorApply();
    Serial.println(F("Actuator reverse (retract)."));
    printActuatorStatus();
    return;
  }

  if (cmd.equalsIgnoreCase("stop")) {
    actuator.dir = ActDir::STOP;
    actuatorApply();
    Serial.println(F("Actuator stopped."));
    printActuatorStatus();
    return;
  }

  if (cmd.equalsIgnoreCase("status")) { printActuatorStatus(); return; }

  if (cmd.equalsIgnoreCase("pins")) {
    Serial.print(F("IN1=D")); Serial.println(ACT_IN1);
    Serial.print(F("IN2=D")); Serial.println(ACT_IN2);
    Serial.print(F("IN3(PWM)=D")); Serial.println(ACT_IN3);
    Serial.print(F("IN4=D")); Serial.println(ACT_IN4);
    return;
  }

  if (cmd.startsWith("speed ")) {
    String arg = cmd.substring(6); arg.trim();
    int sp = arg.toInt();
    if (sp < 0) sp = 0;
    if (sp > 255) sp = 255;
    actuator.speed = (uint8_t)sp;
    actuatorApply();
    Serial.print(F("Actuator speed set to ")); Serial.println(actuator.speed);
    return;
  }

  if (cmd.length() > 0) {
    Serial.println(F("Unknown command. Use: f | r | stop | speed <0-255> | status | pins | b"));
  }
}

static void handleDrillMenu(const String& cmd) {
  if (cmd == "b") {
    menuState = MenuState::MAIN;
    printMainMenu();
    return;
  }
  // Reuse your proven command set inside this submenu
  drillHandleCommandLine(cmd);
}

// ========================= SETUP / LOOP =========================
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* ok on boards with native USB */ }

#if !defined(HAVE_HWSERIAL1)
  // Some cores don't define HAVE_HWSERIAL1; the real compile failure would be "Serial1 not declared".
  // This message is here to make the requirement obvious in Serial Monitor if you happen to run anyway.
  Serial.println(F("NOTE: Drill submenu requires Serial1/Serial2 (e.g., Mega/Due). Uno R3 will not compile with Drill enabled."));
#endif

  Wire.begin();

  // PCA9685
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);

  // Servos default
  for (uint8_t i = 0; i < 3; i++) {
    servos[i].channel = i;
    servos[i].minUs = 500;
    servos[i].maxUs = 2500;
    servos[i].centerUs = 1500;
    servos[i].speedDegPerSec = 60.0f;
    servos[i].currentDeg = 90.0f;
    servos[i].targetDeg = 90.0f;
    servos[i].moving = false;
    servos[i].lastUpdateMs = millis();
    writeServoDeg(i, 90.0f);
  }

  // Relays
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(relays[i].pin, OUTPUT);
    relays[i].state = false;
    applyRelay(i);
  }

  // Actuator pins
  pinMode(ACT_IN1, OUTPUT);
  pinMode(ACT_IN2, OUTPUT);
  pinMode(ACT_IN3, OUTPUT);
  pinMode(ACT_IN4, OUTPUT);
  actuatorApply();

  // RoboClaw setup (from your working sketch)
  Serial1.begin(ROBOCLAW_BAUD);
  roboclaw1.begin(ROBOCLAW_BAUD);

  Serial2.begin(ROBOCLAW_BAUD);
  roboclaw2.begin(ROBOCLAW_BAUD);

  delay(200);
  drillStopAll();
  drillEnterState(DRILL_STEP1);

  Serial.println(F("\nSystem ready."));
  printMainMenu();
}

void loop() {
  // Background updates
  updateServos();
  drillUpdateSequence();

  // Menu input handling
  String cmd = readLineNonBlocking();
  if (cmd.length() == 0) return;

  // Global shortcut
  if (cmd.equalsIgnoreCase("main")) {
    menuState = MenuState::MAIN;
    printMainMenu();
    return;
  }

  switch (menuState) {
    case MenuState::MAIN:              handleMain(cmd); break;
    case MenuState::SERVO_MENU:        handleServoSelect(cmd); break;
    case MenuState::SERVO_1:           handleServoControl(0, cmd); break;
    case MenuState::SERVO_2:           handleServoControl(1, cmd); break;
    case MenuState::SERVO_3:           handleServoControl(2, cmd); break;
    case MenuState::PUMPS_UV_MENU:     handlePumpsUvMenu(cmd); break;
    case MenuState::PUMPS_UV_CONTROL:  handlePumpsUvControl(cmd); break;
    case MenuState::ACTUATOR_MENU:     handleActuatorMenu(cmd); break;
    case MenuState::DRILL_MENU:        handleDrillMenu(cmd); break;
  }
}