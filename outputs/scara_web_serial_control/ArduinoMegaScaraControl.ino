#include <AccelStepper.h>

// Install the AccelStepper library from Arduino IDE Library Manager.
// Use external STEP/DIR stepper drivers. Do not drive motors directly from Arduino pins.
constexpr uint32_t BAUD_RATE = 115200;

constexpr uint8_t J1_STEP_PIN = 22;
constexpr uint8_t J1_DIR_PIN = 23;
constexpr uint8_t J2_STEP_PIN = 24;
constexpr uint8_t J2_DIR_PIN = 25;
constexpr uint8_t Z_STEP_PIN = 26;
constexpr uint8_t Z_DIR_PIN = 27;

constexpr uint8_t ENABLE_PIN = 30; // LOW enables many common drivers.
constexpr uint8_t MAGNET_PIN = 31; // Drive a MOSFET or relay module, not the coil directly.

constexpr float MAX_SPEED = 900.0;
constexpr float ACCELERATION = 600.0;

constexpr long HOME_J1 = 0;
constexpr long HOME_J2 = 0;
constexpr long SAFE_Z = 0;
constexpr long DROP_Z = -450;
constexpr long PICK_J1 = -1200;
constexpr long PICK_J2 = 800;
constexpr long PICK_Z = -450;

// Tune these nine positions after measuring the real 3x3 board.
constexpr long CELL_J1[9] = {
  -900, 0, 900,
  -900, 0, 900,
  -900, 0, 900
};

constexpr long CELL_J2[9] = {
  1000, 1000, 1000,
  0, 0, 0,
  -1000, -1000, -1000
};

AccelStepper arm1(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper arm2(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);
AccelStepper upDown(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

String line;
bool stopped = false;

AccelStepper *axisFromName(const String &name) {
  if (name == "J1") return &arm1;
  if (name == "J2") return &arm2;
  if (name == "Z") return &upDown;
  return nullptr;
}

void configureAxis(AccelStepper &axis) {
  axis.setMaxSpeed(MAX_SPEED);
  axis.setAcceleration(ACCELERATION);
}

String nextToken(String &source) {
  source.trim();
  int splitAt = source.indexOf(' ');
  if (splitAt < 0) {
    String token = source;
    source = "";
    return token;
  }

  String token = source.substring(0, splitAt);
  source = source.substring(splitAt + 1);
  return token;
}

bool allStopped() {
  return arm1.distanceToGo() == 0 && arm2.distanceToGo() == 0 && upDown.distanceToGo() == 0;
}

void pollStopCommand() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      String commandLine = line;
      line = "";
      commandLine.trim();
      commandLine.toUpperCase();
      if (commandLine == "STOP") {
        stopAll();
        Serial.println("OK stop");
      }
    } else if (c != '\r') {
      line += c;
    }
  }
}

void runUntilArrived() {
  while (!stopped && !allStopped()) {
    arm1.run();
    arm2.run();
    upDown.run();
    pollStopCommand();
  }
}

void moveTo(long j1, long j2, long z) {
  if (stopped) return;
  arm1.moveTo(j1);
  arm2.moveTo(j2);
  upDown.moveTo(z);
  runUntilArrived();
}

void stopAll() {
  stopped = true;
  arm1.stop();
  arm2.stop();
  upDown.stop();
  digitalWrite(MAGNET_PIN, LOW);
}

void home() {
  stopped = false;
  moveTo(arm1.currentPosition(), arm2.currentPosition(), SAFE_Z);
  moveTo(HOME_J1, HOME_J2, SAFE_Z);
}

void pickBall() {
  stopped = false;
  moveTo(arm1.currentPosition(), arm2.currentPosition(), SAFE_Z);
  moveTo(PICK_J1, PICK_J2, SAFE_Z);
  moveTo(PICK_J1, PICK_J2, PICK_Z);
  digitalWrite(MAGNET_PIN, HIGH);
  delay(250);
  moveTo(PICK_J1, PICK_J2, SAFE_Z);
}

void placeBall(uint8_t cellNumber) {
  if (cellNumber < 1 || cellNumber > 9) {
    Serial.println("ERR cell must be 1-9");
    return;
  }

  uint8_t index = cellNumber - 1;
  stopped = false;
  moveTo(arm1.currentPosition(), arm2.currentPosition(), SAFE_Z);
  moveTo(CELL_J1[index], CELL_J2[index], SAFE_Z);
  moveTo(CELL_J1[index], CELL_J2[index], DROP_Z);
  digitalWrite(MAGNET_PIN, LOW);
  delay(250);
  moveTo(CELL_J1[index], CELL_J2[index], SAFE_Z);
}

void zeroAll() {
  arm1.setCurrentPosition(0);
  arm2.setCurrentPosition(0);
  upDown.setCurrentPosition(0);
}

void printStatus() {
  Serial.print("POS ");
  Serial.print("ARM1=");
  Serial.print(arm1.currentPosition());
  Serial.print(" ARM2=");
  Serial.print(arm2.currentPosition());
  Serial.print(" Z=");
  Serial.print(upDown.currentPosition());
  Serial.print(" MAG=");
  Serial.println(digitalRead(MAGNET_PIN) == HIGH ? 1 : 0);
}

void handleCommand(String commandLine) {
  commandLine.trim();
  if (commandLine.length() == 0) return;

  String command = nextToken(commandLine);
  command.toUpperCase();

  if (command == "JOG") {
    String axisName = nextToken(commandLine);
    axisName.toUpperCase();
    long steps = commandLine.toInt();

    AccelStepper *axis = axisFromName(axisName);
    if (axis == nullptr) {
      Serial.println("ERR unknown axis");
      return;
    }

    stopped = false;
    axis->move(steps);
    Serial.println("OK jog");
    return;
  }

  if (command == "PICK") {
    pickBall();
    Serial.println("OK pick");
    return;
  }

  if (command == "PLACE") {
    uint8_t cellNumber = commandLine.toInt();
    placeBall(cellNumber);
    Serial.println("OK place");
    return;
  }

  if (command == "MAG") {
    int value = commandLine.toInt();
    digitalWrite(MAGNET_PIN, value == 0 ? LOW : HIGH);
    Serial.println(value == 0 ? "OK magnet off" : "OK magnet on");
    return;
  }

  if (command == "STOP") {
    stopAll();
    Serial.println("OK stop");
    return;
  }

  if (command == "HOME") {
    home();
    Serial.println("OK home");
    return;
  }

  if (command == "ZERO") {
    zeroAll();
    Serial.println("OK zero");
    return;
  }

  if (command == "STATUS") {
    printStatus();
    return;
  }

  Serial.println("ERR unknown command");
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(MAGNET_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(MAGNET_PIN, LOW);

  configureAxis(arm1);
  configureAxis(arm2);
  configureAxis(upDown);

  Serial.begin(BAUD_RATE);
  Serial.println("READY pingpong scara");
}

void loop() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      handleCommand(line);
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }

  if (!stopped) {
    arm1.run();
    arm2.run();
    upDown.run();
  }
}
