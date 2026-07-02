#include <AccelStepper.h>
#include <ESP32Servo.h>


constexpr uint32_t BAUD_RATE = 115200;

// --- ESP32 Pin  ---
constexpr uint8_t J1_STEP_PIN = 14;
constexpr uint8_t J1_DIR_PIN = 27;
constexpr uint8_t J2_STEP_PIN = 18;
constexpr uint8_t J2_DIR_PIN = 19;
constexpr uint8_t ENABLE_PIN = 25; // LOW enables A4988 drivers

constexpr uint8_t Z_SERVO_PIN = 12;
constexpr uint8_t ARM_SERVO_PIN = 13;

// --- Motion Settings ---
constexpr float MAX_SPEED = 900.0;
constexpr float ACCELERATION = 600.0;
constexpr long HOME_J1 = 0;
constexpr long HOME_J2 = 0;
constexpr int SAFE_SERVO_ANGLE = 90; 
// --- Motor Objects ---
AccelStepper arm1(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper arm2(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);

Servo zServo;
Servo armServo;

// --- State Variables ---
String line;
bool stopped = false;
float arm1Speed = 0.0;
float arm2Speed = 0.0;

// --- Function Prototypes ---
void stopAll();
void clearDriveSpeeds();

AccelStepper *axisFromName(const String &name) {
  if (name == "J1") return &arm1;
  if (name == "J2") return &arm2;
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
  return arm1.distanceToGo() == 0 && arm2.distanceToGo() == 0;
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
    pollStopCommand();
  }
}

void moveTo(long j1, long j2) {
  if (stopped) return;
  clearDriveSpeeds();
  arm1.moveTo(j1);
  arm2.moveTo(j2);
  runUntilArrived();
}

void stopAll() {
  stopped = true;
  arm1Speed = 0.0;
  arm2Speed = 0.0;
  arm1.stop();
  arm2.stop();
}

void home() {
  stopped = falses
  moveTo(HOME_J1, HOME_J2);
  zServo.write(SAFE_SERVO_ANGLE);
  armServo.write(SAFE_SERVO_ANGLE);
}

void zeroAll() {
  arm1.setCurrentPosition(0);
  arm2.setCurrentPosition(0);
}

void clearDriveSpeeds() {
  arm1Speed = 0.0;
  arm2Speed = 0.0;
  arm1.setSpeed(0.0);
  arm2.setSpeed(0.0);
}

// 長押し後に「元の座標に戻ろうとする暴走」を防ぐ処理を追加
void setDriveSpeed(const String &axisName, float speed) {
  if (axisName == "J1") {
    arm1Speed = speed;
    arm1.setSpeed(speed);
    if (speed == 0.0) arm1.moveTo(arm1.currentPosition()); // 今の場所を新たな目標にセット
    return;
  }
  if (axisName == "J2") {
    arm2Speed = speed;
    arm2.setSpeed(speed);
    if (speed == 0.0) arm2.moveTo(arm2.currentPosition()); // 今の場所を新たな目標にセット
    return;
  }
}

void printStatus() {
  Serial.print("POS ");
  Serial.print("ARM1=");
  Serial.print(arm1.currentPosition());
  Serial.print(" ARM2=");
  Serial.print(arm2.currentPosition());
  Serial.print(" Z_ANGLE=");
  Serial.print(zServo.read());
  Serial.print(" ARM_ANGLE=");
  Serial.println(armServo.read());
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
      Serial.println("ERR unknown axis or axis is servo");
      return;
    }
    
    stopped = false;
    clearDriveSpeeds();
    axis->move(steps);
    Serial.println("OK jog");
    return;
  }
  
  if (command == "DRIVE") {
    String axisName = nextToken(commandLine);
    axisName.toUpperCase();
    float speed = commandLine.toFloat();
    
    if (axisFromName(axisName) == nullptr) {
      Serial.println("ERR unknown axis or axis is servo");
      return;
    }
    
    stopped = false;
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    setDriveSpeed(axisName, speed);
    Serial.println(speed == 0 ? "OK drive stop" : "OK drive");
    return;
  }

  // Z軸（上下）のサーボ制御
  if (command == "Z") {
    int angle = commandLine.toInt();
    angle = constrain(angle, 0, 180); 
    zServo.write(angle);
    Serial.println("OK Z moved");
    return;
  }

  // アームのサーボ制御
  if (command == "ARM") {
    int angle = commandLine.toInt();
    angle = constrain(angle, 0, 180); 
    armServo.write(angle);
    Serial.println("OK arm moved");
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
  digitalWrite(ENABLE_PIN, LOW); 


  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  zServo.setPeriodHertz(50); 
  armServo.setPeriodHertz(50);
  
  zServo.attach(Z_SERVO_PIN, 500, 2400); 
  armServo.attach(ARM_SERVO_PIN, 500, 2400);
  
 
  zServo.write(SAFE_SERVO_ANGLE);
  armServo.write(SAFE_SERVO_ANGLE);

  configureAxis(arm1);
  configureAxis(arm2);
  
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
    if (arm1Speed != 0.0) {
      arm1.runSpeed();
    } else {
      arm1.run();
    }
    
    if (arm2Speed != 0.0) {
      arm2.runSpeed();
    } else {
      arm2.run();
    }
  }
}
