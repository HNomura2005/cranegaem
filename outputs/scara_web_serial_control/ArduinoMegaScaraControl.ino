#include <AccelStepper.h>
#include <ESP32Servo.h>

constexpr uint32_t BAUD_RATE = 115200;

// --- ESP32 ピン設定 ---
constexpr uint8_t J1_STEP_PIN = 25;
constexpr uint8_t J1_DIR_PIN = 26;
constexpr uint8_t J2_STEP_PIN = 27;
constexpr uint8_t J2_DIR_PIN = 14;

constexpr uint8_t Z_STEP_PIN = 33; 
constexpr uint8_t Z_DIR_PIN = 32;  

constexpr uint8_t ENABLE_PIN = 16;
constexpr uint8_t ARM_SERVO_PIN = 13;

// --- 動作設定 ---
constexpr float ARM_MAX_SPEED = 900.0;
constexpr float ARM_ACCELERATION = 600.0;
constexpr float Z_MAX_SPEED = 2000.0;   
constexpr float Z_ACCELERATION = 600.0; 

constexpr long HOME_J1 = 0;
constexpr long HOME_J2 = 0;
constexpr int SAFE_SERVO_ANGLE = 70; 

AccelStepper arm1(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper arm2(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);
AccelStepper zAxis(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
Servo armServo;

String line;
bool stopped = false;
int servoOffset = 0; 

// --- 手動操作（DRIVE）用の変数 ---
float driveSpeedJ1 = 0.0;
float driveSpeedJ2 = 0.0;
float driveSpeedZ = 0.0;

unsigned long lastStepJ1 = 0;
unsigned long lastStepJ2 = 0;
unsigned long lastStepZ = 0;

void stopAll() {
  stopped = true;
  driveSpeedJ1 = 0.0;
  driveSpeedJ2 = 0.0;
  driveSpeedZ = 0.0;
  arm1.stop();
  arm2.stop();
  zAxis.stop();
}

void moveServo(int logicalAngle) {
  int actualAngle = constrain(logicalAngle + servoOffset, 0, 180);
  armServo.write(actualAngle);
}

AccelStepper *axisFromName(const String &name) {
  if (name == "J1") return &arm1;
  if (name == "J2") return &arm2;
  if (name == "Z") return &zAxis;
  return nullptr; 
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
  return arm1.distanceToGo() == 0 && arm2.distanceToGo() == 0 && zAxis.distanceToGo() == 0;
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
    zAxis.run();
    pollStopCommand();
  }
}

void moveTo(long j1, long j2, long z) {
  if (stopped) return;
  driveSpeedJ1 = 0.0;
  driveSpeedJ2 = 0.0;
  driveSpeedZ = 0.0;
  arm1.moveTo(j1);
  arm2.moveTo(j2);
  zAxis.moveTo(z);
  runUntilArrived();
}

void home() {
  stopped = false;
  moveTo(HOME_J1, HOME_J2, 0); 
  moveServo(SAFE_SERVO_ANGLE);
}

void zeroAll() {
  arm1.setCurrentPosition(0);
  arm2.setCurrentPosition(0);
  zAxis.setCurrentPosition(0);
}

void printStatus() {
  Serial.print("POS ");
  Serial.print("ARM1=");
  Serial.print(arm1.currentPosition());
  Serial.print(" ARM2=");
  Serial.print(arm2.currentPosition());
  Serial.print(" Z_POS=");
  Serial.print(zAxis.currentPosition());
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
    if (axis == nullptr) return;
    
    stopped = false;
    axis->move(steps);
    Serial.println("OK jog");
    return;
  }
  
  if (command == "DRIVE") {
    String axisName = nextToken(commandLine);
    axisName.toUpperCase();
    float speed = commandLine.toFloat();
    
    stopped = false;
    float maxAllowedSpeed = (axisName == "Z") ? Z_MAX_SPEED : ARM_MAX_SPEED;
    speed = constrain(speed, -maxAllowedSpeed, maxAllowedSpeed);
    
    if (axisName == "J1") driveSpeedJ1 = speed;
    else if (axisName == "J2") driveSpeedJ2 = speed;
    else if (axisName == "Z") driveSpeedZ = speed;

    Serial.println(speed == 0 ? "OK drive stop" : "OK drive");
    return;
  }

  if (command == "Z") {
    long position = commandLine.toInt();
    stopped = false;
    zAxis.moveTo(position);
    Serial.println("OK Z move");
    return;
  }

  if (command == "ARM") {
    int angle = commandLine.toInt();
    moveServo(angle);
    Serial.println("OK arm moved");
    return;
  }

  if (command == "SERVO_ADD") {
    int diff = commandLine.toInt();
    int currentActual = armServo.read();
    int newActual = constrain(currentActual + diff, 0, 180);
    armServo.write(newActual);
    Serial.println("OK servo adjusted");
    return;
  }

  if (command == "SERVO_ZERO") {
    servoOffset = armServo.read();
    Serial.println("OK servo zero set");
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
}

// ★ ここが今回の一番の肝です！
void customDriveLoop() {
  unsigned long now = micros();

  // J1軸
  if (driveSpeedJ1 != 0.0) {
    unsigned long interval = 1000000.0 / abs(driveSpeedJ1);
    if (now - lastStepJ1 >= interval) {
      lastStepJ1 = now;
      bool isPositive = (driveSpeedJ1 > 0);
      
      // ① 方向（DIR）をセット
      digitalWrite(J1_DIR_PIN, isPositive ? HIGH : LOW);
      
      // ② ★超重要：モータードライバに方向を認識させるための待機時間！
      delayMicroseconds(20); 
      
      // ③ パルス（STEP）を出す
      digitalWrite(J1_STEP_PIN, HIGH);
      delayMicroseconds(20); // パルスもしっかり読ませる
      digitalWrite(J1_STEP_PIN, LOW);
      
      arm1.setCurrentPosition(arm1.currentPosition() + (isPositive ? 1 : -1));
    }
  }

  // J2軸
  if (driveSpeedJ2 != 0.0) {
    unsigned long interval = 1000000.0 / abs(driveSpeedJ2);
    if (now - lastStepJ2 >= interval) {
      lastStepJ2 = now;
      bool isPositive = (driveSpeedJ2 > 0);
      
      digitalWrite(J2_DIR_PIN, isPositive ? HIGH : LOW);
      delayMicroseconds(20); // ★同様に追加
      digitalWrite(J2_STEP_PIN, HIGH);
      delayMicroseconds(20);
      digitalWrite(J2_STEP_PIN, LOW);
      
      arm2.setCurrentPosition(arm2.currentPosition() + (isPositive ? 1 : -1));
    }
  }

  // Z軸
  if (driveSpeedZ != 0.0) {
    unsigned long interval = 1000000.0 / abs(driveSpeedZ);
    if (now - lastStepZ >= interval) {
      lastStepZ = now;
      bool isPositive = (driveSpeedZ > 0);
      
      digitalWrite(Z_DIR_PIN, isPositive ? HIGH : LOW);
      delayMicroseconds(20); // ★同様に追加
      digitalWrite(Z_STEP_PIN, HIGH);
      delayMicroseconds(20);
      digitalWrite(Z_STEP_PIN, LOW);
      
      zAxis.setCurrentPosition(zAxis.currentPosition() + (isPositive ? 1 : -1));
    }
  }
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pinMode(J1_STEP_PIN, OUTPUT);
  pinMode(J1_DIR_PIN, OUTPUT);
  pinMode(J2_STEP_PIN, OUTPUT);
  pinMode(J2_DIR_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);

  ESP32PWM::allocateTimer(0);
  armServo.setPeriodHertz(50);
  armServo.attach(ARM_SERVO_PIN, 500, 2400);
  moveServo(SAFE_SERVO_ANGLE);

  arm1.setMaxSpeed(ARM_MAX_SPEED);
  arm1.setAcceleration(ARM_ACCELERATION);
  arm2.setMaxSpeed(ARM_MAX_SPEED);
  arm2.setAcceleration(ARM_ACCELERATION);
  zAxis.setMaxSpeed(Z_MAX_SPEED);
  zAxis.setAcceleration(Z_ACCELERATION);
  
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
    if (driveSpeedJ1 != 0.0 || driveSpeedJ2 != 0.0 || driveSpeedZ != 0.0) {
      customDriveLoop();
    } else {
      arm1.run();
      arm2.run();
      zAxis.run();
    }
  }
}
