#include <AccelStepper.h>
#include <ESP32Servo.h>

constexpr uint32_t BAUD_RATE = 115200;

// --- ESP32 ピン設定 ---
constexpr uint8_t J1_STEP_PIN = 14;
constexpr uint8_t J1_DIR_PIN = 27;
constexpr uint8_t J2_STEP_PIN = 18;
constexpr uint8_t J2_DIR_PIN = 19;

// Z軸（昇降用）
constexpr uint8_t Z_STEP_PIN = 26; 
constexpr uint8_t Z_DIR_PIN = 33;  
constexpr uint8_t Z_LIMIT_PIN = 32; // Z軸のリミットスイッチ用ピン

constexpr uint8_t ENABLE_PIN = 25;
constexpr uint8_t ARM_SERVO_PIN = 13;

// --- 動作設定 ---
constexpr float MAX_SPEED = 900.0;
constexpr float ACCELERATION = 600.0;
constexpr float HOMING_SPEED = -300.0; //原点を探すときの速度（マイナスだと下がる方向と仮定）

constexpr long HOME_J1 = 0;
constexpr long HOME_J2 = 0;
constexpr int SAFE_SERVO_ANGLE = 90; 

// --- モーターオブジェクト ---
AccelStepper arm1(AccelStepper::DRIVER, J1_STEP_PIN, J1_DIR_PIN);
AccelStepper arm2(AccelStepper::DRIVER, J2_STEP_PIN, J2_DIR_PIN);
AccelStepper zAxis(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
Servo armServo;

// --- 状態管理の変数 ---
String line;
bool stopped = false;
float arm1Speed = 0.0;
float arm2Speed = 0.0;
float zSpeed = 0.0;

// --- 関数プロトタイプ ---
void stopAll();
void clearDriveSpeeds();

AccelStepper *axisFromName(const String &name) {
  if (name == "J1") return &arm1;
  if (name == "J2") return &arm2;
  if (name == "Z") return &zAxis;
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
  clearDriveSpeeds();
  arm1.moveTo(j1);
  arm2.moveTo(j2);
  zAxis.moveTo(z);
  runUntilArrived();
}

void stopAll() {
  stopped = true;
  clearDriveSpeeds();
  arm1.stop();
  arm2.stop();
  zAxis.stop();
}

// Z軸の原点探し専用の関数を追加した
void homeZAxis() {
  Serial.println("Z axis homing started...");
  
  // スイッチが押されるまで、一定の速度で動き続ける
  zAxis.setSpeed(HOMING_SPEED);
  while (digitalRead(Z_LIMIT_PIN) == HIGH) {
    zAxis.runSpeed(); // 加減速なしで一定速度で動く
  }
  
  // スイッチが押されたら即停止
  zAxis.stop();
  
  // 少しだけ逆に戻して、スイッチから離れる（スイッチの保護と正確な位置決めのため）
  zAxis.move(50); // 50ステップ戻る（方向が逆の場合は -50 にしろ）
  while (zAxis.distanceToGo() != 0) {
    zAxis.run();
  }
  
  // 今いる場所を「原点」として記憶する
  zAxis.setCurrentPosition(0);
  Serial.println("Z axis homing complete!");
}

// HOMEコマンドの動作を変更
void home() {
  stopped = false;
  // 先にZ軸のホーミングを行う
  homeZAxis();
  
  // その後、アームを安全な位置に戻す
  moveTo(HOME_J1, HOME_J2, 0); 
  armServo.write(SAFE_SERVO_ANGLE);
}

void zeroAll() {
  arm1.setCurrentPosition(0);
  arm2.setCurrentPosition(0);
  zAxis.setCurrentPosition(0);
}

void clearDriveSpeeds() {
  arm1Speed = 0.0;
  arm2Speed = 0.0;
  zSpeed = 0.0;
  arm1.setSpeed(0.0);
  arm2.setSpeed(0.0);
  zAxis.setSpeed(0.0);
}

void setDriveSpeed(const String &axisName, float speed) {
  if (axisName == "J1") {
    arm1Speed = speed;
    arm1.setSpeed(speed);
    if (speed == 0.0) arm1.moveTo(arm1.currentPosition()); 
    return;
  }
  if (axisName == "J2") {
    arm2Speed = speed;
    arm2.setSpeed(speed);
    if (speed == 0.0) arm2.moveTo(arm2.currentPosition());
    return;
  }
  if (axisName == "Z") {
    zSpeed = speed;
    zAxis.setSpeed(speed);
    if (speed == 0.0) zAxis.moveTo(zAxis.currentPosition());
    return;
  }
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
    if (axis == nullptr) {
      Serial.println("ERR unknown axis");
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
      Serial.println("ERR unknown axis");
      return;
    }
    stopped = false;
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    setDriveSpeed(axisName, speed);
    Serial.println(speed == 0 ? "OK drive stop" : "OK drive");
    return;
  }

  if (command == "Z") {
    long position = commandLine.toInt();
    stopped = false;
    clearDriveSpeeds();
    zAxis.moveTo(position);
    Serial.println("OK Z move");
    return;
  }

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

  // リミットスイッチのピンを「内部プルアップ付きの入力」として設定
  pinMode(Z_LIMIT_PIN, INPUT_PULLUP); 

  ESP32PWM::allocateTimer(0);
  armServo.setPeriodHertz(50);
  armServo.attach(ARM_SERVO_PIN, 500, 2400);
  armServo.write(SAFE_SERVO_ANGLE);

  configureAxis(arm1);
  configureAxis(arm2);
  configureAxis(zAxis);
  
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
    if (arm1Speed != 0.0) arm1.runSpeed();
    else arm1.run();
    
    if (arm2Speed != 0.0) arm2.runSpeed();
    else arm2.run();

    if (zSpeed != 0.0) zAxis.runSpeed();
    else zAxis.run();
  }
}
