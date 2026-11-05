#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

constexpr uint32_t BAUD_RATE = 115200;
constexpr uint32_t LINK_TIMEOUT_MS = 1000;
constexpr uint8_t ENABLE_PIN = 16;
constexpr uint8_t ARM_SERVO_PIN = 13;
constexpr int SAFE_SERVO_ANGLE = 170;
constexpr float ARM_MAX_SPEED = 4000;
constexpr float ARM_ACCELERATION = 50;
constexpr float Z_MAX_SPEED = 4000;
constexpr float Z_ACCELERATION = 100;

// STEP / 未接続のダミーDIR。実際のDIRは下で個別に制御する。
AccelStepper arm1(AccelStepper::DRIVER, 25, 4);
AccelStepper arm2(AccelStepper::DRIVER, 27, 5);
AccelStepper zAxis(AccelStepper::DRIVER, 33, 18);
AccelStepper *axes[] = {&arm1, &arm2, &zAxis};
const uint8_t dirPins[] = {22, 14, 32};
const float maxSpeeds[] = {
  ARM_MAX_SPEED,
  ARM_MAX_SPEED,
  Z_MAX_SPEED
};

bool driveMode[3] = {false, false, false};
int lastDir[3] = {-1, -1, -1};
Servo armServo;

bool stopped = true;
bool havePing = false;
bool homePending = false;
uint32_t lastPingAt = 0;
int servoAngle = SAFE_SERVO_ANGLE;
int servoOffset = 0;
unsigned long requestId = 0;
char inputLine[128];
size_t inputLength = 0;
bool inputOverflow = false;

void reply(const char *message) {
  if (requestId) {
    Serial.print('@');
    Serial.print(requestId);
    Serial.print(' ');
  }

  Serial.println(message);
}

void haltAxis(int i) {
  // 現在位置を維持したまま、速度と古い目標を取り消す。
  axes[i]->setCurrentPosition(axes[i]->currentPosition());
  driveMode[i] = false;
}

void stopAll() {
  stopped = true;
  homePending = false;
  havePing = false;

  for (int i = 0; i < 3; ++i) {
    haltAxis(i);
  }

  // 保持トルクを残す。サーボも最後の指令角度を保持する。
}

bool allIdle() {
  for (int i = 0; i < 3; ++i) {
    if (axes[i]->speed() != 0 ||
        axes[i]->distanceToGo() != 0) {
      return false;
    }
  }

  return true;
}

void setDrive(int i, float speed) {
  homePending = false;

  if (speed == 0) {
    haltAxis(i);
    return;
  }

  if (!driveMode[i]) {
    haltAxis(i);
  }

  driveMode[i] = true;
  axes[i]->setSpeed(
    constrain(speed, -maxSpeeds[i], maxSpeeds[i])
  );
}

int axisIndex(const char *name) {
  if (!strcmp(name, "J1")) return 0;
  if (!strcmp(name, "J2")) return 1;
  if (!strcmp(name, "Z")) return 2;

  return -1;
}

bool number(const char *s, float &value) {
  char *end;
  errno = 0;
  value = strtof(s, &end);

  return end != s &&
         *end == '\0' &&
         errno == 0 &&
         isfinite(value);
}

bool integer(const char *s, long &value) {
  char *end;
  errno = 0;
  value = strtol(s, &end, 10);

  return end != s &&
         *end == '\0' &&
         errno == 0 &&
         value >= -1000000000L &&
         value <= 1000000000L;
}

void printStatus() {
  char status[220];

  snprintf(
    status,
    sizeof(status),
    "POS PROTO=2 ARM1=%ld ARM2=%ld Z_POS=%ld "
    "ARM_ANGLE=%d ARM_LOGICAL=%d OFFSET=%d STOPPED=%d",
    arm1.currentPosition(),
    arm2.currentPosition(),
    zAxis.currentPosition(),
    servoAngle,
    servoAngle - servoOffset,
    servoOffset,
    stopped ? 1 : 0
  );

  reply(status);
}

void handleCommand(char *text) {
  requestId = 0;

  char *save = nullptr;
  char *token = strtok_r(text, " \t\r", &save);

  if (!token) return;

  // HTML側からの通信番号。通常の文字コマンドも受け付ける。
  if (token[0] == '@') {
    char *end;
    errno = 0;
    requestId = strtoul(token + 1, &end, 10);

    if (!isdigit(token[1]) ||
        *end ||
        errno ||
        !requestId) {
      requestId = 0;
      reply("ERR ID");
      return;
    }

    token = strtok_r(nullptr, " \t\r", &save);
  }

  char *args[5];
  int count = 0;

  while (token && count < 5) {
    args[count++] = token;
    token = strtok_r(nullptr, " \t\r", &save);
  }

  if (!count || token) {
    reply("ERR FORMAT");
    return;
  }

  for (int i = 0; i < count; ++i) {
    for (char *p = args[i]; *p; ++p) {
      *p = toupper((unsigned char)*p);
    }
  }

  const char *cmd = args[0];

  if (!strcmp(cmd, "STOP") && count == 1) {
    stopAll();
    reply("OK STOP");
    return;
  }

  if (!strcmp(cmd, "PING") && count == 1) {
    lastPingAt = millis();
    havePing = true;
    reply("OK PING");
    return;
  }

  if (!strcmp(cmd, "STATUS") && count == 1) {
    printStatus();
    return;
  }

  if (!strcmp(cmd, "RESUME") && count == 1) {
    if (!havePing ||
        uint32_t(millis() - lastPingAt) >= LINK_TIMEOUT_MS) {
      reply("ERR PING_REQUIRED");
      return;
    }

    stopped = false;
    reply("OK RESUME");
    return;
  }

  if ((!strcmp(cmd, "ZERO") ||
       !strcmp(cmd, "SERVO_ZERO")) &&
      count == 1) {
    if (!stopped) {
      reply("ERR STOP_FIRST");
      return;
    }

    if (!strcmp(cmd, "ZERO")) {
      for (int i = 0; i < 3; ++i) {
        axes[i]->setCurrentPosition(0);
      }

      reply("OK ZERO");
    } else {
      servoOffset = servoAngle;
      reply("OK SERVO_ZERO");
    }

    return;
  }

  // STOP後はRESUMEまで移動・サーボ操作を受け付けない。
  if (stopped) {
    reply("ERR STOPPED");
    return;
  }

  if (!strcmp(cmd, "DRIVE3") && count == 4) {
    float values[3];

    for (int i = 0; i < 3; ++i) {
      if (!number(args[i + 1], values[i])) {
        reply("ERR NUMBER");
        return;
      }
    }

    for (int i = 0; i < 3; ++i) {
      setDrive(i, values[i]);
    }

    reply("OK DRIVE3");
    return;
  }

  if (!strcmp(cmd, "DRIVE") && count == 3) {
    int i = axisIndex(args[1]);
    float speed;

    if (i < 0 || !number(args[2], speed)) {
      reply("ERR ARGUMENT");
      return;
    }

    setDrive(i, speed);
    reply("OK DRIVE");
    return;
  }

  if ((!strcmp(cmd, "JOG") && count == 3) ||
      (!strcmp(cmd, "Z") && count == 2)) {
    bool jog = !strcmp(cmd, "JOG");
    int i = jog ? axisIndex(args[1]) : 2;
    long value;

    if (i < 0 ||
        !integer(args[jog ? 2 : 1], value)) {
      reply("ERR ARGUMENT");
      return;
    }

    int64_t target = jog
      ? int64_t(axes[i]->currentPosition()) + value
      : value;

    if (target < -1000000000LL ||
        target > 1000000000LL) {
      reply("ERR RANGE");
      return;
    }

    homePending = false;
    haltAxis(i);
    axes[i]->moveTo(long(target));

    reply(jog ? "OK JOG" : "OK Z");
    return;
  }

  if (!strcmp(cmd, "HOME") && count == 1) {
    for (int i = 0; i < 3; ++i) {
      haltAxis(i);
      axes[i]->moveTo(0);
    }

    homePending = true;
    reply("OK HOME");
    return;
  }

  if ((!strcmp(cmd, "ARM") ||
       !strcmp(cmd, "SERVO_ADD")) &&
      count == 2) {
    long value;

    if (!integer(args[1], value) ||
        value < -180 ||
        value > 180) {
      reply("ERR ANGLE");
      return;
    }

    bool absolute = !strcmp(cmd, "ARM");

    long actual = absolute
      ? servoOffset + value
      : servoAngle + value;

    if (actual < 0 || actual > 180) {
      reply("ERR SERVO_RANGE");
      return;
    }

    servoAngle = int(actual);
    armServo.write(servoAngle);

    reply(absolute ? "OK ARM" : "OK SERVO_ADD");
    return;
  }

  reply("ERR COMMAND");
}

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  for (int i = 0; i < 3; ++i) {
    pinMode(dirPins[i], OUTPUT);
    digitalWrite(dirPins[i], LOW);

    axes[i]->setMaxSpeed(maxSpeeds[i]);
    axes[i]->setAcceleration(
      i == 2 ? Z_ACCELERATION : ARM_ACCELERATION
    );
    axes[i]->setMinPulseWidth(3);
  }

  ESP32PWM::allocateTimer(0);
  armServo.setPeriodHertz(50);
  armServo.attach(ARM_SERVO_PIN, 500, 2400);
  armServo.write(servoAngle);

  stopAll();

  digitalWrite(ENABLE_PIN, LOW);
  Serial.println("READY PAKIRA 2");
}

void loop() {
  if (!stopped &&
      uint32_t(millis() - lastPingAt) >= LINK_TIMEOUT_MS) {
    stopAll();
    Serial.println("EVENT STOP TIMEOUT");
  }

  // 受信が多くてもモーター処理を続ける。
  for (int budget = 0;
       budget < 32 && Serial.available();
       ++budget) {
    char c = char(Serial.read());

    if (c == '\n') {
      if (inputOverflow) {
        stopAll();
        Serial.println("EVENT STOP LINE_TOO_LONG");
      } else {
        inputLine[inputLength] = '\0';
        handleCommand(inputLine);
      }

      inputLength = 0;
      inputOverflow = false;

    } else if (c != '\r') {
      if (inputLength < sizeof(inputLine) - 1 &&
          !inputOverflow) {
        inputLine[inputLength++] = c;
      } else {
        inputOverflow = true;
      }
    }
  }

  if (stopped) return;

  for (int i = 0; i < 3; ++i) {
    float speed = axes[i]->speed();

    if (speed != 0) {
      int direction = speed > 0 ? HIGH : LOW;

      if (direction != lastDir[i]) {
        digitalWrite(dirPins[i], direction);
        delayMicroseconds(2);
        lastDir[i] = direction;
      }
    }

    if (driveMode[i]) {
      axes[i]->runSpeed();
    } else {
      axes[i]->run();
    }
  }

  if (homePending && allIdle()) {
    homePending = false;
    servoAngle = SAFE_SERVO_ANGLE;
    armServo.write(servoAngle);
  }
}
