#include <Arduino.h>

// =================== SENSOR ===================
// Cảm biến: gặp trắng thì đèn sáng, gặp đen thì đèn tắt
// Ta cần line đen => quy về đen = 1, trắng = 0
const int S1 = 25;
const int S2 = 33;
const int S3 = 32;
const int S4 = 35;
const int S5 = 34;

// =================== MOTOR (L298N) ===================
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int pwmFreq = 1000;
const int pwmResolution = 8;

const int IN1 = 16;
const int IN2 = 17;
const int IN3 = 18;
const int IN4 = 19;
const int ENA = 22;
const int ENB = 23;

// Đảo chiều motor nếu bị cắm ngược
const bool INVERT_LEFT_MOTOR  = true;
const bool INVERT_RIGHT_MOTOR = true;

// =================== PID ===================
float Kp = 18.0;
float Kd = 10.0;
int baseSpeed = 100;
int lastError = 0;

// =================== RECOVER ===================
unsigned long recoverStartMs = 0;
const unsigned long LOST_CONFIRM_MS = 50;

// =================== TURN ===================
unsigned long turnStartMs = 0;
const unsigned long TURN_CLEAR_MS = 120;
const unsigned long TURN_TIMEOUT_MS = 900;
const int turnSpeed = 120;

// =================== JUNCTION ===================
bool junctionArmed = true;
unsigned long junctionLockUntil = 0;
const unsigned long JUNCTION_LOCK_MS = 220;

// =================== STATE ===================
enum Task1State {
  T1_FOLLOW,
  T1_RECOVER,
  T1_TURN_LEFT,
  T1_TURN_RIGHT,
  T1_UTURN,
  T1_DONE
};

Task1State task1State = T1_FOLLOW;

// =================== MOTOR CONTROL ===================
void setMotor(int right, int left) {
  if (INVERT_LEFT_MOTOR)  left  = -left;
  if (INVERT_RIGHT_MOTOR) right = -right;

  // Motor trái
  if (left >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    left = -left;
  }

  // Motor phải
  if (right >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    right = -right;
  }

  left  = constrain(left, 0, 255);
  right = constrain(right, 0, 255);

  ledcWrite(pwmChannelA, left);
  ledcWrite(pwmChannelB, right);
}

void stop_robot() {
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);
}

void pivotLeft(int spd) {
  setMotor(-spd, spd);
}

void pivotRight(int spd) {
  setMotor(spd, -spd);
}

// =================== SENSOR READ ===================
int readOne(int pin) {
  int raw = digitalRead(pin);

  // Module sáng khi trắng, tắt khi đen
  // Ta cần line đen = 1
  return (raw == LOW) ? 1 : 0;
}

void readSensors(int &s1, int &s2, int &s3, int &s4, int &s5) {
  s1 = readOne(S1);
  s2 = readOne(S2);
  s3 = 0;
  s4 = readOne(S4);
  s5 = readOne(S5);
}

int activeCount() {
  int s1, s2, s3, s4, s5;
  readSensors(s1, s2, s3, s4, s5);
  return s1 + s2 + s3 + s4 + s5;
}

bool detectJunction() {
  int c = activeCount();
  return (c >= 3);
}

// =================== LINE READ ===================
int readLine() {
  int s1, s2, s3, s4, s5;
  readSensors(s1, s2, s3, s4, s5);

  int c = s1 + s2 + s3 + s4 + s5;

  // mất line: giữ theo sai số cũ
  if (c == 0) return lastError;

  // giao lộ / line quá rộng
  if (c == 5) return 999;

  int error = (s1 * -2) + (s2 * -1) + (s3 * 0) + (s4 * 1) + (s5 * 2);
  return error;
}

void followLinePID() {
  int error = readLine();
  if (error == 999) return;

  int P = error;
  int D = error - lastError;
  int output = (int)(Kp * P + Kd * D);

  int speed = baseSpeed;
  if (abs(error) >= 2) speed = 85;

  int left  = speed + output;
  int right = speed - output;

  setMotor(right, left);
  lastError = error;
}

// =================== TURN ===================
void startLeftTurn() {
  turnStartMs = millis();
  task1State = T1_TURN_LEFT;
  stop_robot();
}

void startRightTurn() {
  turnStartMs = millis();
  task1State = T1_TURN_RIGHT;
  stop_robot();
}

void startUTurn() {
  turnStartMs = millis();
  task1State = T1_UTURN;
  stop_robot();
}

void updateTurnLeft() {
  unsigned long t = millis() - turnStartMs;

  int s1, s2, s3, s4, s5;
  readSensors(s1, s2, s3, s4, s5);
  int c = s1 + s2 + s3 + s4 + s5;

  if (t < TURN_CLEAR_MS) {
    pivotLeft(turnSpeed);
    return;
  }

  // Bắt lại line bằng sensor giữa
  if (s3 == 1 && c <= 3) {
    stop_robot();
    task1State = T1_FOLLOW;
    return;
  }

  if (t > TURN_TIMEOUT_MS) {
    stop_robot();
    task1State = T1_FOLLOW;
    return;
  }

  pivotLeft(turnSpeed);
}

void updateTurnRight() {
  unsigned long t = millis() - turnStartMs;

  int s1, s2, s3, s4, s5;
  readSensors(s1, s2, s3, s4, s5);
  int c = s1 + s2 + s3 + s4 + s5;

  if (t < TURN_CLEAR_MS) {
    pivotRight(turnSpeed);
    return;
  }

  if (s3 == 1 && c <= 3) {
    stop_robot();
    task1State = T1_FOLLOW;
    return;
  }

  if (t > TURN_TIMEOUT_MS) {
    stop_robot();
    task1State = T1_FOLLOW;
    return;
  }

  pivotRight(turnSpeed);
}

void updateUTurn() {
  unsigned long t = millis() - turnStartMs;

  if (t < TURN_CLEAR_MS) {
    pivotLeft(turnSpeed);
    return;
  }

  if (t > 2 * TURN_CLEAR_MS && t < TURN_TIMEOUT_MS) {
    pivotLeft(turnSpeed);
    return;
  }

  stop_robot();
  task1State = T1_FOLLOW;
}

// =================== RECOVER ===================
void recoverLine() {
  int c = activeCount();

  if (c > 0) {
    stop_robot();
    recoverStartMs = 0;
    task1State = T1_FOLLOW;
    return;
  }

  // Quay theo hướng lệch trước đó, nhưng có sửa chiều đúng
  if (lastError < 0) {
    pivotLeft(100);
  } else {
    pivotRight(100);
  }
}

// =================== JUNCTION LOGIC ===================
void handleJunction() {
  unsigned long now = millis();

  if (now < junctionLockUntil) return;
  if (!junctionArmed) return;

  int s1, s2, s3, s4, s5;
  readSensors(s1, s2, s3, s4, s5);

  bool left  = (s1 == 1 || s2 == 1);
  bool front = (s3 == 1);
  bool right = (s4 == 1 || s5 == 1);

  stop_robot();
  delay(25);

  junctionArmed = false;
  junctionLockUntil = now + JUNCTION_LOCK_MS;

  // Ưu tiên trái
  if (left) {
    startLeftTurn();
    return;
  }

  if (front) {
    setMotor(baseSpeed, baseSpeed);
    return;
  }

  if (right) {
    startRightTurn();
    return;
  }

  startUTurn();
}

// =================== TASK UPDATE ===================
void task1Update() {
  unsigned long now = millis();

  if (task1State == T1_TURN_LEFT) {
    updateTurnLeft();
    return;
  }

  if (task1State == T1_TURN_RIGHT) {
    updateTurnRight();
    return;
  }

  if (task1State == T1_UTURN) {
    updateUTurn();
    return;
  }

  if (task1State == T1_RECOVER) {
    recoverLine();
    return;
  }

  // ===== FOLLOW MODE =====
  int c = activeCount();

  if (c == 0) {
    if (recoverStartMs == 0) recoverStartMs = now;

    if (now - recoverStartMs > LOST_CONFIRM_MS) {
      task1State = T1_RECOVER;
      return;
    }

    followLinePID();
    return;
  } else {
    recoverStartMs = 0;
  }

  bool junctionNow = detectJunction();

  if (!junctionNow) {
    junctionArmed = true;
  }

  if (junctionNow && junctionArmed) {
    handleJunction();
    return;
  }

  followLinePID();
}

// =================== SETUP ===================
void setup() {
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelB, pwmFreq, pwmResolution);
  ledcAttachPin(ENA, pwmChannelA);
  ledcAttachPin(ENB, pwmChannelB);

  stop_robot();
}

// =================== LOOP ===================
void loop() {
  task1Update();
}