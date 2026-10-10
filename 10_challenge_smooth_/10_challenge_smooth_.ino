
#include <Servo.h>
#include <math.h>

#define PIN_SERVO 10
#define TRIG_PIN 12
#define ECHO_PIN 13

Servo myServo;

const int CLOSED_ANGLE = 30;
const int OPEN_ANGLE = 120;

const float DETECT_DISTANCE = 15.0;
const unsigned long MOVING_TIME = 2000;
const unsigned long CLEAR_TIME = 1500;

enum GateState { CLOSED, OPENING, OPEN, CLOSING };
GateState gateState = CLOSED;

int currentAngle = CLOSED_ANGLE;
int startAngle = CLOSED_ANGLE;
int targetAngle = CLOSED_ANGLE;

unsigned long moveStartTime = 0;
unsigned long clearStartTime = 0;
unsigned long lastReadTime = 0;
float distance = -1;

// 초음파 거리 측정
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);

  if (duration == 0) return -1;
  return duration * 0.0343 / 2.0;
}

// ServoSmooth: Smoothstep 보간 함수
float ServoSmooth(float t) {
  return t * t * (3.0 - 2.0 * t);
}

// 서보 이동 시작
void startMove(int target) {
  startAngle = currentAngle;
  targetAngle = target;
  moveStartTime = millis();

  gateState = (target == OPEN_ANGLE) ? OPENING : CLOSING;
}

// 서보 각도 갱신
void updateServo() {
  float t = (float)(millis() - moveStartTime) / MOVING_TIME;
  if (t > 1.0) t = 1.0;

  float eased = ServoSmooth(t);

  currentAngle = round(
    startAngle + (targetAngle - startAngle) * eased
  );
  myServo.write(currentAngle);

  if (t >= 1.0) {
    currentAngle = targetAngle;
    myServo.write(currentAngle);

    if (targetAngle == OPEN_ANGLE) {
      gateState = OPEN;
      clearStartTime = 0;
    } else {
      gateState = CLOSED;
    }
  }
}

void setup() {
  Serial.begin(57600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myServo.attach(PIN_SERVO);
  myServo.write(CLOSED_ANGLE);
}

void loop() {
  if (millis() - lastReadTime >= 60) {
    lastReadTime = millis();
    distance = getDistance();

    Serial.print("Distance:");
    Serial.print(distance);
    Serial.print("\tAngle:");
    Serial.println(currentAngle);
  }

  bool carDetected =
    distance > 0 && distance <= DETECT_DISTANCE;

  if (carDetected &&
      (gateState == CLOSED || gateState == CLOSING)) {
    startMove(OPEN_ANGLE);
    clearStartTime = 0;
  }

  if (gateState == OPENING || gateState == CLOSING) {
    updateServo();
  }

  if (gateState == OPEN) {
    if (carDetected) {
      clearStartTime = 0;
    } else {
      if (clearStartTime == 0) clearStartTime = millis();

      if (millis() - clearStartTime >= CLEAR_TIME) {
        startMove(CLOSED_ANGLE);
      }
    }
  }
}
