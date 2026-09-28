// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)    // coefficient to convert duration to distance

unsigned long last_sampling_time;   // unit: msec

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float distance;
  int brightness;

  // wait until next sampling time
  unsigned long current_time = millis();

  if (current_time - last_sampling_time < INTERVAL)
    return;

  last_sampling_time += INTERVAL;

  // read distance
  distance = USS_measure(PIN_TRIG, PIN_ECHO);

  // ---------------------------------------
  // LED brightness control
  // Active Low:
  // analogWrite(pin, 0)   = maximum brightness
  // analogWrite(pin, 225) = LED OFF
  // ---------------------------------------

  if (distance == 0.0 || distance <= _DIST_MIN || distance >= _DIST_MAX) {
    // 100 mm 이하 또는 300 mm 이상이면 LED OFF
    brightness = 225;
  }
  else if (distance <= 200.0) {
    // 100 ~ 200 mm
    // 100 mm -> 225
    // 200 mm -> 0
    brightness = (int)(225.0 * (200.0 - distance) / 100.0);
  }
  else {
    // 200 ~ 300 mm
    // 200 mm -> 0
    // 300 mm -> 225
    brightness = (int)(225.0 * (distance - 200.0) / 100.0);
  }

  // LED PWM output
  analogWrite(PIN_LED, brightness);

  // ---------------------------------------
  // output the distance to the serial port
  // ---------------------------------------

  Serial.print("Min:");
  Serial.print(_DIST_MIN);

  Serial.print(",distance:");
  Serial.print(distance);

  Serial.print(",Max:");
  Serial.print(_DIST_MAX);

  Serial.print(",PWM:");
  Serial.println(brightness);
}


// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
