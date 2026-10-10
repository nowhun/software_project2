// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// Configurable parameters
#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10

#define _DIST_MIN 100
#define _DIST_MAX 300

#define TIMEOUT ((INTERVAL / 2) * 1000UL)
#define SCALE (0.001 * 0.5 * SND_VEL)

// Median filter sample count (N)
#define MEDIAN_N 30

// Global variables
unsigned long last_sampling_time = 0;

// Circular buffer for recent samples
float samples[MEDIAN_N];
int sample_count = 0;
int sample_index = 0;

// Function declarations
float USS_measure(int TRIG, int ECHO);
float median_filter(float new_value);

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_TRIG, LOW);

  Serial.begin(57600);

  // Initialize sample buffer
  for (int i = 0; i < MEDIAN_N; i++) {
    samples[i] = 0.0;
  }
}

void loop() {
  float dist_raw;
  float dist_median;

  // Wait until next sampling time
  if (millis() - last_sampling_time < INTERVAL) {
    return;
  }

  last_sampling_time += INTERVAL;

  // Read ultrasonic sensor
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // Apply ONLY the median filter.
  // Do not replace invalid readings with previous valid values.
  dist_median = median_filter(dist_raw);

  // LED indicates whether the raw reading is out of range
  if (dist_raw < _DIST_MIN || dist_raw > _DIST_MAX) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }

  // Serial Plotter output
  Serial.print("Min:");
  Serial.print(_DIST_MIN);

  Serial.print(",raw:");
  Serial.print(min(dist_raw, (float)(_DIST_MAX + 100)));

  Serial.print(",ema:");
Serial.print(dist_raw);

  Serial.print(",median:");
  Serial.print(min(dist_median, (float)(_DIST_MAX + 100)));

  Serial.print(",Max:");
  Serial.print(_DIST_MAX);

  Serial.println("");
}

// Median filter
// Stores the latest MEDIAN_N readings and returns their median.
float median_filter(float new_value) {
  float sorted[MEDIAN_N];

  // Insert the new sample into the circular buffer
  samples[sample_index] = new_value;
  sample_index = (sample_index + 1) % MEDIAN_N;

  if (sample_count < MEDIAN_N) {
    sample_count++;
  }

  // Copy current samples to a temporary array
  for (int i = 0; i < sample_count; i++) {
    sorted[i] = samples[i];
  }

  // Sort the samples in ascending order (insertion sort)
  for (int i = 1; i < sample_count; i++) {
    float key = sorted[i];
    int j = i - 1;

    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }

    sorted[j + 1] = key;
  }

  // Return the median
  if (sample_count % 2 == 1) {
    return sorted[sample_count / 2];
  } else {
    return (sorted[sample_count / 2 - 1]
          + sorted[sample_count / 2]) / 2.0;
  }
}

// Ultrasonic sensor measurement (unit: mm)
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, TIMEOUT);

  // Timeout returns 0. Keep it as a raw measurement.
  // Do NOT substitute the previous valid distance.
  return duration * SCALE;
}
