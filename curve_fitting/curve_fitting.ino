#define PIN_IR A0

#define N_POINTS   7      // 0 ~ 30cm, 5cm 간격
#define MAX_DEGREE 5

#define DEFAULT_METHOD 2
#define DEFAULT_DEGREE 2
#define DEFAULT_N      30

int method = DEFAULT_METHOD;
int degree = DEFAULT_DEGREE;
int n_repeat = DEFAULT_N;

unsigned int adc[N_POINTS];       // 위치별 센서 값
float coef[MAX_DEGREE + 1];       // 다항식 계수 (상수항부터)
float x_mid, x_half;              // x = (t - x_mid) / x_half

// ---------------------------------------------------------------
// 시리얼 한 줄 읽기 
// ---------------------------------------------------------------
void read_line(char *buf, int size)
{
  int len = 0;
  while (true) {
    while (Serial.available() == 0)
      ;
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      delay(10);   // CRLF의 나머지 문자 버리기
      while (Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r'))
        Serial.read();
      break;
    }
    if (len < size - 1)
      buf[len++] = c;
  }
  buf[len] = '\0';
}

// 설정값 입력 (빈 입력 = 기본값)
int read_setting(int lo, int hi, int def)
{
  char buf[16];
  while (true) {
    Serial.print(F("  범위 ")); Serial.print(lo); Serial.print(F("~")); Serial.print(hi);
    Serial.print(F(", 기본값 ")); Serial.print(def); Serial.print(F(" > "));
    read_line(buf, sizeof(buf));
    if (buf[0] == '\0') {
      Serial.println(def);
      return def;
    }
    int v = atoi(buf);
    Serial.println(v);
    if (v >= lo && v <= hi)
      return v;
    Serial.println(F("  범위를 벗어났습니다. 다시 입력하세요."));
  }
}

// ---------------------------------------------------------------
// 센서
// ---------------------------------------------------------------
int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position)
{
  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1))
    return 0;
  if (position == 1.0)
    position = 0.999;

  unsigned int *ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;
  for (unsigned int i = 0; i < n; i++)
    ir_val[i] = analogRead(PIN_IR);
  qsort(ir_val, n, sizeof(unsigned int), compare);
  unsigned int ret_val = ir_val[(unsigned int)(n * position)];
  free(ir_val);
  return ret_val;
}

// ---------------------------------------------------------------
// 곡선맞춤
// ---------------------------------------------------------------
float to_t(unsigned int v)
{
  return (method == 1) ? (float)v : 100.0 / v;
}

float volt_to_distance(unsigned int a_value)
{
  float x = (to_t(a_value) - x_mid) / x_half;
  float y = 0, p = 1;
  for (int k = 0; k <= degree; k++) {
    y += coef[k] * p;
    p *= x;
  }
  return y;
}

bool fit_curve()
{
  int m = degree + 1;

  // x 정규화 범위
  float t_min = to_t(adc[0]), t_max = t_min;
  for (int i = 1; i < N_POINTS; i++) {
    float t = to_t(adc[i]);
    if (t < t_min) t_min = t;
    if (t > t_max) t_max = t;
  }
  x_mid  = (t_max + t_min) / 2.0;
  x_half = (t_max - t_min) / 2.0;
  if (x_half <= 0)
    return false;

  // 정규방정식
  float A[MAX_DEGREE + 1][MAX_DEGREE + 2];
  for (int r = 0; r < m; r++)
    for (int c = 0; c <= m; c++)
      A[r][c] = 0;

  for (int i = 0; i < N_POINTS; i++) {
    float x = (to_t(adc[i]) - x_mid) / x_half;
    float y = 50.0 * i;            // 실제 위치 0, 50, ..., 300mm
    float pw[2 * MAX_DEGREE + 1];
    pw[0] = 1;
    for (int k = 1; k <= 2 * degree; k++)
      pw[k] = pw[k - 1] * x;
    for (int r = 0; r < m; r++) {
      for (int c = 0; c < m; c++)
        A[r][c] += pw[r + c];
      A[r][m] += pw[r] * y;
    }
  }

  // 가우스 소거
  for (int col = 0; col < m; col++) {
    int piv = col;
    for (int r = col + 1; r < m; r++)
      if (fabs(A[r][col]) > fabs(A[piv][col]))
        piv = r;
    if (fabs(A[piv][col]) < 1e-9)
      return false;
    if (piv != col)
      for (int c = 0; c <= m; c++) {
        float tmp = A[col][c]; A[col][c] = A[piv][c]; A[piv][c] = tmp;
      }
    for (int r = col + 1; r < m; r++) {
      float f = A[r][col] / A[col][col];
      for (int c = col; c <= m; c++)
        A[r][c] -= f * A[col][c];
    }
  }

  // 후진 대입
  for (int r = m - 1; r >= 0; r--) {
    float s = A[r][m];
    for (int c = r + 1; c < m; c++)
      s -= A[r][c] * coef[c];
    coef[r] = s / A[r][r];
  }
  return true;
}

// ---------------------------------------------------------------
// 결과 출력
// ---------------------------------------------------------------
void print_equation()
{
  Serial.println();
  Serial.println(F("==============================="));
  Serial.println(F("float volt_to_distance(unsigned int a_value)   // unit: mm"));
  Serial.println(F("{"));
  if (method == 1) {
    Serial.print(F("  float x = ((float)a_value - "));
    Serial.print(x_mid, 4);
    Serial.print(F(") / "));
    Serial.print(x_half, 4);
  } else {
    Serial.print(F("  float x = (100.0 / a_value - "));
    Serial.print(x_mid, 6);
    Serial.print(F(") / "));
    Serial.print(x_half, 6);
  }
  Serial.println(F(";"));

  Serial.print(F("  return "));
  for (int k = 0; k <= degree; k++) {
    if (k == 0) {
      Serial.print(coef[k], 4);
    } else {
      Serial.print(coef[k] < 0 ? F(" - ") : F(" + "));
      Serial.print(fabs(coef[k]), 4);
      for (int j = 0; j < k; j++)
        Serial.print(F(" * x"));
    }
  }
  Serial.println(F(";"));
  Serial.println(F("}"));
  Serial.println(F("=========================================="));
}

void print_report()
{
  float ss_res = 0, ss_tot = 0;
  Serial.println();
  Serial.println(F("센서값\t실제(mm)\t계산(mm)\t오차(mm)"));
  for (int i = 0; i < N_POINTS; i++) {
    float actual = 50.0 * i;
    float fitted = volt_to_distance(adc[i]);
    float err = fitted - actual;
    ss_res += err * err;
    ss_tot += (actual - 150.0) * (actual - 150.0);   // 150 = 실제 위치 평균
    Serial.print(adc[i]);      Serial.print(F("\t"));
    Serial.print(actual, 0);   Serial.print(F("\t\t"));
    Serial.print(fitted, 1);   Serial.print(F("\t\t"));
    Serial.println(err, 1);
  }
  Serial.print(F("R^2 = "));
  Serial.println(1.0 - ss_res / ss_tot, 5);
  if (degree > 3)
    Serial.println(F("주의: 차수가 높으면 측정점 7개에만 과하게 맞을 수 있습니다 (overfitting)."));
}

// ---------------------------------------------------------------
// 보정
// ---------------------------------------------------------------
void calibrate()
{
  char buf[16];
  while (true) {
    Serial.println();
    Serial.println(F("======================"));
    Serial.println(F("방식 1: 센서 값에 대한 다항식"));
    Serial.println(F("방식 2: (100 / 센서 값)에 대한 다항식"));
    Serial.println(F("방식:"));
    method = read_setting(1, 2, DEFAULT_METHOD);
    Serial.println(F("차수:"));
    degree = read_setting(1, MAX_DEGREE, DEFAULT_DEGREE);
    Serial.println(F("위치마다 반복 측정 횟수 n:"));
    n_repeat = read_setting(1, 99, DEFAULT_N);

    Serial.println();
    for (int i = 0; i < N_POINTS; i++) {
      Serial.print(F("탁구공을 ")); Serial.print(i * 5);
      Serial.print(F("cm에 놓고 Enter > "));
      read_line(buf, sizeof(buf));
      adc[i] = ir_sensor_filtered(n_repeat, 0.5);
      Serial.print(F("센서 값 ")); Serial.println(adc[i]);
      if (adc[i] == 0) {
        Serial.println(F("  센서 값이 0입니다. 배선을 확인하고 다시 Enter를 누르세요."));
        i--;
      }
    }

    if (fit_curve()) {
      print_equation();
      print_report();
      return;
    }
    Serial.println(F("곡선맞춤 실패: 측정값이 모두 같습니다. 센서와 위치를 확인하고 다시 시작합니다."));
  }
}

// ---------------------------------------------------------------
void setup()
{
  Serial.begin(1000000);
  calibrate();
}

void loop()
{
  char buf[16];
  Serial.println();
  Serial.print(F("Enter: 거리 측정 / R + Enter: 다시 보정 > "));
  read_line(buf, sizeof(buf));

  if (buf[0] == 'r' || buf[0] == 'R') {
    Serial.println(F("R"));
    calibrate();
    return;
  }

  unsigned int v = ir_sensor_filtered(n_repeat, 0.5);
  Serial.print(F("센서 값 ")); Serial.print(v);
  Serial.print(F(" => 거리 ")); Serial.print(volt_to_distance(v), 1);
  Serial.println(F(" mm"));
}
