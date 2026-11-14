
#define PIN_IR A0

#define REPEAT 20   // 수행시간 평균을 낼 반복 횟수

// ---------------------------------------------------------------
// 남은 RAM 측정
// ---------------------------------------------------------------
extern int __heap_start, *__brkval;
int free_ram() {
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

int base_free;   // 함수 호출 직전에 남은 RAM
int peak_used;   // 함수 안에서 버퍼를 잡은 시점에 추가로 쓴 RAM

// ---------------------------------------------------------------
// 측정값 공급원
//   use_fake == false : IR 센서 (analogRead)
//   use_fake == true  : 재현 가능한 가짜 데이터 (randomSeed로 순서 고정)
// ---------------------------------------------------------------
bool use_fake = false;

unsigned int read_sample() {
  if (!use_fake)
    return analogRead(PIN_IR);
  unsigned int v = 300 + random(0, 20);   // 평소 값: 300~319
  if (random(0, 10) == 0)                 // 10% 확률로 스파이크
    v = random(0, 1024);
  return v;
}

// ---------------------------------------------------------------
// 원본 방식: n개를 전부 저장 → qsort → n*position 번째 값
// 메모리: n개 (2n byte)
// ---------------------------------------------------------------
int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int filter_orig(unsigned int n, float position)
{
  if ((n == 0) || (n > 1000) || (position < 0.0) || (position > 1))
    return 0;
  if (position == 1.0)
    position = 0.999;

  unsigned int *ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;

  for (unsigned int i = 0; i < n; i++)
    ir_val[i] = read_sample();

  peak_used = base_free - free_ram();

  qsort(ir_val, n, sizeof(unsigned int), compare);
  unsigned int ret_val = ir_val[(unsigned int)(n * position)];

  free(ir_val);
  return ret_val;
}

// ---------------------------------------------------------------
// 메모리 절감 방식
//   오름차순 k번째 값(k = n*position)만 필요하므로
//   - k가 앞쪽이면 "지금까지 가장 작은 k+1개"만 정렬된 상태로 유지
//   - k가 뒤쪽이면 "지금까지 가장 큰 n-k개"만 유지
//   측정할 때마다 바로 삽입하므로 n개를 다 모을 필요가 없다.
// 메모리: min(k+1, n-k)개 → 최대 약 n/2개
// ---------------------------------------------------------------
unsigned int filter_lowmem(unsigned int n, float position)
{
  if ((n == 0) || (n > 1000) || (position < 0.0) || (position > 1))
    return 0;
  if (position == 1.0)
    position = 0.999;

  unsigned int k = (unsigned int)(n * position);   // 오름차순 기준 목표 순위(0부터)
  bool keep_small = (k + 1 <= n - k);              // 작은 쪽을 남기는 게 더 적은가?
  unsigned int m = keep_small ? (k + 1) : (n - k); // 버퍼 크기

  unsigned int *buf = (unsigned int *)malloc(sizeof(unsigned int) * m);
  if (buf == NULL)
    return 0;

  unsigned int cnt = 0;   // 버퍼에 들어 있는 개수 (항상 오름차순 유지)

  for (unsigned int i = 0; i < n; i++) {
    unsigned int v = read_sample();
    // 큰 쪽을 남길 때는 값을 뒤집어(1023 - v) "작은 쪽 남기기"와 같은 코드로 처리
    if (!keep_small)
      v = 1023 - v;

    if (cnt == m) {
      if (v >= buf[m - 1])   // 이미 남긴 값들보다 크면 버림
        continue;
      cnt--;                 // 가장 큰 값을 밀어내고 그 자리에 삽입
    }

    // 삽입 정렬 한 단계: 뒤에서부터 v보다 큰 값을 한 칸씩 민다
    int j = (int)cnt - 1;
    while (j >= 0 && buf[j] > v) {
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = v;
    cnt++;
  }

  peak_used = base_free - free_ram();

  unsigned int ret_val = buf[m - 1];   // 남긴 m개 중 가장 큰 값 = 전체의 k번째
  if (!keep_small)
    ret_val = 1023 - ret_val;

  free(buf);
  return ret_val;
}

// ---------------------------------------------------------------
// 1) 자체 검증: 같은 가짜 데이터에 대해 두 함수의 결과가 같은지
// ---------------------------------------------------------------
void self_test()
{
  const unsigned int ns[] = {1, 2, 3, 5, 7, 10, 30, 99};
  const float poss[] = {0.0, 0.25, 0.5, 0.75, 1.0};
  unsigned int cases = 0, mismatches = 0;

  use_fake = true;
  for (unsigned int t = 0; t < 20; t++) {
    for (unsigned int a = 0; a < sizeof(ns) / sizeof(ns[0]); a++) {
      for (unsigned int b = 0; b < sizeof(poss) / sizeof(poss[0]); b++) {
        unsigned long seed = 1000UL * t + 10 * a + b + 1;   // +1: randomSeed(0)은 무시됨
        randomSeed(seed);
        unsigned int r1 = filter_orig(ns[a], poss[b]);
        randomSeed(seed);
        unsigned int r2 = filter_lowmem(ns[a], poss[b]);
        cases++;
        if (r1 != r2) {
          mismatches++;
          Serial.print("MISMATCH n="); Serial.print(ns[a]);
          Serial.print(" pos=");       Serial.print(poss[b]);
          Serial.print(" orig=");      Serial.print(r1);
          Serial.print(" lowmem=");    Serial.println(r2);
        }
      }
    }
  }
  use_fake = false;

  Serial.print("self-test: ");
  Serial.print(cases);
  Serial.print(" cases, ");
  Serial.print(mismatches);
  Serial.println(mismatches == 0 ? " mismatches -> PASS" : " mismatches -> FAIL");
}

// ---------------------------------------------------------------
// 2) 비교표: n별 평균 수행시간(us)과 추가 RAM(byte)
// ---------------------------------------------------------------
void benchmark(bool fake)
{
  const unsigned int ns[] = {5, 10, 30, 50, 99};
  use_fake = fake;

  Serial.println(fake ? "== fake data (sort cost only differs) =="
                      : "== IR sensor (analogRead included) ==");
  Serial.println("n,orig_us,lowmem_us,orig_bytes,lowmem_bytes,orig_val,lowmem_val");

  for (unsigned int a = 0; a < sizeof(ns) / sizeof(ns[0]); a++) {
    unsigned int n = ns[a];
    unsigned long t_orig = 0, t_low = 0;
    int m_orig = 0, m_low = 0;
    unsigned int v_orig = 0, v_low = 0;

    for (unsigned int r = 0; r < REPEAT; r++) {
      if (fake) randomSeed(r + 1);   // +1: randomSeed(0)은 무시됨
      base_free = free_ram();
      unsigned long t0 = micros();
      v_orig = filter_orig(n, 0.5);
      t_orig += micros() - t0;
      m_orig = peak_used;

      if (fake) randomSeed(r + 1);   // +1: randomSeed(0)은 무시됨
      base_free = free_ram();
      t0 = micros();
      v_low = filter_lowmem(n, 0.5);
      t_low += micros() - t0;
      m_low = peak_used;
    }

    Serial.print(n);                 Serial.print(",");
    Serial.print(t_orig / REPEAT);   Serial.print(",");
    Serial.print(t_low / REPEAT);    Serial.print(",");
    Serial.print(m_orig);            Serial.print(",");
    Serial.print(m_low);             Serial.print(",");
    Serial.print(v_orig);            Serial.print(",");
    Serial.println(v_low);
  }
  use_fake = false;
}

void setup()
{
  Serial.begin(1000000);
  while (!Serial) ;

  Serial.println("19C08 ir_sensor_filtered: orig vs lowmem");
  self_test();
  Serial.println("Press <Enter> to run benchmark.");
}

void loop()
{
  if (Serial.available() == 0)
    return;
  while (Serial.available() > 0)
    Serial.read();   // 엔터 입력 비우기

  benchmark(false);  // 실제 센서
  benchmark(true);   // 가짜 데이터
  Serial.println("Press <Enter> to run again.");
}



/*
들어온 값, 버퍼
58	[58]
79	[58, 79]
59	[58, 59, 79]
59	[58, 59, 59] ← 79 밀려남
58	[58, 58, 59] ← 59 밀려남 → 답 59
*/
