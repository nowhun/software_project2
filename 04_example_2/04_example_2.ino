unsigned int count, toggle;
void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial. begin(115200); // Initialize serial port
  while (!Serial) {
    ;
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(LED_BUILTIN, toggle); // turn off LED.
}
void loop() {
  Serial.println(++count);
  toggle = toggle_state(toggle); //toggle LED value.
  digitalWrite(LED_BUILTIN, toggle); // update LED status. 
  delay(1000); // wait for 1,000 milliseconds
}

int toggle_state(int toggle) {
  return !toggle;
}
