#define BUILTIN_LED 2

void setup() {
  Serial.begin(115200);
  pinMode(BUILTIN_LED, OUTPUT);
  Serial.println("ready");
}

void loop() {
  Serial.println(hallRead());
  digitalWrite(BUILTIN_LED, HIGH);
  delay(1000);
  digitalWrite(BUILTIN_LED, LOW);
  delay(1000); 
}
