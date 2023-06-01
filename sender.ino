int conteur = 0;
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  String hello = "HelloWorld " + String(conteur);
  conteur += 1; 
  Serial1.println(hello);
  Serial.println("I send: " + hello);
  delay(1000);
}
