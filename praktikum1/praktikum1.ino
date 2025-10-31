const int buttonPin = 3;
const int ledPin = LED_BUILTIN;


void setup() {
  Serial.begin(9600);

  while (!Serial) {
    delay(1);
  }  

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  bool btnState = digitalRead(buttonPin);
  digitalWrite(ledPin, btnState);
  Serial.println(btnState);
}
