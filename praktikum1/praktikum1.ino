const int buttonPin = 3;
const int ledPin = LED_BUILTIN;


void setup() {
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, digitalRead(buttonPin));
}
