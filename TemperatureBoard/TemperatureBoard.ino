int MUXA_PIN = 51;
int MUXB_PIN = 52;
int MUXC_PIN = 53;

float MAX_RESISTANCE = 1455.4; // from NTCS0805E3103FLT datasheet corresponding to 85 C

void setup() {
  pinMode(MUXA_PIN, OUTPUT);
  pinMode(MUXB_PIN, OUTPUT);
  pinMode(MUXC_PIN, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
  Serial.begin(9600);
  Serial.println("Hello, world!");
}

void loop() {
  // loop through binary 0-7 for mux selectors
  // and check for out of bounds temperatures
  for (int bit1 = 0; bit1 < 2; bit1++) {
    for (int bit2 = 0; bit2 < 2; bit2++) {
      for (int bit3 = 0; bit3 < 2; bit3++) {
        // write to selectors
        if (bit1 == 1) {
          digitalWrite(MUXA_PIN, HIGH);
        }
        else {
          digitalWrite(MUXA_PIN, LOW);
        }
        if (bit2 == 1) {
          digitalWrite(MUXB_PIN, HIGH);
        }
        else {
          digitalWrite(MUXB_PIN, LOW);
        }
        if (bit3 == 1) {
          digitalWrite(MUXC_PIN, HIGH);  
        }
        else {
          digitalWrite(MUXC_PIN, LOW);
        }
        
        for (int n = 0; n < 4; n++) {
          int analogSignal = analogRead(n);
          float voltage = analogSignal * (3.3 / 1023.0);
          float current = voltage / 10000.0;
          float resistance = (3.3 - voltage) / current;
          if (resistance > MAX_RESISTANCE) {
            Serial.println("Exceeded Max Temp");
          }
        }
        
      }
    }
  }
}
