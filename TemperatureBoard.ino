const int MUXA_PIN = 51; //lsb
const int MUXB_PIN = 52;
const int MUXC_PIN = 53; //msb

float MIN_RESISTANCE = 1455.4; // from NTCS0805E3103FLT datasheet corresponding to 85 C

void setup() {
  pinMode(MUXA_PIN, OUTPUT);
  pinMode(MUXB_PIN, OUTPUT);
  pinMode(MUXC_PIN, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
  Serial.begin(9600);
  Serial.println("Connected");
}

float resistances[8] = {};
void loop() {
  // loop through binary 0-7 for mux selectors
  // and check for out of bounds temperatures
  for (int bit1 = 0; bit1 < 2; bit1++) {
    for (int bit2 = 0; bit2 < 2; bit2++) {
      for (int bit3 = 0; bit3 < 2; bit3++) {
        // write to selectors
        int decimal = 0;
        if (bit1 == 1) {
          digitalWrite(MUXC_PIN, HIGH);
          decimal = decimal + 4;
        }
        else {
          digitalWrite(MUXC_PIN, LOW);
        }
        
        if (bit2 == 1) {
          digitalWrite(MUXB_PIN, HIGH);
          decimal = decimal + 2;
        }
        else {
          digitalWrite(MUXB_PIN, LOW);
        }
        
        if (bit3 == 1) {
          digitalWrite(MUXA_PIN, HIGH);
          decimal = decimal + 1;  
        }
        else {
          digitalWrite(MUXA_PIN, LOW);
        }
        
        float prev_resistance = resistances[decimal];
        //for (int n = 0; n < 4; n++) {
        int n = 2;
          int analogSignal = analogRead(n);
          float voltage = analogSignal * (3.3 / 1023.0);
          float current = voltage / 10000.0;
          float resistance = (3.3 - voltage) / current;
          float diff = resistance - prev_resistance;
          resistances[decimal] = resistance;
            Serial.print("Mux in: ");
            //Serial.print(bit1);
            //Serial.print(bit2);
            //Serial.print(bit3);
            Serial.print(decimal);
            Serial.print(" Analog read: ");
            Serial.println(analogSignal); 
            //Serial.print(" Resistance: ");
            //Serial.print(resistance);
            //Serial.print(" Delta R: ");
          //Serial.println(diff);
          delay(200);
          
//          if (resistance < MIN_RESISTANCE) {
//            Serial.print("Voltage: ");
//            Serial.print(voltage);
//            Serial.print(" Mux in");
//            Serial.print(bit1);
//            Serial.print(bit2);
//            Serial.print(bit3);
//            Serial.print(" Analog pin:");
//            Serial.println(n);
//            
//          }
        //}
        
      }
    }
  }
}
