#include <RoboCodeFingerprint.h>

RoboCodeFingerprint finger = RoboCodeFingerprint(2, 3);

void setup() {
  Serial.begin(9600);
  Serial.println("Fingerprint id database");  
  finger.begin(57600);

  Serial.println("ID: ");
  for (int id = 0; id < 127; id++) {
    if (finger.searchFinger(id)) { 
      Serial.print((String)id + "; ");                                  
    }
  } 
}

void loop() {

}
