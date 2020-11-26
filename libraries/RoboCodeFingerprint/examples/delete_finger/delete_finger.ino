#include <RoboCodeFingerprint.h>

RoboCodeFingerprint finger = RoboCodeFingerprint(2, 3);

void setup() {
  Serial.begin(9600);
  Serial.println("Delete Finger");
  
  finger.begin(57600);

  Serial.println("Please type in the ID # (from 1 to 127) you want to delete...");
}

void loop() {
  if (Serial.available() > 0) {
    int id = Serial.parseInt();
    Serial.println(id);
    finger.deleteFinger(id);
    Serial.println("Please type in the ID # (from 1 to 127) that you want to delete...");
  }
}
