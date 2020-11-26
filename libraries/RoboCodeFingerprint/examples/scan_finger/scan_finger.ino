#include <RoboCodeFingerprint.h>

RoboCodeFingerprint finger = RoboCodeFingerprint(2, 3);

void setup() {
  Serial.begin(9600);
  finger.begin(57600);

  finger.getTemplateCount();
  Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  Serial.println("Waiting for valid finger...");
}

void loop() {
  int id = finger.getFingerID();
  if (id >= 0) {
    if (id == 0) {
      Serial.println("No such Fingerprint");
    } else {
      Serial.print("I have found this fingerprint with id: "); Serial.println(id);
    }
  }
  delay(50);
}
