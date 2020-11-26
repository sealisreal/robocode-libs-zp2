#include <RoboCodeFingerprint.h>

RoboCodeFingerprint finger = RoboCodeFingerprint(2, 3);

void setup() {
  Serial.begin(9600);
  finger.begin(57600);
}

void loop() {
  Serial.println("Ready to write a new fingerprint!");
  Serial.println("Please type in the ID # (from 1 to 127) you want to save this finger as...");
  int id = readNumber();
  Serial.print("Enrolling ID #");
  Serial.println(id);
  finger.writeNewFinger(id);
}

int readNumber() {
  int num = 0;
  while (num == 0) {
    while (!Serial.available());
    num = Serial.parseInt();
  }
  return num;
}
