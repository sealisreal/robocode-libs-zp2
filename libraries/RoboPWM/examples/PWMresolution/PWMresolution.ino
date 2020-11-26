#include <RoboPWM.h>

void setup() {
  pinMode(9, OUTPUT);

  // запустить ШИМ на D9, разрядность 12 бит (0-4095), частота 3.9 кГц (см. таблицу №1 в RoboPWM.h), режим FAST_PWM
  resolPWM(9, 12, FAST_PWM);
}

void loop() {
  setPWM(9, 2842);   // установить заполнение ШИМ на D9, равное 2842 из 4095, частота 3.9 кГц (см. таблицу №1 в RoboPWM.h)
}
