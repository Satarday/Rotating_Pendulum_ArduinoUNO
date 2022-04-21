#include <GyverStepper.h>
#include <EncButton.h>                                            
int encVal = 0;                                                  // показания энкодера
const float stp = encVal;                                        // координата требуемой точки баланса
const float kp = 72;                                             // коэффициент пропорционального компонента
const float ki = 36;                                             // коэффициент интегрального компонента
const float kd = 3.6;                                            // коэффициент дифференциального компонента 
const float td = 10;                                             // задержка работы цикла
unsigned long tmr = 0;                                           // таймер для работы регулятора
unsigned long tmr_s = 0;                                         // таймер для вывода в порт
float angFinal = 0;                                              // угол, на который должен повернуться мотор                                            
EncButton<EB_TICK, 2, 3> enc;                                    // подключение энкодера без кнопки
GStepper<STEPPER2WIRE> stepper(6400, 4, 7, 8);                   // подключение драйвера шагового двигателя (res, step, dir, en)



void setup() {
  Serial.begin(115200)
  stepper.setRunMode(FOLLOW_POS);                                // установк режима следования к заданной позиции для драйвера мотора
  stepper.setMaxSpeedDeg(360);                                   // установка максимальной скорости мотора
  stepper.setAccelerationDeg(720);                               // установка ускорения мотора
  stepper.autoPower(true);                                       // установка автоматического отключения мотора по достижению цели
  attachInterrupt(0, isr, CHANGE);  // D2                        // подключение прерывания к функции обработки сигнала энкодера
  attachInterrupt(1, isr, CHANGE);  // D3                        // подключение прерывания к функции обработки сигнала энкодера
  tmr = micros();                                                // установка таймера по micros()
}                                                               

                                                                 ///все прерывания подключены к одной функции    
                                                                 
void loop() {                                                    // основной цикл 
  enc.tick();                                                    // опрос энкодера
  stepper.tick();                                                // опрос мотора
  if (micros() - tmr > td){                                                                                 // проверка таймера 
    tmr = micros();                                                                                         // перезапись таймера                                                                   
    if (enc.turn()) {                                                                                       // проверка события "поворот энкодера"
      encVal = constrain(enc.counter,-2000,2000);                                                           // обработка данных с энкодера (приведение данных к одному обороту) 
      int ang = map(encVal,-2000,2000,-360,360);                                                            // перевод пулученных значений в градусы
      angFinal = constrain(computePID(-ang, stp, kp, ki, kd, td, -90.00, 90.00),-90, 90);                   // вызов функции-регулятора
      stepper.setTargetDeg(angFinal, ABSOLUTE);                                                             // передача полученного от регулятора значения драйверу мотора 
    }
  }
  if (micros() - tmr_s > 100){                                                                              // проверка таймера 
    tmr_s = micros();                                                                                       // перезапись таймера
    if (enc.turn()) {                                                                                       // проверка события "поворот энкодера"
      Serial.println(encVal);                                                                               // вывод показаний энкодера в монитор порта 
    }
  }
}




void isr() {                                                                                                // функция обработчик прерываний
  enc.tickISR();                                                                                            // в прерывании вызывается тик ISR, происходит опрос энкодера
}


int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {     // функция-регулятор 
  float err = setpoint - input;                                                                                   // рассчёт ошибки (пропорциональной составляющей)
  static float integral = 0, prevErr = 0;                                                                         
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);                                          // рассчёт интегральной составляющей и приведение её к требуемому диапазону
  float D = (err - prevErr) / dt;                                                                                 // рассчёт дифференциальной составляющей
  prevErr = err;                                                                                                  // перезапись показаний ошибки 
  return constrain(err * kp + integral + D * kd, minOut, maxOut);                                                 // рассчёт и возврат значения угла поворота мотора в требуемом диапазоне
}
