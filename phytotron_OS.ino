/////////////////////////////////////////////////////
//                   БИБЛИОТЕКИ                    //
/////////////////////////////////////////////////////

//#include <Arduino.h>                // Библиотека "Arduino" (встроенная библиотека)
//#include <Wire.h>                   // Библиотека "Wire" (встроенная библиотека)
//#include <SPI.h>                    // Библиотека "SPI" (встроенная библиотека)
#include <SD.h>                     // Библиотека урезанная SD-картридера "SD" (встроенная библиотека)
#include <CD74HC4067.h>             // Библиотека мультиплексера "CD74HC4067" / waspinator (https://github.com/waspinator/CD74HC4067)
#include <microDS3231.h>            // Библиотека часов "microDS3231" / GyverLibs (https://github.com/GyverLibs/microDS3231)
#include <LiquidCrystal_I2C.h>      // Библиотека дисплея по I2C "LiquidCrystal_I2C" / Frank de Brabander (https://github.com/johnrickman/LiquidCrystal_I2C)
#include <PCF8574.h>                // Библиотека расширителя PCF8574 "PCF8574" / Rob Tillaart (https://github.com/RobTillaart/PCF8574)
#include <GyverDS18Array.h>         // Библиотека датчиков температуры DS18 "GyverDS18" / AlexGyver (https://github.com/GyverLibs/GyverDS18)
#include <DHT.h>                    // Библиотека датчика температуры и влажности DHT22 "DHT sensor library" / Adafruit (https://github.com/adafruit/DHT-sensor-library)





/////////////////////////////////////////////////////
//               ИНИЦИАЛИЗАЦИЯ ПИНОВ               //
/////////////////////////////////////////////////////

// Мультиплесор - управление
#define pin_mux A0   // Пин получения сигнала с Мультиплексора
#define pin_S0 2     // Пин S0 управления Мультиплексора
#define pin_S1 4     // Пин S1 управления Мультиплексора
#define pin_S2 7     // Пин S2 управления Мультиплексора
#define pin_S3 8     // Пин S3 управления Мультиплексора

// Мультиплесор - выхода
#define pin_T10 0    // Пин мультиплексора для термистора NTC T10
#define pin_T11 1    // Пин мультиплексора для термистора NTC T11
#define pin_T12 2    // Пин мультиплексора для термистора NTC T12
#define pin_T13 3    // Пин мультиплексора для термистора NTC T13
#define pin_T14 4    // Пин мультиплексора для термистора NTC T14
#define pin_T20 5    // Пин мультиплексора для термистора NTC T20
#define pin_T21 6    // Пин мультиплексора для термистора NTC T21
#define pin_T22 7    // Пин мультиплексора для термистора NTC T22
#define pin_T23 8    // Пин мультиплексора для термистора NTC T23
#define pin_T24 9    // Пин мультиплексора для термистора NTC T24
//#define mux10 10   // Пин мультиплексора 10
#define pin_SH1 11   // Пин мультиплексора для датчика влажности почвы #1
#define pin_SH2 12   // Пин мультиплексора для датчика влажности почвы #2
#define pin_SH3 13   // Пин мультиплексора для датчика влажности почвы #3
#define pin_SH4 14   // Пин мультиплексора для датчика влажности почвы #4
#define pin_SL1 15   // Пин мультиплексора для датчика уровня воды в баке

// ШИМ-выходы на светодиодные ленты
#define pin_led_W 5  // Пин управления белым освещением
#define pin_led_R 3  // Пин управления красным освещением
#define pin_led_G 6  // Пин управления зелёным освещением
#define pin_led_B 9  // Пин управления синим освещением

// SD-картридер
#define pin_CS 10    // Пин общения с SD картой

// Датчик DHT22
#define pin_DHT1 A1   // Пин общения с внутренним датчиком DHT22
#define pin_DHT2 A3   // Пин общения с внешним датчиком DHT22

// Датчик DS18
#define pin_DS18 A2   // Пин общения с датчиком температуры DS18

// PCF1
//#define P0 0       // Пин PCF1 P0 - управление H1 (Увлажнитель / Humider)
#define pin_F1 1     // Пин PCF1 P1 - управление F1 (Вентилятор прямой / Fan direct)
#define pin_P1 2     // Пин PCF1 P2 - управление P1 (Клапан 1 / Pump 1)
#define pin_P2 3     // Пин PCF1 P3 - управление P2 (Клапан 2 / Pump 2)
#define pin_P3 4     // Пин PCF1 P4 - управление P3 (Клапан 3 / Pump 3)
#define pin_P4 5     // Пин PCF1 P5 - управление P4 (Клапан 4 / Pump 4)
#define pin_RT1 6    // Пин PCF1 P6 - управление RT1 (Нагреватель / Heater)
#define pin_F2 7     // Пин PCF1 P7 - управление F2 (Вентилятор обратный / Fan reverse)

// PCF2
#define pin_SB1_Right 0    // Пин PCF2 P0 - кнопка SB1 (Кнопка Вправо)
#define pin_SB2_Down 1     // Пин PCF2 P1 - кнопка SB2 (Кнопка Вниз)
#define pin_SB3_Left 2     // Пин PCF2 P2 - кнопка SB3 (Кнопка Влево)
#define pin_SB4_Up 3       // Пин PCF2 P3 - кнопка SB4 (Кнопка Вверх)
#define pin_HL1 4          // Пин PCF2 P4 - управление HL1 (Светодиод программы / Program LED)
//#define P5 5             // Пин PCF2 P5 - Пустой
//#define P6 6             // Пин PCF2 P6 - Пустой
//#define P7 7             // Пин PCF2 P7 - Пустой




/////////////////////////////////////////////////////
//              ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ              //
/////////////////////////////////////////////////////


//////////////////
//  I2C адреса  //
//////////////////
/*
I2C Адреса для PCF8574 (ver. T/AT/MH)
№  Адрес      A0  A1  A2
1. 0x20     | 0 | 0 | 0 |
2. 0x21     | 0 | 0 | 1 |
3. 0x22     | 0 | 1 | 0 |
4. 0x23     | 0 | 1 | 1 |
5. 0x24     | 1 | 0 | 0 |
6. 0x25     | 1 | 0 | 1 |
7. 0x26     | 1 | 1 | 0 |
8. 0x27     | 1 | 1 | 1 |

I2C Адреса для PCF8574 (ver. IIC)
№  Адрес      A0  A1  A2
1. 0x20     | 0 | 0 | 0 |
2. 0x21     | 1 | 0 | 0 |
3. 0x22     | 0 | 1 | 0 |
4. 0x23     | 1 | 1 | 0 |
5. 0x24     | 0 | 0 | 1 |
6. 0x25     | 1 | 0 | 1 |
7. 0x26     | 0 | 1 | 1 |
8. 0x27     | 1 | 1 | 1 |
*/

#define pcf1_I2C_adress 0x24          // Определение I2C адреса регистра сдвига №1
#define pcf2_I2C_adress 0x22          // Определение I2C адреса регистра сдвига №2
#define rtc_I2C_adress 0x68           // Определение I2C адреса часов реального времени
#define lcd1_I2C_adress 0x27          // Определение I2C адреса дисплея



//////////////////
//   RTC-часы   //
//////////////////

#define rtc_second_shift 14           // Компенсация задержки загрузки программы для синхронизации часов [сек] | Системный
#define rtc_region 4                  // Часовой пояс для коррекции часов: 4 - Ижевск; 5 - Пермь | Настраиваемый



//////////////////
//     Mux      //
//////////////////

#define mux_delay_def 5               // Стандартная задержка переключения мультиплексора [мсек] | Системный



//////////////////
//   SD карта   //
//////////////////

String log_path;                      // Путь файла для записи показаний | Системный
#define sd_delay 25                   // Задержка для инициализации SD карты [мсек] | Системный
#define strings_folder "/str/"        // Путь к файлам с неотсортированными строками | Системный
#define headers_folder "/head/"       // Путь к файлам со строками заголовков записываемых параметров | Системный
#define menu_folder "/menu/"          // Путь к файлам со строками экранов меню | Системный

//////////////////
// Эксперимент  //
//////////////////

// Время эксперимента
uint32_t time_exp_start = 0;          // Время начала работы системы (unix) [сек] | Системный
uint32_t time_exp_count = 0;          // Переменная подсчёта длительности эксперимента (unix) [сек] | Системный
uint32_t experiment_long = 90;        // Продолжительность эксперимента [мин] | Настраиваемый

// Тик-Таймер
unsigned long timer_tick = 0;         // Таймер для отсчтёта минимального временного периода для чтения [мсек] | Системный
#define timer_tick_period 1           // Минимальный временной период [сек] | Настраиваемый
uint16_t log_tick_count = 0;          // Счётчик периода записи log-данных [сек] | Системный
uint16_t log_period = 60;             // Период записи log-данных [сек] | Настраиваемый

// Таймер увлажения
unsigned long timer_pump = 0;         // Таймер дозированного полива [мсек] | Системный
#define timer_pump_period 1500        // Период дозированного полива [мсек] | Настраиваемый
uint32_t timer_humMeas;               // Таймер для отсчёта периода измерения влажности почвы (unix) [сек] | Системный
#define timer_humMeas_period 1800     // Период измерения влажности почвы (unix) [сек] | Настраиваемый

// Влажность почвы
bool water_lvl = false;               // Переменная наличия воды в баке | Системный
uint8_t pot_num = 0;                  // Текущий активный горшок | Системный
bool humMeas_switch = true;           // Готовность измерять влажность почвы | Системный
uint16_t hum_curr = 0;                // Измеренная влажность почвы | Системный
uint16_t hum_set_1 = 0;               // Эталонная влажность для горшка №1 | Системный
uint16_t hum_set_2 = 0;               // Эталонная влажность для горшка №2 | Системный
uint16_t hum_set_3 = 0;               // Эталонная влажность для горшка №3 | Системный
uint16_t hum_set_4 = 0;               // Эталонная влажность для горшка №4 | Системный
uint16_t hum_pot_1 = 0;               // Измеренная влажность в горшке №1 | Системный
uint16_t hum_pot_2 = 0;               // Измеренная влажность в горшке №2 | Системный
uint16_t hum_pot_3 = 0;               // Измеренная влажность в горшке №3 | Системный
uint16_t hum_pot_4 = 0;               // Измеренная влажность в горшке №4 | Системный
#define hum_beta 0.035                // Корректировочный коэффициент ёмкостного датчика [%/℃] | Константа
#define T_hum_ref 25.0                // Комнатная температура, при которой рассчитана работа датчика [℃] | Константа

// Регулировка температуры
float soil_temp_current_avr;          // Текущая температура у всей почвы [℃] | Системный
float air_temp_current;               // Текущая температура воздуха [℃] | Системный
float air_out_temp_current;           // Текущая температура наружнего воздуха [℃] | Системный

float temp_air_set = 22;              // Заданная температура воздуха [℃] | Настраиваемый
float temp_soil_set = 27;             // Заданная температура почвы [℃] | Настраиваемый
float temp_air_set_day = 22;          // Заданная температура воздуха в режиме День [℃] | Настраиваемый
float temp_air_set_night = 22;        // Заданная температура воздуха в режиме Ночь [℃] | Настраиваемый
float temp_soil_set_day = 27;         // Заданная температура почвы в режиме День [℃] | Настраиваемый
float temp_soil_set_night = 27;       // Заданная температура почвы в режиме Ночь [℃] | Настраиваемый

// Освещение
uint8_t light_s_W = 100;              // Процентное включение цвета W в спектре цвета [%] | Настраиваемый
uint8_t light_s_R = 100;              // Процентное включение цвета R в спектре цвета [%] | Настраиваемый
uint8_t light_s_G = 100;              // Процентное включение цвета G в спектре цвета [%] | Настраиваемый
uint8_t light_s_B = 100;              // Процентное включение цвета B в спектре цвета [%] | Настраиваемый
uint8_t light_day = 100;              // Процентное включение фитолампы в режиме День [%] | Настраиваемый
uint8_t light_night = 100;            // Процентное включение фитолампы в режиме Ночь [%] | Настраиваемый
uint8_t light_set = 100;              // Процентное установленное значение включения фитолампы [%] | Настраиваемый

// Режимы работы
bool experiment_inWork = false;       // Переменная переключения состояния автоматики эксперемента | Системный
uint8_t cfgID = 0;                    // Номер cfg файла | Системный

// Режим цикла дня и ночи
uint8_t day_long = 18;                // Длительность фазы дня [час] | Настраиваемый
uint8_t night_long = 2;               // Длительность фазы ночи [час] | Настраиваемый
uint8_t day_night_lvl = 0;            // Управляющий уровень режимом дня и ночи (0 => 0%; 255 => 100%) | Системный



//////////////////
//   Датчики    //
//////////////////

// Адреса датчиков DS18
uint64_t DS18_addr[] = {
  0x602255C21E64FF28,                 // Адрес 1 - Красный - 0x602255C21E64FF28
  0x0ADD14C21E64FF28,                 // Адрес 2 - Зеленый - 0x0ADD14C21E64FF28
  0xEC246ECD1E64FF28,                 // Адрес 3 - Желтый  - 0xEC246ECD1E64FF28
  0xC0021AC21E64FF28,                 // Адрес 4 - Черный  - 0xC0021AC21E64FF28
};



//////////////////
//     ПИД      //
//////////////////

// ПИД-регулятор
unsigned long timer_PID_period = 0;   // Таймер отсчёта периода ШИМ ПИД-регулятора | Системный
#define PID_PWM_period 500            // Длительность периода ШИМ регулирования ПИД-регулятора [мсек] | Системный
#define PWM_min 50                    // Ограничение импульса ШИМ ПИД-регулятора по минимальному значению | Системный
float PID_value = 0;                  // Значение ПИД-регулятора (нужно для эксперимента, можно будет убрать) | Системный
const int PID_PWM_steps = (           // Количество ступеней ШИМ регулирования ПИД-регулятора | Системный
  round((PID_PWM_period) / 
  (float)PWM_min));
#define PID_div 100                   // Делитель коэффициентов ПИД-регулятора | Системный

// ПИД Нагревателя
#define RT1_kP 35                     // P = 0.35 коэффициент ПИД-регулятора Нагревателя | Настраиваемый
#define RT1_kI 50                     // I = 0.50 коэффициент ПИД-регулятора Нагревателя | Настраиваемый
#define RT1_kD 12000                  // D = 120 коэффициент ПИД-регулятора Нагревателя | Настраиваемый

// ПИД Вентилятора
#define F1_kP 200                     // P = 2.00 коэффициент ПИД-регулятора Вентилятора | Настраиваемый
#define F1_kI 800                     // I = 8.00 коэффициент ПИД-регулятора Вентилятора | Настраиваемый
#define F1_kD 600                     // D = 6.00 коэффициент ПИД-регулятора Вентилятора | Настраиваемый



//////////////////
//    Кнопки    //
//////////////////

// Время кнопок
unsigned long timer_btn = 0;          // Таймер отсчёта бездействия кнопок [мсек] | Системный
#define btn_delay 400                 // Задержка срабатывания кнопок [мсек] | Системный

// Флаги нажатия кнопок
bool btn_flag_Up = false;             // Переменная определяющая была ли уже нажата кнопка Вверх | Системный
bool btn_flag_Down = false;           // Переменная определяющая была ли уже нажата кнопка Вниз | Системный
bool btn_flag_Right = false;          // Переменная определяющая была ли уже нажата кнопка Вправо | Системный
bool btn_flag_Left = false;           // Переменная определяющая была ли уже нажата кнопка Влево | Системный



//////////////////
//     Меню     //
//////////////////

// Переключение экранов
uint8_t work_sreen_id = 0;            // Переменная активного экрана меню | Системный
bool menu_update = true;              // Переменная обновления экрана | Системный

// Экраны Меню
#define Menu_Start 0                  // Номер экрана "Стартовое меню" | Системный
#define Menu_cfg 1                    // Номер экрана "Меню выбора cfg" | Системный
#define Menu_cfg_err 2                // Номер экрана "Ошибка cfg-файла" | Системный
#define Menu_Log 3                    // Номер экрана "Создание log-файла" | Системный
#define Menu_Log_err 4                // Номер экрана "Ошибка log-файла" | Системный
#define Menu_Exp_Done 5               // Номер экрана "Эксперимент выполнен" | Системный
#define Menu_Exp_Stop 6               // Номер экрана "Остановка эксперимента" | Системный
#define Menu_Exp_1 7                  // Номер экрана "Эксперимент 1" | Системный
#define Menu_Exp_2 8                  // Номер экрана "Эксперимент 2" | Системный









/////////////////////////////////////////////////////
//                 ОСНОВНЫЕ ОБЪЕКТЫ                //
/////////////////////////////////////////////////////

CD74HC4067 mux(pin_S0, pin_S1, pin_S2, pin_S3);   // Создание объекта Мультиплексора и привязка контактов S0, S1, S2 и S3
MicroDS3231 rtc;                                  // Создание объекта часов c периодом синхронизации в секундах, поумолч. 1 час
LiquidCrystal_I2C lcd1(lcd1_I2C_adress, 20, 4);   // Создание объекта основного дисплея с адресом lcd1_I2C_adress и размером 20 х 4
PCF8574 pcf1(pcf1_I2C_adress);                    // Создание объекта регистра сдвига PCF1
PCF8574 pcf2(pcf2_I2C_adress);                    // Создание объекта регистра сдвига PCF2
DHT DHT1(pin_DHT1, DHT22);                        // Инициация внутреннего датчика DHT22
DHT DHT2(pin_DHT2, DHT22);                        // Инициация внешнего датчика DHT22
GyverDS18Array DS18(pin_DS18, DS18_addr, 4);      // Инициация датчиков DS18




/////////////////////////////////////////////////////
//                     КЛАССЫ                      //
///////////////////////////////////////////////////// 

// ПИД-регулятор
class DevicePID
{
  private:
    // Внутринние переменные
    float last_err;                       // Последняя ошибка для Диффирициальной составляющей
    float I = 0;                          // Интегральная составляющая
    bool I_switch = false;                // Переключатель интегральной составляющей
    unsigned long pwm_pulse_timer;        // Таймер длительности импульса ШИМ
    unsigned long pwm_period_timer;       // Таймер периода ШИМ
    
    // Вводимые переменные
    uint8_t device_id;                    // Номер пина устройства на PCF (0...255)
    bool device_pwr_sig;                  // Каким сигналом включается устройство (false, true)

  public:
    uint16_t pwm_pulse;                   // Продолжительно импульса ПИД [мсек]
    uint16_t pwm_period;                  // Продолжительно периода ШИМ (0...65535) [мсек]
    uint16_t kP, kI, kD;                  // Коэффициенты ПИД (0...65535) [%]

    // Метод вызываемый при создании объекта
    DevicePID(uint8_t _device_id,         // Номер пина устройства на PCF (0...255)
              bool _device_pwr_sig,       // Каким сигналом включается устройство (false, true)
              uint16_t _kP,               // Коэффициент P (0...65535) [%]
              uint16_t _kI,               // Коэффициент I (0...65535) [%]
              uint16_t _kD,               // Коэффициент D (0...65535) [%]
              uint16_t _pwm_period        // Длительность периода ШИМ (0...65535) [мсек]
             )
    {
      device_id = _device_id;
      device_pwr_sig = _device_pwr_sig;
      kP = _kP;
      kI = _kI;
      kD = _kD;
      pwm_period = _pwm_period;
    }
    

    // Метод обработки постоянного включения
    // Вызывается в любом месте единожды или каждый цикл
    void workON(bool devise_state)
    {
      switch (devise_state)
      {
        case true:
          pcf1.write(device_id, device_pwr_sig);
          break;
        case false:
          pcf1.write(device_id, !device_pwr_sig);
          break;
      } 
    }



    // Метод расчёта времени импульса ПИД
    // Вызывается один раз в период ПИД
    void pidPWM(float setpoint, float input)
    {
      // Вычисление ПИД
      float err;                        // Ошибка регулирования
      float P;                          // Пропорциональная составляющая
      float D;                          // Диффиринциальная составляющая
      
      // Пропорциональная составляющая
      // Если устройство работает обратно (снижает контролируемый параметр)
      if(device_id == pin_F1)           // Перечисляются через ИЛИ все устройства
      {
        err = input - setpoint;
      }
      else
      {
        err = setpoint - input;
      }
      
      P = min(1, max(err, -1));
      
      // Интегральная составляющая
      // Если ошибка регулирования больше 0
      if (err > 0)
      {
        I_switch = true;               // Включить интегральную составляющую
      }
      if (I_switch == true)
      {
        I += err * (float)pwm_period / 14400000;
      }
      I = min(1, max(I, -1));

      // Диффиринциальная составляющая
      D = (err - last_err) / ((float)pwm_period / 1000);
      D = min(1, max(D, -1));
      last_err = err;

      // Результат
      float result = (float)kP/PID_div * P + (float)kI/PID_div * I + (float)kD/PID_div * D;
      
      // Вычисление длительности импульса [мсек]
      result = min(max(result, 0), 1);
      result = max(0, round((result * (float)PID_PWM_steps)) * ((float)pwm_period / (float)PID_PWM_steps));

      // Вывод результата
      pwm_pulse = (int)result;
      return;
    }



    // Метод отработки импульса ПИД
    // Вызывается каждый цикл функции loop() при активном режиме выращивания
    void workPWM(float setpoint, float input)
    {
      // Таймер периода ПИД
      if (millis() - pwm_period_timer >= pwm_period)
      {
        pwm_period_timer = millis();
        pwm_pulse_timer = millis();
        pidPWM(setpoint, input);
      }

      // Если время импульса ещё не закончилось
      if (millis() - pwm_pulse <= pwm_pulse_timer && pwm_pulse >= PWM_min)
      {
        workON(true);
      }
      // Если время импульса закончилось
      else
      {
        workON(false);
      }
    }
};




/////////////////////////////////////////////////////
//                 ОБЪЕКТЫ КЛАССОВ                 //
/////////////////////////////////////////////////////

// Инициализация нагревателя как устройства воздействия в режиме ПИД
DevicePID Heater(pin_RT1,           // Номер пина устройства на PCF (0...255)
                 true,              // Каким сигналом включается устройство (false, true)
                 RT1_kP,            // Коэффициент P (0...65535) [%]
                 RT1_kI,            // Коэффициент I (0...65535) [%]
                 RT1_kD,            // Коэффициент D (0...65535) [%]
                 10000              // Длительность периода ШИМ (0...65535) [мсек]
                );

// Инициализация вентилятора как устройства воздействия в режиме ПИД
DevicePID Fan(pin_F1,               // Номер пина устройства на PCF (0...255)
              false,                // Каким сигналом включается устройство (false, true)
              F1_kP,                // Коэффициент P (0...65535) [%]
              F1_kI,                // Коэффициент I (0...65535) [%]
              F1_kD,                // Коэффициент D (0...65535) [%]
              PID_PWM_period        // Длительность периода ШИМ (0...65535) [мсек]
             );                






/////////////////////////////////////////////////////
//                  ОБЩИЕ ФУНКЦИИ                  //
///////////////////////////////////////////////////// 

// Обработка нажатия кнопок
bool pcf_click(int pin)
{
  if (!pcf2.readButton(pin) == HIGH && millis() - timer_btn >= btn_delay)
  {
    timer_btn = millis();
    return true;
  }
  else
  {
    return false;
  }
}

// Функция вывода "0" перед малым числом
void zeroWrite(int num)
{
  if (num < 10)
  {
    lcd1.print("0");
  }
}

// Пересчёт влажности с учётом температуры (мнимая влажность)
uint16_t imaginaryHum(uint16_t hum, float T)
{
  return (float)hum * (1 + hum_beta * (T - T_hum_ref));
}

// Фиксация эталонной влажности почвы
void humSet()
{
  float T = 25;
  DS18.requestTemp();
  delay(mux_delay_def);

  // Влажность почвы №1
  DS18.readTemp(0);
  T = DS18.getTemp();
  mux.channel(pin_SH1);
  DS18.requestTemp();
  delay(mux_delay_def);
  hum_set_1 = imaginaryHum(analogRead(pin_mux), T);
  hum_pot_1 = hum_set_1;

  // Влажность почвы №2
  DS18.readTemp(1);
  T = DS18.getTemp();
  mux.channel(pin_SH2);
  DS18.requestTemp();
  delay(mux_delay_def);
  hum_set_2 = imaginaryHum(analogRead(pin_mux), T);
  hum_pot_2 = hum_set_2;

  // Влажность почвы №3
  DS18.readTemp(2);
  T = DS18.getTemp();
  mux.channel(pin_SH3);
  DS18.requestTemp();
  delay(mux_delay_def);
  hum_set_3 = imaginaryHum(analogRead(pin_mux), T);
  hum_pot_3 = hum_set_3;
  
  // Влажность почвы №4
  DS18.readTemp(3);
  T = DS18.getTemp();
  mux.channel(pin_SH4);
  DS18.requestTemp();
  delay(mux_delay_def);
  hum_set_4 = imaginaryHum(analogRead(pin_mux), T);
  hum_pot_4 = hum_set_4;
}

// Статический класс функций SD карты
namespace sdFunc {

  // Функция чтения строки из файла SD-карты
  //String readLine(const char* filename, uint32_t lineNumber)
  String readLine(String filename, uint8_t id, uint32_t lineNumber)
  {
    filename = filename + id;
    // Отработка ошибок
    if (lineNumber == 0 || !SD.exists(filename))
    {
      return String();
    }

    File f = SD.open(filename);
    if (!f) return String();

    uint32_t current = 1;
    String line;

    while (f.available())
    {
      // читаем строку до '\n' (не включает '\n')
      line = f.readStringUntil('\n');

      // удаляем возможный '\r' в конце (Windows EOL)
      if (line.endsWith("\r")) {
        line.remove(line.length() - 1);
      }

      if (current == lineNumber) {
        f.close();
        return line;                      // найденная строка
      }
      current++;
    }

    f.close();
    return String();                      // не найдено (файл короче)
  }
  
  void logPathTime()
  {
    log_path = "";
    for (int i = 0; i < 4; i++)
    {
      uint8_t path;
      switch (i)
      {
        case 0:
          path = rtc.getMonth();
          break;
        case 1:
          path = rtc.getDate();
          break;
        case 2:
          path = rtc.getHours();
          break;
        case 3:
          path = rtc.getMinutes();
          break;
      }
      if (path < 10)
      {
        log_path += sdFunc::readLine(strings_folder, 1, 1);
      }
      log_path += path;
    }
    return;
  }

  // Функция создания log файла
  bool LogCreate()
  {
    SD.begin(pin_CS);                         // Инициализация SD карты
    delay(sd_delay);                          // Ожидание инициализации
    sdFunc::logPathTime();
    File Logfile = SD.open(log_path, FILE_WRITE);
    if (Logfile)
    {
      // Запись заголовков №1:
      /* 
        experiment_long 
        day_long 
        night_long 
        log_period 
        temp_air_set_day 
        temp_air_set_night 
        temp_soil_set_day 
        temp_soil_set_night 
        light_s_W 
        light_s_R 
        light_s_G 
        light_s_B 
        light_day 
        light_night 
        hum_set_1 
        hum_set_2 
        hum_set_3 
        hum_set_4 
      */
      for (uint8_t param_i = 1; param_i <= 18; param_i++)
      {
        Logfile.print(sdFunc::readLine(headers_folder, 1, param_i));
      }
      Logfile.println();                        // Запись на карту переноса "\n"

      // Запись значений №1
      for (uint8_t param_i = 1; param_i <= 18; param_i++)
      {
        switch (param_i)
        {
          // Запись длительности эксперимента - experiment_long
          case 1:
            Logfile.print(experiment_long);
            break;
          // Запись длительности режима Дня - day_long
          case 2:
            Logfile.print(day_long);
            break;
          // Запись длительности режима Ночи - night_long
          case 3:
            Logfile.print(night_long);
            break;
          // Запись переодичности записи log-данных - log_period
          case 4:
            Logfile.print(log_period);
            break;
          // Запись установленной температуры воздуха в режиме Дня - temp_air_set_day
          case 5:
            Logfile.print(temp_air_set_day);
            break;
          // Запись установленной температуры воздуха в режиме Ночи - temp_air_set_night
          case 6:
            Logfile.print(temp_air_set_night);
            break;
          // Запись установленной температуры почвы в режиме Дня - temp_soil_set_day
          case 7:
            Logfile.print(temp_soil_set_day);
            break;
          // Запись установленной температуры почвы в режиме Ночи - temp_soil_set_night
          case 8:
            Logfile.print(temp_soil_set_night);
            break;
          // Запись значения Белого цвета в спектре - light_s_W
          case 9:
            Logfile.print(light_s_W);
            break;
          // Запись значения Красного цвета в спектре - light_s_R
          case 10:
            Logfile.print(light_s_R);
            break;
          // Запись значения Зелёного цвета в спектре - light_s_G
          case 11:
            Logfile.print(light_s_G);
            break;
          // Запись значения Синего цвета в спектре - light_s_B
          case 12:
            Logfile.print(light_s_B);
            break;
          // Запись уровня яркости фитолампы в режиме Дня - light_day
          case 13:
            Logfile.print(light_day);
            break;
          // Запись уровня яркости фитолампы в режиме Дня - light_night
          case 14:
            Logfile.print(light_night);
            break;
          // Запись Эталанной влажности для горшка №1 - hum_set_1
          case 15:
            Logfile.print(hum_set_1);
            break;
          // Запись Эталанной влажности для горшка №2 - hum_set_2
          case 16:
            Logfile.print(hum_set_2);
            break;
          // Запись Эталанной влажности для горшка №3 - hum_set_3
          case 17:
            Logfile.print(hum_set_3);
            break;
          // Запись Эталанной влажности для горшка №4 - hum_set_4
          case 18:
            Logfile.print(hum_set_4);
            break;
        }
        Logfile.print(" ");
      }
      Logfile.println();                        // Запись на карту переноса "\n"

      // Запись заголовков №2 в формате:
      /* 
        time_exp_count 
        day_night_lvl 
        air_out_temp_current 
        air_temp_current 
        soil_temp_current_avr 
        temp_air_set 
        temp_soil_set 
        light_set 
        water_lvl 
        hum_pot_1 
        hum_pot_2 
        hum_pot_3 
        hum_pot_4 
      */

      for (uint8_t param_i = 1; param_i <= 13; param_i++)
      {
        Logfile.print(sdFunc::readLine(headers_folder, 2, param_i));
      }
      Logfile.println();                        // Запись на карту переноса "\n"
      Logfile.close();                          // Закрытие файла
      return true;
    }
    else
    {
      return false;
    }
  }

  // Функция записи данных в конец файла
  void LogData()
  {
    File logFile = SD.open(log_path, FILE_WRITE);     // Создание или открытие файла для записи
    if (logFile)                                      // Если файл открылся успешно
    {
      // Запись значений №2
      for (uint8_t param_i = 1; param_i <= 13; param_i++)
      {
        switch (param_i)
        {
          // Запись времени - time_exp_count
          case 1:
            logFile.print(time_exp_count);
            break;
          // Запись значения состояния режима Дня и Ночи - day_night_lvl
          case 2:
            logFile.print(day_night_lvl);
            break;
          // Запись внешней температуры воздуха - air_out_temp_current
          case 3:
            logFile.print(air_out_temp_current);
            break;
          // Запись внутренней температуры воздуха - air_temp_current
          case 4:
            logFile.print(air_temp_current);
            break;
          // Запись средней температуры почвы - soil_temp_current_avr
          case 5:
            logFile.print(soil_temp_current_avr);
            break;
          // Запись заданной температуры воздуха - temp_air_set
          case 6:
            logFile.print(temp_air_set);
            break;
          // Запись заданной температуры почвы - temp_soil_set
          case 7:
            logFile.print(temp_soil_set);
            break;
          // Запись заданной яркости фитолампы - light_set
          case 8:
            logFile.print(light_set);
            break;
          // Запись значения наличия воды в баке - water_lvl
          case 9:
            logFile.print(water_lvl);
            break;
          // Запись текущей влажности SH1
          case 10:
            logFile.print(hum_pot_1);
            break;
          // Запись текущей влажности SH2
          case 11:
            logFile.print(hum_pot_2);
            break;
          // Запись текущей влажности SH3
          case 12:
            logFile.print(hum_pot_3);
            break;
          // Запись текущей влажности SH4
          case 13:
            logFile.print(hum_pot_4);
            break;
        }
        logFile.print(" ");
      }
      // Завершение записи
      logFile.println();                        // Запись на карту переноса "\n"
      logFile.close();                          // Закрытие файла
    }
  }

  // Функция записи данных из cfg файлов в переменные
  bool cfgRead(String cfg_path)
  {
    SD.begin(pin_CS);                         // Инициализация SD карты
    delay(sd_delay);                          // Ожидание инициализации
    File cfgFile;                             // Инициализация файла
    cfgFile = SD.open(cfg_path);              // Открытие файла cfg
    
    // Если файл не открывается
    if (!cfgFile)
    {
      return false;
    }

    uint8_t param_i = 0;                // Переменная перечисления параметров из cfg
    const uint8_t param_i_last = 13;    // Переменная номера последнего параметра из cfg
    uint8_t write_state = 0;            // Переменная состояния записи:
                                        // 0 - Запись не началась
                                        // 1 - Запись началась (записал первое значение)
                                        // 2 - Запись закончилось (записал последнее занчение)
    String line;                        // Переменная записи строки данных из cfg

    // Цикл пока имеются данные в файле И не заполнены все индексы
    while (cfgFile.available() && param_i <= param_i_last)
    {
      line = cfgFile.readStringUntil('\n');           // Прочитать строку до переноса
      line.trim();                                    // Удаление системных символов

      // Если строка пустая или начинается с комментария "#"
      if (line.length() == 0 || line.startsWith("#"))
      {}
      else
      {
        // Встатвка значений по индексу
        switch (param_i)
        {
          case 0: experiment_long = line.toInt(); write_state = 1; break;
          case 1: day_long = line.toInt(); break;
          case 2: night_long = line.toInt(); break;
          case 3: log_period = line.toInt(); break;
          case 4: temp_air_set_day = atof(line.c_str()); break;
          case 5: temp_air_set_night = atof(line.c_str()); break;
          case 6: temp_soil_set_day = atof(line.c_str()); break;
          case 7: temp_soil_set_night = atof(line.c_str()); break;
          case 8: light_s_W = line.toInt(); break;
          case 9: light_s_R = line.toInt(); break;
          case 10: light_s_G = line.toInt(); break;
          case 11: light_s_B = line.toInt(); break;
          case 12: light_day = line.toInt(); break;
          case param_i_last: light_night = line.toInt(); write_state = 2; break;
        }
        param_i++; 
      }
    }
    cfgFile.close();
    if (write_state == 2)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}







/////////////////////////////////////////////////////
//                  ФУНКЦИИ ЦИКЛА                  //
///////////////////////////////////////////////////// 




///////////////////////////////
//     Обработчик кнопок     //
///////////////////////////////
namespace btnFunc {
  
  // Функция сброса флаго кнопок (предотвращение двойного нажатия)
  void flagDrop()
  {
    btn_flag_Up = false;
    btn_flag_Down = false;
    btn_flag_Right = false;
    btn_flag_Left = false;
  }

  // Функция обработки считывания кнопок
  void Loop()
  {
    // Сброс флагов кнопок
    btnFunc::flagDrop();

    // Нажатие вправо
    if (pcf_click(pin_SB1_Right) == true)
    {
      btn_flag_Right = true;
    }

    // Нажатие вниз
    if (pcf_click(pin_SB2_Down) == true)
    {
      btn_flag_Down = true;
    }
    
    // Нажитие влево
    if (pcf_click(pin_SB3_Left) == true)
    {
      btn_flag_Left = true;
    }
    
    // Нажатие вверх
    if (pcf_click(pin_SB4_Up) == true)
    {
      btn_flag_Up = true;
    }
  }
}





///////////////////////////////
//      Интерфейс (Menu)     //
///////////////////////////////
namespace menuFunc {

  // Функция переключения меню
  void switchTo(uint8_t screen_id)
  {
    work_sreen_id = screen_id;        // Указание активного экрана
    lcd1.clear();
    for (int line_i = 1; line_i <= 4; line_i++)
    {
      lcd1.setCursor(0, line_i - 1);
      lcd1.print(sdFunc::readLine(menu_folder, screen_id, line_i));
    }
  }

  // Функция циклов меню
  void Loop()
  {
    // Экран №0 - Стартовое меню (Menu_Start)
    if (work_sreen_id == Menu_Start)
    {
      // Управление на экране
      if (btn_flag_Right)
      {
        menuFunc::switchTo(Menu_cfg);
        lcd1.setCursor(4, 2);
        lcd1.print(cfgID);
        btnFunc::flagDrop();
      }
    }
    
    // Экран №1 - Меню выбора cfg (Menu_cfg)
    if (work_sreen_id == Menu_cfg)
    {
      // Управление на экране
      if (btn_flag_Right)
      {
        humSet();                       // Фиксация эталонной влажности в почве
        // Чтение параметров из cfg
        String cfg_path_current = "/" + String(cfgID) + ".cfg";
        if (sdFunc::cfgRead(cfg_path_current))
        {
          // Создание log файла
          if(sdFunc::LogCreate())
          {
            menuFunc::switchTo(Menu_Log);
            lcd1.setCursor(6, 1);
            lcd1.print(log_path);
          }
          else
          {
            menuFunc::switchTo(Menu_Log_err);
          }
        }
        else
        {
          menuFunc::switchTo(Menu_cfg_err);
        }
        btnFunc::flagDrop();
      }
      if (btn_flag_Left)
      {
        menuFunc::switchTo(Menu_Start);
        btnFunc::flagDrop();
      }
      if (btn_flag_Up)
      {
        cfgID = cfgID + 1;
        lcd1.setCursor(4, 2);
        lcd1.print("   ");
        lcd1.setCursor(4, 2);
        lcd1.print(cfgID);
        btnFunc::flagDrop();
      }
      if (btn_flag_Down)
      {
        cfgID = cfgID - 1;
        lcd1.setCursor(4, 2);
        lcd1.print("   ");
        lcd1.setCursor(4, 2);
        lcd1.print(cfgID);
        btnFunc::flagDrop();
      }
    }
    
    // Экран №2 - Ошибка cfg-файла (Menu_cfg_err)
    if (work_sreen_id == Menu_cfg_err)
    {
      // Управление на экране
      if (btn_flag_Left)
      {
        menuFunc::switchTo(Menu_Start);
        btnFunc::flagDrop();
      }
    }

    // Экран №3 - Создание log-файла (Menu_Log)
    if (work_sreen_id == Menu_Log)
    {
      // Управление на экране
      if (btn_flag_Right)
      {
        menuFunc::switchTo(Menu_Exp_1);
        btnFunc::flagDrop();
        menu_update = true;
        experiment_inWork = true;                     // Включение экспермента
        time_exp_start = rtc.getUnix(rtc_region);     // Фиксация времени начала эксперимента
        time_exp_count = 0;                           // Обнуление таймера эксперимента
      }
      if (btn_flag_Left)
      {
        menuFunc::switchTo(Menu_Start);
        btnFunc::flagDrop();
      }
    }
  
    // Экран №4 - Ошибка log-файла (Menu_Log_err)
    if (work_sreen_id == Menu_Log_err)
    {
      // Управление на экране
      if (btn_flag_Left)
      {
        menuFunc::switchTo(Menu_Start);
        btnFunc::flagDrop();
      }
    }

    // Экран №5 - Эксперимент выполнен (Menu_Exp_Done)
    if (work_sreen_id == Menu_Exp_Done)
    {
      // Управление на экране
      if (btn_flag_Right)
      {
        menuFunc::switchTo(Menu_Start);
        btnFunc::flagDrop();
      }
    }

    // Экран №6 - Остановка эксперимента (Menu_Exp_Stop)
    if (work_sreen_id == Menu_Exp_Stop)
    {
      // Управление на экране
      if (btn_flag_Right)
      {
        menuFunc::switchTo(Menu_Start);
        btnFunc::flagDrop();
        experiment_inWork = false;             // Выключение экспермента
      }
      if (btn_flag_Left)
      {
        menuFunc::switchTo(Menu_Exp_1);
        btnFunc::flagDrop();
        menu_update = true;
      }
    }

    // Экран №7 - Эксперимент 1 (Menu_Exp_1)
    if (work_sreen_id == Menu_Exp_1)
    {
      // Динамичные элементы
      if (menu_update == true)
      {
        menu_update = false;
        lcd1.setCursor(10, 0);
        lcd1.print(experiment_long);
        lcd1.setCursor(10, 1);
        uint32_t time_curr = (float)time_exp_count / 60.0f;
        lcd1.print(time_curr);
        lcd1.setCursor(10, 2);
        uint8_t process = 100 * (time_exp_count / (experiment_long * 60.0f));
        zeroWrite(process);
        lcd1.print(process);
      }

      // Управление на экране
      if (btn_flag_Left)
      {
        menuFunc::switchTo(Menu_Exp_Stop);
        btnFunc::flagDrop();
      }
      if (btn_flag_Up || btn_flag_Down)
      {
        menuFunc::switchTo(Menu_Exp_2);
        btnFunc::flagDrop();
        menu_update = true;
      }
    }

    // Экран №8 - Эксперимент 2 (Menu_Exp_2)
    if (work_sreen_id == Menu_Exp_2)
    {
      // Динамичные элементы
      if (menu_update == true)
      {
        menu_update = false;
        lcd1.setCursor(11, 0);
        zeroWrite(air_out_temp_current);
        lcd1.print(air_out_temp_current);
        lcd1.setCursor(11, 1);
        zeroWrite(air_temp_current);
        lcd1.print(air_temp_current);
        lcd1.setCursor(11, 2);
        zeroWrite(soil_temp_current_avr);
        lcd1.print(soil_temp_current_avr);
      }
      
      // Управление на экране
      if (btn_flag_Left)
      {
        menuFunc::switchTo(Menu_Exp_Stop);
        btnFunc::flagDrop();
      }
      if (btn_flag_Up || btn_flag_Down)
      {
        menuFunc::switchTo(Menu_Exp_1);
        btnFunc::flagDrop();
        menu_update = true;
      }
    }    
  }
}





///////////////////////////////
//  Обработчик выращивания   //
///////////////////////////////
namespace expFunc {

  // Расчёт управляющего уровня режима дня и ночи
  void DayNight(uint32_t time_curr)
  {
    uint32_t all_day_clock = 24UL * 3600;                                 // Длительность суток (сек)
    uint32_t switch_long = ((24 - (day_long + night_long)) / 2) * 3600;   // Длительность перехода День / Ночь (сек)
    uint8_t morn_clock = 0;                                               // Значение часов (24 формат) начала утра (сек)
    uint32_t day_clock = switch_long;                                     // Значение часов (24 формат) начала дня (сек)
    uint32_t evening_clock = day_clock + day_long * 3600;                 // Значение часов (24 формат) начала вечера (сек)
    uint32_t night_clock = evening_clock + switch_long;                   // Значение часов (24 формат) начала ночи (сек)

    uint32_t time_clock = time_curr % (all_day_clock);                    // Значение часов (24 формат) в данный момент (сек)

    // Фаза утра
    if (morn_clock <= time_clock && time_clock < day_clock)
    {
      day_night_lvl = 255 * ((float)time_clock / (float)switch_long);
    }
    // Фаза дня
    if (day_clock <= time_clock && time_clock < evening_clock)
    {
      day_night_lvl = 255;
    }
    // Фаза вечера
    if (evening_clock <= time_clock && time_clock < night_clock)
    {
      day_night_lvl = 255 * (1 - ((float)time_clock - (float)evening_clock) / (float)switch_long);
    }
    // Фаза ночи
    if (night_clock <= time_clock && time_clock <= all_day_clock)
    {
      day_night_lvl = 0;
    }
    
    light_set = (float)light_night + (float)day_night_lvl/255 * ((float)light_day - (float)light_night);
    temp_air_set = (float)temp_air_set_night + (float)day_night_lvl/255 * ((float)temp_air_set_day - (float)temp_air_set_night);
    temp_soil_set = (float)temp_soil_set_night + (float)day_night_lvl/255 * ((float)temp_soil_set_day - (float)temp_soil_set_night);
  }

  // Функция управления Фитосветильником
  void light(uint8_t state)
  {
    uint8_t pins[] = {pin_led_W, pin_led_R, pin_led_G, pin_led_B};
    uint8_t light_s[] = {light_s_W, light_s_R, light_s_G, light_s_B};
    uint8_t signal_all = state * map(light_set, 0, 100, 0, 255);
    for (int i = 0; i < 4; i++)
    {
      uint8_t sig = light_s[i];
      sig = (float)signal_all * (float)sig / 100.0f;
      analogWrite(pins[i], sig);
    }
  }

  // Функция замера влажности
  void irrigating()
  {
    if (time_exp_count - timer_humMeas >= timer_humMeas_period)
    {
      uint8_t SH_pin = 255;
      uint8_t pump_pin = 255;
      uint16_t hum_set = 0;
      switch (pot_num)
      {
        case 0:
          SH_pin = pin_SH1;
          pump_pin = pin_P1;
          hum_set = hum_set_1;
          break;
        case 1:
          SH_pin = pin_SH2;
          pump_pin = pin_P2;
          hum_set = hum_set_2;
          break;
        case 2:
          SH_pin = pin_SH3;
          pump_pin = pin_P3;
          hum_set = hum_set_3;
          break;
        case 3:
          SH_pin = pin_SH4;
          pump_pin = pin_P4;
          hum_set = hum_set_4;
          break;
      }
      if (humMeas_switch == true)
      {
        mux.channel(SH_pin);                      // Устанавливаем мультиплексор на канал датчика влажности
        delay(mux_delay_def);
        hum_curr = analogRead(pin_mux);           // Измерение влажности
        // Пересчёт влажности с учётом температуры
        hum_curr = imaginaryHum(hum_curr, soil_temp_current_avr);        
        mux.channel(pin_SL1);                     // Устанавливаем мультиплексор на канал датчика уровня воды
        water_lvl = digitalRead(pin_mux);
        switch (pot_num)
        {
          case 0:
            hum_pot_1 = hum_curr;
            break;
          case 1:
            hum_pot_2 = hum_curr;
            break;
          case 2:
            hum_pot_3 = hum_curr;
            break;
          case 3:
            hum_pot_4 = hum_curr;
            break;
        }
        humMeas_switch = false;
        timer_pump = millis();
      }
      if (millis() - timer_pump <= timer_pump_period && hum_curr > hum_set && water_lvl)
      {
        pcf1.write(pump_pin, !true);
      }
      else
      {
        pcf1.write(pump_pin, !false);
        pot_num = pot_num + 1;
        humMeas_switch = true;
      }
      if (pot_num >= 4)
      {
        pot_num = 0;
        timer_humMeas = time_exp_count;
      }
    }
  }

  // Функция получения средней температуры почвы
  float DS18Avr()
  {
    float val = 0;
    // Считывание температуры DS18 - DS1 ... DS4
    for (int DS_num = 0; DS_num < 4; DS_num++)
    {
      DS18.readTemp(DS_num);                  // Активация нужного датчика DS18
      val = val + DS18.getTemp();             // Считывание температуры с активого датичка DS18
    }

    val = val / 4;
    DS18.requestTemp();
    return val;
  }

  // Функция записи лог данных
  void timerTick()
  {
    // Если прошло время периода записи
    if (millis() - timer_tick >= timer_tick_period * 1000)
    {
      timer_tick = millis();
      time_exp_count = rtc.getUnix(rtc_region) - time_exp_start;
      expFunc::DayNight(time_exp_count);
      log_tick_count = log_tick_count + 1;
      menu_update = true;                                 // Сигнал на обновления экрана
    }
    if ((float)log_tick_count >= (float)log_period / (float)timer_tick_period)
    {
      sdFunc::LogData();
      log_tick_count = 0;
    }
  }

  // Функция циклов режима выращивания
  void Loop()
  {
    // Если эксперимент активен
    if (experiment_inWork)
    {
      // Если время эксперимента НЕ закончилось
      if (time_exp_count <= experiment_long * 60.0f)
      { 
        expFunc::timerTick();
        // Чтение датчиков
        soil_temp_current_avr = expFunc::DS18Avr();
        air_temp_current = DHT1.readTemperature();
        air_out_temp_current = DHT2.readTemperature();

        // Работа устройств воздействия
        Heater.workPWM(temp_soil_set, soil_temp_current_avr);
        Fan.workPWM(temp_air_set, air_temp_current);
        expFunc::light(1);                   // Работа освещения
        expFunc::irrigating();
        
      }
      // Если время эксперимента закончилось
      else
      {
        experiment_inWork = false;            // Остановка эксперимента
        menuFunc::switchTo(Menu_Exp_Done);        // Переключение экрана
      }
    }
    // Если эксперимент НЕ активен
    else
    {
      // Выключение элементов
      Heater.workON(false);
      Fan.workON(false);
      pcf1.write(pin_F2, !false);
      expFunc::light(0);
    }
    
  }
}






/////////////////////////////////////////////////////
//                   УСТАНОВКИ                     //
/////////////////////////////////////////////////////

void setup()
{
  //Serial.begin(9600);                       // Запуск монитора порта с частотой 9600 бад

  ///////////////////////////////
  //  Инициализация объектов   //
  ///////////////////////////////
  Wire.begin();                             // Инициализация шины Wire
  delay(25);                                // Ожидание инициализации
  rtc.begin();                              // Инициализация часов реального времени и синхронизация времени
  DS18.requestTemp();                       // Запуск DS18 и первый запрос на измерение
  DHT1.begin();                             // Запуск внутреннего датчика DHT22
  DHT2.begin();                             // Запуск внешнего датчика DHT22
  pcf1.begin();                             // Инициализация PCF1
  pcf2.begin();                             // Инициализация PCF2
  lcd1.init();                              // Инициализация lcd1
  lcd1.backlight();                         // Включить подсветку lcd1
  SD.begin(pin_CS);                         // Инициализация SD карты

  ///////////////////////////////
  //      Настройка пинов      //
  ///////////////////////////////
  pinMode(pin_mux, INPUT);                  // Мультиплексор
  pinMode(pin_DS18, INPUT);                 // Датчики DS18
  // LED освещение WRGB
  pinMode(pin_led_W, OUTPUT);
  pinMode(pin_led_R, OUTPUT);
  pinMode(pin_led_G, OUTPUT);
  pinMode(pin_led_B, OUTPUT);
  /*
  // Сброс времени часов (синхронизация с ПК)
  if (!pcf2.readButton(pin_SB4_Up) && !pcf2.readButton(pin_SB3_Left))
  {
    rtc.setTime(BUILD_SEC + rtc_second_shift, BUILD_MIN, BUILD_HOUR, BUILD_DAY, BUILD_MONTH, BUILD_YEAR);  // Установка времени с ПК при компиляции
  }
  */
  ///////////////////////////////
  //     Начальные сигналы     //
  ///////////////////////////////
  /*
  pcf1.write(pin_RT1, false);
  analogWrite(pin_led_W, 0);
  analogWrite(pin_led_R, 0);
  analogWrite(pin_led_G, 0);
  analogWrite(pin_led_B, 0);
  */

  //Serial.println(sdFunc::readLine(strings_folder, 1, 2));
  menuFunc::switchTo(Menu_Start);
}





/////////////////////////////////////////////////////
//                   ЦИКЛ РАБОТЫ                   //
///////////////////////////////////////////////////// 

void loop()
{ 
  expFunc::Loop();          // Обработка режима выращивания
  menuFunc::Loop();         // Обработка экранов меню
  btnFunc::Loop();          // Обработка считывания кнопок
}
