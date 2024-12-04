/*********************************************************************************
 * 
 * Работа АЦП с резистивным делителем
 *
 ********************************************************************************/
#pragma once
#include <Arduino.h>

#if defined(__AVR__)
  #define _GPIO_REG_TYPE    uint8_t
  #define _DEFAULT_VREF     5
#elif defined(ARDUINO_ARCH_ESP32)
  #define _GPIO_REG_TYPE    uint8_t
  #define _DEFAULT_VREF     3.3
#elif defined(ARDUINO_ARCH_ESP8266)
  #define _GPIO_REG_TYPE    uint8_t
  #define _DEFAULT_VREF     3.3
#elif defined(ARDUINO_ARCH_STM32)
  #define _GPIO_REG_TYPE    uint32_t
  #define _DEFAULT_VREF     3.3
#endif

class AdvADC {
public:
  /**
  *  Конструктор
  *  Принимает пин, номиналы сопротивлений делителя, опорное напряжение и разрешение АЦП
  */
  AdvADC(_GPIO_REG_TYPE pin, uint16_t R_up, uint16_t R_down, float vRef = _DEFAULT_VREF
        #if defined (ARDUINO_ARCH_ESP32)  /* ESP32 */
          , adc_attenuation_t db = ADC_11db
        #endif
        )
  {
    init(pin, R_up, R_down, vRef
          #if defined (ARDUINO_ARCH_ESP32)
            , db
          #endif
          );
  };

  /**
   * Конструктор для случая без резистивного делителя, подключение напрямую к пину
   */
  AdvADC(_GPIO_REG_TYPE pin, float vRef = _DEFAULT_VREF     
        #if defined (ARDUINO_ARCH_ESP32) /* ESP32 */
          , adc_attenuation_t db = ADC_11db 
        #endif
        )  
  {
    init( pin, 0, 1000, vRef
            #if defined (ARDUINO_ARCH_ESP32) /* ESP32 */
              , db
            #endif
            );
  }

/*--------STM32---------*/
  #if defined (ARDUINO_ARCH_STM32)
  /**
  *  Конструктор - для STM32
  *  Принимает пин типа PinName (пример: PC_2), номиналы сопротивлений делителя, опорное напряжение и разрешение АЦП
  */
  AdvADC(PinName pin, uint16_t R_up, uint16_t R_down, float vRef = _DEFAULT_VREF) {
    init(digitalPinToPinName(pin), R_up, R_down, vRef);
  };

  /**
   * Конструктор, принимает пин типа PinName (пример: PC_2) - для STM32
   */
  AdvADC(PinName pin, float vRef = 3.3)   {
    init( digitalPinToPinName(pin), 0, 1000, vRef );
  }
  #endif

  /**
   * Читать напряжение в вольтах
   */
  float getVoltage () {  
    #if defined(__AVR__)
      return  koeff * analogRead(_pin);
    #elif defined (ARDUINO_ARCH_ESP32) 
      return koeff * analogReadMilliVolts(_pin) / 1000;  
    #elif defined (ARDUINO_ARCH_ESP8266) 
      return koeff * analogRead(_pin);
    #elif defined (ARDUINO_ARCH_STM32)
      PinName p = analogInputToPinName(_pin);
      return koeff * adc_read_value(p, resolution);
    #endif
  }

  /**
   * Читать напряжение в отсчетах АЦП
   */
  float getAdc () {  
    #if defined(__AVR__)
      return  analogRead(_pin);
    #elif defined (ARDUINO_ARCH_ESP32)  
      return analogRead(_pin);  
    #elif defined (ARDUINO_ARCH_ESP8266)
      return analogRead(_pin);
    #elif defined (ARDUINO_ARCH_STM32) 
      PinName p = analogInputToPinName(_pin);
      return adc_read_value(p, resolution);
    #endif
  }

private:
  /**
   * Инициализация для конструкторов
   */
  void init(_GPIO_REG_TYPE pin, uint16_t R_up, uint16_t R_down, float vRef = _DEFAULT_VREF
            /* ESP32 */
            #if defined (ARDUINO_ARCH_ESP32)
              , adc_attenuation_t db = ADC_11db
            #endif
            ) 
  {
    _pin = pin;
    pinMode(_pin, INPUT);

    #if defined (__AVR__) 
      resolution = 10;          //Разрешение по умолчанию
      koeff = ((R_up + R_down) / R_down) * (vRef / pow(2, resolution));   //Коэфф. приведения к вольтам  
    #elif defined (ARDUINO_ARCH_ESP32)  
      adcAttachPin(_pin);                 //Присоединить пин если был перенастроен
      analogSetPinAttenuation(_pin, db);  //Аттенюатор
      koeff = (R_up + R_down) / R_down;   //Коэфф. приведения к вольтам 
    #elif defined (ARDUINO_ARCH_ESP8266)  
      resolution = 10;
      koeff = (R_up + R_down) / R_down * (vRef / pow(2, resolution));   //Коэфф. приведения к вольтам 
    #elif defined  (ARDUINO_ARCH_STM32) 
      resolution = 12;          //Разрешение по умолчанию
      koeff = ((R_up + R_down) / R_down) * (vRef / pow(2, resolution));   //Коэфф. приведения к вольтам  
    #endif
  }

  _GPIO_REG_TYPE _pin;
  int resolution;
  float koeff;
};
