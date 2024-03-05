//------------------------------------------------------------------------------------
//  Работа АЦП с резистивным делителем
//------------------------------------------------------------------------------------
#pragma once
#include <Arduino.h>

class AdvADC {
public:
  //------------------------------------------------------------------------------------
  //  Конструктор
  //  Принимает пин, номиналы сопротивлений делителя, опорное напряжение и разрешение АЦП
  //------------------------------------------------------------------------------------ 
  AdvADC(uint32_t pin, 
        float R_up = 0, float R_down = 10000, 
        float vRef = 3.3, 
        int bitRes = 12)
  {
    _pin = pin;
    _vRef = vRef;
    float kmV = vRef / pow(2, bitRes);          //Коэфф. приведения к mV
    float kDiv = (R_up + R_down) / R_down;      //Коэфф. делителя
    k = kmV * kDiv;                             //Общий коэфф.
    analogReadResolution(bitRes);               //Установить разрешение
    pinMode(_pin, INPUT);
  }
  
  //------------------------------------------------------------------------------------
  //  Считать напряжение в вольтах. 
  //------------------------------------------------------------------------------------
  float voltage () {
    uint32_t adc = analogRead(_pin);
    return adc * k;
    }

private:
  uint32_t _pin;
  float _vRef, k;
};
