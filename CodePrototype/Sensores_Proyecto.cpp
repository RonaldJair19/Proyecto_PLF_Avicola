#include "Sensores_Proyecto.h"
#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <BH1750.h>

/*METODOS DE LA CLASE NodoSensor*/
//void NodeSensor::addValue(float Value){  
//    if(isnan(Value)){
//        
//    }
//    
//    if(_Counter_Sensor == 1){
//        _MaxValue = Value;
//        _MinValue = Value;
//    }
//    else{
//        if (_MaxValue < Value){
//            _MaxValue = Value;
//        }
//        else if(_MinValue > Value){
//            _MinValue = Value;
//        }
//    }
//    _Counter_Sensor++;
//    _Values = _Values + Value;
//        
//}

int NodeSensor::addValue(float Value){
    if(isnan(Value)){ //Comprobar funcionamiento del sensor
        return 0;
    }
    else{
        if(_Counter_Sensor == 1){
            _MaxValue = Value;
            _MinValue = Value;
        }
        else{
            if (_MaxValue < Value){
                _MaxValue = Value;
            }
            else if(_MinValue > Value){
                _MinValue = Value;
            }
        }
        _Counter_Sensor++;
        _Values = _Values + Value;
        return 1;
    }
}

int NodeSensor::getTypeVariable(){  
        return _Variable;
}

int NodeSensor::getCounterSensor(){
    return _Counter_Sensor;
}

float NodeSensor::getValues(){
    return _Values;
}

float NodeSensor::getMaxValue(){
    return _MaxValue;
}

float NodeSensor::getMinValue(){
    return _MinValue;
}

NodeSensor::NodeSensor(Type_Variable Type_Variable_Node){
    _Counter_Sensor = 0;
    //_Avarage_Value = 0;
    _Values = 0;
    _MaxValue = 0; 
    _MinValue = 0;
    _Variable = Type_Variable_Node;
}


//NodeSensor::~NodeSensor();

void NodeSensor::CalculateAvarageValue(){
    switch (_Variable)
    {
        case VARIABLE_TEMPERATURE:
        case VARIABLE_HUMIDITY:
        case VARIABLE_TOXIC:
        case VARIABLE_FLAMMABLE:
        case VARIABLE_LIGHT:
            _Avarage_Value = _Values / _Counter_Sensor;
            if (isnan(_Avarage_Value)){
                _Avarage_Value = 0;
            }
        break;
    default:
        break;
    }
}

float NodeSensor::getAvarage(){
    return _Avarage_Value;
} 

void NodeSensor::resetCounterAvg(){
    _Counter_Sensor = 0 ;
    //_Avarage_Value = 0;
    _Values  = 0;
    //_MaxValue, _MinValue = 0; //Si no se resetea se puede tener la lectura maxima y minima de todas las lecturas
}






/*METODOS DE LA CLASE SENSOR*/

Sensor::Sensor(int pin, Type_Sensor type){
    _Pin = pin;
    _Type = type;
    _Value_Sensor, _Values_Temp, _Values_Hum = 0;
}

Sensor::Sensor(Type_Sensor type){
    _Type = type;
}


void Sensor::setSensorPIN(int Pin){
    _Pin = Pin;
}

void Sensor::setSensorTYPE(Type_Sensor type){
    _Type = type;
}

void Sensor::readValueSensor(){
    if(_Type == Type_Sensor::MQ_135){ //Sensor MQ135 de Gas
        pinMode(_Pin,INPUT);
        _Value_Sensor = analogRead(_Pin);
    }
    else if(_Type == Type_Sensor::MQ_2){ //Sensor MQ2 de Gas
        pinMode(_Pin,INPUT);
        _Value_Sensor = analogRead(_Pin);
    }
    else if(_Type == Type_Sensor::BH_1735){
        BH1750 luxometro;
        Wire.begin();
        luxometro.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
        _Value_Sensor = luxometro.readLightLevel();
    }
    else if (_Type == Type_Sensor::DHT_11){ //Sensor DHT11 de temperatura
        DHT dht11(_Pin, DHT11);
        dht11.begin();
        _Values_Temp = dht11.readTemperature();
        _Values_Hum = dht11.readHumidity();
    }
    else if(_Type == Type_Sensor::DHT_22){ //Sensor DHT22 de temperatura
        DHT dht22(_Pin, DHT22);
        dht22.begin();
        _Values_Temp = dht22.readTemperature();
        _Values_Hum = dht22.readHumidity();
    }
    else if (_Type == Type_Sensor::UNCONFIGURED_SENSOR){
        _Values_Temp = 0;
        _Values_Hum = 0;
        _Value_Sensor = 0;
    }
}

float Sensor::getValueSensor(){
    return _Value_Sensor;
}

float Sensor::getValueSensor(Type_DHT Type_DHT){
    if (_Type == Type_Sensor::DHT_11){ //Sensor DHT11 de temperatura
        switch (Type_DHT){
        case DHT_11_TEMPERATURE:
                return _Values_Temp;
            break;
        case DHT_11_HUMIDITY:
                return _Values_Hum;
            break;
        default:
            break;
        }
    }
    else if(_Type == Type_Sensor::DHT_22){ //Sensor DHT22 de temperatura
        switch (Type_DHT){
        case DHT_22_TEMPERATURE:
                return _Values_Temp;
            break;
        case DHT_22_HUMIDITY:
                return _Values_Hum;
            break;
        default:
            break;
        }
    }
}

int Sensor::getPinSensor(){
    return _Pin;
}



/*METODOS DE LA CLASE CONTROL STATE*/
bool Control_Element::getState(){
        return _state;
    }
Control_Element::Control_Element(int pin){
    _pin_element = pin;
    pinMode(_pin_element, OUTPUT);
    _state = false;
}

Control_Element::Control_Element(){
    _state = false;
}

void Control_Element::setOnElement(){
    _state = true;
    digitalWrite(_pin_element, HIGH);
}

void Control_Element::setOffElement(){
    _state = false;
    digitalWrite(_pin_element, LOW);
}