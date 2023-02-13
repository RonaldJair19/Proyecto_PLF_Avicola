#include <Arduino.h>

class NodeSensor
{
public:
    enum Type_Variable{
        UNCONFIGURED,
        VARIABLE_TEMPERATURE,
        VARIABLE_HUMIDITY,
        VARIABLE_TOXIC,
        VARIABLE_FLAMMABLE,
        VARIABLE_LIGHT
    };
    NodeSensor(Type_Variable Type_Variable_Node); //Constructor
    int addValue(float Value);
    //void addValue(float Value);
    void CalculateAvarageValue();
    float getAvarage();
    int getTypeVariable();
    void resetCounterAvg();
    int getCounterSensor();
    float getValues();
    float getMaxValue();
    float getMinValue();
    //~NodeSensor();
    
    private:
    int _Counter_Sensor;
    float _Values, _MaxValue, _MinValue;
    float _Avarage_Value;
    Type_Variable _Variable = UNCONFIGURED;
};





class Sensor{
    private:
        int _Pin; //Pin asignado al sensor
        float _Value_Sensor, _Values_Temp, _Values_Hum; //Valor obtenido del sensor
    public:
        //Sensor();
        //Sensor(int pin, int type);
        enum Type_Sensor{
            UNCONFIGURED_SENSOR,
            DHT_11,
            DHT_22,
            MQ_135,
            MQ_2,
            BH_1735,
            KY_001
        };

        enum Type_DHT{
            UNCONFIGURED_DHT,
            DHT_11_TEMPERATURE,
            DHT_11_HUMIDITY,
            DHT_22_TEMPERATURE,
            DHT_22_HUMIDITY
        };


        Type_Sensor _Type = UNCONFIGURED_SENSOR;
        Type_DHT _Type_DHT = UNCONFIGURED_DHT;
        Sensor(int pin, Type_Sensor modo);
        Sensor(Type_Sensor modo);
        void setSensorPIN(int Pin);
        void setSensorTYPE(Type_Sensor modo);
        int getPinSensor(void);
        int getSensorTYPE(void);
        void readValueSensor();
        float getValueSensor();
        float getValueSensor(Type_DHT Type_DHT);
};


class IdentifierEmployer{
    public:
        enum Employer_Identifier{
        NO_USE,
        TEMPERATURE_DOWN,
        TEMPERATURE_UP,
        HUMIDITY_DOW,
        HUMIDITY_UP,
        TOXIC_UP,
        FLAMMABLE_UP,
        LUMINOSITY_DOWN
    };
};

class ControlElement : public IdentifierEmployer{
    private:    
        int _pin_element;
        bool _state;
        Employer_Identifier _useIdentifier;
    public:
    ControlElement(int pin);
    ControlElement();
    bool getState();
    void setOnElement(Employer_Identifier useIdentifier);
    void setOffElement();
    Employer_Identifier getUseIdentifier();
};


class Evaluator : public IdentifierEmployer{
    public:
    NodeSensor *nodeSensor;
    ControlElement *controlElement;
    float _latestAverage;
    float _minimumRangeValue, _maximumRangeValue;
    Employer_Identifier _employer_identifier;

    Evaluator(NodeSensor *nodeSensor, ControlElement *controlElement);
    Evaluator(NodeSensor *nodeSensor, ControlElement *controlElement, Employer_Identifier employerId);
    void evaluateVariable();
    void setMinimumRangeValue(float minimumRangeValue);
    void setMaximumRangeValue(float maximumRangeValue);
    void setEmployerIdentifier(Employer_Identifier employerId);
    float getMinimumRangeValue();
    float getMaximumRangeValue();
    float getAvgEvaluator();
    Employer_Identifier getEmployerIdentifier();
};

