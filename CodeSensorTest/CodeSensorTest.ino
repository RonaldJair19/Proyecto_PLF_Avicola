#include <Ticker.h>
#include <DHT.h>
#include <DHT_U.h>
#include "Sensores_Proyecto.h"
#include "TTN_CayenneLPP.h"

TTN_CayenneLPP lpp;

char payloadString[50] = {0};

//DEFINICION PINES SENSORES Y ACTUADORES
#define DHT11_2_PIN 2 
#define DHT11_1_PIN 3
#define DHT11_0_PIN 4
#define DHT22_2_PIN 5
#define DHT22_1_PIN 6
#define DHT22_0_PIN 7
#define MQ2_0_PIN A3
#define MQ2_1_PIN A6
#define MQ135_0_PIN A0
#define MQ135_1_PIN A1
#define MQ135_2_PIN A2
#define RELAY_FAN_PIN 8
#define RELAY_HUMIDITY_SYSTEM_PIN 9
#define RELAY_INFRARED_BULBS_PIN 10
#define RELAY_INCANDESCENT_BULBS_PIN 11


// DEFINICION DE LOS OBJETOS NODO PARA CADA VARIABLE A MONITORIZAR
NodeSensor NODO_TEMPERATURE(NodeSensor::VARIABLE_TEMPERATURE);
NodeSensor NODO_HUMIDITY(NodeSensor::VARIABLE_HUMIDITY);
NodeSensor NODO_TOXIC(NodeSensor::VARIABLE_TOXIC);
NodeSensor NODO_FLAMMABLE(NodeSensor::VARIABLE_FLAMMABLE);
NodeSensor NODO_LIGHT(NodeSensor::VARIABLE_LIGHT);

//DEFINICION DE LOS VALORES DE REFERENCIA PARA LOS SENSORES
const float TEMPERATURE_DOWN_MAX_REFERENCE = 29.0;
const float TEMPERATURE_DOWN_MIN_REFERENCE = 0;

const float TEMPERATURE_UP_MAX_REFERENCE = 50.0;
const float TEMPERATURE_UP_MIN_REFERENCE = 30.10;

const float HUMIDITY_DOWN_MAX_REFERENCE = 40.1;
const float HUMIDITY_DOWN_MIN_REFERENCE = 0;

const float HUMIDITY_UP_MAX_REFERENCE = 100;
const float HUMIDITY_UP_MIN_REFERENCE = 54.0;

const float FLAMMABLE_UP_MAX_REFERENCE = 1000;
const float FLAMMABLE_UP_MIN_REFERENCE = 400;

const float TOXIC_UP_MAX_REFERENCE = 400;
const float TOXIC_UP_MIN_REFERENCE = 1000;

const float LIGHT_DOWN_MAX_REFERENCE = 400;
const float LIGHT_DOWN_MIN_REFERENCE = 0;

//DEFINICION DE LAS FUNCIONES PARA LEER DATOS DESDE LOS SENSORES Y OBTENER UN PROMEDIO
void Rutina_Temperatura();
void Rutina_Humedad();
void Rutina_Gases_Inflamables();
void Rutina_Gases_Toxicos();
void Rutina_Iluminacion();
void SendPayload_I2C();
// void Elementos_Control();
void TemperatureEvaluation();
void HumidityEvaluation();
void ToxicGasEvaluation();
void FlammableGasEvaluation();
void LuminosityEvaluation();

//DEFINICION DE LOS TICKERS QUE EJECUTARAN LA LECTURA DE LOS SENSORES EN UN TIEMPO DETERMINADO
Ticker RUTINA_TEMP(Rutina_Temperatura, 14000); //En micros segundos
Ticker RUTINA_HUM(Rutina_Humedad, 9000); //En micros segundos
Ticker RUTINA_MQ135(Rutina_Gases_Toxicos, 11000); //En micros segundos
Ticker RUTINA_MQ2(Rutina_Gases_Inflamables, 19000); //En micros segundos
Ticker RUTINA_LUZ(Rutina_Iluminacion, 29000); //En micros segundos

//TickTwo para el envio de la informacion
Ticker RUTINA_ENVIO_I2C(SendPayload_I2C, 60000);

//DEFINICION DE TICKER PARA LOS ACTUADORES
Ticker ROUTINE_TEMPERATURE_EVALUATION(TemperatureEvaluation, 30000);
Ticker ROUTINE_HUMIDITY_EVALUATION(HumidityEvaluation, 32000);
Ticker ROUTINE_TOXIC_GAS_EVALUATION(ToxicGasEvaluation, 34000);
Ticker ROUTINE_FLAMMABLE_GAS_EVALUATION(FlammableGasEvaluation, 36000);
Ticker ROUTINE_LUMINOSITY_EVALUATION(LuminosityEvaluation, 38000);

//DEFINICION DE LOS PINES Y TIPO DE SENSOR

//----------Para sensores KY001----------

//----------Para sensores DHT22----------
Sensor Sensor_DHT22_0(DHT22_0_PIN, Sensor::DHT_22);
Sensor Sensor_DHT22_1(DHT22_1_PIN, Sensor::DHT_22);
Sensor Sensor_DHT22_2(DHT22_2_PIN, Sensor::DHT_22);

//----------Para sensores DHT11----------
Sensor Sensor_DHT11_0(DHT11_0_PIN, Sensor::DHT_11);
Sensor Sensor_DHT11_1(DHT11_1_PIN, Sensor::DHT_11);
Sensor Sensor_DHT11_2(DHT11_2_PIN, Sensor::DHT_11);


//----------Para sensores MQ_135----------
Sensor SensorGasMQ135_0(MQ135_0_PIN,Sensor::MQ_135);
Sensor SensorGasMQ135_1(MQ135_1_PIN,Sensor::MQ_135);
Sensor SensorGasMQ135_2(MQ135_2_PIN,Sensor::MQ_135);

//----------Para sensores MQ_2----------
Sensor SensorGasMQ2_0(MQ2_0_PIN,Sensor::MQ_2);
Sensor SensorGasMQ2_1(MQ2_1_PIN,Sensor::MQ_2);

//----------Para sensores BH1750----------
Sensor SensorBH1750_1(Sensor::BH_1735); // SCL ---> A5 | SDA ---> A4



//Controls element (fan, humidity system, infrared bulbs, incandescent bulb)
ControlElement RelayFan(RELAY_FAN_PIN);
ControlElement RelayHumiditySystem(RELAY_HUMIDITY_SYSTEM_PIN);
ControlElement RelayInfraredBulbs(RELAY_INFRARED_BULBS_PIN);
ControlElement RelayIncandescentBulbs(RELAY_INCANDESCENT_BULBS_PIN);

//Evaluators
Evaluator TemperatureDownEvaluator(&NODO_TEMPERATURE, &RelayFan, Evaluator::TEMPERATURE_DOWN);
Evaluator TemperatureUpEvaluator(&NODO_TEMPERATURE, &RelayInfraredBulbs, Evaluator::TEMPERATURE_UP);
Evaluator HumidityDownEvaluator(&NODO_HUMIDITY, &RelayHumiditySystem, Evaluator::HUMIDITY_DOW);
Evaluator HumidityUpEvaluator(&NODO_HUMIDITY, &RelayInfraredBulbs, Evaluator::HUMIDITY_UP);
Evaluator ToxicUpEvaluator(&NODO_TOXIC, &RelayFan, Evaluator::TOXIC_UP);
Evaluator FlammableUpEvaluator(&NODO_FLAMMABLE, &RelayFan, Evaluator::FLAMMABLE_UP);
Evaluator LuminosityDownEvaluator(&NODO_LIGHT, &RelayIncandescentBulbs, Evaluator::LUMINOSITY_DOWN);



void setup() {
  TemperatureDownEvaluator.setMaximumRangeValue(TEMPERATURE_DOWN_MAX_REFERENCE);
  TemperatureDownEvaluator.setMinimumRangeValue(TEMPERATURE_DOWN_MIN_REFERENCE);

  TemperatureUpEvaluator.setMaximumRangeValue(TEMPERATURE_UP_MAX_REFERENCE);
  TemperatureUpEvaluator.setMinimumRangeValue(TEMPERATURE_UP_MIN_REFERENCE);

  HumidityDownEvaluator.setMaximumRangeValue(HUMIDITY_DOWN_MAX_REFERENCE);
  HumidityDownEvaluator.setMinimumRangeValue(HUMIDITY_DOWN_MIN_REFERENCE);

  HumidityUpEvaluator.setMaximumRangeValue(HUMIDITY_UP_MAX_REFERENCE);
  HumidityUpEvaluator.setMinimumRangeValue(HUMIDITY_UP_MIN_REFERENCE);

  ToxicUpEvaluator.setMaximumRangeValue(TOXIC_UP_MAX_REFERENCE);
  ToxicUpEvaluator.setMinimumRangeValue(TOXIC_UP_MIN_REFERENCE);

  FlammableUpEvaluator.setMaximumRangeValue(FLAMMABLE_UP_MAX_REFERENCE);
  FlammableUpEvaluator.setMinimumRangeValue(FLAMMABLE_UP_MIN_REFERENCE);

  LuminosityDownEvaluator.setMaximumRangeValue(LIGHT_DOWN_MAX_REFERENCE);
  LuminosityDownEvaluator.setMinimumRangeValue(LIGHT_DOWN_MIN_REFERENCE);


  Serial.begin(9600);
  RUTINA_TEMP.start();
  RUTINA_HUM.start();
  RUTINA_MQ135.start();
  RUTINA_MQ2.start();
  RUTINA_LUZ.start();
  RUTINA_ENVIO_I2C.start();
  ROUTINE_TEMPERATURE_EVALUATION.start();
  ROUTINE_HUMIDITY_EVALUATION.start();
  ROUTINE_TOXIC_GAS_EVALUATION.start();
  ROUTINE_FLAMMABLE_GAS_EVALUATION.start();
  ROUTINE_LUMINOSITY_EVALUATION.start();
  Serial.println(F("Iniciando..."));
}

void loop() {
  RUTINA_TEMP.update();
  RUTINA_HUM.update();
  RUTINA_MQ135.update();
  RUTINA_MQ2.update();
  RUTINA_LUZ.update();
  RUTINA_ENVIO_I2C.update();
  if(RUTINA_ENVIO_I2C.counter() == 5){
    //Serial.println(F("Cambio del intervalo"));
    RUTINA_ENVIO_I2C.interval(60000*20);
    RUTINA_TEMP.interval(59000);
    RUTINA_HUM.interval(58000);
    RUTINA_MQ135.interval(57000);
    RUTINA_MQ2.interval(56000);
    RUTINA_LUZ.interval(55000);
  }
  ROUTINE_TEMPERATURE_EVALUATION.update();
  ROUTINE_HUMIDITY_EVALUATION.update();
  ROUTINE_TOXIC_GAS_EVALUATION.update();
  ROUTINE_FLAMMABLE_GAS_EVALUATION.update();
  ROUTINE_LUMINOSITY_EVALUATION.update();
}

void Rutina_Temperatura(){
  Serial.println(F("========== TEMPERATURE SENSORS =========="));

  Sensor_DHT22_0.readValueSensor();
  Serial.print(F("Value of sensor DHT22_0: "));
  Serial.println(Sensor_DHT22_0.getValueSensor(Sensor::DHT_22_TEMPERATURE));
  if(!NODO_TEMPERATURE.addValue(Sensor_DHT22_0.getValueSensor(Sensor::DHT_22_TEMPERATURE))){
      Serial.println(F("- Error de lectura de temperatura en el sensor DHT22_0"));
  }

  Sensor_DHT22_1.readValueSensor();
  Serial.print(F("Value of sensor DHT22_1: "));
  Serial.println(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_TEMPERATURE));
  if(!NODO_TEMPERATURE.addValue(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_TEMPERATURE))){
      Serial.println(F("- Error de lectura de temperatura en el sensor DHT22_1"));
  }

  Sensor_DHT22_2.readValueSensor();
  Serial.print(F("Value of sensor DHT22_2: "));
  Serial.println(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_TEMPERATURE));
  if(!NODO_TEMPERATURE.addValue(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_TEMPERATURE))){
      Serial.println(F("- Error de lectura de temperatura en el sensor DHT22_2"));
  }

  Sensor_DHT11_0.readValueSensor();
  Serial.print(F("Value of sensor DHT11_0: "));
  Serial.println(Sensor_DHT11_0.getValueSensor(Sensor::DHT_11_TEMPERATURE));
  if(!NODO_TEMPERATURE.addValue(Sensor_DHT11_0.getValueSensor(Sensor::DHT_11_TEMPERATURE))){
      Serial.println(F("- Error de lectura de temperatura en el sensor DHT11_0"));
  }

  Sensor_DHT11_1.readValueSensor();
  Serial.print(F("Value of sensor DHT11_1: "));
  Serial.println(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_TEMPERATURE));
  if(!NODO_TEMPERATURE.addValue(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_TEMPERATURE))){
      Serial.println(F("- Error de lectura de temperatura en el sensor DHT11_1"));
  }

  Sensor_DHT11_2.readValueSensor();
  Serial.print(F("Value of sensor DHT11_2: "));
  Serial.println(Sensor_DHT11_2.getValueSensor(Sensor::DHT_11_TEMPERATURE));
  if(!NODO_TEMPERATURE.addValue(Sensor_DHT11_2.getValueSensor(Sensor::DHT_11_TEMPERATURE))){
      Serial.println(F("- Error de lectura de temperatura en el sensor DHT11_2"));
  }


  // Serial.print(F("+ Promedio de temperatura: " ));
  // NODO_TEMPERATURE.CalculateAvarageValue();
  // Serial.println(NODO_TEMPERATURE.getAvarage());
  // Serial.print(F("\tValor maximo de temperatura: "));
  // Serial.println(NODO_TEMPERATURE.getMaxValue());
  // Serial.print(F("\tValor minimo de temperatura: ")); 
  // Serial.println(NODO_TEMPERATURE.getMinValue());
  // NODO_TEMPERATURE.resetCounterAvg();
  // Serial.println();

  //Serial.println(RUTINA_TEMP.elapsed());
}

void Rutina_Humedad(){
  Serial.println(F("========== HUMIDITY SENSORS =========="));
  Sensor_DHT22_0.readValueSensor();
  Serial.print(F("Value of sensor DHT22_0: "));
  Serial.println(Sensor_DHT22_0.getValueSensor(Sensor::DHT_22_HUMIDITY));
  if(!NODO_HUMIDITY.addValue(Sensor_DHT22_0.getValueSensor(Sensor::DHT_22_HUMIDITY))){
    Serial.println(F("- Error en la lectura de la humedad en el sensor DHT22_0"));
  }

  Sensor_DHT22_1.readValueSensor();
  Serial.print(F("Value of sensor DHT22_1: "));
  Serial.println(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_HUMIDITY));
  if(!NODO_HUMIDITY.addValue(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_HUMIDITY))){
    Serial.println(F("- Error en la lectura de la humedad en el sensor DHT22_1"));
  }
  
  Sensor_DHT22_2.readValueSensor();
  Serial.print(F("Value of sensor DHT22_2: "));
  Serial.println(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_HUMIDITY));
  if(!NODO_HUMIDITY.addValue(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_HUMIDITY))){
    Serial.println(F("- Error en la lectura de la humedad en el sensor DHT22_2"));
  }



  Sensor_DHT11_0.readValueSensor();
  Serial.print(F("Value of sensor DHT11_0: "));
  Serial.println(Sensor_DHT11_0.getValueSensor(Sensor::DHT_11_HUMIDITY));
  if(!NODO_HUMIDITY.addValue(Sensor_DHT11_0.getValueSensor(Sensor::DHT_11_HUMIDITY))){
    Serial.println(F("- Error en la lectura de la humedad en el sensor DHT11_0"));
  }

  Sensor_DHT11_1.readValueSensor();
  Serial.print(F("Value of sensor DHT11_1: "));
  Serial.println(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_HUMIDITY));
  if(!NODO_HUMIDITY.addValue(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_HUMIDITY))){
    Serial.println(F("- Error en la lectura de la humedad en el sensor DHT11_1"));
  }

  Sensor_DHT11_2.readValueSensor();
  Serial.print(F("Value of sensor DHT11_2: "));
  Serial.println(Sensor_DHT11_2.getValueSensor(Sensor::DHT_11_HUMIDITY));
  if(!NODO_HUMIDITY.addValue(Sensor_DHT11_2.getValueSensor(Sensor::DHT_11_HUMIDITY))){
    Serial.println(F("- Error en la lectura de la humedad en el sensor DHT11_2"));
  }

  // Serial.print(F("+ Promedio de humedad: "));
  // NODO_HUMIDITY.CalculateAvarageValue();
  // Serial.println(NODO_HUMIDITY.getAvarage());
  // Serial.print(F("\tValor maximo de humedad: "));
  // Serial.println(NODO_HUMIDITY.getMaxValue());
  // Serial.print(F("\tValor minimo de humedad: ")); 
  // Serial.println(NODO_HUMIDITY.getMinValue());
  // NODO_HUMIDITY.resetCounterAvg();
  // Serial.println();

  //Serial.println(Sensor_DHT11_2.getValueSensor(Sensor::DHT_11_HUMIDITY));
  //Serial.println(Sensor_DHT11_0.getValueSensor(Sensor::DHT_11_HUMIDITY));
  //Serial.println(RUTINA_HUM.elapsed());
}


void Rutina_Gases_Inflamables(){
  Serial.println(F("========== FLAMABLE SENSORS =========="));

  SensorGasMQ2_0.readValueSensor();
  Serial.print(F("Value of sensor MQ2_0: "));
  Serial.println(SensorGasMQ2_0.getValueSensor());
  if(!NODO_FLAMMABLE.addValue(SensorGasMQ2_0.getValueSensor())){
    Serial.println(F("- Error en la lectura de gases inflamables en el sensor MQ2_0"));
  }

  SensorGasMQ2_1.readValueSensor();
  Serial.print(F("Value of sensor MQ2_1: "));
  Serial.println(SensorGasMQ2_1.getValueSensor());
  if(!NODO_FLAMMABLE.addValue(SensorGasMQ2_1.getValueSensor())){
    Serial.println(F("- Error en la lectura de gases inflamables en el sensor MQ_2_1"));
  }
  
  // Serial.print(F("+ Promedio de valor de deteccion de gases inflamables: "));
  // NODO_FLAMMABLE.CalculateAvarageValue();
  // Serial.println(NODO_FLAMMABLE.getAvarage());
  // NODO_HUMIDITY.resetCounterAvg();
}

void Rutina_Gases_Toxicos(){
  Serial.println(F("========== TOXIC SENSORS =========="));

  SensorGasMQ135_0.readValueSensor();
  Serial.print(F("Value of sensor MQ135_0: "));
  Serial.println(SensorGasMQ135_0.getValueSensor());
  if(!NODO_TOXIC.addValue(SensorGasMQ135_0.getValueSensor())){
    Serial.println(F("- Error en la lectura de gases inflamables en el sensor MQ135_0"));
  }
  SensorGasMQ135_1.readValueSensor();
  Serial.print(F("Value of sensor MQ135_1: "));
  Serial.println(SensorGasMQ135_1.getValueSensor());
  if(!NODO_TOXIC.addValue(SensorGasMQ135_1.getValueSensor())){
    Serial.println(F("- Error en la lectura de gases inflamables en el sensor MQ135_1"));
  }

  SensorGasMQ135_2.readValueSensor();
  Serial.print(F("Value of sensor MQ135_2: "));
  Serial.println(SensorGasMQ135_2.getValueSensor());
  if(!NODO_TOXIC.addValue(SensorGasMQ135_2.getValueSensor())){
    Serial.println(F("- Error en la lectura de gases inflamables en el sensor MQ135_2"));
  }


  // Serial.print(F("+ Promedio de valor de deteccion de gases toxicos: "));
  // NODO_TOXIC.CalculateAvarageValue();
  // Serial.println(NODO_TOXIC.getAvarage());
  // NODO_TOXIC.resetCounterAvg();
}

void Rutina_Iluminacion(){
  Serial.println(F("========== LUMINOSITY SENSORS =========="));
  Serial.print("Value of sensor BH1750: ");
  SensorBH1750_1.readValueSensor();
  Serial.println(SensorBH1750_1.getValueSensor());
  if (NODO_LIGHT.addValue(SensorBH1750_1.getValueSensor()) == 0){
    Serial.println(F("- Error en la lectura de luminosidad del sensor BH_1735_1"));
  // }
  // Serial.print(F("+ Promedio del valor de luminosidad: "));
  // NODO_LIGHT.CalculateAvarageValue();
  // Serial.println(NODO_LIGHT.getAvarage());
  // NODO_LIGHT.resetCounterAvg();
  }
}


//Control elements routines
void TemperatureEvaluation(){
  TemperatureDownEvaluator.evaluateVariable();
  Serial.print(F("Temp down:"));
  Serial.println(TemperatureDownEvaluator.getAvgEvaluator());
  TemperatureUpEvaluator.evaluateVariable();
  Serial.print(F("Temp up:"));
  Serial.println(TemperatureUpEvaluator.getAvgEvaluator());
}

void HumidityEvaluation(){
  HumidityDownEvaluator.evaluateVariable();
  Serial.print(F("Hum down:"));
  Serial.println(HumidityDownEvaluator.getAvgEvaluator());
  HumidityUpEvaluator.evaluateVariable();
  Serial.print(F("Hum up:"));
  Serial.println(HumidityUpEvaluator.getAvgEvaluator());
}

void ToxicGasEvaluation(){
  ToxicUpEvaluator.evaluateVariable();
  Serial.print(F("Toxic up:"));
  Serial.println(ToxicUpEvaluator.getAvgEvaluator());
}

void FlammableGasEvaluation(){
  FlammableUpEvaluator.evaluateVariable();
  Serial.print(F("Flammable up:"));
  Serial.println(FlammableUpEvaluator.getAvgEvaluator());
}

void LuminosityEvaluation(){
  LuminosityDownEvaluator.evaluateVariable();
  Serial.print(F("Lumm down:"));
  Serial.println(LuminosityDownEvaluator.getAvgEvaluator());
}

void SendPayload_I2C(){
  Serial.println(F("------------------------------- AVERAGES VALUES -------------------------------"));
  /*=================== TEMPERATURE ===================*/
  Serial.println(F("========== TEMPERATURE  =========="));
  Serial.println("=> Cantidad de lecturas: "+String(NODO_TEMPERATURE.getCounterSensor()));
  Serial.println("=> Suma de las lecturas: "+String(NODO_TEMPERATURE.getValues()));
  Serial.print(F("+ Promedio de temperatura " ));
  NODO_TEMPERATURE.CalculateAvarageValue();
  Serial.println(NODO_TEMPERATURE.getAvarage());
  Serial.print(F("\tValor maximo de temperatura: "));
  Serial.println(NODO_TEMPERATURE.getMaxValue());
  Serial.print(F("\tValor minimo de temperatura: ")); 
  Serial.println(NODO_TEMPERATURE.getMinValue());
  NODO_TEMPERATURE.resetCounterAvg();
  lpp.addTemperature(1, NODO_TEMPERATURE.getAvarage());
  Serial.println();
  /*=================== HUMIDITY ===================*/
  Serial.println(F("========== HUMIDITY  =========="));
  Serial.println("=> Cantidad de lecturas: "+String(NODO_HUMIDITY.getCounterSensor()));
  Serial.println("=> Suma de las lecturas: "+String(NODO_HUMIDITY.getValues()));
  Serial.print(F("+ Promedio de humedad: "));
  NODO_HUMIDITY.CalculateAvarageValue();
  Serial.println(NODO_HUMIDITY.getAvarage());
  Serial.print(F("\tValor maximo de humedad: "));
  Serial.println(NODO_HUMIDITY.getMaxValue());
  Serial.print(F("\tValor minimo de humedad: ")); 
  Serial.println(NODO_HUMIDITY.getMinValue());
  NODO_HUMIDITY.resetCounterAvg();
  lpp.addRelativeHumidity(1,int(NODO_HUMIDITY.getAvarage())/2);
  Serial.println();
  /*=================== LUMINOSITY ===================*/
  Serial.println(F("========== LUMINOSITY  =========="));
  Serial.println("=> Cantidad de lecturas: "+String(NODO_LIGHT.getCounterSensor()));
  Serial.println("=> Suma de las lecturas: "+String(NODO_LIGHT.getValues()));
  Serial.print(F("+ Promedio del valor de luminosidad: "));
  NODO_LIGHT.CalculateAvarageValue();
  Serial.println(NODO_LIGHT.getAvarage());
  NODO_LIGHT.resetCounterAvg();
  lpp.addLuminosity(1, NODO_LIGHT.getAvarage());
  Serial.println();
  /*=================== TOXIC ===================*/
  Serial.println(F("========== TOXIC  =========="));
  Serial.println("=> Cantidad de lecturas: "+String(NODO_TOXIC.getCounterSensor()));
  Serial.println("=> Suma de las lecturas: "+String(NODO_TOXIC.getValues()));
  Serial.print(F("+ Promedio de valor de deteccion de gases toxicos: "));
  NODO_TOXIC.CalculateAvarageValue();
  Serial.println(NODO_TOXIC.getAvarage());
  NODO_TOXIC.resetCounterAvg();
  lpp.addGasToxic(1, NODO_TOXIC.getAvarage());
  Serial.println();
  /*=================== FLAMABLE ===================*/
  Serial.println(F("========== FLAMABLE  =========="));
  Serial.println("=> Cantidad de lecturas: "+String(NODO_FLAMMABLE.getCounterSensor()));
  Serial.println("=> Suma de las lecturas: "+String(NODO_FLAMMABLE.getValues()));
  Serial.print(F("+ Promedio de valor de deteccion de gases inflamables: "));
  NODO_FLAMMABLE.CalculateAvarageValue();
  Serial.println(NODO_FLAMMABLE.getAvarage());
  NODO_FLAMMABLE.resetCounterAvg();
  lpp.addGasFlamable(1, NODO_FLAMMABLE.getAvarage());
  Serial.println();
  /*=================== ENVIO DE DATOS POR I2C ===================*/
  sprintf(payloadString,"Send Payload I2C: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                          lpp.getBuffer()[0],lpp.getBuffer()[1],
                          lpp.getBuffer()[2],lpp.getBuffer()[3],
                          lpp.getBuffer()[4],lpp.getBuffer()[5],
                          lpp.getBuffer()[6],lpp.getBuffer()[7],
                          lpp.getBuffer()[8]);
  Serial.println(payloadString);
  lpp.reset();
}
