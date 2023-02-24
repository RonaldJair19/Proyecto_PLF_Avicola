#include <Ticker.h>
#include <DHT.h>
#include <DHT_U.h>
#include "Sensores_Proyecto.h"
#include "TTN_CayenneLPP.h"
#include <Wire.h>
TTN_CayenneLPP lpp;

char payloadString[50] = {0};
char payloadStringReceived[50] = {0};
bool S = false;

// byte bufferReceived[4];
uint8_t* bufferReceived;
uint8_t i = 0;

//Pin definitions
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

//Instances of objects referring to variables
NodeSensor NODO_TEMPERATURE(NodeSensor::VARIABLE_TEMPERATURE);
NodeSensor NODO_HUMIDITY(NodeSensor::VARIABLE_HUMIDITY);
NodeSensor NODO_TOXIC(NodeSensor::VARIABLE_TOXIC);
NodeSensor NODO_FLAMMABLE(NodeSensor::VARIABLE_FLAMMABLE);
NodeSensor NODO_LIGHT(NodeSensor::VARIABLE_LIGHT);

//Definitions of reference values for sensors
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

//Sensor sampling routines per monitored variable
void RoutineTemperature();
void RoutineHumidity();
void RoutineFlammableGas();
void RoutineToxicGas();
void RoutineLighting();

void AddAveragesLPP();
void RequestingEvent();
void ReceptionEvent();

//Monitoring routines for actuator activation
void TemperatureEvaluation();
void HumidityEvaluation();
void ToxicGasEvaluation();
void FlammableGasEvaluation();
void LuminosityEvaluation();


//Definition of threads for the execution of sampling for sensors and actuators
//Ticker for sensors
Ticker ROUTINE_TEMP(RoutineTemperature, 14000); //En milisegundos
Ticker ROUTINE_HUM(RoutineHumidity, 9000); //En milisegundos
Ticker ROUTINE_MQ135(RoutineToxicGas, 11000); //En milisegundos
Ticker ROUTINE_MQ2(RoutineFlammableGas, 19000); //En milisegundos
Ticker ROUTINE_LIGHT(RoutineLighting, 29000); //En milisegundos

//Tickers for actuators
Ticker ROUTINE_TEMPERATURE_EVALUATION(TemperatureEvaluation, 30000);
Ticker ROUTINE_HUMIDITY_EVALUATION(HumidityEvaluation, 32000);
Ticker ROUTINE_TOXIC_GAS_EVALUATION(ToxicGasEvaluation, 34000);
Ticker ROUTINE_FLAMMABLE_GAS_EVALUATION(FlammableGasEvaluation, 36000);
Ticker ROUTINE_LUMINOSITY_EVALUATION(LuminosityEvaluation, 38000);

//TickTwo para el envio de la informacion
// Ticker RUTINE_AVERAGES(AddAveragesLPP, 21000);

//Definition of sensor objects
//----------DHT22----------
Sensor Sensor_DHT22_0(DHT22_0_PIN, Sensor::DHT_22);
Sensor Sensor_DHT22_1(DHT22_1_PIN, Sensor::DHT_22);
Sensor Sensor_DHT22_2(DHT22_2_PIN, Sensor::DHT_22);

//----------DHT11----------
Sensor Sensor_DHT11_0(DHT11_0_PIN, Sensor::DHT_11);
Sensor Sensor_DHT11_1(DHT11_1_PIN, Sensor::DHT_11);
Sensor Sensor_DHT11_2(DHT11_2_PIN, Sensor::DHT_11);

//----------MQ_135----------
Sensor SensorGasMQ135_0(MQ135_0_PIN,Sensor::MQ_135);
Sensor SensorGasMQ135_1(MQ135_1_PIN,Sensor::MQ_135);
Sensor SensorGasMQ135_2(MQ135_2_PIN,Sensor::MQ_135);

//----------MQ_2----------
Sensor SensorGasMQ2_0(MQ2_0_PIN,Sensor::MQ_2);
Sensor SensorGasMQ2_1(MQ2_1_PIN,Sensor::MQ_2);

//----------BH1750----------
Sensor SensorBH1750_1(Sensor::BH_1735); // SCL ---> A5 | SDA ---> A4

//Definition of actuating objects (fan, humidity system, infrared bulbs, incandescent bulb)
ControlElement RelayFan(RELAY_FAN_PIN);
ControlElement RelayHumiditySystem(RELAY_HUMIDITY_SYSTEM_PIN);
ControlElement RelayInfraredBulbs(RELAY_INFRARED_BULBS_PIN);
ControlElement RelayIncandescentBulbs(RELAY_INCANDESCENT_BULBS_PIN);

//Definition of evaluator objects
Evaluator TemperatureDownEvaluator(&NODO_TEMPERATURE, &RelayFan, Evaluator::TEMPERATURE_DOWN);
Evaluator TemperatureUpEvaluator(&NODO_TEMPERATURE, &RelayInfraredBulbs, Evaluator::TEMPERATURE_UP);
Evaluator HumidityDownEvaluator(&NODO_HUMIDITY, &RelayHumiditySystem, Evaluator::HUMIDITY_DOW);
Evaluator HumidityUpEvaluator(&NODO_HUMIDITY, &RelayInfraredBulbs, Evaluator::HUMIDITY_UP);
Evaluator ToxicUpEvaluator(&NODO_TOXIC, &RelayFan, Evaluator::TOXIC_UP);
Evaluator FlammableUpEvaluator(&NODO_FLAMMABLE, &RelayFan, Evaluator::FLAMMABLE_UP);
Evaluator LuminosityDownEvaluator(&NODO_LIGHT, &RelayIncandescentBulbs, Evaluator::LUMINOSITY_DOWN);



void setup() {
  Serial.begin(9600);
  // Serial.println(F("Iniciando..."));
  Wire.begin(23);                // join the i2c bus with address #23
  Wire.onRequest(RequestingEvent); // record data request event
  Wire.onReceive(ReceptionEvent); // record data reception event
  //Setting values for automation 
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


  ROUTINE_TEMP.start();
  ROUTINE_HUM.start();
  ROUTINE_MQ135.start();
  ROUTINE_MQ2.start();
  ROUTINE_LIGHT.start();

  ROUTINE_TEMPERATURE_EVALUATION.start();
  ROUTINE_HUMIDITY_EVALUATION.start();
  ROUTINE_TOXIC_GAS_EVALUATION.start();
  ROUTINE_FLAMMABLE_GAS_EVALUATION.start();
  ROUTINE_LUMINOSITY_EVALUATION.start();
  Serial.println(F("Iniciando..."));

  // RUTINE_AVERAGES.start();
  // RUTINA_RELAY_1.start();
}

void loop() {
  ROUTINE_TEMP.update();
  ROUTINE_HUM.update();
  ROUTINE_MQ135.update();
  ROUTINE_MQ2.update();
  ROUTINE_LIGHT.update();
  if(ROUTINE_TEMP.counter() == 20){
    ROUTINE_TEMP.interval(59000);
  }
  if(ROUTINE_HUM.counter() == 30){
    ROUTINE_HUM.interval(58000);
  }
  if(ROUTINE_MQ135.counter() == 25){
    ROUTINE_MQ135.interval(57000);
  }
  if(ROUTINE_MQ2.counter() == 15){
    ROUTINE_MQ2.interval(56000);
  }
  if(ROUTINE_LIGHT.counter() == 10){
    ROUTINE_LIGHT.interval(55000);
  }
  // RUTINE_AVERAGES.update();
  // RUTINA_ENVIO_I2C.update();
  // if(RUTINA_ENVIO_I2C.counter() == 4){
  //   //Serial.println(F("Cambio del intervalo"));
  //   RUTINA_ENVIO_I2C.interval(50000);
  //   ROUTINE_TEMP.interval(30000);
  //   ROUTINE_HUM.interval(35000);
  //   ROUTINE_MQ135.interval(25000);
  //   ROUTINE_MQ2.interval(20000);
  //   ROUTINE_LIGHT.interval(27000);
  // }
  ROUTINE_TEMPERATURE_EVALUATION.update();
  ROUTINE_HUMIDITY_EVALUATION.update();
  ROUTINE_TOXIC_GAS_EVALUATION.update();
  ROUTINE_FLAMMABLE_GAS_EVALUATION.update();
  ROUTINE_LUMINOSITY_EVALUATION.update();
}

void RoutineTemperature(){
  Sensor_DHT22_0.readValueSensor();
  NODO_TEMPERATURE.addValue(Sensor_DHT22_0.getValueSensor(Sensor::DHT_22_TEMPERATURE));

  Sensor_DHT22_1.readValueSensor();
  NODO_TEMPERATURE.addValue(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_TEMPERATURE));

  Sensor_DHT22_2.readValueSensor();
  NODO_TEMPERATURE.addValue(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_TEMPERATURE));

  Sensor_DHT11_0.readValueSensor();
  NODO_TEMPERATURE.addValue(Sensor_DHT11_0.getValueSensor(Sensor::DHT_11_TEMPERATURE));

  Sensor_DHT11_1.readValueSensor();
  NODO_TEMPERATURE.addValue(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_TEMPERATURE));

  Sensor_DHT11_2.readValueSensor();
  NODO_TEMPERATURE.addValue(Sensor_DHT11_2.getValueSensor(Sensor::DHT_11_TEMPERATURE));
}

void RoutineHumidity(){
  Sensor_DHT22_0.readValueSensor();
  NODO_HUMIDITY.addValue(Sensor_DHT22_0.getValueSensor(Sensor::DHT_22_HUMIDITY));

  Sensor_DHT22_1.readValueSensor();
  NODO_HUMIDITY.addValue(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_HUMIDITY));
  
  Sensor_DHT22_2.readValueSensor();
  NODO_HUMIDITY.addValue(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_HUMIDITY));

  Sensor_DHT11_0.readValueSensor();
  NODO_HUMIDITY.addValue(Sensor_DHT11_0.getValueSensor(Sensor::DHT_11_HUMIDITY));

  Sensor_DHT11_1.readValueSensor();
  NODO_HUMIDITY.addValue(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_HUMIDITY));

  Sensor_DHT11_2.readValueSensor();
  NODO_HUMIDITY.addValue(Sensor_DHT11_2.getValueSensor(Sensor::DHT_11_HUMIDITY));
}


void RoutineFlammableGas(){
  SensorGasMQ2_0.readValueSensor();
  NODO_FLAMMABLE.addValue(SensorGasMQ2_0.getValueSensor());

  SensorGasMQ2_1.readValueSensor();
  NODO_FLAMMABLE.addValue(SensorGasMQ2_1.getValueSensor());
}

void RoutineToxicGas(){
  SensorGasMQ135_0.readValueSensor();
  NODO_TOXIC.addValue(SensorGasMQ135_0.getValueSensor());

  SensorGasMQ135_1.readValueSensor();
  NODO_TOXIC.addValue(SensorGasMQ135_1.getValueSensor());

  SensorGasMQ135_2.readValueSensor();
  NODO_TOXIC.addValue(SensorGasMQ135_2.getValueSensor());
}

void RoutineLighting(){
  SensorBH1750_1.readValueSensor();
  NODO_LIGHT.addValue(SensorBH1750_1.getValueSensor());
}

//Control elements routines
void TemperatureEvaluation(){
  TemperatureDownEvaluator.evaluateVariable();
  TemperatureUpEvaluator.evaluateVariable();
}

void HumidityEvaluation(){
  HumidityDownEvaluator.evaluateVariable();
  HumidityUpEvaluator.evaluateVariable();
}

void ToxicGasEvaluation(){
  ToxicUpEvaluator.evaluateVariable();
}

void FlammableGasEvaluation(){
  FlammableUpEvaluator.evaluateVariable();
}

void LuminosityEvaluation(){
  LuminosityDownEvaluator.evaluateVariable();
}


//Calculate averages of variables per I2C request

void AddAveragesLPP(){
  NODO_TEMPERATURE.CalculateAvarageValue();
  NODO_TEMPERATURE.resetCounterAvg();
  lpp.addTemperature(1, NODO_TEMPERATURE.getAvarage());

  /*=================== HUMIDITY ===================*/
  NODO_HUMIDITY.CalculateAvarageValue();
  NODO_HUMIDITY.resetCounterAvg();
  lpp.addRelativeHumidity(1,int(NODO_HUMIDITY.getAvarage())/2);

  /*=================== LUMINOSITY ===================*/
  NODO_LIGHT.CalculateAvarageValue();
  NODO_LIGHT.resetCounterAvg();
  lpp.addLuminosity(1, NODO_LIGHT.getAvarage());

  /*=================== TOXIC ===================*/
  NODO_TOXIC.CalculateAvarageValue();
  NODO_TOXIC.resetCounterAvg();
  lpp.addGasToxic(1, NODO_TOXIC.getAvarage());

  /*=================== FLAMABLE ===================*/

  NODO_FLAMMABLE.CalculateAvarageValue();
  NODO_FLAMMABLE.resetCounterAvg();
  lpp.addGasFlamable(1, NODO_FLAMMABLE.getAvarage()); 
  /*=================== TEMPERATURE ===================*/

  // Serial.println();
  /*=================== ENVIO DE DATOS POR I2C ===================*/
  // sprintf(payloadString,"Send Payload I2C: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
  //                         lpp.getBuffer()[0],lpp.getBuffer()[1],
  //                         lpp.getBuffer()[2],lpp.getBuffer()[3],
  //                         lpp.getBuffer()[4],lpp.getBuffer()[5],
  //                         lpp.getBuffer()[6],lpp.getBuffer()[7],
  //                         lpp.getBuffer()[8]);
  // Serial.println(payloadString);
  // lpp.reset();
}

void ReceptionEvent(){
  //Agregar el Wire.available();
  switch (Wire.read())
  {
  case 'S':
      S = true;
      //Es mejor hacer una rutina para SendPayload() para evitar desborde de memoria
      AddAveragesLPP();
    break;
  case 'R':
      Serial.println(F("Recibiendo mensaje del Maestro: "));
      byte sizeBufferReceived = Wire.read();
      Serial.println("Cantidad de Bytes a recibir: "+String(sizeBufferReceived));
      bufferReceived = (uint8_t*)malloc(sizeBufferReceived);
      while(Wire.available()){
        byte byteReceived = Wire.read();
        Serial.println("Byte["+String(i)+"]: "+ String(byteReceived));
        bufferReceived[i] = byteReceived;
        i++;
      }
      // Serial.println();
      Serial.print(F("Payload recibido:"));
      sprintf(payloadStringReceived,"%02X %02X %02X %02X %02X",bufferReceived[0],bufferReceived[1],bufferReceived[2],bufferReceived[3],bufferReceived[4]);
      Serial.println(payloadStringReceived);
      Serial.println(F("============================================================="));
      free(bufferReceived);
      i=0;
    break;

  default:
    break;
  }
}

void RequestingEvent() {
  if( S == true ){
    Wire.write(lpp.getSize()+2); //Aquí es que se envía la cantidad de Bytes
    // Serial.println("Se envia cantidad de bytes!");
    // Wire.write(message.length()); //Aquí es que se envía la cantidad de Bytes
    S = false;
  }
  else{
    // Wire.println(message);
    Wire.write(1);
    Wire.write(lpp.getBuffer(),lpp.getSize());
    Wire.write(9);
    sprintf(payloadString,"%02X %02X %02X %02X %02X %02X %02X %02X %02X",lpp.getBuffer()[0],lpp.getBuffer()[1],lpp.getBuffer()[2],lpp.getBuffer()[3],lpp.getBuffer()[4],lpp.getBuffer()[5],lpp.getBuffer()[6],lpp.getBuffer()[7],lpp.getBuffer()[8]);
    Serial.println(payloadString);
    // Serial.println(lpp.getBuffer()[1]);
    lpp.reset();
  }
}
