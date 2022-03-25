#include <Ticker.h>
#include <DHT.h>
#include <DHT_U.h>
#include "Sensores_Proyecto.h"
#include "TTN_CayenneLPP.h"
#include <Wire.h>
TTN_CayenneLPP lpp;

bool S = false;
char payloadString[50] = {0};

//DEFINICION PINES SENSORES Y ACTUADORES
#define DHT22_PIN 4
#define DHT22_2_PIN 6
#define DHT22_3_PIN 8
#define DHT11_PIN 2
#define MQ2_1_PIN A2
#define MQ135_1_PIN A7
#define MQ135_2_PIN A1

// DEFINICION DE LOS OBJETOS NODO PARA CADA VARIABLE A MONITORIZAR
NodeSensor NODO_TEMPERATURE(NodeSensor::VARIABLE_TEMPERATURE);
NodeSensor NODO_HUMIDITY(NodeSensor::VARIABLE_HUMIDITY);
NodeSensor NODO_TOXIC(NodeSensor::VARIABLE_TOXIC);
NodeSensor NODO_FLAMMABLE(NodeSensor::VARIABLE_FLAMMABLE);
NodeSensor NODO_LIGHT(NodeSensor::VARIABLE_LIGHT);

//DEFINICION DE LOS VALORES DE REFERENCIA PARA LOS SENSORES
const int LIGHT_REFERENCE = 400;

const float TEMPERATURE_MIN_REFERENCE = 10.0;
const float TEMPERATURE_MAX_REFERENCE = 30.0;

const int HUMIDITY_MIN_REFERENCE = 10;
const int HUMIDITY_MAX_REFERENCE = 90;

const int FLAMMABLE_MAX_REFERENCE = 200;

const int TOXIC_MAX_REFERENCE = 900;

//DEFINICION DE LAS FUNCIONES PARA LEER DATOS DESDE LOS SENSORES Y OBTENER UN PROMEDIO
void Rutina_Temperatura();
void Rutina_Humedad();
void Rutina_Gases_Inflamables();
void Rutina_Gases_Toxicos();
void Rutina_Iluminacion();
void SendPayload_I2C();
void eventoSolicitud();
void eventoRecepcion();
// void Elementos_Control();

//DEFINICION DE LOS TICKERS QUE EJECUTARAN LA LECTURA DE LOS SENSORES EN UN TIEMPO DETERMINADO
Ticker RUTINA_TEMP(Rutina_Temperatura, 17000); //En micros segundos
Ticker RUTINA_HUM(Rutina_Humedad, 21000); //En micros segundos
Ticker RUTINA_MQ135(Rutina_Gases_Toxicos, 10000); //En micros segundos
Ticker RUTINA_MQ2(Rutina_Gases_Inflamables, 6000); //En micros segundos
Ticker RUTINA_LUZ(Rutina_Iluminacion, 10000); //En micros segundos

//TickTwo para el envio de la informacion
// Ticker RUTINA_ENVIO_I2C(SendPayload_I2C, 25000);

//DEFINICION DE TICKER PARA LOS ACTUADORES
// Ticker RUTINA_RELAY_1(Elementos_Control, 30000);

//DEFINICION DE LOS PINES Y TIPO DE SENSOR

//----------Para sensores KY001----------

//----------Para sensores DHT22----------
Sensor Sensor_DHT22_1(DHT22_PIN, Sensor::DHT_22);
Sensor Sensor_DHT22_2(DHT22_2_PIN, Sensor::DHT_22);
Sensor Sensor_DHT22_3(DHT22_3_PIN, Sensor::DHT_22);

//----------Para sensores DHT11----------
Sensor Sensor_DHT11_1(DHT11_PIN,Sensor::DHT_11);
// Sensor Sensor_DHT11_2(10,Sensor::DHT_11);

//----------Para sensores MQ_135----------
Sensor SensorGasMQ135_1(MQ135_1_PIN,Sensor::MQ_135);
Sensor SensorGasMQ135_2(MQ135_2_PIN,Sensor::MQ_135);

//----------Para sensores MQ_2----------
Sensor SensorGasMQ2_1(MQ2_1_PIN,Sensor::MQ_2);

//----------Para sensores BH1750----------
Sensor SensorBH1750_1(Sensor::BH_1735); // SCL ---> A5 | SDA ---> A4



//ELEMENTOS DE CONTROL
// Control_Element Relay_Abanicos(8);
// Control_Element Relay_Inflarojos(7);
// Control_Element Relay_Iluminacion(6);
// Control_Element Relay_Humificador(2);

void setup() {
  Serial.begin(9600);
  Serial.println(F("Iniciando..."));
  Wire.begin(23);                // unirse al bus i2c con la direccion #23
  Wire.onRequest(eventoSolicitud); // registrar evento de solicitud de datos
  Wire.onReceive(eventoRecepcion); // registrar evento de recepcion de datos
  RUTINA_TEMP.start();
  RUTINA_HUM.start();
  RUTINA_MQ135.start();
  RUTINA_MQ2.start();
  RUTINA_LUZ.start();
  // RUTINA_ENVIO_I2C.start();
  // RUTINA_RELAY_1.start();
}

void loop() {
  RUTINA_TEMP.update();
  RUTINA_HUM.update();
  RUTINA_MQ135.update();
  RUTINA_MQ2.update();
  RUTINA_LUZ.update();
  // RUTINA_ENVIO_I2C.update();
  // if(RUTINA_ENVIO_I2C.counter() == 4){
  //   //Serial.println(F("Cambio del intervalo"));
  //   RUTINA_ENVIO_I2C.interval(50000);
  //   RUTINA_TEMP.interval(30000);
  //   RUTINA_HUM.interval(35000);
  //   RUTINA_MQ135.interval(25000);
  //   RUTINA_MQ2.interval(20000);
  //   RUTINA_LUZ.interval(27000);
  // }
  // RUTINA_RELAY_1.update();
  // if(RUTINA_RELAY_1.counter() == 1) RUTINA_RELAY_1.interval(5000);
}

void Rutina_Temperatura(){
  Sensor_DHT22_1.readValueSensor();
  NODO_TEMPERATURE.addValue(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_TEMPERATURE));

  Sensor_DHT11_1.readValueSensor();
  NODO_TEMPERATURE.addValue(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_TEMPERATURE));

  Sensor_DHT22_2.readValueSensor();
  NODO_TEMPERATURE.addValue(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_TEMPERATURE));

  Sensor_DHT22_3.readValueSensor();
  NODO_TEMPERATURE.addValue(Sensor_DHT22_3.getValueSensor(Sensor::DHT_22_TEMPERATURE));
}

void Rutina_Humedad(){
  Sensor_DHT22_1.readValueSensor();
  NODO_HUMIDITY.addValue(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_HUMIDITY));

  Sensor_DHT11_1.readValueSensor();
  NODO_HUMIDITY.addValue(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_HUMIDITY));

  Sensor_DHT22_2.readValueSensor();
  NODO_HUMIDITY.addValue(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_HUMIDITY));

  Sensor_DHT22_3.readValueSensor();
  NODO_HUMIDITY.addValue(Sensor_DHT22_3.getValueSensor(Sensor::DHT_22_HUMIDITY));
}


void Rutina_Gases_Inflamables(){
  SensorGasMQ2_1.readValueSensor();
  NODO_FLAMMABLE.addValue(SensorGasMQ2_1.getValueSensor());
}

void Rutina_Gases_Toxicos(){
  SensorGasMQ135_1.readValueSensor();
  NODO_TOXIC.addValue(SensorGasMQ135_1.getValueSensor());

  SensorGasMQ135_2.readValueSensor();
  NODO_TOXIC.addValue(SensorGasMQ135_2.getValueSensor());
}

void Rutina_Iluminacion(){
  SensorBH1750_1.readValueSensor();
  NODO_LIGHT.addValue(SensorBH1750_1.getValueSensor());
}

void SendPayload_I2C(){
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

void eventoRecepcion(){
  //Agregar el Wire.available();
  switch (Wire.read())
  {
  case 'S':
      S = true;
      SendPayload_I2C();
      
    break;
  case 1:
    break;

  default:
    break;
  }
}

void eventoSolicitud() {
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
    // sprintf(payloadString,"%02X %02X %02X %02X %02X %02X %02X %02X %02X\n",lpp.getBuffer()[0],lpp.getBuffer()[1],lpp.getBuffer()[2],lpp.getBuffer()[3],lpp.getBuffer()[4],lpp.getBuffer()[5],lpp.getBuffer()[6],lpp.getBuffer()[7],lpp.getBuffer()[8]);
    // Serial.println(payloadString);
    // Serial.println(lpp.getBuffer()[1]);
    lpp.reset();
  }
}




// void Elementos_Control(){
//   //TEMPERATURA ALTA
//   if (NODO_TEMPERATURE.getAvarage() > TEMPERATURE_MAX_REFERENCE){
//     if (!Relay_Abanicos.getState()){
//       Relay_Abanicos.setOnElement();
//       Serial.println(F("\t - Abanicos encendidos"));
//     }
//     if(Relay_Inflarojos.getState()){
//       Relay_Inflarojos.setOffElement();
//       Serial.println(F("\t - Luces Inflarojas encendidas"));
//     }
//   }
//   //TEMPERATURA BAJA
//   if(NODO_TEMPERATURE.getAvarage() < TEMPERATURE_MIN_REFERENCE){
//     if(Relay_Abanicos.getState()){
//       Relay_Abanicos.setOffElement();
//       Serial.println(F("\t - Abanicos apagados"));
//     }
//     if(!Relay_Inflarojos.getState()){
//       Relay_Inflarojos.setOnElement();
//       Serial.println(F("\t - Luces inflarojas encendidas"));
//     }
//   }
//   //HUMEDAD RELATIVA BAJA
//   if (NODO_HUMIDITY.getAvarage() <= HUMIDITY_MIN_REFERENCE){
//     if (!Relay_Humificador.getState()){
//       Relay_Humificador.setOnElement();
//       Serial.println(F("\t - Humificador encendido"));
//     }
//   }

//   //HUMEDAD RELATIVA ALTA
//   if (NODO_HUMIDITY.getAvarage() >= HUMIDITY_MAX_REFERENCE){
//     if(Relay_Humificador.getState()){
//       Relay_Humificador.setOffElement();
//       Serial.println(F("\t - Humificador apagado"));
//     }
//     if(!Relay_Abanicos.getState()){
//       Relay_Abanicos.setOnElement();
//       Serial.println(F("\t - Abanicos encendido"));
//     }
//     if(!Relay_Inflarojos.getState()){
//       Relay_Inflarojos.setOnElement();
//       Serial.println(F("\t - Luces inflarojas encendidas"));
//     }      
//   }

//   //NIVEL DE GAS INFLAMABLE ALTO
//   if (NODO_FLAMMABLE.getAvarage() > FLAMMABLE_MAX_REFERENCE){
//     if(!Relay_Abanicos.getState()){
//       Relay_Abanicos.setOnElement();
//       Serial.println(F("\t - Abanicos encendidos [FLAMMABLE]"));
//     } 
//   }
  
//   //NIVEL DE GASES TOXICOS ALTO
//   if (NODO_TOXIC.getAvarage() > TOXIC_MAX_REFERENCE){
//     if(!Relay_Abanicos.getState())
//       Relay_Abanicos.setOnElement();
//       Serial.println(F("\t - Abanicos encendidos"));
//   }

//   //ILUMINACION BAJA
//   if (NODO_LIGHT.getAvarage() < LIGHT_REFERENCE){
//     if (!Relay_Iluminacion.getState()){
//       Relay_Iluminacion.setOnElement();
//       Serial.println(F("\t - Luces encendidas "));
//     }
//   }
//   else{
//     Relay_Iluminacion.setOffElement();
//     Serial.println(F("\t - Luces apagadas "));
//   }
  
// }