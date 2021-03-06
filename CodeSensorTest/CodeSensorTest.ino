#include <Ticker.h>
#include <DHT.h>
#include <DHT_U.h>
#include "Sensores_Proyecto.h"
#include "TTN_CayenneLPP.h"

TTN_CayenneLPP lpp;

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
// void Elementos_Control();

//DEFINICION DE LOS TICKERS QUE EJECUTARAN LA LECTURA DE LOS SENSORES EN UN TIEMPO DETERMINADO
Ticker RUTINA_TEMP(Rutina_Temperatura, 14000); //En micros segundos
Ticker RUTINA_HUM(Rutina_Humedad, 9000); //En micros segundos
Ticker RUTINA_MQ135(Rutina_Gases_Toxicos, 11000); //En micros segundos
Ticker RUTINA_MQ2(Rutina_Gases_Inflamables, 19000); //En micros segundos
Ticker RUTINA_LUZ(Rutina_Iluminacion, 29000); //En micros segundos

//TickTwo para el envio de la informacion
Ticker RUTINA_ENVIO_I2C(SendPayload_I2C, 60000);

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
  RUTINA_TEMP.start();
  RUTINA_HUM.start();
  RUTINA_MQ135.start();
  RUTINA_MQ2.start();
  RUTINA_LUZ.start();
  RUTINA_ENVIO_I2C.start();
  // RUTINA_RELAY_1.start();
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
  // RUTINA_RELAY_1.update();
  // if(RUTINA_RELAY_1.counter() == 1) RUTINA_RELAY_1.interval(5000);
}

void Rutina_Temperatura(){
  Serial.println(F("========== TEMPERATURE SENSORS =========="));
  Sensor_DHT22_1.readValueSensor();
  Serial.print(F("Value of sensor DHT22_1: "));
  Serial.println(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_TEMPERATURE));
  if(!NODO_TEMPERATURE.addValue(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_TEMPERATURE))){
      Serial.println(F("- Error de lectura de temperatura en el sensor DHT22_1"));
  }

  Sensor_DHT11_1.readValueSensor();
  Serial.print(F("Value of sensor DHT11_1: "));
  Serial.println(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_TEMPERATURE));
  if(!NODO_TEMPERATURE.addValue(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_TEMPERATURE))){
      Serial.println(F("- Error de lectura de temperatura en el sensor DHT11_1"));
  }

  Sensor_DHT22_2.readValueSensor();
  Serial.print(F("Value of sensor DHT22_2: "));
  Serial.println(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_TEMPERATURE));
  if(!NODO_TEMPERATURE.addValue(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_TEMPERATURE))){
      Serial.println(F("- Error de lectura de temperatura en el sensor DHT22_2"));
  }

  Sensor_DHT22_3.readValueSensor();
  Serial.print(F("Value of sensor DHT22_3: "));
  Serial.println(Sensor_DHT22_3.getValueSensor(Sensor::DHT_22_TEMPERATURE));
  if(!NODO_TEMPERATURE.addValue(Sensor_DHT22_3.getValueSensor(Sensor::DHT_22_TEMPERATURE))){
      Serial.println(F("- Error de lectura de temperatura en el sensor DHT22_3"));
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
  Sensor_DHT22_1.readValueSensor();
  Serial.print(F("Value of sensor DHT22_1: "));
  Serial.println(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_HUMIDITY));
  if(!NODO_HUMIDITY.addValue(Sensor_DHT22_1.getValueSensor(Sensor::DHT_22_HUMIDITY))){
    Serial.println(F("- Error en la lectura de la humedad en el sensor DHT22_1"));
  }

  Sensor_DHT11_1.readValueSensor();
  Serial.print(F("Value of sensor DHT11_1: "));
  Serial.println(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_HUMIDITY));
  if(!NODO_HUMIDITY.addValue(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_HUMIDITY))){
    Serial.println(F("- Error en la lectura de la humedad en el sensor DHT11_1"));
  }

  Sensor_DHT22_2.readValueSensor();
  Serial.print(F("Value of sensor DHT22_2: "));
  Serial.println(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_HUMIDITY));
  if(!NODO_HUMIDITY.addValue(Sensor_DHT22_2.getValueSensor(Sensor::DHT_22_HUMIDITY))){
    Serial.println(F("- Error en la lectura de la humedad en el sensor DHT22_2"));
  }

  Sensor_DHT22_3.readValueSensor();
  Serial.print(F("Value of sensor DHT22_3: "));
  Serial.println(Sensor_DHT22_3.getValueSensor(Sensor::DHT_22_HUMIDITY));
  if(!NODO_HUMIDITY.addValue(Sensor_DHT22_3.getValueSensor(Sensor::DHT_22_HUMIDITY))){
    Serial.println(F("- Error en la lectura de la humedad en el sensor DHT22_3"));
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
  //Serial.println(Sensor_DHT11_1.getValueSensor(Sensor::DHT_11_HUMIDITY));
  //Serial.println(RUTINA_HUM.elapsed());
}


void Rutina_Gases_Inflamables(){
  Serial.println(F("========== FLAMABLE SENSORS =========="));
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

void SendPayload_I2C(){
  Serial.println(F("========== AVERAGES VALUES =========="));
  /*=================== TEMPERATURE ===================*/
  Serial.println("=> Cantidad de lecturas: "+String(NODO_TEMPERATURE.getCounterSensor()));
  Serial.println("=> Suma de las lecturas: "+String(NODO_TEMPERATURE.getValues()));
  Serial.print(F("+ Promedio de temperatura: " ));
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
  Serial.println("=> Cantidad de lecturas: "+String(NODO_LIGHT.getCounterSensor()));
  Serial.println("=> Suma de las lecturas: "+String(NODO_LIGHT.getValues()));
  Serial.print(F("+ Promedio del valor de luminosidad: "));
  NODO_LIGHT.CalculateAvarageValue();
  Serial.println(NODO_LIGHT.getAvarage());
  NODO_LIGHT.resetCounterAvg();
  lpp.addLuminosity(1, NODO_LIGHT.getAvarage());
  /*=================== TOXIC ===================*/
  Serial.println("=> Cantidad de lecturas: "+String(NODO_TOXIC.getCounterSensor()));
  Serial.println("=> Suma de las lecturas: "+String(NODO_TOXIC.getValues()));
  Serial.print(F("+ Promedio de valor de deteccion de gases toxicos: "));
  NODO_TOXIC.CalculateAvarageValue();
  Serial.println(NODO_TOXIC.getAvarage());
  NODO_TOXIC.resetCounterAvg();
  lpp.addGasToxic(1, NODO_TOXIC.getAvarage());
  /*=================== FLAMABLE ===================*/
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