#include <Wire.h>
#include "Arduino.h"
#include "heltec.h"
#include <TickTwo.h>
#include <TTN_esp32.h>

/*  OTAA Activation   */
// const char* devEui = ""; // Change to TTN Device EUI
// const char* appEui = ""; // Change to TTN Application EUI
// const char* appKey = ""; // Chaneg to TTN Application Key


/* ABP ACTIVATION*/
const char* devAddr = ""; // Change to TTN Device Address
const char* nwkSKey = ""; // Change to TTN Network Session Key
const char* appSKey = ""; // Change to TTN Application Session Key

TTN_esp32 ttn ;

#if defined( Wireless_Stick_Lite )
  #include <Wire.h>
  #include "oled/SSD1306Wire.h"
  static const uint8_t SCL_OLED = 15;
  static const uint8_t SDA_OLED = 4;
  // static const uint8_t BYTE_CONTROL_BEGIN = 1;
  // static const uint8_t BYTE CONTROL_END = 9;
#endif

void receivePayload();
void message(const uint8_t* payload, size_t size, int rssi);

  /* ======================================================================*/
  /*      Inicia leyendo (I2C) y enviando (loRa) lecturas cada minuto      */
  /* ======================================================================*/
TickTwo routineReceivePayload(receivePayload, 60000);

uint8_t BYTE_CONTROL_BEGIN = 1;
uint8_t BYTE_CONTROL_END = 9;
uint8_t* buffer;
//Arreglo de Bytes de prueba
const byte data[] = {1, 2, 3, 4, 5};
const size_t dataLength = sizeof(data);

int i = 0;
char payloadString[50] = {0};
byte byteReceivedEnd, byteReceivedInit;

void setup() {
  Heltec.begin(false /*Display Enable*/, true /*LoRa Disable*/, true /*Serial Enable*/);
  ttn.begin();
  ttn.onMessage(message); // declare callback function when is downlink from server
  /* =========ABP Activation==========*/
  ttn.personalize(devAddr, nwkSKey, appSKey);
  /* =========OTAA Activation==========*/
  // ttn.join(devEui, appEui, appKey);
  // Serial.print("Joining TTN ");
  // while (!ttn.isJoined())
  // {
  //     Serial.print(".");
  //     delay(500);
  // }
  // Serial.println("\njoined !");
  /* ===================*/
  ttn.showStatus();
  Wire.begin(SDA,SCL);
  Wire.setClock(100000);
  Serial.println(F("Iniciando..."));
  routineReceivePayload.start();
}

void loop() {
  routineReceivePayload.update();
  /* =============================================================*/
  /*      Pasado la lectura (I2C) y envío (loRa) de 5 paquetes    */
  /*      Se ajusta para leer/enviar cada 5 minutos               */
  /* =============================================================*/
  if(routineReceivePayload.counter() == 5){
    routineReceivePayload.interval(60000*5);
  }
  /* ===========================================================================*/
  /*      Pasado la lectura (I2C) y envío (loRa) de 4 paquetes cada 5 minutos   */
  /*      En el contador se suman las 5 paquetes ya pasadas mas las 4 nuevas    */
  /*      Se ajusta para leer/enviar cada 20 minutos                            */
  /* ===========================================================================*/
  if(routineReceivePayload.counter() == 9){
    routineReceivePayload.interval(60000*20);
  }
}


void receivePayload(){
  Wire.beginTransmission(23);
  Wire.write('S');
  Wire.endTransmission();

  Wire.requestFrom(23, 1);
  
  byte sizeBuffer = Wire.read();
  buffer = (uint8_t*)malloc(sizeBuffer);
  Serial.println(F("===================================================="));
  Serial.println("Cantidad del Buffer a recibir: "+ String(sizeBuffer));
  Wire.requestFrom(23, (int)sizeBuffer); 

  byteReceivedInit = Wire.read();
  Serial.println("Byte de control inicial: "+ String(byteReceivedInit));
  
  if(byteReceivedInit == BYTE_CONTROL_BEGIN){
    while (Wire.available()) { 
      byte byteReceived = Wire.read();
      if(i < 9){
        buffer[i] = byteReceived;
        Serial.println("Byte["+String(i)+"]: "+String(byteReceived));         // print the character
        // Serial.print(buffer[1]);
      }
      if(i == 9){
        byteReceivedEnd = byteReceived;
        Serial.println("Byte de control final: "+ String(byteReceivedEnd));
      }
      i++;
    }
    if(byteReceivedEnd == BYTE_CONTROL_END){
      Serial.print(F("Imprimiendo buffer verificado: "));
      // for(int j=0;j<i;j++){
      //   Serial.print(buffer[j], HEX);
      // }
      sprintf(payloadString,"%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",byteReceivedInit,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8], byteReceivedEnd);
      Serial.println(F(payloadString));
      Serial.println(F("===================================================="));
      /* ===========Envio a TTN============ */ 
      if (ttn.sendBytes(buffer, (sizeBuffer-2))){
        Serial.println("Size buffer sending: " + String(sizeBuffer-2));
        Serial.printf("=> Payload sending to TTN: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8]);
      }
      /* ======================= */ 
      Wire.beginTransmission(23);
      Wire.write('R');
      Wire.write(dataLength);
      Wire.write(data,dataLength);
      Wire.endTransmission();
      /* ======================= */ 
    }
    else{
      Serial.println("Mensaje corrupto en su Byte final: " + String(byteReceivedEnd));
    }
    i = 0;
    free(buffer);
  }
  else{
    Serial.println("Mensaje corrupto en su Byte inicial: " + String(byteReceivedInit));
  }
  // }
}


void message(const uint8_t* payload, size_t size, int rssi){
  Serial.println(F("Mensaje recibido: "));
  Serial.print("Recibido " + String(size) + " bytes RSSI= " + String(rssi) + "dB");
  for (int i = 0; i < size; i++)
  {
    Serial.print(" " + String(payload[i])+" ");
    // Serial.write(payload[i]);
  }
  // Wire.beginTransmission(23);
  // Wire.write('R');
  // Wire.write(payload,size);
  // Wire.endTransmission();
  Serial.println();
}
