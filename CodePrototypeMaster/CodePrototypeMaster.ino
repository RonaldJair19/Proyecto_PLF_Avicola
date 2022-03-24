//#include <Wire.h>
#include "Arduino.h"
#include "heltec.h"
#include <TickTwo.h>

#if defined( Wireless_Stick_Lite )
  #include <Wire.h>
  #include "oled/SSD1306Wire.h"
  static const uint8_t SCL_OLED = 15;
  static const uint8_t SDA_OLED = 4;
  // static const uint8_t BYTE_CONTROL_BEGIN = 1;
  // static const uint8_t BYTE CONTROL_END = 9;
#endif

void receivePayload();

TickTwo routineReceivePayload(receivePayload, 12000);

uint8_t BYTE_CONTROL_BEGIN = 1;
uint8_t BYTE_CONTROL_END = 9;
uint8_t* buffer;
int i = 0;
char payloadString[50] = {0};
byte byteReceivedEnd, byteReceivedInit;

void setup() {
  Heltec.begin(false /*Display Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
  // Wire1.begin(SDA_OLED, SCL_OLED);
  Wire.begin(SDA,SCL);
  Wire.setClock(100000);
  //Serial.begin(115200);
  Serial.println(F("Iniciando..."));
  routineReceivePayload.start();
  // Serial.begin(9600);  // configurar monitor serie a 9600
}

void loop() {
  routineReceivePayload.update();
}


void receivePayload(){
  Wire.beginTransmission(23);
  Wire.write('S');
  Wire.endTransmission();
  Wire.requestFrom(23, 1);
  byte sizeBuffer = Wire.read();
  
  // if (sizeBuffer == 9){
  buffer = (uint8_t*)malloc(sizeBuffer);
  Serial.println("Cantidad del Buffer a recibir: "+ String(sizeBuffer)+"\n");
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
      Serial.println();
      Serial.println(F("Imprimiendo buffer verificado: "));
      // for(int j=0;j<i;j++){
      //   Serial.print(buffer[j], HEX);
      // }
      sprintf(payloadString,"%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",byteReceivedInit,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8], byteReceivedEnd);
      Serial.println(F(payloadString));
      // i = 0;
      Wire.beginTransmission(23);
      Wire.write(1);
      Wire.endTransmission();
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
