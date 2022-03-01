//#include <Wire.h>
#include "Arduino.h"
#include "heltec.h"
#include <TickTwo.h>

#if defined( Wireless_Stick_Lite )
  #include <Wire.h>
  #include "oled/SSD1306Wire.h"
  static const uint8_t SCL_OLED = 15;
  static const uint8_t SDA_OLED = 4;
#endif

void receivePayload();

TickTwo routineReceivePayload(receivePayload, 5000);

uint8_t* buffer;
int i = 0;
char payloadString[50] = {0};

void setup() {
  Heltec.begin(false /*Display Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
  // Wire1.begin(SDA_OLED, SCL_OLED);
  Wire.begin(SDA,SCL);
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
  buffer = (uint8_t*)malloc(sizeBuffer);
  Serial.println("Cantidad Buffer: "+ String(sizeBuffer)+"\n");
  Wire.requestFrom(23, (int)sizeBuffer); 
  
  while (Wire.available()) { 
    byte byteReceived = Wire.read();
    buffer[i] = byteReceived;
    Serial.println("Byte["+String(i)+"]: "+String(byteReceived));         // print the character
    i++;
  }
  // Serial.print(buffer[1]);
  Serial.println();

  // delay(5000);
  Serial.println("Imprimiendo buffer: ");
  // for(int j=0;j<i;j++){
  //   Serial.print(buffer[j], HEX);
  // }
  sprintf(payloadString,"%02X %02X %02X %02X %02X %02X %02X %02X %02X\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7],buffer[8]);
  Serial.println(payloadString);
  i = 0;
  free(buffer);
}