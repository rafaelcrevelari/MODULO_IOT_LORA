/**********************************************************************************************
 * MODULO END NODE LORA - Version 1.0.0
 * by Carlos Rafael Crevelari <rafaelcrevelari@gmail.com>
 * 
 * @file    MODULO_END_NODE_LORA.cpp
 * @date    20 Nov 2020
 *
 * Código desenvolvido para protótipo utilizado no trabalho de conclusão de curso entitulado
 * DESENVOLVIMENTO DE UM MÓDULO PROGRAMAVEL IOT COM TECNOLOGIA LORA
 * 
 * Realiza leitura de temperatura, umidade e pressao do sensor BME280 e envia valores via LoRa
 *
 * Desenvolvido com framework Arduino utilizando Arduino core support for STM32 disponivel em:
 * https://github.com/stm32duino/Arduino_Core_STM32
 * 
 * Utilizada biblioteca Arduino LoRa For STM32F103CB disponivel em:
 * https://github.com/armtronix/arduino-LoRa-STM32
 * 
 * Utilizada biblioteca Adafruit BME280 disponivel em:
 * https://github.com/adafruit/Adafruit_BME280_Library
 **********************************************************************************************/
 
#include <SPI.h>
#include <Wire.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LoRa_STM32.h>


/***************************************************************************
 *  DEFINES DE SOFTWARE 
 ***************************************************************************/
#define LED_ON  0
#define LED_OFF 1

/***************************************************************************
 *  VARIAVEIS GLOBAIS
 ***************************************************************************/
Adafruit_BME280 bme; // I2C
unsigned status;

char  data_buffer[128];
uint8_t aux_temperature = 0;
uint8_t aux_humidity = 0;
uint8_t count_send = 0;
uint16_t aux_pressure = 0;


void setup() 
{
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LED_OFF);

    //Inicializa sensor Bosch BME280
    status = bme.begin(0x76);

    //Inicializa modulo RFM95W na frequencia de 915Mhz
    if (!LoRa.begin(915E6)) 
    {
     while (1);
    }
}

void loop() 
{
  //Incrementa contador de envio
  if (count_send <= 100)
  {
    count_send++;
  }
  else
  {
    count_send = 0;
  }

  //Realiza leitura do sensor BME280
  aux_temperature = (uint8_t)bme.readTemperature();
  aux_humidity = (uint8_t)bme.readHumidity();
  aux_pressure = (uint16_t)(bme.readPressure() / 100.0F);

  //Formata string de envio
  snprintf(data_buffer,sizeof(data_buffer),"%02d%03d%05d%03d",aux_temperature,aux_humidity,aux_pressure,count_send);
  String LoRa_Data = String(data_buffer);

  //Envia pacote LoRa
  LoRa.beginPacket();
  LoRa.print(LoRa_Data);
  LoRa.endPacket();

  //Pisca led da placa Blue Pill para sinalizar envio
  digitalWrite(PC13, LED_ON);   
  delay(500);              
  digitalWrite(PC13, LED_OFF);   
  delay(500);  

  //Aguarda 15 segundos para proximo envio (1 seg do led + 14 seg de espera)
  delay(14000);
}
