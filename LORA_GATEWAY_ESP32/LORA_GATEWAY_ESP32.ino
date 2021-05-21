/**********************************************************************************************
 * LORA GATEWAY ESP32 - Version 1.0.0
 * by Carlos Rafael Crevelari <rafaelcrevelari@gmail.com>
 * 
 * @file    LORA_GATEWAY_ESP32.cpp
 * @date    20 Nov 2020
 *
 * Código desenvolvido para protótipo utilizado no trabalho de conclusão de curso entitulado
 * DESENVOLVIMENTO DE UM MÓDULO PROGRAMAVEL IOT COM TECNOLOGIA LORA
 * 
 * Verifica recepcao de pacotes LoRa pelo modulo RFM95W, formata dados para JSON e realiza post HTTPS para plataforma TagoIO
 *
 * Utiliado como base o projeto open source Medidor de Energia IoT desenvolvido por Eder Andrade, disponivel em
 * https://github.com/EderAndrade/Medidor-de-Energia-IoT
 * 
 * Utilizada biblioteca Arduino LoRa disponivel em:
 * https://github.com/sandeepmistry/arduino-LoRa
 * 
 **********************************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include <iostream>
#include <string> 

#include <LoRa.h>

/*
 *  DEFINES MUST BE HERE! ********************************************** 
 */
#define CORE_0    (unsigned char)0 
#define CORE_1    (unsigned char)1

#define WIFISSID  " "  
#define PASSWORD  " "   

#define IO_LED    (uint8_t) 2

//define the pins used by the transceiver module
#define ss      5
#define rst     15
#define dio0    4

//#define DEBUG_TAGO    1
/***********************************************************************/

/*
 *  FUNCTION PROTOTYPES MUST BE HERE! *********************************** 
 */
void prvSetupHardware (void);
void vPrintStr        (const    char *pcString);
void vPrintStrNum     (const    char *pcString, uint32_t uiValue);
void vPrintStrFloat   (const    char *pcString, float fValue);
void vPrintHex        (uint32_t uiValue);
void vTimerTask       (void     *pvParameters);
void vHttpTask        (void     *pvParameters);
void vSensorTask      (void     *pvParameters);
/***********************************************************************/

/*
 *  GLOBAL VARIABLES MUST BE HERE! ************************************** 
 */
/* Wi-Fi --------------------------------------------------------------*/
WiFiClient      client;
WiFiMulti       wifiMulti;

/* FreeRTOS -----------------------------------------------------------*/
portMUX_TYPE    myMutex           = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t   xSensorQueue        = NULL;

/* HTTP ---------------------------------------------------------------*/
//String          apiKey            = ""; 
//const char*     server            = "api.tago.io";

String          apiKey            = " "; 
const char*     server            = "api.tago.io";

/* LoRa Sensor-----------------------------------------------------*/
bool flag_sending = false;

/***********************************************************************/

/*
 *  STRUCTS MUST BE HERE! *********************************************** 
 */
typedef struct
{
  uint8_t  ucChipRev        ;// ESP32 chip revision
  uint32_t uiCpuFreq        ;// CPU frequency value
  uint32_t uiFreeHeapSize   ;// Free heap memory size
  uint64_t ulMac            ;// MAC address of ESP32
    
}SysStatus_t;

SysStatus_t stSysStatus = 
{
  .ucChipRev        = 0,
  .uiCpuFreq        = 0,
  .uiFreeHeapSize   = 0,
  .ulMac            = 0
};

int temperature = 0;
int humidity = 0;
int pressure = 0;
int postData = 0;
String LoRa_RSSI = " ";

int count_end_node = 0;
int count_gateway = 0;
float perda = 0;
/***********************************************************************/

/**
  * @brief  : Print strings
  * @param  : const char*
  * @retval : void
  */
void vPrintStr(const char *pcString)
{
  portENTER_CRITICAL(&myMutex);
  {
    Serial.print((char*)pcString);
    Serial.flush();
  }
  portEXIT_CRITICAL(&myMutex);
}

/**
  * @brief  : Print strings and int numbers
  * @param  : const char* and uint32_t
  * @retval : void
  */
void vPrintStrNum(const char *pcString, uint32_t uiValue)
{
  portENTER_CRITICAL(&myMutex);
  {
    char buffer[64] = {0}; 
    sprintf(buffer, "%s %lu\r", pcString, uiValue);
    Serial.println((char*)buffer);
  }
  portEXIT_CRITICAL(&myMutex);
}

/**
  * @brief  : Print strings and float numbers
  * @param  : const char* and float
  * @retval : void
  */
void vPrintStrFloat(const char *pcString, float fValue)
{
  portENTER_CRITICAL(&myMutex);
  {
    char buffer[64] = {0}; 
    sprintf(buffer, "%s %4.2f\r", pcString, fValue);
    Serial.println((char*)buffer);
  }
  portEXIT_CRITICAL(&myMutex);
}

/**
  * @brief  : Print hexadecimal values
  * @param  : uint32_t
  * @retval : void
  */
void vPrintHex(uint32_t uiValue)
{
  portENTER_CRITICAL(&myMutex);
  {
    char buffer[64] = {0}; 
    sprintf(buffer, " %08X", uiValue);
    Serial.print((char*)buffer);
  }
  portEXIT_CRITICAL(&myMutex);
}

/**
  * @brief  : HTTP task. All about connection, JSON formating and post values.
  * @param  : void *
  * @retval : void
  */
void vHttpTask(void *pvParameters)
{
  vPrintStr("HTTP task has started...");
    
  for(;;)
  {            
    // Wi-Fi is connected?
    if((wifiMulti.run() == WL_CONNECTED))
    {
      if(flag_sending == true)
      {
        //Inicia um client TCP para o envio dos dados
        if(client.connect(server, 80)) 
        {
          #ifdef DEBUG_TAGO
          vPrintStr("\n\n****************************************************\n");
          vPrintStr("*************** CONNECTED TO TAGOIO ****************\n");
          vPrintStr("****************************************************\n");
          #endif
          String      strPost     = ""; 
          String      strPostData = "";

          /* Formatacao JSON  ******************************************************************************/
          strPostData  = "[\n{\n\"variable\": \"temperatura\",\n\"value\": "  + String(temperature)     +"\n},\n";
          strPostData += "{\n\"variable\": \"umidade\",\n\"value\": "         + String(humidity)        +"\n},\n";
          strPostData += "{\n\"variable\": \"pressao\",\n\"value\": "         + String(pressure)        +"\n},\n";
          strPostData += "{\n\"variable\": \"count_gateway\",\n\"value\": "   + String(count_gateway)   +"\n},\n";
          strPostData += "{\n\"variable\": \"count_end_node\",\n\"value\": "  + String(count_end_node)  +"\n},\n";
          strPostData += "{\n\"variable\": \"perda\",\n\"value\": "           + String(perda)           +"\n},\n";
          strPostData += "{\n\"variable\": \"lora_rssi\",\n\"value\": "       + String(LoRa_RSSI)       +"\n}\n]";
          /******************************************************************************************************/
          
          strPost = "POST /data HTTP/1.1\n";
          strPost += "Host: api.tago.io\n";
          strPost += "Device-Token: "+apiKey+"\n";
          strPost += "_ssl: false\n";
          strPost += "Content-Type: application/json\n";
          strPost += "Content-Length: "+String(strPostData.length())+"\n";
          strPost += "\n";
          strPost += strPostData;
    
          // Post header and content
          client.print(strPost);
          
          uint32_t uiTimeout = millis();
          while(client.available() == 0) 
          {
            if(millis() - uiTimeout > 5000) 
            {
              #ifdef DEBUG_TAGO
              Serial.print(">>> Client Timeout !\n");
              #endif
              
              client.stop();
              flag_sending = false;
              break;
            }
          } 

          while(client.available())
          {
            String strLine = client.readStringUntil('\r');
            #ifdef DEBUG_TAGO
            Serial.print(strLine);
            #endif
          } 
        }
        // Close TCP/IP connection
        client.stop();
        digitalWrite(IO_LED, LOW);
        flag_sending = false;
      }
    }
    else
    {
      #ifdef DEBUG_TAGO
      vPrintStr("\n\n****************************************************\n");
      vPrintStr("!!!!! Trying to reconnect to local network... !!!!!!");
      vPrintStr("\n****************************************************\n\n");
      #endif
      // Disconnected to local network
      wifiMulti.addAP(WIFISSID, PASSWORD); 
    }
    vTaskDelay(15000/portTICK_PERIOD_MS);
  }
}

/**
  * @brief  : LoRa Sensor task. All about read values and cheking communicaiont with LoRa Sensor.
  * @param  : void *
  * @retval : void
  */
void vSensorTask(void *pvParameters)
{
  vPrintStr("LoRa Sensor task has started...");

  //unsigned long time_trigger=0;
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(915E6)) 
  {
    Serial.println(".");
    vTaskDelay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
    

  // Loop
  for( ;; )
  {
    // try to parse packet
    int packetSize = LoRa.parsePacket();
    if (packetSize) 
    {
      // received a packet
      Serial.print("\nReceived packet '");
  
      // read packet
      while (LoRa.available()) 
      {
        String LoRaData = LoRa.readString();
        Serial.print(LoRaData); 

        // print RSSI of packet
        LoRa_RSSI = LoRa.packetRssi();
        Serial.print("' with RSSI ");
        Serial.println(LoRa_RSSI);

        digitalWrite(IO_LED, HIGH);
      
        //validacao da string recebida
        if(LoRaData.length() == 13)
        {
          //Separa valores da string recebida
          temperature = (LoRaData.substring(0,2)).toInt();
          humidity = (LoRaData.substring(3,5)).toInt();
          pressure = (LoRaData.substring(6,10)).toInt();
          count_end_node = (LoRaData.substring(11,13)).toInt();
          
          Serial.print("\ntemperature = "); Serial.print(temperature);
          Serial.print("\nhumidity = "); Serial.print(humidity);
          Serial.print("\npressure = "); Serial.print(pressure);


          if(count_gateway<=100)
          {
            count_gateway++;
            
          }
          else
          {
            count_gateway=0;
          }

          //reseta contador 
          if (count_gateway>count_end_node)
          {
            count_end_node = 1;
            count_gateway = 1;
            perda = 0.0;
          }

          perda = (count_end_node*1.0 - count_gateway*1.0 ) / count_end_node*1.0;
          perda = perda*100.0;
          Serial.print("\ncount_end_node = "); Serial.print(count_end_node);
          Serial.print("\ncount_gateway = "); Serial.print(count_gateway);
          Serial.print("\nperda = "); Serial.print(perda);

          flag_sending = true;
        }
        else 
        {
          digitalWrite(IO_LED, LOW);
          Serial.print("\nString invalida!");
          flag_sending = false;
        }
      }
    }

    // Delay
    vTaskDelay(50/portTICK_PERIOD_MS);
  }

}

/**
  * @brief  : Timer task.
  * @param  : void *
  * @retval : void
  */
void vTimerTask(void *pvParameters)
{
  static uint8_t __c_100 = 0;
  static uint8_t __c_1s = 0;
  static uint8_t __c_60s = 0;
    
  // Loop
  for( ;; )
  {
    __c_100++;
    //contador 100ms
    if (__c_100>=100)
    {
      __c_100 = 0;
      
      __c_1s++;
      //contador 1 segundo
      if (__c_1s>=10) 
      { 
        __c_1s = 0;


        //contador 1 minuto
        __c_60s++;
        if (__c_60s>=60) 
        { 
          __c_60s = 0;

        }
      }
    }
    // Delay
    vTaskDelay(1/portTICK_PERIOD_MS);
  }
}

/**
  * @brief  : Setting up UART, GPIOs, Wi-Fi connection, System Infos, etc.
  * @param  : void 
  * @retval : void
  */
void prvSetupHardware(void)
{
  //
  uint32_t uiTimeout = 0;
  
  /* UART **********************************************************/
  Serial.begin(115200); 
  delay(1000);
  /*****************************************************************/  

  /* GPIO **********************************************************/ 
  pinMode     (IO_LED   , OUTPUT);
  digitalWrite(IO_LED   , LOW);
  /*****************************************************************/     

  /* Wi-Fi *********************************************************/
  // Connect to the local network
  wifiMulti.addAP(WIFISSID, PASSWORD); 
  // Delay 50ms
  delay(50);
  
  // Get current millis value
  uiTimeout = millis();
  
  // Check if is connected to local network
  while(wifiMulti.run() != WL_CONNECTED)
  {
    // Print point indicating waiting...
    vPrintStr(".");
    
    // Check timeout
    if(millis() - uiTimeout > 15000)
    {
      vPrintStr("\n\n****************************************************\n");
      vPrintStr("!!!!!!!!!!!!!! Restarting ESP32 module !!!!!!!!!!!!!\n");
      vPrintStr("****************************************************\n\n");  
      ESP.restart();
    }
    delay(500);
  }
  // Indicating that Wi-Fi is connected!
  //digitalWrite(IO_LED, HIGH);
  /*****************************************************************/  

  /* Getting ESP32 system informations *****************************/
  stSysStatus.ucChipRev         = ESP.getChipRevision();
  stSysStatus.uiCpuFreq         = ESP.getCpuFreqMHz();
  stSysStatus.uiFreeHeapSize    = ESP.getFreeHeap();
  stSysStatus.ulMac             = ESP.getEfuseMac();
  /*****************************************************************/

  vPrintStr   ("\n\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
  vPrintStr   ("**************Mini Gateway Sensor LoRa *************\n");
  vPrintStr   ("************* Carlos Rafael Crevelari *************\n");
  vPrintStr   ("************ ESP32 + FreeRTOS + TagoIO *************\n");
  vPrintStr   ("****************************************************\n");
  vPrintStr   ("----------------------------------------------------\n");
  vPrintStr   ("****************************************************\n");
  vPrintStr   ("************* ESP32 SYSTEM INFORMATION *************\n");
  vPrintStr   ("****************************************************\n");
  vPrintStrNum("Chip revision -----------:", stSysStatus.ucChipRev);
  vPrintStrNum("CPU frequency ------(MHz):", stSysStatus.uiCpuFreq);
  vPrintStrNum("Free Heap Size -------(B):", stSysStatus.uiFreeHeapSize);
  vPrintStr   ("MAC address -------------:");
  vPrintHex   ((uint16_t)(stSysStatus.ulMac >> 32));
  vPrintHex   (stSysStatus.ulMac);
  vPrintStr   ("\nWi-Fi -------------------: OK");
  vPrintStr   ("\nIP ----------------------: ");
  Serial.println(WiFi.localIP());   
  vPrintStr   ("****************************************************\n");
  vPrintStr   ("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n\n");

}

/**
  * @brief  : Setting up hardware and creating tasks.
  * @param  : void 
  * @retval : void
  */
void setup(void) 
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  prvSetupHardware(); 
                          
  xTaskCreatePinnedToCore(vTimerTask  , 
                          "Timer task", 
                          (configMINIMAL_STACK_SIZE + 1024), 
                          NULL, 
                          1, 
                          NULL, 
                          CORE_1);
                          

                          
  xTaskCreatePinnedToCore(vHttpTask , 
                          "HTTP task"  , 
                          (configMINIMAL_STACK_SIZE + 2048), 
                          NULL, 
                          2, 
                          NULL, 
                          CORE_1);   

  xTaskCreatePinnedToCore(vSensorTask  , 
                          "Sensor task", 
                          (configMINIMAL_STACK_SIZE + 1024), 
                          NULL, 
                          3, 
                          NULL, 
                          CORE_1);
}

void loop(void) 
{
  vTaskDelay(10/portTICK_PERIOD_MS);
}
