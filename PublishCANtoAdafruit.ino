#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "WiFi.h"

///////////initialize CAN Data/////////////////
CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size
CAN_frame_t rx_frame;
unsigned long currentMillis = millis();
/////////////initialize Cloud Data////////////////
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME  "$$$$$$$$"
#define AIO_KEY  "$$$$$$$$$$$$$$$$"


WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish Feed_Publish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/MohanadFeed_OnOff");
Adafruit_MQTT_Subscribe Feed_Subscribe = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Feed_Subscribe");


boolean MQTT_connect() 
{  
  int8_t ret; 
  if (mqtt.connected()) 
  {    
    return true; 
  }  
  uint8_t retries = 3;  
  while ((ret = mqtt.connect()) != 0) 
  { 
    mqtt.disconnect(); delay(2000);  
    retries--;
    if (retries == 0) 
    { 
      return false;       
    }    
  }
  return true;  
}
 
void task1(void* parameters)
 {
     for(;;)
     {
      SubscribeCAN();
      //use it to delay the TASK for 1 minutes
      //vTaskDelay(1000/portTICK_PERIOD_MS); 
     }
     
 }
 void task2(void* parameters)
 {
     for(;;)
     {
      PublishCAN();
      //use it to delay the TASK for 1 minutes
      //vTaskDelay(1000/portTICK_PERIOD_MS); 
     }
     
 }
void SubscribeCAN()
{
    
     char * CANMessagesFromCloud;
     String SplitetCanMessages[9];
     int FromStrToLongArray[9];
     char *endptr;
     int Digit=0;   
     int EndIndexFrameID=0;
     bool ExcuteOneTimeGetFrameID=true;
      if (MQTT_connect())
      { 
         Adafruit_MQTT_Subscribe *subscription_name;      
       while ((subscription_name = mqtt.readSubscription(5000))) 
      {
                    
            if (subscription_name == &Feed_Subscribe) {
               ExcuteOneTimeGetFrameID=true;
              Serial.println("Subscription ...... ");             
              Serial.println(((char *)Feed_Subscribe.lastread));
              CANMessagesFromCloud=(char *)Feed_Subscribe.lastread;
             /////////////////////////////////////
               for(int i=0;i<9;i++)
               {
                   if(ExcuteOneTimeGetFrameID)
                   {                       
                     for(int j=0;j<strlen(CANMessagesFromCloud);j++)
                     {
                       if(CANMessagesFromCloud[j]==':')   
                       {                      
                         Digit=j-1;                                                 
                         char sub[j+1];
                         strncpy(sub, CANMessagesFromCloud,j);
                         sub[j]='\0';                       
                         SplitetCanMessages[i]=sub;
                         FromStrToLongArray[i] = strtol(SplitetCanMessages[i].c_str(), &endptr, 16);
                         ExcuteOneTimeGetFrameID=false;                   
                         continue;
                       }
                     }
                    }else
                    {
                    
                    /////////////////////////////////////////////
                    /////////////////////////////////////////////                      
                     SplitetCanMessages[i]=String(CANMessagesFromCloud[Digit])+String(CANMessagesFromCloud[Digit+1]);                       
                     FromStrToLongArray[i] = strtol(SplitetCanMessages[i].c_str(), &endptr, 16);            
                    }
                     Digit+=2;                                
               } 
              rx_frame.FIR.B.FF = CAN_frame_std;
              rx_frame.MsgID = FromStrToLongArray[0];
             rx_frame.FIR.B.DLC = 8;
             rx_frame.data.u8[0] = FromStrToLongArray[1];
             rx_frame.data.u8[1] = FromStrToLongArray[2];
             rx_frame.data.u8[2] = FromStrToLongArray[3];
             rx_frame.data.u8[3] = FromStrToLongArray[4];
             rx_frame.data.u8[4] = FromStrToLongArray[5];
             rx_frame.data.u8[5] = FromStrToLongArray[6];
             rx_frame.data.u8[6] = FromStrToLongArray[7];
             rx_frame.data.u8[7] = FromStrToLongArray[8];     
              ESP32Can.CANWriteFrame(&rx_frame);         
            }      
      }
     }
}
 void PublishCAN()
 {
   if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) 
   {
    if (rx_frame.FIR.B.FF == CAN_frame_std) {
    printf("Publishing");
    }
    else {
    printf("New extended frame");
    }
    if (rx_frame.FIR.B.RTR == CAN_RTR) {
    printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    }
    else 
    {
        String Uint8TOStr="";
        printf(" 0x%08X, DLC %d, Data ", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
        Uint8TOStr+=String(rx_frame.MsgID,HEX); 
        Uint8TOStr+=String(" : "); 
        for (int i = 0; i < rx_frame.FIR.B.DLC; i++) 
        {
            printf("0x%02X ", rx_frame.data.u8[i]);
            if(rx_frame.data.u8[i]<=9)
            Uint8TOStr+="0"+String(rx_frame.data.u8[i],HEX)+":";
            else
            Uint8TOStr+=String(rx_frame.data.u8[i],HEX)+":";        
        }
        printf("\n");
        if (MQTT_connect())
        {                  
          Feed_Publish.publish(Uint8TOStr.c_str());
          delay(2000);
        }
    }
  }
     
 }
 void ConnectToWiFi()
 {
   WiFi.disconnect();
  delay(3000);
  Serial.println("START");
   WiFi.begin("&&&&&&&","&&&&&&&&&");
  while ((!(WiFi.status() == WL_CONNECTED))){
    delay(300);
    Serial.print("..");
  }
  Serial.println("Connected");
  Serial.println("Your IP is");
  Serial.println((WiFi.localIP()));
 }
void setup() 
{   
  Serial.begin(9600);            
  
  // Init CAN Module  
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_21;
  CAN_cfg.rx_pin_id = GPIO_NUM_22;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();

  //ConnectToWifi
  ConnectToWiFi();
  
  //Start Subscribe for CAN Messages From the Cloud
  mqtt.subscribe(&Feed_Subscribe);
  
  //Activating MultiTask : 1.TaskePublish  2.TaskSubscribe 
  xTaskCreate(task1,"Task 1",2000,NULL,1,NULL);
  xTaskCreate(task2,"Task 2",2000,NULL,1,NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
     
}