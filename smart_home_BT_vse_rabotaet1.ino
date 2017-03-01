
#define DEBUG
#define CLIMATE_MODULE
#define BLUETOOTH_MODULE

boolean debug = true;

#include <QuadDisplay.h>
#include <TroykaDHT11.h>
#include <QueueList.h>

#define PIN_DISPLAY   3
#define PIN_BT_LED    13
#define PIN_FOTO_R    A1  //фоторезистор
#define PIN_DOOR_LED  6   // инициализируем пин для светодиода
#define PIN_PIR       7   // инициализируем пин для получения сигнала от пироэлектрического датчика движения
#define PIN_BUZZER    A5  //пьезодинамик (пищалка)
#define PIN_LASER     5   //лазер от охранной сигнализации
#define PIN_COOLER    8   //кондиционер
#define PIN_GERKON    2   //Геркон
#define PIN_DHT       9  //датчит DHT11



//---------------------------------- НАСТРОЙКА BLUETOOTH_MODULE ---------------------

// RemoteXY select connection mode and include library  

#define REMOTEXY_MODE__SOFTSERIAL
#include <SoftwareSerial.h> 
#include "bluetooth.h"


// RemoteXY connection settings  
#define REMOTEXY_SERIAL_RX 11 
#define REMOTEXY_SERIAL_TX 12 
#define REMOTEXY_SERIAL_SPEED 9600

// RemoteXY configurate   
#pragma pack(push, 1) 
uint8_t RemoteXY_CONF[] = 
  { 2,0,28,0,6,6,0,1,0,3
  ,7,43,43,0,76,69,68,0,2,0
  ,55,16,41,26,2,79,78,0,79,70
  ,70,0 }; 
   
// this structure defines all the variables of your control interface  
 struct { 

    // input variable
  uint8_t button_1; // =1 if button pressed, else =0 
  uint8_t switch_1; // =1 if switch ON and =0 if OFF 
  uint8_t switch_2;

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY; 
#pragma pack(pop) 

///////////////////////////////////////////// 
//           END RemoteXY include          // 
///////////////////////////////////////////// 



#include "functions.h"
#include "climate.h"
#include "display.h"

// состояние сенсоров 
int pirState  = LOW;      // начинаем работу программы, предполагая, что движения нет
int tempState = 0;        // состояние температуры (предыдущее значение)
int gerkonState = LOW;    // состояние геркона

//состояние устройств  (исполнители)
int doorLedState  = LOW;  //состояние светодиода над дверью



//----------------------------------- НАСТРОЙКА ВСЕГО -------------------------------------------
void setup()
{
    pinMode(PIN_FOTO_R,   INPUT);  //контакт А0 как вход.
    pinMode(PIN_PIR,      INPUT);  // объявляем датчик в качестве INPUT
    pinMode(PIN_GERKON,   INPUT);
    
    pinMode(PIN_DOOR_LED, OUTPUT);  // объявляем светодиод в качестве  OUTPUT
    pinMode(PIN_BUZZER,   OUTPUT); //контакт 10 как выход
    pinMode(PIN_COOLER,   OUTPUT);
    pinMode(PIN_LASER,    OUTPUT);
    pinMode(PIN_BT_LED,   OUTPUT);
    pinMode(PIN_LASER,    OUTPUT);

    dht.begin();
    Serial.begin(9600);

    #ifdef BLUETOOTH_MODULE
    RemoteXY_Init();
    #endif


}

//------------------------------------ ГЛАВНЫЙ ЦИКЛ ПРОГРАММЫ --------------------------------------
void loop()
{
    int fotoRVal    = 0;
    int pirVal      = 0;      // переменная для чтения состояния пина  
    int tempStatus  = 0;      // статус чтения информации с датчика температуры (ошибка или нет)
    int tempVal     = 0;
    int gerkonVal   = HIGH;    //Геркон
    
static  byte coolState   = LOW;  // состояние кондиционера 
static  byte coolVal  =  coolState;    //Предлагаемое состояние кондиционера равно текущему состоянию 
static  int doorLedVal  =  doorLedState;    //Предлагаемое состояние светодиода выключено
   
    //--------------------------------- ЧТЕНИЕ ПОКАЗАНИЙ СЕНСОРОВ -------------------------------------
    tempVal   = getTemperature (); 
    pirVal    = digitalRead (PIN_PIR);      // считываем значение с датчика
    fotoRVal  = analogRead  (PIN_FOTO_R);   //прочитать показания сигнализации
    gerkonVal = digitalRead (PIN_GERKON);   //прочитать состояние двери
    
    #ifdef BLUETOOTH_MODULE
    RemoteXY_Handler();                     //читаем состояние смартфона
    #endif   
    //-------------------------------  АНАЛИЗ ПОКАЗАНИЙ СЕНСОРОВ -------------------------------------
    // выводим показания влажности и температуры

    if (tempVal != tempState) {
          onStateChanged(tempState, tempVal, "Temp changed: " + String(tempVal));
          
          displayTemperatureC(PIN_DISPLAY, tempVal);

          if (tempVal >= COOLER_ON_BARRIER)   coolVal = STATE_COOLER_ON;      
          if (tempVal <= COOLER_OFF_BARRIER)  coolVal = STATE_COOLER_OFF;
          
          tempState = tempVal;
    }

    if (pirVal != pirState) {
          onStateChanged(pirState, pirVal, "PIR changed: " + String(pirState) + " -> " + String(pirVal));
          
          if (HIGH == pirVal)           doorLedVal = HIGH;
          else   if (HIGH == gerkonVal) doorLedVal = LOW;
          
          pirState  = pirVal; 
    }

    if (gerkonVal != gerkonState) {
          onStateChanged(gerkonState, gerkonVal, "Door state changed: " + String(gerkonState) + 
                                                    " -> " + String(gerkonVal));
          if (LOW == gerkonVal)      doorLedVal = HIGH;
          else  if (LOW == pirVal)   doorLedVal = LOW;
          
          gerkonState = gerkonVal;
    }

      
  
    #ifdef BLUETOOTH_MODULE
    if (fotoRVal < FOTO_R_BARRIER && RemoteXY.switch_1==1) //если показания с фоторезистора ниже 900 кнопка на телефоне нажата то...
                            tone(PIN_BUZZER, map(fotoRVal, 0, 1023, 2500, 4500), TIME_BUZZER);
    else noTone(PIN_BUZZER);
    #endif

//------------------------------------- ИЗМЕНЕНИЕ СОСТОЯНИЙ ИСПОЛНИТЕЛЬНЫХ УСТРОЙСТВ -------------------------------
    if (doorLedVal != doorLedState) {
        onStateChanged(doorLedState, doorLedVal, "#Led state changed: " + String(doorLedState) + 
                                                          " -> " + String(doorLedVal));
    
        doorLedState = doorLedVal;
        digitalWrite(PIN_DOOR_LED, doorLedState);
    }

    if (coolVal != coolState) {
         onStateChanged(coolState, coolVal, "#Cooler state changed: " + String(coolState) + 
                                                          " -> " + String(coolVal));
         
         if (STATE_COOLER_ON == coolVal) analogWrite(PIN_COOLER, 255);  else analogWrite(PIN_COOLER, 0);         
         
         coolState=coolVal;
    }
   
    //----------------------------------- ДЕМОНСТРАЦИЯ ЧТЕНИЯ С ТЕЛЕФОНА -------------------------------------------------       
    
    #ifdef BLUETOOTH_MODULE
    digitalWrite(PIN_BT_LED, (RemoteXY.button_1==0)?LOW:HIGH);
    digitalWrite(PIN_LASER, (RemoteXY.switch_1==0)?LOW:HIGH);
    #endif
   
}     



             
