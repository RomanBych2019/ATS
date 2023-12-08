#pragma once

// #define PRINTDEBUG
// #define verATP
#define verAnalogInput

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <freertos/task.h>
#include <FreeRTOSConfig.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include "Preferences.h"
// #include "SoftwareSerial.h" 
#include <SoftwareSerial.h>
#include <SimpleModbusSlave_DUE.h>
#include <RtcDS3231.h>
#ifdef verAnalogInput
#include <Adafruit_ADS1X15.h>
#include "LS_ANALOG_U.h"
#include "LS_ANALOG_F.h"
#endif
#include "COUNTER.h"
#include "TARRING.h"
#include "TANK.h"
#include "NEXTION.h"
#include "Out.h"
#include "LS_RS485.h"
#include "LS_BLE.h"
#include "LS_EMPTY.h"

#define serialLS Serial1
#define serialMB Serial2

#define PLATE_v1 // PLATE_v1 - плата вер1

#ifdef PLATE_v1
static const uint8_t INDI_F_PIN_ = GPIO_NUM_25;     // индикатор включения частотного ДУТа
static const uint8_t OUT_PUMP = GPIO_NUM_12;        // вывод управления насосом
static const uint8_t INIDICATE_COUNT = GPIO_NUM_13; // вывод индикатора входных импульсов
static const uint8_t IN_KCOUNT = GPIO_NUM_5;        // вход счетчика топлива
static const uint8_t RXDNEX = GPIO_NUM_23;          //
static const uint8_t TXDNEX = GPIO_NUM_19;          //
static const uint8_t RXLS = GPIO_NUM_27;
static const uint8_t TXLS = GPIO_NUM_14;
#endif

// режимы работы
enum type
{
  CALIBR,      // Калибровка счетчика
  PUMPINGAUTO, // Откачка топлива автоматом
  TAR,         // Тарировка
  SETTING,     // Настройка тарировки
  MENU,        // Меню
  COUNT,       // Счетчик
  PUMPINGOUT,  // Выдача топлива
  END_TAR,
  END_TAR_HMI,
  MESSAGE,
  SEARCH_BLE,
  PAUSE,
  SET_DATE
};

const uint8_t SIZE = 22;
union
{
  struct
  {
    unsigned int mode;          //  1 режим работы станции
    unsigned int kRefillNum;    //  2 номер пролива
    unsigned int v_full;        //  3 объем залитого топлива всего в 0,1 литра 1-2 байт
    unsigned int k_v_full;      //  3.1 объем залитого топлива всего в 0,1 литра 3-4 байт
    unsigned int resultNProliv; //  5 N тарируемого ДУТа зафиксированный станцией в проливах
    unsigned int id1;           //  6 номер автомобиля 1-2 байт
    unsigned int id2;           //  6.1 номер автомобиля 3-4 байт
    unsigned int resultN;       //  8 N тарируемого ДУТА постоянно получаемы данные
    unsigned int adress;        //  9 сетевой адресс ДУТа 1-2 байт
    unsigned int k_adress;      //  9.1 сетевой адресс ДУТа 3-4 байт
    unsigned int vtank;         //  11 объем тарируемого бака, литр
    unsigned int kRefill;       //  12  количество проливов
    unsigned int flowRate;      //  13  скорость потока, литр/мин
    unsigned int pause;         //  14  длительность паузы между проливами, сек
    unsigned int k_in_Litr;     //  15  количество импульсов на 10 литров
    unsigned int timetarring;   //  16  время выполнения тарировки
    unsigned int typells;       //  17  тип ДУТ , 0 - аналоговый_U , 1 - аналоговый_F , 2 - цифровой по rs485 , 3 - цифровой BLE
    unsigned int typetarring;   //  19  режим тарировки 1 - автоматический, 0 - ручной
    unsigned int error;         //  19  код ошибки
    unsigned int rssi;          //  20  RSSI ДУТ BLE
    unsigned int llsATP;        //  21  N ДУТа емкости АПТ (lls adr=100)
    bool controlFlowrate;       //  22  Флаг контроля скорости потока
  };
  unsigned int au16data[SIZE];
} datemod;

int counter_display_resetring = 0;
volatile unsigned time_counter_imp = 0;
const long MIN_DURATION = 500;
const uint16_t TIME_UPDATE_LLS = 10000;       // период обновления данных ДУТ
const uint16_t TIME_UPDATE_HMI = 300;         // период обновления данных на дисплее, мсек
const uint16_t TIME_UPDATE_SPEED_PUMP = 2000; // период обновления скорости потока
const uint16_t TIME_PAUSE_END_TAR = 20000;    // пауза в конце тарировки для передаче данных в систему мониторинга

unsigned long start_pause, worktime, time_start_refill, time_LLS_update, time_stop_flow_rate;
bool autostop = false;
bool flag_HMI_send = false; 
bool flag_conect_ok = true; // флаг удачного получения данных от ДУТ

const char *LOG_FILE_NAME = "log.csv";

AsyncWebServer server(80);

void rpmFun();
void modeMenu();
void modePumpOut();
void modeTarring();
void modePumpAuto();
void endTarring();
void endRefill();
void proceedTarring();
void errors();
void modbus();
void digitalpause();
void startPump();
void stopPump();
String makeLlsDateToDisplay(ILEVEL_SENSOR *_lls);
void onHMIEvent(String messege, String data, String response);
void exitTarring();
String saveLog();
void wifiInit();

void wifiInit();
void delete_lls();
String saveLog();
String deleteLog();

String listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void getDataLog(AsyncWebServerRequest *request, String file);

void updateLS(void *pvParameters);
void updateLS();
void sendNextion(void *pvParameters);
void readNextion(void *pvParameters);
void calculate_speedPump(void *pvParameters);
void onHMIEvent(String messege, String data, String response);
void printDebugLog(void *pvParameters);

hw_timer_t *My_timer = NULL;
void IRAM_ATTR onTimer();

RtcDS3231<TwoWire> Rtc(Wire);
#ifdef verAnalogInput
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
LS_ANALOG_F *lls_analog_f;
LS_ANALOG_U *lls_analog_u;
#endif
Preferences flash;
EspSoftwareSerial::UART serialHMI;

// void test();

// ДУТ
ILEVEL_SENSOR *lls;
LS_RS485 *lls_RS485;
LS_BLE *lls_Ble;
LS_EMPTY *lls_Empty;

#ifdef verATP
// ДУТ в емкости АТП
LS_RS485 *lls_ATP;
#endif

// насос
Out *pump;

// запасной выход
Out *out_tmp;

// счетчик
COUNTER *countV;

// бак
TANK *tank;

// тарировка
TARRING *tar;

// дисплей
NEXTION hmi(serialHMI);
