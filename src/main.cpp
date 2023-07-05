#include "main.h"
#ifdef verAnalogInput
#include "LS_ANALOG_U.h"
#include "LS_ANALOG_F.h"
#endif
#include "LS_RS485.h"
#include "LS_BLE.h"
#include <TimeUtil.h>

void setup()
{
  pinMode(GPIO_NUM_2, OUTPUT);
  pinMode(INDI_F_PIN_, OUTPUT);
  digitalWrite(INDI_F_PIN_, HIGH); // индикация загрузки

  Serial.begin(115200);
  Serial2.begin(19200);
  serialLS.begin(19200, SERIAL_8N1, RXLS, TXLS);
  serialNextion.begin(19200, SWSERIAL_8N1, RXDNEX, TXDNEX);

  hmi.echoEnabled(true);
  hmi.hmiCallBack(onHMIEvent);

  Rtc.Begin();
  SPIFFS.begin(true);

  digitalWrite(GPIO_NUM_2, HIGH);

#ifdef verAnalogInput
  ads.setGain(GAIN_ONE);
  ads.begin();
#endif

  wifiInit();
  server.begin();
  // AsyncElegantOTA.begin(&server); // Start ElegantOTA

  flash.begin("eerom", false);

  int k = flash.getInt("impulse_count", 2000); // чтение из eerom значения K счетчика
  countV = new COUNTER(k);

  tank = new TANK(countV);
  tar = new TARRING(countV, tank);
  tar->setType(tarring::MANUAL);
  pump = new Out(OUT_PUMP);

  datemod.mode = MENU;
  lls = nullptr;

#ifdef verATP
  lls_ATP = new LS_RS485(&serialLS, 100);
#endif

  pinMode(IN_KCOUNT, INPUT_PULLUP);           // инициализация входа импульсов ДАРТ
  attachInterrupt(IN_KCOUNT, rpmFun, CHANGE); // функция прерывания

  modbus_configure(&Serial2, 19200, 1, 0, SIZE, datemod.au16data);
  modbus_update_comms(19200, 1);

  hmi("rest");

  // tickerspeedPump.attach_ms(2000, flowRate);
  // tickerupdateLS.attach_ms(TIME_UPDATE_LLS, updateLS);
  // tickermodbus.attach_ms(100, modbus);

  // страница  списка файлов
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html", "<p>ATS 2023</p><p>" + String(__DATE__) + "</p><p>" + listDir(SPIFFS, "/", 0) + "</p>"); });

  // скачивание лог-файла
  server.on("/log.csv", HTTP_GET, [file = "/log.csv"](AsyncWebServerRequest *request)
            { getDataLog(request, file); });

  // удаление файла логов
  server.on("/delete", [](AsyncWebServerRequest *request)
            { request->send(200, "text/html", "<p>ATS - delete log file</p>" + deleteLog()); });

  xTaskCreatePinnedToCore(
      calculate_speedPump,        /* Вычисление скорости потока */
      "Task_calculate_speedPump", /* Название задачи */
      2048,                       /* Размер стека задачи */
      NULL,                       /* Параметр задачи */
      1,                          /* Приоритет задачи */
      NULL,                       /* Идентификатор задачи, чтобы ее можно было отслеживать */
      tskNO_AFFINITY);            /* Ядро для выполнения задачи (0) */

  xTaskCreatePinnedToCore(
      updateLS,        /* */
      "Task_updateLS", /* Название задачи */
      2048,            /* Размер стека задачи */
      NULL,            /* Параметр задачи */
      1,               /* Приоритет задачи */
      NULL,            /* Идентификатор задачи, чтобы ее можно было отслеживать */
      tskNO_AFFINITY); /* Ядро для выполнения задачи (0) */

#ifdef PRINTDEBUG
  xTaskCreatePinnedToCore(
      printDebugLog,        /* Обновление данных для протокола MB */
      "Task_printDebugLog", /* Название задачи */
      4096,                 /* Размер стека задачи */
      NULL,                 /* Параметр задачи */
      3,                    /* Приоритет задачи */
      NULL,                 /* Идентификатор задачи, чтобы ее можно было отслеживать */
      tskNO_AFFINITY);      /* Ядро для выполнения задачи (0) */
#endif
  xTaskCreatePinnedToCore(
      sendNextion,        /* обновление данных HMI */
      "Task_sendNextion", /* Название задачи */
      4096,               /* Размер стека задачи */
      NULL,               /* Параметр задачи */
      4,                  /* Приоритет задачи */
      NULL,               /* Идентификатор задачи, чтобы ее можно было отслеживать */
      1);

  xTaskCreatePinnedToCore(
      readNextion,        /* чтение данных от HMI */
      "Task_readNextion", /* Название задачи */
      4096,               /* Размер стека задачи */
      NULL,               /* Параметр задачи */
      2,                  /* Приоритет задачи */
      NULL,               /* Идентификатор задачи, чтобы ее можно было отслеживать */
      tskNO_AFFINITY);

  delay(2000);
  digitalWrite(INDI_F_PIN_, LOW);
  pinMode(INIDICATE_COUNT, OUTPUT);
  digitalWrite(INIDICATE_COUNT, LOW);
}
void loop()
{
  errors();

  modbus();

  switch (datemod.mode)
  {
  case MENU:
    modeMenu();
    break;
  case PUMPINGAUTO:
    modePumpAuto();
    break;
  case PAUSE:;
  case TAR:
    modeTarring();
    break;
  case COUNT:
    modeCounter();
    break;
  case PUMPINGOUT:
    modePumpOut();
    break;
  case CALIBR:
    modeCalibr();
    break;
  case SETTING:
    modeSetting();
    break;
  case END_TAR:
    modeEndTar();
    break;
  case SEARCH_BLE:
    break;
  }
}

// void test()
// {
//   if (pump->get() == ON)
//   {
//     rpmFun();
//   }
// }

void sendNextion(void *pvParameters)
{
  for (;;)
  {
    String str = {};
    String res = {};
    uint level;

    RtcDateTime dt = Rtc.GetDateTime();
    const uint SIZE = 20;
    char datestring[SIZE];
    int bt = 0;
    int j0 = 0;

    switch (datemod.mode)
    {
    case MENU:
      snprintf_P(datestring,
                 SIZE,
                 PSTR("%02u.%02u.%04u %02u:%02u"),
                 dt.Day(),
                 dt.Month(),
                 dt.Year(),
                 dt.Hour(),
                 dt.Minute());

      if (lls != nullptr)
        str = makeLlsDateToDisplay(lls);
      else
        str = "";
      if (lls != nullptr)
        bt = static_cast<int>(lls->getType());

#ifdef verATP
      if (lls_ATP->getError() == ILEVEL_SENSOR::NO_ERROR)
        res = String(lls_ATP->getTarLevel(), 1);
      else
        res = "---";
#endif

      hmi.sendScreenMenu(datestring, countV->getKinLitr(), res, str, bt);
      break;

    case PUMPINGOUT:
      hmi.sendScreenPump_Out(tar->getVfuel(), countV->getFlowRate());
      hmi("pump_out.b1.picc", pump->get() == OFF ? 20 : 15);
      break;

    case PUMPINGAUTO:
      if (lls != nullptr)
        str = makeLlsDateToDisplay(lls);
      else
        str = "";
      hmi.sendScreenPump_Auto(tar->getVfuel(), countV->getFlowRate(), str);
      hmi("pump_auto.b4.picc", pump->get() == OFF ? 14 : 15);
      break;

    case COUNT:
      if (lls != nullptr)
        str = makeLlsDateToDisplay(lls);
      else
        str = "";
      if (counter_display_resetring != tar->getCountReffil())
      {
        for (int i = 0; i < tar->getCountReffil(); i++)
        {
          res += String(tar->getRefill(i) / 10.0, 1) + " l\\r";
        }
        hmi.sendScreenCounter(res);
        counter_display_resetring++;
      }
      hmi.sendScreenCounter(tar->getVfuel() - tar->getBackRefill(), tar->getVfuel(), countV->getFlowRate(), str);
      hmi("counter.b4.picc", pump->get() == OFF ? 10 : 11);
      break;

    case CALIBR:
      hmi.sendScreenCalibration(countV->getVFuelCalibr(), countV->getK());
      hmi("calibr.b4.picc", pump->get() == OFF ? 2 : 3);
      break;

    case PAUSE:;

    case TAR:
      if (lls != nullptr)
      {
        str = makeLlsDateToDisplay(lls);
      }
      else
        str = "ДУТ не подключен";
      level = map(tar->getVfuel(), 0, tar->getVTank(), 0, 100);
      hmi("tar.b4.picc", pump->get() == OFF ? 23 : 24);
      uint tmp_time_pause;
      if (autostop && tar->getType() == tarring::MANUAL)
      {
        tmp_time_pause = 60 * (tar->getTimePause()) - (millis() - start_pause) / 1000;
        if (tmp_time_pause == 0)
          tmp_time_pause = 1;
      }
      else
        tmp_time_pause = tar->getTimePause() * 60;
      hmi.sendScreenTarring(tar->getVfuel() - tar->getBackRefill(), tar->getVfuel(), tar->getCountReffil(), tar->getNumRefill() - tar->getCountReffil(), countV->getFlowRate(), str, tar->getTimeTarring(), level, tmp_time_pause);
      break;

    case MESSAGE:
      if (datemod.error & ILEVEL_SENSOR::error::CLIFF)
      {
        str = "Данные с ДУТ ниже минимального значения\\rПроверте настройки ДУТ (min уровень))";
      }
      else if (datemod.error & ILEVEL_SENSOR::error::CLOSURE)
      {
        str = "Данные с ДУТ выше максимального значения\\rПроверте настройки и подключение ДУТ";
      }
      else if (datemod.error & ILEVEL_SENSOR::error::NOT_FOUND)
      {
        str = "ДУТ не найден\\rПроверте настройки и подключение ДУТ";
      }
      else if (datemod.error & ILEVEL_SENSOR::error::LOST)
      {
        str = "ДУТ потерян\\rПроверте подключение ДУТ";
      }
      else if (datemod.error & 32)
      {
        str = "Нет изменения значений ДУТ\\rПроверте поступление топлива в бак";
      }
      else if (datemod.error & 64)
      {
        str = "Низкая скорость потока топлива\\rПроверте прохождение топлива через счетчик";
      }
      else if (datemod.error & 128)
      {
        str = "Высокие начальные показания ДУТ\\rПроверте калибровку ДУТ, убедитесь в отсутствии топлива в баке";
      }
      if (str)
        hmi.sendScreenMessage(str);
      break;

    case END_TAR:
      str = "ID: " + tar->getId() + "\\rN  | LLS   | V";
      for (int i = 0; i < tar->getNRefill()->size(); i++)
      {
        uint n = tar->getNRefill()->at(i);
        String res_n = "";
        if (n < 10)
          res_n = "       " + String(n);
        else if (n < 100)
          res_n = "    " + String(n);
        else if (n < 1000)
          res_n = "  " + String(n);
        else
          res_n = " " + String(n);
        if (i < 10)
          str += "\\r" + String(i) + "   |";
        else
          str += "\\r" + String(i) + " |";
        str += res_n + "| " + String(tar->getVRefill()->at(i) / 10.0, 1);
      }
      if (start_pause > millis())
        j0 = map(start_pause - millis(), TIME_PAUSE_END_TAR, 100, 100, 0);
      if (lls != nullptr)
        hmi.sendScreenEnd_Tar(str, j0, tar->getVRefill(), tar->getNRefill());
      else
        hmi.sendScreenEnd_Tar(str, j0);
      break;

    case SETTING:
      if (lls != nullptr)
      {
        str = makeLlsDateToDisplay(lls);
      }
      else
      {
        str = " не подключен";
      }
      hmi.sendScreenSetting(tar->getTimeTarring(), str);
      break;

    case SEARCH_BLE:
      if (lls != nullptr)
      {
        if (lls->getNameBLE() != "")
        {
          if (lls->getError() == ILEVEL_SENSOR::NOT_FOUND)
            str = "ДУТ не найден";
          else if (lls->getError() == ILEVEL_SENSOR::NO_ERROR)
          {
            str = "N " + String(lls->getLevel()) + " | RSSI " + String(lls->getRSSI()) + " | V " + String(lls->getDataBLE(1) / 10) + "." + String(lls->getDataBLE(1) % 10) + " | T " + String(lls->getDataBLE(2)) + " | D " + String(lls->getDataBLE(3));
          }
        }
        else
          str = "";
        hmi.sendScreenSearch_BLE(str);
      }
      break;
    default:
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(TIME_UPDATE_HMI));
  }
  vTaskDelete(NULL);
}
// Режим Меню
void modeMenu()
{
  tar->reset();
  pump->off();
  datemod.id1 = 0;
  datemod.id2 = 0;
  counter_display_resetring = 0;

  // сброс ДУТа BLE без номера и не найденного в поиске
  if (lls != nullptr && lls->getType() == ILEVEL_SENSOR::type::BLE_ESKORT && (lls->getNameBLE() == "" || lls->getError() == ILEVEL_SENSOR::error::NOT_FOUND))
  {
    delete_lls();
  }
}
void modeEndTar()
{
  exitTarring();
}
void modeSetting()
{
}
// Режим Автоматическая выдача топлива
void modePumpOut()
{
  if (tar->getVTank() <= tar->getVfuel()) //
  {
    pump->off();
  }
}
// Режим Тарировка
void modeTarring()
{
  if (tar->getVTankRefill() * tar->getCountReffil() <= tar->getVfuel() && tar->getCountReffil() != tar->getNumRefill()) // условие окончания очередного пролива
  {
    endRefill();
  }

  if (tar->getVTank() <= tar->getVfuel()) // условие окончания тарировки
  {
    endTarring();
  }
}
// Режим Счетчик________________________________________________________________________________________________
void modeCounter()
{
}
// Режим Автоматическое выкачивание________________________________________________________________________________________________
void modePumpAuto()
{
  if (millis() - worktime > 30000)
  {
    if (pump->get() == ON && countV->getFlowRate() < 5) // скорость потока менее 5 л/мин
    {
      pump->off();
    }
  }
  if (pump->get() == OFF)
    worktime = millis();
}

// настройка счетчика
void modeCalibr()
{
}

// продолжение тарировки
void proceedTarring()
{
  if (lls != nullptr)
  {
    tar->saveResultRefuil(lls->getLevel());
  }
  else
    tar->saveResultRefuil();

  autostop = false;
  time_start_refill = millis();
  datemod.mode = TAR;
  pump->on();
}

// окончание очередного пролива
void endRefill()
{
  stopPump();
  if (datemod.mode == TAR)
  {
    autostop = true;
    start_pause = millis();
    datemod.mode = PAUSE;
  }

  if (millis() - start_pause < 10000)
    return;

  if (countV->getFlowRate() > 0) // проверка, что топливо больше не поступает в бак
  {
    delay(100);
    return;
  }

  if (tar->getType() == tarring::AUTO) // пауза если ДУТ цифровой
    digitalpause();
  else if (millis() - start_pause > tar->getTimePause() * 60000) // пауза если ДУТ аналоговый
    proceedTarring();
}

// финал тарировки
void endTarring()
{
  stopPump();
  if (datemod.mode == TAR)
  {
    start_pause = millis();
    datemod.mode = PAUSE;
  }

  if (datemod.mode == PAUSE)
  {
    if (millis() - start_pause < 1000)
      return;
    else
      datemod.mode = END_TAR;
  }

  if (datemod.mode == END_TAR)
  {
    // страница окончания тарировки
    hmi("page t_end");
    while (countV->getFlowRate() > 0) // проверка, что топливо больше не поступает в бак
      delay(1000);

    if (lls != nullptr)
      tar->saveResultRefuil(lls->getLevel());
    else
      tar->saveResultRefuil();
  }
}

// выход из тарировки
void exitTarring()
{
  if (start_pause < millis())
  {
    start_pause = millis() + TIME_PAUSE_END_TAR;
  }
  if (start_pause - millis() < 500)
  {
    // hmi("vis b1,1");
    // hmi("vis b4,1");
  }
}

// считывание данных с ДУТ
void updateLS(void *pvParameters)
{
  for (;;)
  {
#ifdef verATP
    lls_ATP->update();
#endif

    if (datemod.mode != MENU && datemod.mode != CALIBR && datemod.mode != PUMPINGOUT)
      if (lls != nullptr)
      {
        lls->update();
        // test();
      }
    vTaskDelay(pdMS_TO_TICKS(TIME_UPDATE_LLS));
  }
  vTaskDelete(NULL);
}

// вычисление скорости потока
void calculate_speedPump(void *pvParameters)
{
  for (;;)
  {
    countV->updateFlowRate();
    vTaskDelay(pdMS_TO_TICKS(TIME_UPDATE_SPEED_PUMP));
  }
  vTaskDelete(NULL);
}

// ошибки ДУТ
void errors()
{
  if (datemod.mode == MESSAGE || datemod.mode == END_TAR || datemod.mode == CALIBR)
    return;

  int error = 0;
  if (lls != nullptr && tar->getType() == tarring::AUTO)
  {
    error |= lls->getError();
    // проверка увеличения данных с ДУТа в проливах
    if (tar->getCountReffil() > 2)
    {
      if (tar->getNRefill(tar->getCountReffil() - 1) < 10 + tar->getNRefill(tar->getCountReffil() - 2))
        error |= 1 << 5;
    }
  }
  // проверка, что после 30 секунд после включения насоса скорость пролива не меньше 5л/мин
  if (datemod.mode != PUMPINGOUT)
    if (pump->get() == ON && millis() > pump->getTimeStart() + 30000)
    {
      if (countV->getFlowRate() < 5)
      {
        if (!(lls != nullptr && lls->getType() == ILEVEL_SENSOR::type::BLE_ESKORT)) // если ДУТ BLE, скорость не контролируем
          error |= 1 << 6;
      }
    }
  // проверка, что тарировка начинается с приемлемого уровня ДУТ
  if (datemod.mode == SETTING)
  {
    if (lls != nullptr)
      if (lls->getLevel() > lls->getLevelStart())
        error |= 1 << 7;
  }

  if (datemod.mode == SEARCH_BLE && error == ILEVEL_SENSOR::NOT_FOUND)
  {
    return;
  }

  if (error && (datemod.error != error))
  {
    // Serial.print"\nErr: %d", error);
    pump->off();
    hmi("page message");
    datemod.error = error;
    datemod.mode = MESSAGE;
  }
}

// функция подсчета импульсов с ДАРТ
void rpmFun()
{
  if (micros() - duratiom_counter_imp > MIN_DURATION)
  {
    countV->setKcount();
    digitalWrite(INIDICATE_COUNT, !digitalRead(INIDICATE_COUNT));
  }
  duratiom_counter_imp = micros();
}

// парсинг полученых данных от дисплея Nextion
void onHMIEvent(String messege, String data, String response)
{

  {
    //  Экран Меню--------------------------------------------------------------------------------------------------------------------------
    if (messege == "menu")
    {
      int k = flash.getInt("impulse_count", 2000); // чтение из eerom значения K счетчика
      countV->setKinLitr(k);
      datemod.mode = MENU;
      datemod.error = 0;
      if (lls != nullptr)
        if (lls->getType() != ILEVEL_SENSOR::type::BLE_ESKORT)
          delete_lls();
    }

    else if (messege == "au!")
    {
#ifdef verAnalogInput
      delete_lls();
      lls = new LS_ANALOG_U(ads, 2);
      lls->search();
      tar->setType(tarring::AUTO);
#endif
    }

    if (messege == "af!")
    {
#ifdef verAnalogInput
      delete_lls();
      lls = new LS_ANALOG_AF;
      lls->search();
      tar->setType(tarring::AUTO);
#endif
    }

    if (messege == "rs485!")
    {
      delete_lls();
      lls = new LS_RS485(&serialLS);
      if (lls->search())
        tar->setType(tarring::AUTO);
      // else
      //   delete_lls();
    }

    if (messege == "ble!")
    {
      delete_lls();
      lls = new LS_BLE();
      datemod.mode = SEARCH_BLE;
    }

    if (messege == "no_lls!")
    {
      delete_lls();
    }

    if (messege == "TD_") // получение имени ДУТа BLE Эскорт
    {
      lls->setNameBLE(messege + data);
      lls->search();
    }

    if (messege == "pump")
    {
      pump->get() ? stopPump() : startPump();
    }

    if (messege == "p-")
    {
      if (tar->getTimePause() > 1)
        tar->setTimePause(tar->getTimePause() - 1);
    }

    if (messege == "p+")
      if (tar->getTimePause() < 15)
      {
        tar->setTimePause(tar->getTimePause() + 1);
      }

    if (messege == "pump_out")
    {
      datemod.mode = PUMPINGOUT;
    }

    if (messege == "set_id")
    {
      datemod.mode = SETTING;
      tank->reset();
    }

    if (messege == "endtarr") // окончание тарировки
    {
      datemod.mode = END_TAR;
      endTarring();
    }

    if (messege == "save") // сохранение тарировки
    {
      saveLog();
    }

    //  Экран Счетчик--------------------------------------------------------------------------------------------------------------------------
    if (messege == "counter")
    {
      datemod.mode = COUNT;
    }
    if (messege == "resetring")
    {
      if (tar->getVfuel())
        if (lls != nullptr)
          tar->saveResultRefuil(lls->getLevel());
        else
          tar->saveResultRefuil();
      return;
    }
    if (messege == "reset")
    {
      tar->reset();
      counter_display_resetring = -1;
    }
    //  Экран Автоматического выкачивания-------------------------------------------------------------------------------------------------------------------
    if (messege == "pump_auto")
    {
      datemod.mode = PUMPINGAUTO;
    }

    //  Экран Калибровка--------------------------------------------------------------------------------------------------------------------------
    if (messege == "calibr")
    {
      datemod.mode = CALIBR;
    }
    if (messege == "calibr_on")
    {
      countV->reset();
    }
    if (messege == "calibr_off")
    {
      stopPump();
    }

    if (messege == "k_count") // получение промежуточного значения k_in_Litr
    {
      countV->setKinLitrCalibr(data.toInt());
    }

    if (messege == "save_k") // получение нового К, запись в память
    {
      flash.putInt("impulse_count", data.toInt()); // запись в eerom значения K счетчика
      countV->setKinLitr(data.toInt());
    }
    if (messege == "id") // получение номера тарируемого объекта
    {
      tar->setId(data);
    }
    if (messege == "vtank") // получение объема бака
    {
      tank->setVTank(data.toInt() * 10);
    }
    if (messege == "krefill") // получение количества проливов
    {
      tar->setNumRefill(data.toInt());
      datemod.mode = SETTING;
    }

    if (messege == "set_dat")
    {
      datemod.mode = SET_DATE;
    }

    //  Получениее времени --------------------------------------------------------------------------------------------------------------------------
    if (messege == "date")
    {
      Serial.println();
      auto index_sym = data.indexOf(';');
      auto DATE = data.substring(0, index_sym);
      auto TIME = data.substring(index_sym + 1, data.length());
      Serial.printf("Date %s Time %s\n", DATE, TIME);

      // Для установки  времени
      RtcDateTime compiled = RtcDateTime(DATE.c_str(), TIME.c_str());
      Rtc.SetDateTime(compiled);
    }

    //  Экран Тарировка--------------------------------------------------------------------------------------------------------------------------

    if (messege == "tar_start")
    {
      if (lls != nullptr)
        tar->saveResultRefuil(lls->getLevel());
      else
        tar->saveResultRefuil();
      tar->setTStart(Rtc.GetDateTime());
      datemod.mode = TAR;
    }

    if (messege == "pause")
    {
      if (datemod.mode != TAR && datemod.mode != PAUSE)
        datemod.mode = SETTING;
      if (data.toInt())
      {
        tar->setType(tarring::MANUAL);
        tar->setTimePause(data.toInt());
      }
      else
      {
        tar->setType(tarring::AUTO);
        tar->setTimePause(0);
      }
    }
    if (messege == "clear_err0")
    {
      flag_dell_lls = true;
    }
    if (messege == "clear_err5")
    {
      datemod.mode = SETTING;
    }
    if (messege == "clear_err7")
    {
      datemod.mode = TAR;
    }
  }
}

// данные с Nextion
void readNextion(void *pvParameters)
{
  for (;;)
  {
    hmi.listen();
    //  Serial.print("!");

    // vPrintString("readNextion");
    // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TIME_UPDATE_HMI));
    digitalWrite(GPIO_NUM_2, !digitalRead(GPIO_NUM_2));
    vTaskDelay(pdMS_TO_TICKS(TIME_UPDATE_HMI));
  }
  vTaskDelete(NULL);
}

#ifdef PRINTDEBUG
void printDebugLog(void *pvParameters)
{
  for (;;)
  {
    Serial.println();
    Serial.printf("Mem: %d", ESP.getFreeHeap());
    Serial.println("   _____________");
    Serial.printf("\nРежим работы\t\t");
    switch (datemod.mode)
    {
    case PUMPINGAUTO:
      Serial.print("Откачка топлива автоматом");
      break;
    case CALIBR:
      Serial.print("Калибровка счетчика");
      break;
    case MENU:
      Serial.print("Меню");
      break;
    case TAR:
      Serial.println("Тарировка");
      break;
    case SETTING:
      Serial.print("Настройка тарировки");
      break;
    case COUNT:
      Serial.print("Счетчик");
      break;
    case PUMPINGOUT:
      Serial.print("Выдача топлива");
      break;
    case END_TAR:
      Serial.print("Конец тарировки");
      break;
    case PAUSE:
      Serial.print("Пауза");
      break;
    case MESSAGE:
      Serial.print("Собщения");
      break;
    }
    Serial.printf("\nТекущее значение К\t%d", countV->getKinLitr());
    Serial.printf("\nОбъем бака\t\t%d", tar->getVTank());
    Serial.printf("\nЗалито топлива\t\t%d", tar->getVfuel());
    Serial.printf("\nНомер автомобиля\t%s", tar->getId());
    Serial.printf("\nОбъем проливов\t\t%d", tar->getVTankRefill());
    Serial.printf("\nКоличество проливов\t%d", tar->getNumRefill());
    Serial.printf("\nПауза между проливами\t%d", tar->getTimePause());
    Serial.print("\nТип тарировки\t\t");
    if (tar->getType() == tarring::AUTO)
      Serial.println("автомат.");
    else
      Serial.println("ручная");
    Serial.print("Тип ДУТа\t\t");
    if (lls != nullptr)
    {
      switch (lls->getType())
      {
      case ILEVEL_SENSOR::ANALOGE_U:
        Serial.print("аналоговый U");
        break;
      case ILEVEL_SENSOR::ANALOGE_F:
        Serial.print("аналоговый F");
        break;
      case ILEVEL_SENSOR::RS485:
        Serial.print("цифровой RS485");
        break;
      case ILEVEL_SENSOR::BLE_ESKORT:
        Serial.print("цифровой BLE");
        break;
      }

      Serial.printf("\nАдрес ДУТа\t\t%d", lls->getNetadres());
      Serial.printf("\nЗначение N ДУТа\t\t%d", lls->getLevel());
      Serial.printf("\nУровень сигнала ble\t%d", lls->getRSSI());
    }
    else
      Serial.printf(("не выбран"));
    Serial.printf("\nНомер пролива\t\t%d", tar->getCountReffil());
    Serial.printf("\nОшибки\t\t\t%d", datemod.error);
    if (lls != nullptr)
      Serial.printf("\nОшибки LLS \t\t%d", lls->getError());

    Serial.println();
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  vTaskDelete(NULL);
}
#endif

// Обновление данных таблицы modbus
void modbus()
{
  modbus_update();
  if (tar->getNumRefill() >= tar->getVRefill()->size())
    datemod.kRefillNum = tar->getVRefill()->size();
  if (millis() > time_start_refill + 12000) // задержка в обновлении объема топлива для корректного составления отчета в Виалоне
  {
    datemod.v_full = tank->getFuelInTank();
    datemod.k_v_full = (tank->getFuelInTank() >> 16);
  }
  datemod.id1 = tar->getId_int();
  datemod.id2 = tar->getId_int() >> 16;
  // Serial.print"Id: %s %d %d\n", tar->getId(), datemod.id1, datemod.id2);
  datemod.k_in_Litr = countV->getKinLitr();
  datemod.vtank = tar->getVTank() / 10;
  datemod.kRefill = tar->getNumRefill();
  datemod.flowRate = countV->getFlowRate();
  datemod.pause = tar->getTimePause();
  datemod.timetarring = tar->getTimeTarring();
  datemod.typetarring = tar->getType();
  if (lls != nullptr)
  {
    datemod.typells = lls->getType();
    if (lls->getType() == ILEVEL_SENSOR::type::RS485)
    {
      datemod.adress = lls->getNetadres();
      datemod.k_adress = lls->getNetadres() >> 16;
    }
    if (lls->getType() == ILEVEL_SENSOR::type::BLE_ESKORT)
    {
      datemod.adress = lls->getNameBLE_int();
      datemod.k_adress = lls->getNameBLE_int() >> 16;
      datemod.rssi = lls->getRSSI();
    }
    datemod.resultN = lls->getLevel();
    datemod.resultNProliv = tar->getBackNRefill();
  }
  else
  {
    datemod.typells = ILEVEL_SENSOR::type::NO_LLS;
    datemod.adress = 0;
    datemod.k_adress = 0;
    datemod.resultN = 0;
    datemod.resultNProliv = 0;
    datemod.adress = 0;
    datemod.k_adress = 0;
    datemod.rssi = 0;
  }

#ifdef verATP
  datemod.llsATP = lls_ATP->getLevel();
#endif
}

void digitalpause()
{

  if (lls->getVecLevel()->size() < 20)
    return;

  uint32_t res = 0;
  for (auto vol : *lls->getVecLevel())
  {
    res += vol;
  }

  if (abs(static_cast<uint16_t>(res / lls->getVecLevel()->size()) - lls->getLevel()) < 3) // условие проверки уровня топлива по ДУТ (топливо перестало перетекать из других секций)
  {
    proceedTarring();
  }
}

void startPump()
{
  pump->on();
  if (datemod.mode != TAR && datemod.mode != PAUSE)
    countV->reset();
  else if (tar->getType() == tarring::MANUAL && autostop)
    proceedTarring();
}

void stopPump()
{
  pump->off();
}

String makeLlsDateToDisplay(ILEVEL_SENSOR *_lls)
{
  String ch{};
  if (datemod.mode == TAR || datemod.mode == PAUSE)
    ch = "\\r";
  else
    ch = " | ";
  if (_lls->getType() == ILEVEL_SENSOR::RS485)
  {
    if (lls->getError() == ILEVEL_SENSOR::error::NOT_FOUND)
      return "RS485 не найден!";
    else if (lls->getError() == ILEVEL_SENSOR::error::LOST)
      return "RS485" + ch + "Adr: " + String(_lls->getNetadres()) + ch + "Потерян!";
    else
      return "RS485" + ch + "Adr: " + String(_lls->getNetadres()) + ch + "N= " + String(_lls->getLevel());
  }
  else if (_lls->getType() == ILEVEL_SENSOR::BLE_ESKORT)
  {
    if (lls->getError() == ILEVEL_SENSOR::error::NOT_FOUND)
      return "BLE не найден!";
    else
      return "BLE" + ch + "RSSI: " + String(_lls->getRSSI()) + ch + "N= " + String(_lls->getLevel());
  }
#ifdef verAnalogInput
  else if (_lls->getType() == ILEVEL_SENSOR::ANALOGE_U)
  {
    if (lls->getError() == ILEVEL_SENSOR::error::NOT_FOUND)
      return "Analoge U не найден!";
    else
      return "Analoge U" + ch + "U= " + String(_lls->getLevel() / 100.0, 2) + " V";
  }
  else if (_lls->getType() == ILEVEL_SENSOR::ANALOGE_F)
  {
    if (lls->getError() == ILEVEL_SENSOR::error::NOT_FOUND)
      return "Analoge F не найден!";
    return "Analoge F" + ch + "F= " + String(_lls->getLevel()) + " Hz";
  }
#endif
  return {};
}

String saveLog()
{
  // if (!SPIFFS.begin(true))
  // {
  //   return "not mounting SPIFFS";
  // }
  String patch = "/" + String(LOG_FILE_NAME);

  if (!SPIFFS.exists(patch))
  {
    File file = SPIFFS.open(patch, FILE_WRITE);
    file.printf("Log tar\n ");
  }

  File file = SPIFFS.open(patch, FILE_APPEND);
  if (!file)
  {
    return "failed to open log file";
  }

  file.printf("\n\nID: %s\n", tar->getId());
  file.printf("Vtank: %d L | Ref: %d | k: %d imp/L\n", tar->getVTank() / 10, tar->getNumRefill(), countV->getKinLitr());

  if (lls == nullptr)
  {
    file.printf("No LLS | Time pause: %d min.\n", tar->getTimePause());
  }
  else
  {
    String type = "";
    String adr = "";
    switch (lls->getType())
    {
    case ILEVEL_SENSOR::ANALOGE_U:
      type = "Analoge U";
      break;
    case ILEVEL_SENSOR::ANALOGE_F:
      type = "Analoge F";
      break;
    case ILEVEL_SENSOR::RS485:
      type = "RS 485";
      adr = String(lls->getNetadres());
      break;
    case ILEVEL_SENSOR::BLE_ESKORT:
      type = "BLE ESCORT";
      adr = lls->getNameBLE();
      break;

    default:
      break;
    }

    file.printf("Type LLS: %s | Adr LLS: %s | Time pause: %d min.\n", type, adr, tar->getTimePause());
  }
  RtcDateTime dt = tar->getTStart();
  const uint SIZE = 20;
  char datestring[SIZE];

  snprintf_P(datestring,
             SIZE,
             PSTR("%02u.%02u.%04u %02u:%02u"),
             dt.Day(),
             dt.Month(),
             dt.Year(),
             dt.Hour(),
             dt.Minute());
  file.printf("Start: %s\n", datestring);

  dt = Rtc.GetDateTime();
  snprintf_P(datestring,
             SIZE,
             PSTR("%02u.%02u.%04u %02u:%02u"),
             dt.Day(),
             dt.Month(),
             dt.Year(),
             dt.Hour(),
             dt.Minute());
  file.printf("End: %s\n", datestring);
  file.printf("N,LLS,V\n");

  for (int i = 0; i < tar->getCountReffil(); i++)
  {
    file.println(tar->getResultRefill(i));
  }

  file.close();

  if ((SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024 < 300)
    return "no free space SPIFFS";

  return "";
}

String deleteLog()
{
  String patch = "/" + String(LOG_FILE_NAME);

  if (SPIFFS.exists(patch))
  {
    SPIFFS.remove(patch);
    if (!SPIFFS.exists(patch))
      return "<p><b>File delete successfully</b></p>";
    else
      return "<p><b>Deletion error. Try again...</b></p>";
  }
  return "<p><b>Deletion error. No files...</b></p>";
}

void wifiInit()
{
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("ATS");
  const char *SSID = "Trivi Tacho";
  const char *PASWD = "Rus__687";
  WiFi.begin(SSID, PASWD);
  int counter_WiFi = 0;
  while (WiFi.status() != WL_CONNECTED && counter_WiFi < 10)
  {
    delay(500);
    counter_WiFi++;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    const char *SSID = "ATS";
    const char *PASWD = "12_04_19";
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, PASWD, 1, 0, 2);
    WiFi.setTxPower(WIFI_POWER_7dBm);
  }
  Serial.printf(__DATE__);
  Serial.printf("Mac Address:\t");
  Serial.println(WiFi.macAddress());
  Serial.printf("IP Address:\t");
  Serial.println(WiFi.localIP());
}

String listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
  String response{};
  // Serial.print"Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root)
  {
    // Serial.println("- failed to open directory");
    return response;
  }
  if (!root.isDirectory())
  {
    // Serial.println(" - not a directory");
    return response;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      // Serial.print("  DIR : ");
      // Serial.println(file.name());
      if (levels)
      {
        listDir(fs, file.name(), levels - 1);
      }
    }
    else
    {
      // Serial.print("  FILE: ");
      // Serial.print(file.name());
      // Serial.print("\t\tSIZE: ");
      // Serial.println(file.size());
      response += "<p><a href='" + String(file.name()) + "'>" + file.name() + "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;" + file.size() + " byte" + "</a>" + "</p>";
    }
    file = root.openNextFile();
  }
  return response;
}

void getDataLog(AsyncWebServerRequest *request, String file)
{
  AsyncWebServerResponse *response = request->beginResponse(SPIFFS, file, String(), true);
  response->addHeader("Content-Type", "application/octet-stream");
  response->addHeader("Content-Description", "File Transfer");
  // response->addHeader("Content-Disposition", "attachment; filename='data.csv'");
  response->addHeader("Pragma", "public");
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
}

void delete_lls()
{
  if (lls != nullptr)
  {
    while (lls->getFlagUpgate())
      delay(10);
    delete lls;
    lls = nullptr;
    flag_dell_lls = false;
  }
}