#include "main.h"
#ifdef verAnalogInput
#include "LS_ANALOG_U.h"
#include "LS_ANALOG_F.h"
#endif
#include "LS_RS485.h"
#include "LS_BLE.h"
#include "LS_EMPTY.h"
// #include <TimeUtil.h>

void setup()
{
    pinMode(GPIO_NUM_2, OUTPUT);
    pinMode(INDI_F_PIN_, OUTPUT);
    digitalWrite(INDI_F_PIN_, HIGH); // индикация загрузки
    pinMode(INIDICATE_COUNT, OUTPUT);
    digitalWrite(INIDICATE_COUNT, LOW);

    Serial.begin(115200);
    serialLS.begin(19200, SERIAL_8N1, RXLS, TXLS);
    serialHMI.begin(19200, SWSERIAL_8N1, RXDNEX, TXDNEX, false, 256);

    serialMB.begin(19200);

    hmi.echoEnabled(false);
    hmi.hmiCallBack(onHMIEvent);
    hmi("rest");

    Rtc.Begin();
    SPIFFS.begin(true);

#ifdef verAnalogInput
    ads.setGain(GAIN_ONE);
    ads.begin();
    lls_analog_u = new LS_ANALOG_U(ads, 2);
    lls_analog_f = new LS_ANALOG_F();
#endif

    flash.begin("eerom", false);
    int k = flash.getInt("impulse_count", 1680); // чтение из eerom значения K счетчика
    countV = new COUNTER(k);

    lls_Empty = new LS_EMPTY();
    lls = lls_Empty;

    tank = new TANK(countV);
    tar = new TARRING(countV, tank);
    pump = new Out(OUT_PUMP);

    lls_RS485 = new LS_RS485(&serialLS, 1);
    lls_Ble = new LS_BLE();
    lls_Ble->echoEnabled(false);

    pinMode(IN_KCOUNT, INPUT_PULLUP);           // инициализация входа импульсов ДАРТ
    attachInterrupt(IN_KCOUNT, rpmFun, CHANGE); // функция прерывания

    modbus_configure(&serialMB, 19200, 1, 0, SIZE, datemod.au16data);
    modbus_update_comms(19200, 1);

#ifdef verATP
    lls_ATP = new LS_RS485(&serialLS, 100);
#endif

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
        4096,                       /* Размер стека задачи */
        NULL,                       /* Параметр задачи */
        1,                          /* Приоритет задачи */
        NULL,                       /* Идентификатор задачи, чтобы ее можно было отслеживать */
        1);                         /* Ядро для выполнения задачи (0) */

    xTaskCreatePinnedToCore(
        updateLS,        /* */
        "Task_updateLS", /* Обновление данных от ДУТ */
        16364,           /* Размер стека задачи */
        NULL,            /* Параметр задачи */
        1,               /* Приоритет задачи */
        NULL,            /* Идентификатор задачи, чтобы ее можно было отслеживать */
        0);              /* Ядро для выполнения задачи (0) */

#ifdef PRINTDEBUG
    xTaskCreatePinnedToCore(
        printDebugLog,        /*  */
        "Task_printDebugLog", /* Печать отладочной информации*/
        4096,                 /* Размер стека задачи */
        NULL,                 /* Параметр задачи */
        3,                    /* Приоритет задачи */
        NULL,                 /* Идентификатор задачи, чтобы ее можно было отслеживать */
        tskNO_AFFINITY);      /* Ядро для выполнения задачи (0) */
#endif

    xTaskCreatePinnedToCore(
        sendNextion,        /* обновление данных HMI */
        "Task_sendNextion", /* Название задачи */
        8192,               /* Размер стека задачи */
        NULL,               /* Параметр задачи */
        4,                  /* Приоритет задачи */
        NULL,               /* Идентификатор задачи, чтобы ее можно было отслеживать */
        1);

    xTaskCreatePinnedToCore(
        readNextion,        /* чтение данных от HMI */
        "Task_readNextion", /* Название задачи */
        8192,               /* Размер стека задачи */
        NULL,               /* Параметр задачи */
        5,                  /* Приоритет задачи */
        NULL,               /* Идентификатор задачи, чтобы ее можно было отслеживать */
        1);

    wifiInit();
    server.begin();
    AsyncElegantOTA.begin(&server); // Start ElegantOTA

    digitalWrite(INDI_F_PIN_, LOW);
    datemod.controlFlowrate = true;

    // таймер для светодиода индикации счетчика
    My_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(My_timer, &onTimer, true);
    timerAlarmWrite(My_timer, 1000000, true);
    timerAlarmEnable(My_timer);
}

void loop()
{
    errors();
    hmi.listen();
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
    case PUMPINGOUT:
        modePumpOut();
        break;
    case END_TAR:
        exitTarring();
        break;
    case MESSAGE:
        stopPump();
        break;
    case SEARCH_BLE:
        break;
    }

    if (lls->getType() == ILEVEL_SENSOR::ANALOGE_F)
        digitalWrite(INDI_F_PIN_, HIGH);
    else
        digitalWrite(INDI_F_PIN_, LOW);
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

        if (flag_HMI_send)
            hmi("sendme");

        else
        {
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

                lls->getType() == ILEVEL_SENSOR::NO_LLS ? str = "" : str = makeLlsDateToDisplay(lls);

                bt = static_cast<int>(lls->getType());

#ifdef verATP
                if (lls_ATP->getError() == ILEVEL_SENSOR::NO_ERROR)
                {
                    if (lls_ATP->getTarLevel() != -1.0)
                    {
                        res = String(lls_ATP->getTarLevel(), 1);
                        res += " l";
                    }
                }
                if (lls_ATP->getError() == ILEVEL_SENSOR::CLOSURE)
                    res = "closure";
#endif

                hmi.sendScreenMenu(datestring, countV->getKinLitr(), res, str, bt);
                break;

            case PUMPINGOUT:
                hmi.sendScreenPump_Out(tar->getVfuel(), countV->getFlowRate());
                hmi("pump_out.b1.picc", pump->get() == OFF ? 20 : 15);
                break;

            case PUMPINGAUTO:

                hmi.sendScreenPump_Auto(tar->getVfuel(), countV->getFlowRate(), str);
                hmi("pump_auto.b4.picc", pump->get() == OFF ? 14 : 15);
                break;

            case COUNT:
                lls->getType() == ILEVEL_SENSOR::NO_LLS ? str = "" : str = makeLlsDateToDisplay(lls);

                if (counter_display_resetring != tar->getCountReffil())
                {
                    for (int i = 0; i < tar->getCountReffil(); i++)
                        res += String(tar->getRefill(i) / 10.0, 1) + " l\\r";
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
                lls->getType() == ILEVEL_SENSOR::NO_LLS ? str = "ДУТ не подключен" : str = makeLlsDateToDisplay(lls);

                level = map(tar->getVfuel(), 0, tar->getVTank(), 0, 100);
                hmi("tar.b4.picc", pump->get() == OFF ? 23 : 24);
                uint tmp_time_pause;
                // if (autostop && tar->getType() == tarring::MANUAL)
                if (autostop && tar->getTimePause() != 0)
                {
                    tmp_time_pause = 60 * (tar->getTimePause()) - (millis() - start_pause) / 1000;
                    if (tmp_time_pause == 0)
                        tmp_time_pause = 1;
                }
                else
                    tmp_time_pause = tar->getTimePause() * 60;

                if (tmp_time_pause > tar->getTimePause() * 60)
                    tmp_time_pause = tar->getTimePause() * 60;

                hmi.sendScreenTarring(tar->getVfuel() - tar->getBackRefill(), tar->getVfuel(), tar->getCountReffil(), tar->getNumRefill() - tar->getCountReffil(), countV->getFlowRate(), str, tar->getTimeTarring(), level, tmp_time_pause, lls->getDoConnect());
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

            case END_TAR_HMI:
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
                lls->getType() == ILEVEL_SENSOR::NO_LLS ? hmi.sendScreenEnd_Tar(str, j0) : hmi.sendScreenEnd_Tar(str, j0, tar->getVRefill(), tar->getNRefill());
                break;

            case SETTING:
                lls->getType() == ILEVEL_SENSOR::NO_LLS ? str = " не подключен" : str = makeLlsDateToDisplay(lls);

                hmi.sendScreenSetting(tar->getTimeTarring(), str);
                break;

            case SEARCH_BLE:
                if (lls->getType() != ILEVEL_SENSOR::NO_LLS)
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
        }

        flag_HMI_send = !flag_HMI_send;
        vTaskDelay(pdMS_TO_TICKS(TIME_UPDATE_HMI));
    }
    vTaskDelete(NULL);
}

/*  ---------- Режим Меню ---------- */
void modeMenu()
{
    tar->reset();
    pump->off();
    datemod.id1 = 0;
    datemod.id2 = 0;
    counter_display_resetring = 0;
    autostop = false;
    flag_conect_ok = true;
}

/*  ---------- Режим Автоматическая выдача топлива ---------- */
void modePumpOut()
{
    if (tar->getVTank() <= tar->getVfuel())
        pump->off();
}

/*  ---------- Режим Тарировка ---------- */
void modeTarring()
{
    if (tar->getCountReffil() == 0)
    {
        if (tar->getVTank() > 200 && tar->getNumRefill() > 4)
            tar->saveResultRefuil(lls);
        else
            hmi("page menu"); //   возврат в меню при некорректных данных
    }

    if (tar->getVTank() <= tar->getVfuel()) // условие окончания тарировки
        endTarring();

    if (tar->getVTankRefill() * tar->getCountReffil() <= tar->getVfuel() && tar->getCountReffil() != tar->getNumRefill()) // условие окончания очередного пролива
        endRefill();
}

/*  ---------- Режим Автоматическое выкачивание ---------- */
void modePumpAuto()
{
    if (millis() - worktime > 30000)
    {
        if (pump->get() == ON && countV->getFlowRate() < 5) // скорость потока менее 5 л/мин
            pump->off();
    }
    if (pump->get() == OFF)
        worktime = millis();
}

/*  ---------- Продолжение тарировки  ---------- */
void proceedTarring()
{
    tar->saveResultRefuil(lls); // запись результата пролива
    errors();
    if (datemod.error == 32)
        return;
    autostop = false;
    time_start_refill = millis();
    datemod.mode = TAR;
    pump->on();
}

/*  ---------- Окончание очередного пролива  ---------- */
void endRefill()
{
    stopPump();
    if (datemod.mode == TAR && autostop == false)
    {
        autostop = true;
        start_pause = millis();
        datemod.mode = PAUSE;
    }

    if (millis() - start_pause < 5000)
        return;

    if (countV->getFlowRate() > 0) // проверка, что топливо больше не поступает в бак
    {
        delay(100);
        return;
    }

    if (tar->getTimePause() == 0) // пауза если ДУТ цифровой
        digitalpause();
    else if (millis() - start_pause > tar->getTimePause() * 60000) // пауза если ДУТ аналоговый
        proceedTarring();
}

/*  ---------- Финал тарировки  ---------- */
void endTarring()
{
    autostop = true;
    stopPump();

    while (countV->getFlowRate() > 0) // проверка, что топливо больше не поступает в бак
        delay(1000);
    exitTarring();
}

/*  ---------- Выход из тарировки  ---------- */
void exitTarring()
{
    if (datemod.mode != END_TAR)
    {
        datemod.mode = END_TAR;
        if (tar->getTimePause() == 0)
            if (lls->getVecLevel()->size() == 0)
            {
                delay(100);
                return;
            }
        // start_pause = millis() + TIME_PAUSE_END_TAR / 4;
        tar->saveResultRefuil(lls);
    }
    // if (start_pause - millis() > 50)
    //     return;
    hmi("page t_end");
    // delay(200);
    datemod.mode = END_TAR_HMI;
}

/*  ---------- Считывание данных с ДУТ  ---------- */
void updateLS(void *pvParameters)
{
    for (;;)
    {
#ifdef verATP
        if (datemod.mode == MENU)
            lls_ATP->update();
#endif
        if (datemod.mode == TAR || datemod.mode == PAUSE || datemod.mode == COUNT)
            if (lls->getType() != ILEVEL_SENSOR::NO_LLS)
            {
                lls->update();
                // test();
                digitalWrite(GPIO_NUM_2, ON);
                delay(10);
                digitalWrite(GPIO_NUM_2, OFF);
            }

        vTaskDelay(pdMS_TO_TICKS(TIME_UPDATE_LLS));
    }
    vTaskDelete(NULL);
}

/*  ---------- Считывание данных с ДУТ  ---------- */
void updateLS()
{
#ifdef verATP
    if (datemod.mode == MENU)
        lls_ATP->update();
#endif

    if (datemod.mode == TAR || datemod.mode == PAUSE || datemod.mode == COUNT)
        if (lls->getType() != ILEVEL_SENSOR::NO_LLS)
        {
            lls->update();
            // test();
        }
}

/*  ---------- Вычисление скорости потока  ---------- */
void calculate_speedPump(void *pvParameters)
{
    for (;;)
    {
        countV->updateFlowRate();
        vTaskDelay(pdMS_TO_TICKS(TIME_UPDATE_SPEED_PUMP));
    }
    vTaskDelete(NULL);
}

/*  ---------- Ошибки ДУТ  ---------- */
void errors()
{
    int error = 0;
    if (datemod.mode == MESSAGE || datemod.mode == END_TAR || datemod.mode == CALIBR || datemod.mode == MENU)
        return;

    if (lls->getType() != ILEVEL_SENSOR::NO_LLS) // если ДУТ подключен
    {
        if (datemod.mode == SEARCH_BLE) // ДУТ BLE и режим поиск
            return;

        // if (tar->getType() == tarring::AUTO) //  если тарировка в автоматическом режиме
        if (tar->getTimePause() == 0)
        {
            error = lls->getError(); // чтение ошибок ДУТ

            if (datemod.mode == TAR)
                if (tar->getCountReffil() > 2) // проверка увеличения данных с ДУТа в проливах
                    if (tar->getNRefill(tar->getCountReffil() - 1) < 10 + tar->getNRefill(tar->getCountReffil() - 2))
                        error |= 1 << 5;

            if (datemod.mode == SETTING) // проверка, что тарировка начинается с приемлемого уровня ДУТ
                if (lls->getLevel() > lls->getLevelStart())
                    error |= 1 << 7;
        }
    }

    // проверка, что 30 секунд скорость пролива меньше 2л/мин
    if (datemod.controlFlowrate)
    {
        if (pump->get() == ON)
        {
            if (countV->getFlowRate() < 2)
            {
                if (millis() > pump->getTimeStart() + 30000)
                    error |= 1 << 6;
            }
            else
                pump->setTimeStart();
        }
    }
    if (error)
    {
        // Serial.printf("\nErr: %d", error);
        if (datemod.mode != MESSAGE)
            hmi("page message");
        datemod.mode = MESSAGE;
        pump->off();
        datemod.error = error;
    }
}

/*  ---------- Функция подсчета импульсов с ДАРТ  ---------- */
void rpmFun()
{
    if (micros() - time_counter_imp > MIN_DURATION)
    {
        countV->setKcount();
        if (countV->getK() % 20 == 0)
            digitalWrite(INIDICATE_COUNT, !digitalRead(INIDICATE_COUNT));
    }
    time_counter_imp = micros();
}

/*  ---------- Парсинг полученых данных от дисплея Nextion  ---------- */
void onHMIEvent(String messege, String data, String response)
{
    if (messege.isEmpty())
        return;

    switch (messege.toInt())
    {
    case 1:
        datemod.mode = MENU;
        break;
    case 2:
        datemod.mode = CALIBR;
        break;
    case 3:
        datemod.mode = COUNT;
        break;
    case 7:
        datemod.mode = SEARCH_BLE;
        break;
    case 8:
        if (!autostop)
            datemod.mode = TAR;
        else if (datemod.mode != END_TAR)
            datemod.mode = PAUSE;
        break;
    case 9:
        datemod.mode = PUMPINGAUTO;
        break;
    case 10:
        datemod.mode = PUMPINGOUT;
        break;
    case 11:
        datemod.mode = END_TAR_HMI;
        break;
    case 12:
        datemod.mode = MESSAGE;
        break;
    case 13:
        datemod.mode = SETTING;
        break;
    default:
        break;
    }

    /*  ----------  Экран Меню  ---------- */
    if (messege == "menu")
    {
        int k = flash.getInt("impulse_count", 2000); // чтение из eerom значения K счетчика
        countV->setKinLitr(k);
        datemod.error = 0;
        if (lls->getType() != ILEVEL_SENSOR::NO_LLS)
        {
            // сброс ДУТа BLE без номера и не найденного в поиске
            if (lls->getNameBLE() == "" || lls->getError() == ILEVEL_SENSOR::error::NOT_FOUND)
                delete_lls();
        }
    }

#ifdef verAnalogInput
    else if (messege == "au!")
    {
        lls = lls_analog_u;
        lls->search();
    }

    else if (messege == "ag!")
    {
        lls = lls_analog_f;
        digitalWrite(INDI_F_PIN_, HIGH);
        lls->search();
    }
#endif

    else if (messege == "rs485!")
    {
        lls = lls_RS485;
        if (lls->search())
            tar->setTimePause(0);
    }

    else if (messege == "ble!")
    {
        lls = lls_Ble;
        lls->newBLE("");
    }

    else if (messege == "no_lls!")
    {
        delete_lls();
    }

    else if (messege == "TD_") // получение имени ДУТа BLE Эскорт
    {
        lls->newBLE(messege + data);
        lls->search();
    }

    else if (messege == "pump")
    {
        pump->get() ? stopPump() : startPump();
    }

    else if (messege == "endtarr") // окончание тарировки
    {
        exitTarring();
    }

    else if (messege == "save") // сохранение тарировки
    {
        saveLog();
    }

    else if (messege == "resetring")
    {
        if (tar->getVfuel())
            tar->saveResultRefuil(lls);
        return;
    }
    else if (messege == "reset")
    {
        tar->reset();
        counter_display_resetring = -1;
    }

    else if (messege == "CALIBR")
    {
        data.toInt() ? countV->reset() : stopPump();
    }

    else if (messege == "k_count") // получение промежуточного значения k_in_Litr
    {
        countV->setKinLitrCalibr(data.toInt());
    }

    else if (messege == "save_k") // получение нового К, запись в память
    {
        flash.putInt("impulse_count", data.toInt()); // запись в eerom значения K счетчика
        countV->setKinLitr(data.toInt());
    }
    else if (messege == "id") // получение номера тарируемого объекта
    {
        tar->setId(data);
        tar->setTStart(Rtc.GetDateTime());
    }
    else if (messege == "vtank") // получение объема бака
    {
        tank->setVTank(data.toInt() * 10);
    }
    else if (messege == "qt") // получение количества проливов
    {
        tar->setNumRefill(data.toInt());
    }

    /*  ----------  Получениее времени  ---------- */
    if (messege == "date")
    {
        auto index_sym = data.indexOf(';');
        auto DATE = data.substring(0, index_sym);
        auto TIME = data.substring(index_sym + 1, data.length());
        // Serial.printf("Date %s Time %s\n", DATE, TIME);

        // Для установки  времени
        RtcDateTime compiled = RtcDateTime(DATE.c_str(), TIME.c_str());
        Rtc.SetDateTime(compiled);
    }

    if (messege == "pause")
    {
        if (data.toInt())
        {
            tar->setTimePause(data.toInt());
        }
        else
        {
            // tar->setType(tarring::AUTO);
            if (lls->getType() != ILEVEL_SENSOR::NO_LLS)
                tar->setTimePause(0);
            else
                hmi("select0.val", tar->getTimePause());
        }
    }

    if (messege == "clear_err")
    {
        // switch (datemod.error)
        // {
        // case ILEVEL_SENSOR::NOT_FOUND:
        //   datemod.mode = TAR;
        //   if (lls->getType() != ILEVEL_SENSOR::NO_LLS)
        //     if (lls->getType() == ILEVEL_SENSOR::RS485)
        //     {
        //       Serial.println("Дут RS485 не найден. Ищем...");
        //       lls->search();
        //     }
        //     if (lls->getType() == ILEVEL_SENSOR::BLE_ESKORT)
        //     {
        //       Serial.println("Дут BLE не найден. Ищем...");
        //       lls->search();
        //     }
        //   datemod.error = 0;
        //   autostop = false;
        //   break;
        // case ILEVEL_SENSOR::LOST:
        //   datemod.mode = TAR;
        //   if (lls->getType() != ILEVEL_SENSOR::NO_LLS)
        //     if (lls->getType() == ILEVEL_SENSOR::RS485)
        //     {
        //       Serial.println("Дут RS485 потерян. Ищем...");
        //       lls->searchLost();
        //     }
        //     if (lls->getType() == ILEVEL_SENSOR::BLE_ESKORT)
        //     {
        //       Serial.println("Дут BLE потерян. Ищем...");
        //       lls->search();
        //     }
        //   datemod.error = 0;
        //   autostop = false;
        //   break;

        // default:
        //   break;
        // }
        if (datemod.error == 32)
            tar->deleteResultRefuil(lls);

        datemod.error = 0;
        lls->clearError();
    }

    if (messege == "contrFlowrate")
    {
        if (data == "1")
            datemod.controlFlowrate = true;
        if (data == "0")
            datemod.controlFlowrate = false;
    }
}

/*  ---------- данные с Nextion ---------- */
void readNextion(void *pvParameters)
{
    for (;;)
    {
        hmi.listen();
        // vPrintString("readNextion");
        // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TIME_UPDATE_HMI));
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
            Serial.print("Откачка топлива автоматом\n");
            break;
        case CALIBR:
            Serial.print("Калибровка счетчика\n");
            break;
        case MENU:
            Serial.print("Меню\n");
            break;
        case TAR:
            Serial.print("Тарировка\n");
            break;
        case SETTING:
            Serial.print("Настройка тарировки\n");
            break;
        case COUNT:
            Serial.print("Счетчик\n");
            break;
        case PUMPINGOUT:
            Serial.print("Выдача топлива\n");
            break;
        case END_TAR:
            Serial.print("Конец тарировки\n");
            break;
        case PAUSE:
            Serial.print("Пауза\n");
            break;
        case MESSAGE:
            Serial.print("Собщения\n");
            break;
        case SEARCH_BLE:
            Serial.print("Поиск BLE\n");
            break;
        default:
            Serial.print(datemod.mode + "\n");
        }
        // Serial.printf("\nТекущее значение К\t%d", countV->getKinLitr());
        Serial.printf("Объем бака\t\t%d\n", tar->getVTank());
        Serial.printf("Залито топлива\t\t%d\n", tar->getVfuel());
        Serial.printf("Номер автомобиля\t%s\n", tar->getId());
        Serial.printf("Объем проливов\t\t%d\n", tar->getVTankRefill());
        Serial.printf("Количество проливов\t%d\n", tar->getNumRefill());
        Serial.printf("Пауза между проливами\t%d\n", tar->getTimePause());
        Serial.print("Тип тарировки\t\t");
        if (tar->getTimePause())
            Serial.println("ручная");
        else
            Serial.println("автомат.");
        Serial.print("Тип ДУТа\t\t");
        if (lls->getType() != ILEVEL_SENSOR::NO_LLS)
        {
            switch (lls->getType())
            {
            case ILEVEL_SENSOR::ANALOGE_U:
                Serial.print("аналоговый U\n");
                break;
            case ILEVEL_SENSOR::ANALOGE_F:
                Serial.print("аналоговый F\n");
                break;
            case ILEVEL_SENSOR::RS485:
                Serial.print("цифровой RS485\n");
                break;
            case ILEVEL_SENSOR::BLE_ESKORT:
                Serial.print("цифровой BLE\n");
                break;
            }

            Serial.printf("Адрес ДУТа\t\t%d\n", lls->getNetadres());
            Serial.printf("Значение N ДУТа\t\t%d\n", lls->getLevel());
            Serial.printf("Уровень сигнала ble\t%d\n", lls->getRSSI());
        }
        else
            Serial.printf(("не выбран\n"));
        // Serial.printf("\nНомер пролива\t\t%d", tar->getCountReffil());
        Serial.printf("Ошибки\t\t\t%d\n", datemod.error);
        Serial.printf("ДУТ АТП\t\t\t%d\n", lls_ATP->getLevel());

#ifdef verATP
        Serial.printf("Ошибки LLS АТП \t\t%d", lls_ATP->getError());
#endif
        Serial.println();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelete(NULL);
}
#endif

/*  ---------- Обновление данных таблицы modbus ---------- */
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
    datemod.k_in_Litr = countV->getKinLitr();
    datemod.vtank = tar->getVTank() / 10;
    datemod.kRefill = tar->getNumRefill();
    datemod.flowRate = countV->getFlowRate();
    datemod.pause = tar->getTimePause();
    datemod.timetarring = tar->getTimeTarring();
    // datemod.typetarring = tar->getType();
    if (lls->getType() != ILEVEL_SENSOR::NO_LLS)
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
    if (lls->getVecLevel()->size() < ILEVEL_SENSOR::MAX_SIZE)
        return;

    if (!flag_conect_ok)
        return;

    uint32_t res = 0;
    for (auto vol : *lls->getVecLevel())
        res += vol;

    if (abs(static_cast<uint16_t>(res / lls->getVecLevel()->size()) - lls->getLevel()) < 3) // условие проверки уровня топлива по ДУТ (топливо перестало перетекать из других секций)
    {
        proceedTarring();
    }
}

void startPump()
{
    pump->on();
    if (datemod.mode != TAR && datemod.mode != PAUSE)
    {
        countV->reset();
    }
    else if (tar->getTimePause() != 0 && autostop)
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
        ch = " ";
    if (_lls->getType() != ILEVEL_SENSOR::NO_LLS)
    {
        if (_lls->getType() == ILEVEL_SENSOR::RS485)
        {
            if (_lls->getError() == ILEVEL_SENSOR::error::NOT_FOUND)
                return "ДУТ не найден!";
            else if (_lls->getError() == ILEVEL_SENSOR::error::LOST)
                return "ДУТ (RS485)" + ch + "Adr: " + String(_lls->getNetadres()) + ch + "Потерян!";
            else
            {
                String str = "---";
                if (_lls->getLevel() <= ILEVEL_SENSOR::MAX_DIGITAL_N && _lls->getLevel() >= ILEVEL_SENSOR::MIN_DIGITAL_N)
                    str = String(_lls->getLevel());
                return "ДУТ (RS485)" + ch + "Adr: " + String(_lls->getNetadres()) + ch + "N= " + str;
            }
        }
        else if (_lls->getType() == ILEVEL_SENSOR::BLE_ESKORT)
        {
            if (_lls->getError() == ILEVEL_SENSOR::error::NOT_FOUND)
                return "ДУТ не найден!";
            else
                return _lls->getNameBLE() + ch + "RSSI: " + String(_lls->getRSSI()) + ch + "N=" + String(_lls->getLevel());
        }
#ifdef verAnalogInput
        else if (_lls->getType() == ILEVEL_SENSOR::ANALOGE_U)
        {
            if (_lls->getError() == ILEVEL_SENSOR::error::NOT_FOUND)
                return "ДУТ не найден!";
            else
                return "ДУТ (U)" + ch + "U= " + String(_lls->getLevel() / 100.0, 2) + " V";
        }
        else if (_lls->getType() == ILEVEL_SENSOR::ANALOGE_F)
        {
            if (_lls->getError() == ILEVEL_SENSOR::error::NOT_FOUND)
                return "ДУТ не найден!";
            return "ДУТ (F)" + ch + "F= " + String(_lls->getLevel()) + " Hz";
        }
#endif
    }
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
        return "failed to open log file";

    file.printf("\n\nID: %s\n", tar->getId());
    file.printf("Vtank: %d L | Ref: %d | k: %d imp/L\n", tar->getVTank() / 10, tar->getNumRefill(), countV->getKinLitr());

    if (lls == nullptr)
        file.printf("No LLS | Time pause: %d min.\n", tar->getTimePause());
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
        file.println(tar->getResultRefill(i));

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
    WiFi.setHostname("ATS");
    WiFi.mode(WIFI_AP);
    if (WiFi.status() != WL_CONNECTED)
    {
        const char *SSID = "ATS";
        const char *PASWD = "12_04_19";
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
                listDir(fs, file.name(), levels - 1);
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
    // Serial.println("Delete LLS");
    if (lls->getType() == ILEVEL_SENSOR::BLE_ESKORT)
        lls->newBLE("");
    lls = lls_Empty;
}

void IRAM_ATTR onTimer()
{
    digitalWrite(INIDICATE_COUNT, OFF);
}