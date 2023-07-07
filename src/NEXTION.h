#pragma once

#include <Arduino.h>
// #include <vector>
// #include <string>
#include "SoftwareSerial.h"

#define CMD_READ_TIMEOUT 250
#define READ_TIMEOUT 50

#define MIN_ASCII 32
#define MAX_ASCII 255

class NEXTION
{
private:
    SoftwareSerial *_nextionSerial;
    bool _echo; // Response Command Show
    // Callback Function
    typedef void (*hmiListner)(String messege, String date, String response);
    hmiListner listnerCallback;
    boolean flag = false;
    struct graph
    {
        uint16_t x0_ = 380;
        uint16_t y0_ = 330;
        uint16_t x_max_ = 787;
        uint16_t y_min_ = 50;
    } graph_;

public:
    NEXTION(SoftwareSerial &port) : _nextionSerial(&port) {}

    void echoEnabled(bool echoEnabled)
    {
        _echo = echoEnabled;
    }

    // SET CallBack Event
    void hmiCallBack(hmiListner callBack)
    {
        listnerCallback = callBack;
    }

    void operator()(const String &data) const
    {
        send(data);
    }

    void operator()(const String &dev, const double data) const
    {
        send(dev, data);
    }

    void operator()(const String &dev, const String &data) const
    {
        send(dev, data);
    }

    // Listen For incoming callback event from HMI
    void listen()
    {
        // xSemaphoreTake(nexton_mutex, portMAX_DELAY);
        handle();
        // xSemaphoreGive(nexton_mutex);
    }

    //  данные на экране Меню
    void sendScreenMenu(char const *ch, uint const n0, String const &v_atp, String const &t1, int const bt) const
    {
        send("menu.select0.val", bt);
        send("menu.t0.txt", ch);
        send("menu.t1.txt", t1);
        send("calibr.n0.val", n0);
        send("menu.t2.txt", v_atp);
    }

    //  данные на экране Автоматическая выдача топлива
    void sendScreenPump_Out(uint32_t const x1, uint16_t const x2) const
    {
        send("pump_out.x1.val", x1);
        send("pump_out.x2.val", x2);
    }

    //  данные на экране Автоматическое выкачивание
    void sendScreenPump_Auto(uint32_t const x0, uint16_t const x2, String const &t1) const
    {
        send("pump_auto.x0.val", x0);
        send("pump_auto.x2.val", x2);
        send("pump_auto.t1.txt", t1);
    }

    //  данные на экране Счетчик
    void sendScreenCounter(uint32_t const x0, uint16_t const x1, uint16_t const x2, String const &t0) const
    {
        send("counter.x0.val", x0);
        send("counter.x1.val", x1);
        send("counter.x2.val", x2);
        send("counter.t0.txt", t0);
    }
    //  данные на экране Счетчик
    void sendScreenCounter(String const &t1) const
    {
        send("counter.t1.txt", t1);
    }

    //  данные на экране Калибровка счетчика
    void sendScreenCalibration(uint32_t const x0, uint32_t const n1) const
    {
        send("calibr.x0.val", x0);
        send("calibr.n1.val", n1);
    }

    //  данные на экране Сообщения
    void sendScreenMessage(String const &t0) const
    {
        send("message.t0.txt", t0);
    }

    //  данные на экране Настройка паузы
    void sendScreenSetting(long const t1, String const &t3)
    {
        send("set_lls.t1.txt", convertStringTime_(t1));
        send("set_lls.t3.txt", t3);
    }

    //  данные на экране Тарировка
    void sendScreenTarring(uint32_t const x0, uint32_t const x1, uint n1, uint const n2, uint16_t const x3, String const &t0, uint16_t const t7, char const j0, uint16_t const t6)
    {
        send("tar.x0.val", x0);
        send("tar.x1.val", x1);
        send("tar.n1.val", n1);
        send("tar.n2.val", n2);

        String dateConvert = convertStringTime_(t7);
        send("tar.t7.txt", dateConvert);
        send("tar.j0.val", j0);
        send("tar.x3.val", x3);
        send("tar.t0.txt", t0);
        if (t6 == 0)
            send("tar.t6.txt", "AUTO");
        else
        {
            dateConvert = convertStringTime_(t6);
            send("tar.t6.txt", dateConvert);
        }
        flag = false;
    }

    //  данные на экране Окончания тарировки при автоматической тарировке (с выводом графика тарировки)
    void sendScreenEnd_Tar(String const &t0, uint16_t j0, std::vector<uint32_t> *n, std::vector<uint32_t> *v)
    {
        if (!flag)
        {
            send("t_end.t0.txt", t0);
            uint x_old = map(n->at(0), 0, n->back(), graph_.x0_, graph_.x_max_);
            uint y_old = map(v->at(0), 0, v->back(), graph_.y0_, graph_.y_min_);
            send("cir " + String(x_old) + "," + String(y_old) + ",4,RED");

            for (uint i = 1; i < v->size(); ++i)
            {
                uint x = map(n->at(i), 0, n->back(), graph_.x0_, graph_.x_max_);
                uint y = map(v->at(i), 0, v->back(), graph_.y0_, graph_.y_min_);
                send("line " + String(x_old) + "," + String(y_old) + "," + String(x) + "," + String(y) + ",RED");
                send("cir " + String(x) + "," + String(y) + ",4,RED");
                x_old = x;
                y_old = y;
            }
            flag = true;
        }
        send("t_end.j0.val", j0);
    }

    //  данные на экране Окончания тарировки при ручной тарировке (без вывода графика тарировки)
    void sendScreenEnd_Tar(String const &t0, uint16_t j0)
    {
        if (!flag)
        {
            send("t_end.t0.txt", t0);
            flag = true;
        }
        send("t_end.j0.val", j0);
    }

    void sendScreenSearch_BLE(String const &t1) const
    {
        if (t1.endsWith("ДУТ не найден"))
            send("search_ble.t1.pco=RED");
        else
            send("search_ble.t1.pco", 50712);
        if (t1.startsWith("N 0 | RSSI 0"))
            send("search_ble.t1.txt", "Идет поиск...");
        else
            send("search_ble.t1.txt", t1);
    }

private:
    // отправка на Nextion
    void send(const String &dev) const
    {
        _nextionSerial->print(dev); // Отправляем данные dev(номер экрана, название переменной) на Nextion
        sendEnd();
    }
    void send(const String &dev, const double data) const
    {
        _nextionSerial->print(dev + "=");
        _nextionSerial->print(data, 0);
        sendEnd();
    }
    void send(const String &dev, const String &data) const
    {
        _nextionSerial->print(dev + "=\"");
        _nextionSerial->print(data + "\"");
        sendEnd();
    }
    void sendEnd() const
    {
        _nextionSerial->write(0xff);
        _nextionSerial->write(0xff);
        _nextionSerial->write(0xff);
    }

    String checkHex(byte currentNo)
    {
        if (currentNo < 10)
        {
            return "0x" + String(currentNo, HEX);
        }
        return String(currentNo, HEX);
    }

    String handle()
    {
        String response;
        String messege;
        String date;
        bool charEquals = false;
        unsigned long startTime = millis();

        // while ((millis() - startTime < CMD_READ_TIMEOUT))
        // {
        while (_nextionSerial->available())
        {
            int inc = _nextionSerial->read();
            response.concat(checkHex(inc) + " ");
            if (inc == 0x23)
            {
                messege.clear();
                date.clear();
                response.clear();
                charEquals = false;
            }
            else if (inc == 0xff)
            {
                response.clear();
            }
            else if (inc == 0x0A)
            {
                if (_echo)
                {
                    Serial.println("OnEvent : [ M : " + messege + " | D : " + date + " | R : " + response + " ]");
                }
                listnerCallback(messege, date, response);
                messege.clear();
                response.clear();
                date.clear();
                charEquals = false;
            }

            else if (inc == 0x66)
            {
                inc = _nextionSerial->read();
                response.concat(checkHex(inc) + " ");
                messege += String(inc, DEC);
                if (_echo)
                {
                    // Serial.println("OnEvent : [ M : " + String(messege.toInt()) + " | R : " + response + " ]");
                }

                listnerCallback(messege, date, response);
                messege.clear();
                date.clear();
                charEquals = false;
            }
            else
            {
                if (inc <= MAX_ASCII && inc >= MIN_ASCII)
                {
                    if (!charEquals)
                    {
                        if (char(inc) == '=')
                        {
                            charEquals = true;
                            date.clear();
                        }
                        else
                        {
                            messege += char(inc);
                        }
                    }
                    else
                    {
                        date += char(inc);
                    }
                }
                else
                {
                    messege.clear();
                    date.clear();
                    charEquals = false;
                }
            }
            delay(10);
        }
        return response;
        // }
        // return {};
    }

    String convertStringTime_(long date)
    {
        String temp = String(date / 60, DEC);
        temp += ':';
        if (date % 60 < 10)
            temp += '0';
        temp += String(date % 60, DEC);
        return temp;
    }
};