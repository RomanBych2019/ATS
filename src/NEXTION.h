#pragma once

#include <Arduino.h>
#include <vector>
#include <string>
#include "SoftwareSerial.h"

class NEXTION
{
private:
    SoftwareSerial *serialNextion_;
    boolean flag = false;
    struct graph
    {
        uint x0_ = 380;
        uint y0_ = 330;
        uint x_max_ = 787;
        uint y_min_ = 50;
    } graph_;

public:
    NEXTION(SoftwareSerial &port) : serialNextion_(&port) {}

    void send(String const &data) const
    {
        send_(data);
    }

    void send(String const &dev, const double data) const
    {
        send_(dev, data);
    }

    void send(String const &dev, const String &data) const
    {
        send_(dev, data);
    }

    //  данные на экране Меню
    void sendScreenMenu(char const *ch, uint const n0, String const &v_atp, String const &t1, int const bt) const
    {
        send_("menu.cb0.val", bt);
        send_("menu.t0.txt", ch);
        send_("menu.t1.txt", t1);
        send_("calibr.n0.val", n0);
        send_("menu.t2.txt", v_atp);
    }

    //  данные на экране Автоматическая выдача топлива
    void sendScreenPump_Out(uint32_t const x1, uint16_t const x2) const
    {
        send_("pump_out.x1.val", x1);
        send_("pump_out.x2.val", x2);
    }

    //  данные на экране Автоматическое выкачивание
    void sendScreenPump_Auto(uint32_t const x0, uint16_t const x2, String const &t1) const
    {
        send_("pump_auto.x0.val", x0);
        send_("pump_auto.x2.val", x2);
        send_("pump_auto.t1.txt", t1);
    }

    //  данные на экране Счетчик
    void sendScreenCounter(uint32_t const x0, uint16_t const x1, uint16_t const x2, String const &t0) const
    {
        send_("counter.x0.val", x0);
        send_("counter.x1.val", x1);
        send_("counter.x2.val", x2);
        send_("counter.t0.txt", t0);
    }
    //  данные на экране Счетчик
    void sendScreenCounter(String const &t1) const
    {
        send_("counter.t1.txt", t1);
    }

    //  данные на экране Калибровка счетчика
    void sendScreenCalibration(uint32_t const x0, uint32_t const n1) const
    {
        send_("calibr.x0.val", x0);
        send_("calibr.n1.val",n1);
    }

    //  данные на экране Сообщения
    void sendScreenMessage(String const &t0) const
    {
        send_("message.t0.txt", t0);
    }

    //  данные на экране Настройка паузы
    void sendScreenSetting(long const t1, String const &t3)
    {
        send_("set_lls.t1.txt", convertStringTime_(t1));
        send_("set_lls.t3.txt", t3);
    }

    //  данные на экране Тарировка
    void sendScreenTarring(uint32_t const x0, uint32_t const x1, uint n1, uint const n2, uint16_t const x3, String const &t0, uint16_t const t7, char const j0, uint16_t const t6)
    {
        send_("tarring.x0.val", x0);
        send_("tarring.x1.val", x1);
        send_("tarring.n1.val", n1);
        send_("tarring.n2.val", n2);

        String dateConvert = convertStringTime_(t7);
        send_("tarring.t7.txt", dateConvert);
        send_("tarring.j0.val", j0);
        send_("tarring.x3.val", x3);
        send_("tarring.t0.txt", t0);
        if (t6 == 0)
            send_("tarring.t6.txt", "AUTO");
        else
        {
            dateConvert = convertStringTime_(t6);
            send_("tarring.t6.txt", dateConvert);
        }
        flag = false;
    }

    //  данные на экране Окончания тарировки при автоматической тарировке (с выводом графика тарировки)
    void sendScreenEnd_Tar(String const &t0, uint16_t j0, std::vector<uint32_t> *n, std::vector<uint32_t> *v)
    {
        if (!flag)
        {
            send_("t0.txt", t0);
            uint x_old = graph_.x0_;
            uint y_old = graph_.y0_;
            for (uint i = 1; i < v->size(); ++i)
            {
                uint x = map(n->at(i), 0, n->back(), graph_.x0_, graph_.x_max_);
                uint y = map(v->at(i), 0, v->back(), graph_.y0_, graph_.y_min_);
                send_("line " + String(x_old) + "," + String(y_old) + "," + String(x) + "," + String(y) + ",RED");
                send_("cir " + String(x) + "," + String(y) + ",4,RED");
                x_old = x;
                y_old = y;
            }
            flag = true;
        }
        send_("j0.val", j0);
    }

    //  данные на экране Окончания тарировки при ручной тарировке (без вывода графика тарировки)
    void sendScreenEnd_Tar(String const &t0, uint16_t j0)
    {
        if (!flag)
        {
            send_("t0.txt", t0);
            flag = true;
        }
        send_("j0.val", j0);
    }

    void sendScreenSearch_BLE(String const &t1) const
    {
        if (t1.endsWith("ДУТ не найден"))
            send_("search_ble.t1.pco=RED");
        else
            send_("search_ble.t1.pco", 50712);
        send_("search_ble.t1.txt", t1);
    }

private:
    // отправка на Nextion
    void send_(String const &dev) const
    {
        serialNextion_->print(dev); // Отправляем данные dev(номер экрана, название переменной) на Nextion
        sendEnd_();
    }
    void send_(String const &dev, double data) const
    {
        serialNextion_->print(dev + "=");
        serialNextion_->print(data, 0);
        sendEnd_();
    }
    void send_(String const &dev, const String &data) const
    {
        serialNextion_->print(dev + "=\"");
        serialNextion_->print(data + "\"");
        sendEnd_();
    }
    void sendEnd_() const
    {
        serialNextion_->write(0xff);
        serialNextion_->write(0xff);
        serialNextion_->write(0xff);
        // delay(5);
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