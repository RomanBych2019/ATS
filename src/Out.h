#include <Arduino.h>

#define ON HIGH
#define OFF LOW

class Out
{
private:
    int pin_, pin_indicate_;
    u_long time_start_ = 0;
    uint32_t lls_start_ = 0;

public:
    Out() {}
    Out(int pin, int indicate) : pin_(pin) //конструктор для выходов на плате + индикатор включения (при необходимости)
    {
        pinMode(pin, OUTPUT);
        // if (indicate == 2)
        // {
        //     pin_indicate_ = indicate;
        //     pinMode(pin_indicate_, OUTPUT);
        // }
        digitalWrite(pin_, OFF);
        off();
    }
    Out(int pin) : pin_(pin) //конструктор для выходов на плате
    {
        pinMode(pin, OUTPUT);
        off();
    }
    //возращение статуса насоса
    int get()
    {
        return digitalRead(pin_);
    }
    // включение насоса
    void on()
    {
        time_start_ = millis();
        digitalWrite(pin_, ON);
        // digitalWrite(pin_indicate_, ON);
    }
    //выключение насоса
    void off()
    {
        digitalWrite(pin_, OFF);
        // digitalWrite(pin_indicate_, OFF);
    }

    u_long getTimeStart()
    {
        return time_start_;
    }

    // void setLlsStart(uint32_t l)
    // {
    //     lls_start_ = l;
    // }

    // uint32_t getLlsStart()
    // {
    //     return lls_start_;
    // }

    ~Out(){};
};