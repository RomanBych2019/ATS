#pragma once

#include <Arduino.h>
#include <driver/adc.h>
#include "esp_adc_cal.h"

#include "LEVEL_SENSOR.h"

class LS_ANALOG_U : public ILEVEL_SENSOR
{
private:
    uint GPIO_ = 0;
    int val_ = 0, median_ = 0, newest_ = 0, recent_ = 0, oldest_ = 0;
    float k_ = 0.2;
    Adafruit_ADS1115 ads_;
    int num_ads_ = -1;

    // ошибки
    void set_error_()
    {
        if (level_ < MIN_ANALOGE_U)
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_SEARCH_ERROR)
                error_ = error::NOT_FOUND; // обрыв датчика
        }
        else if (level_ > MAX_ANALOGE_U)
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_SEARCH_ERROR)
                error_ = error::CLOSURE; // показания датчика выше нормы (замыкание на питание)
        }
        else
        {
            counter_errror_ = 0;
            error_ = error::NO_ERROR;
        }
    }

public:
    LS_ANALOG_U(uint pin = 34) : GPIO_(pin) // конструктор  GPIO34
    {
        pinMode(GPIO_, INPUT);
        type_ = ILEVEL_SENSOR::ANALOGE_U;
        level_start_ = MIN_ANALOGE_U_START;
        // Serial.printf("\n  - Create Analoge_U, level_start: %d", level_start_);
    }

    LS_ANALOG_U(Adafruit_ADS1115 &ads, const int num_ads = 1) : ads_(ads), num_ads_(num_ads) // конструктор для каналов ADC
    {
        type_ = ILEVEL_SENSOR::ANALOGE_U;
        level_start_ = MIN_ANALOGE_U_START;
        Serial.printf("\n  - Create Analoge_U, level_start: %d", level_start_);
    }

    // обновление показаний
    void update() override
    {
        flag_upgate_ = true;
        if (GPIO_)
            val_ = analogRead(GPIO_);
        else
            val_ = ads_.readADC_SingleEnded(num_ads_);

        oldest_ = recent_;
        recent_ = newest_;
        newest_ = val_;
        median_ = median_of_3(oldest_, recent_, newest_);
        if (median_ < 20)
            level_ = 0;
        else
            level_ = constrain(map(median_, 145, 22330, 17, 1843), 0, MAX_ANALOGE_U);

        Serial.printf("\n\t-Analoge_U:  %d:   %d", val_, level_);

        setVLevel();
        set_error_();
        flag_upgate_ = false;
    }

    const bool search() override
    {
        // Serial.print("\nSearch AnalogeU\n");
        for (int i = 0; i < 20; i++)
        {
            update();
            delay(100);
        }
        return !error_;
    }

    ~LS_ANALOG_U(){
        // Serial.print("\n  - Kill analogU");
    };
};
