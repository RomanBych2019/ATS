#pragma once

#include <Arduino.h>
#include "LEVEL_SENSOR.h"

class LS_EMPTY : public ILEVEL_SENSOR
{
public:
    LS_EMPTY()
    {
        // Serial.print("\n  - Create empty \n");
        type_ = ILEVEL_SENSOR::NO_LLS;
    }

    // ошибки
    void set_error_()
    {
    }

    void setNetadress(int netadress) override
    {
    }
    const int getNetadres() const override
    {
        return {};
    }

    void update() override
    {    
    }

    const bool search() override
    {
        return false;
    }

    const bool searchLost() override
    {
        return false;
    }

    ~LS_EMPTY()
    {
        Serial.print("\n  - Kill empty\n");
    };
};
