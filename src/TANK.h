#pragma once

#include <Arduino.h>

#include "COUNTER.h"

class TANK : private COUNTER
{
private:
    COUNTER *counter_;
    uint32_t v_tank_ = 0;

public:
    TANK(COUNTER *count) : counter_(count) {}
    
    const uint32_t getFuelInTank()
    {
        return counter_->getVFuel();
    }

    void setVTank(uint32_t v)
    {
        v_tank_ = v;
    }

    void reset()
    {
        setVTank(0);
        counter_->reset();
    }

    const uint32_t getVTank() {
        return v_tank_;
    }
};