#pragma once

#include <Arduino.h>
class COUNTER
{
    unsigned long Kcount_ = 0;
    uint16_t k_in_Litr_ = 1000;
    uint16_t k_in_Litr_calibr_ = 1000;
    uint16_t flow_rate_ = 0; // скорость потока топлива
    uint32_t v_fuel_save_ = 0;

public:
    COUNTER(const uint16_t k_in_Litr = 1000)
    {
        if (k_in_Litr > 0)
        {
            k_in_Litr_ = k_in_Litr;
            k_in_Litr_calibr_ = k_in_Litr;
        }
    }

    void setKcount()
    {
        Kcount_++;
    }

    void setKinLitr(const uint16_t k_in_Litr)
    {
        if (k_in_Litr > 0)
            k_in_Litr_ = k_in_Litr;
    }

    void setKinLitrCalibr(const uint16_t k_in_Litr)
    {
        if (k_in_Litr > 0)
            k_in_Litr_calibr_ = k_in_Litr;
    }

    uint16_t getKinLitr() const
    {
        return k_in_Litr_;
    }

    const uint16_t updateFlowRate(const unsigned long period)
    {
        if (period && v_fuel_save_ != getVFuel())
            flow_rate_ = 0.2 * flow_rate_ + 0.8 * (6000 * (getVFuel() - v_fuel_save_) / period);
        else
            flow_rate_ = 0;
        v_fuel_save_ = getVFuel();
        return flow_rate_;
    }

    const uint16_t getFlowRate() const
    {
        return flow_rate_;
    }

    // protected:
    const float getVFuel() const
    {
        return round(100.0 * Kcount_ / (1.0 * k_in_Litr_)); // вычисление объема пролитого топлива
    }

    const float getVFuelCalibr() const
    {
        return round(100.0 * Kcount_ / (1.0 * k_in_Litr_calibr_)); // вычисление объема пролитого топлива для калибровки
    }

    void reset()
    {
        Kcount_ = 0;
        flow_rate_ = 0;
        v_fuel_save_ = 0;
    }
};