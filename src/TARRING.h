#pragma once

#include <Arduino.h>
#include <RtcDS3231.h>

#include "LEVEL_SENSOR.h"
#include "COUNTER.h"
#include "TANK.h"

namespace tarring
{
    enum mode
    {
        MANUAL,
        AUTO
    };
};

class TARRING
{
private:
    uint32_t refill_ = 0; // залито топлива в проливе
    static const int MAX_SIZE = 30;

    RtcDateTime t_start_; // время старта тарировки

    std::vector<uint32_t> v_ref_; // вектор объемов проливов
    std::vector<uint32_t> n_ref_; // вектор значений ДУТ проливов

    std::vector<String> v_total_; // вектор итогов тарировкиnum_reffil_

    uint32_t vtank_refill_ = 0; // объем пролива
    String id_ = "";             // id номер тарируемого объкта

    uint time_pause_ = 3; // пауза между проливами, мин

    uint num_reffil_ = 15;  // плановое кол-во проливов

    const int PUMPSPEED = 500; // скорость потока 10*литр/минута для вычисления времени тарировки
    COUNTER *countV_;
    TANK *tank_;
    tarring::mode mode_;

public:
    TARRING()
    {
        v_ref_.reserve(MAX_SIZE);
        n_ref_.reserve(MAX_SIZE);
        v_total_.reserve(MAX_SIZE);
    }

    TARRING(COUNTER *count, TANK *tank) : countV_(count), tank_(tank)
    {
        v_ref_.reserve(MAX_SIZE);
        n_ref_.reserve(MAX_SIZE);
        v_total_.reserve(MAX_SIZE);
    }

    std::vector<uint32_t> *getNRefill()
    {
        return &n_ref_;
    }

    uint32_t getNRefill(uint i)
    {
        if (n_ref_.size())
            return n_ref_.at(i);
        else
            return 0;
    }

    uint32_t getBackNRefill()
    {
        if (n_ref_.size())
            return n_ref_.back();
        else
            return 0;
    }

    std::vector<uint32_t> *getVRefill()
    {
        return &v_ref_;
    }

    uint32_t getBackRefill()
    {
        if (v_ref_.size())
            return v_ref_.back();
        else
            return 0;
    }

    uint32_t getRefill(uint i)
    {
        if (v_ref_.size())
            return v_ref_.at(i);
        else
            return 0;
    }

    void setId(String id)
    {
        id_ = id;
    }
    const String getId()
    {
        return id_;
    }

    const int getId_int()
    {
        String result{};
        for (auto s : id_)
        {
            if (s >= 0x30 && s <= 0x39)
                result += s;
        }
        return result.toInt();
    }

//запись результатов пролива
    void saveResultRefuil(uint16_t n = 0)
    {
        String str = String(getCountReffil()) + "," + String(n) + "," + String(getVfuel() / 10.0, 1);
        v_total_.push_back(str);
        v_ref_.push_back(getVfuel());
        n_ref_.push_back(n);
        Serial.printf("\n%s\n", str);
    }

// выдача результатов пролива i
    String getResultRefill(int i)
    {
        return v_total_.at(i);
    }

    // выдача режима тарировки
    const tarring::mode getType()
    {
        return mode_;
    }

    // установка режима тарировки
    void setType(tarring::mode m)
    {
        mode_ = m;
    }

    //  выдача объема топлива в баке
    const uint32_t getVfuel()
    {
        return countV_->getVFuel();
    }

    // выдача объема бака
    const uint32_t getVTank()
    {
        return tank_->getVTank();
    }

    // установка объема бака и вычисление объема проливов
    void setVTank(uint32_t v)
    {
        tank_->setVTank(v);
        if (num_reffil_)
            vtank_refill_ = v / num_reffil_;
        else
            vtank_refill_ = v;
    }

    // выдача объема одного пролива
    const uint32_t getVTankRefill()
    {
        return vtank_refill_;
    }

    // выдача времени паузы между проливами
    const uint getTimePause()
    {
        return time_pause_;
    }

    // установка вемени паузы между проливами
    void setTimePause(uint t)
    {
        time_pause_ = t;
    }

    // получить общее время тарировки
    const uint32_t getTimeTarring()
    {
        if (getVTank() - getVfuel() > 0)
            if (mode_ == tarring::MANUAL)
                return time_pause_ * (num_reffil_ - getCountReffil()) + (getVTank() - getVfuel()) / PUMPSPEED;
            else
                return num_reffil_ - getCountReffil() + (getVTank() - getVfuel()) / PUMPSPEED;
        else
            return 0;
    }

    // сброс настроек тарировки
    void reset()
    {
        mode_ = tarring::MANUAL;
        refill_ = 0;
        vtank_refill_ = 0;
        time_pause_ = 3;
        num_reffil_ = 15;
        tank_->reset();
        n_ref_.clear();
        v_ref_.clear();
        v_total_.clear();
        countV_->reset();
        id_ = "";
    }

    // установка кол-ва проливов
    void setNumRefill(uint count)
    {
        num_reffil_ = count;
        if (count)
            vtank_refill_ = getVTank() / count;
        else
            vtank_refill_ = getVTank();
    }

    // выдача кол-ва проливов
    const uint getNumRefill()
    {
        return num_reffil_;
    }

    // выдача номера пролива
    uint getCountReffil()
    {
        return v_ref_.size();
    }

    // сохранения времени начала тарировки
    void setTStart(RtcDateTime t)
    {
        t_start_ = t;
    }

    const RtcDateTime getTStart()
    {
        return t_start_;
    }
};