#pragma once

#include <Arduino.h>

class ILEVEL_SENSOR
{

protected:
    uint16_t level_ = 0, counter_errror_ = 0;
    static const int MAX_SIZE = 20;
    std::vector<uint16_t> v_level_; // вектор последних значений уровня
    uint8_t v_count_ = 0;
    static const uint COUNT_SEARCH_ERROR = 10; // максимальное количество ошибок
    bool flag_upgate_ = false;                 // флаг нахождения в функции update
    uint16_t level_start_;

    ILEVEL_SENSOR()
    {
        // Serial.print("\n  - Create lls");
        v_level_.reserve(MAX_SIZE);
    }

    int median_of_3(int a, int b, int c) // медианное усреднение
    {
        int the_max_ = max(max(a, b), c);
        int the_min_ = min(min(a, b), c);
        return the_max_ ^ the_min_ ^ a ^ b ^ c;
    }

    virtual void set_error_() = 0;

    static const uint16_t MIN_ANALOGE_U = 8;    // минимальное напряжение на рабочем ДУТ (аналог, напряжение в 0.01 В)
    static const uint16_t MAX_ANALOGE_U = 2000; // максимальное напряжение на рабочем ДУТ (аналог, напряжение в 0.01 В)

    static const uint16_t MIN_ANALOGE_F = 20;  // минимальная частота на рабочем ДУТ (аналог, частота в Гц)
    static const uint16_t MAX_ANALOGE_F = 3000; // максимальная частота на рабочем ДУТ (аналог, частота в Гц)

    static const uint16_t MIN_DIGITAL_N = 1;    // минимальное значение ДУТ RS485
    static const uint16_t MAX_DIGITAL_N = 4095; // максимальнoе значение ДУТ RS485

    static const uint16_t MIN_DIGITAL_B = 1;    // минимальное значение ДУТ BLE
    static const uint16_t MAX_DIGITAL_B = 4095; // максимальнoе значение ДУТ BLE

public:
    static const uint16_t MIN_ANALOGE_U_START = 100;     // минимальное напряжение для начала тарировки (аналог, напряжение в 0.01 В)
    static const uint16_t MIN_ANALOGE_F_START = 2000;    // минимальное напряжение для начала тарировки (аналог, частота в Гц)
    static const uint16_t MIN_ANALOGE_RS485_START = 100; // минимальное напряжение для начала тарировки (RS485)
    static const uint16_t MIN_ANALOGE_BLE_START = 100;     // минимальное напряжение для начала тарировки (BLE)

    enum type
    {
        NO_LLS,    // нет подключенного ДУТа
        ANALOGE_U, // аналоговый, напряжение
        ANALOGE_F, // аналоговый, частота
        RS485,     // цифровой, rs485
        BLE_ESKORT // BLE Эскорт
    };

    enum error
    {
        NO_ERROR,
        CLIFF,         // обрыв
        CLOSURE,       // замыкание
        NOT_FOUND = 4, // ДУТ не найден
        LOST = 8       // ДУТ потерян
    };

    const uint16_t getLevel() const
    {
        return level_;
    }

    const uint16_t getLevelStart() const
    {
        return level_start_;
    }

    const bool getFlagUpgate() const
    {
        return flag_upgate_;
    }

    const type getType() const
    {
        return type_;
    }

    const error getError() const
    {
        return error_;
    }

    virtual void update() = 0;

    virtual void setNetadress(int){};

    virtual const int getNetadres() const
    {
        return {};
    }

    virtual const bool search() = 0;

    virtual void setNameBLE(const String &name)
    {
    }

    virtual const String getNameBLE() const
    {
        return {};
    }

    virtual const int getNameBLE_int() const
    {
        return {};
    }

    virtual std::vector<uint16_t> *getVecLevel()
    {
        return &v_level_;
    }

    void test()
    {
        level_++;
    }

    void setVLevel()
    {
        if (v_level_.size() < MAX_SIZE)
        {
            v_level_.push_back(level_);
        }
        else
        {
            if (v_count_ < MAX_SIZE)
            {
                v_level_.at(v_count_) = level_;
                v_count_++;
            }
            else
            {
                v_level_.at(0) = level_;
                v_count_ = 1;
            }
        }
    }

    virtual const int getRSSI() const
    {
        return {};
    }

    virtual const uint16_t getDataBLE(uint i) const
    {
        return {};
    }

    virtual ~ILEVEL_SENSOR()
    {
        Serial.print("\n  - Kill lls");
    }

protected:
    type type_;
    error error_ = NO_ERROR;
};