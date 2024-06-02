#pragma once

#include <Arduino.h>
#include "LEVEL_SENSOR.h"
#include <map>

class LS_RS485 : public ILEVEL_SENSOR
{
private:
    uint8_t netadress_;
    String ttydata_;
    HardwareSerial *port_ = nullptr;
    boolean doConnect_ = false;

    static const uint8_t PROGMEM DSCRC_TABLE[];

    bool update_()
    {
        if (netadress_ == 0xFF)
            return false;
        // Serial.printf("\nUpdate RS485 adr: %d\n", netadress_);
        doConnect_ = false;
        std::vector<uint8_t> bufferRead485{};
        uint8_t rs485TransmitArray[] = {0x31, 0x00, 0x06, 0x00};
        rs485TransmitArray[1] = netadress_;
        rs485TransmitArray[3] = crc8(rs485TransmitArray, 3);
        for (int i = 0; i < 3; i++)
        {
            port_->write(rs485TransmitArray, 4);
            delay(100);
            while (port_->available())
            {
                bufferRead485.push_back(port_->read());
                delay(1);
            }
            if (bufferRead485.size() >= 9)
            {
                if (bufferRead485[0] == 0x3E && bufferRead485[1] == netadress_ && !controlCrc8(bufferRead485))
                {
                    level_ = bufferRead485[5] << 8 | bufferRead485[4];
                    setVLevel();
                    doConnect_ = true;
                    set_error_();
                    return true;
                }
            }
            bufferRead485.clear();
            delay(100);
        }
        level_ = 5000;
        set_error_();
        return false;
    }

    // ошибки
    void set_error_()
    {
        if (netadress_ == 0xFF) // ДУТ не найден
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_ERROR)
                error_ = error::NOT_FOUND;
        }
        else if (!doConnect_) // ДУТ потерян
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_ERROR)
            {
                error_ = error::LOST;
                return;
            }
        }

        else if (level_ < MIN_DIGITAL_N)
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_ERROR)
                error_ = error::CLIFF;
        }
        else if (level_ >= MAX_DIGITAL_N)
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_ERROR)
                error_ = error::CLOSURE; // показания датчика выше нормы (неисправность датчика)
        }
        else
        {
            clearError();
        }
    }

public:
    LS_RS485() {}
    LS_RS485(HardwareSerial *port, uint16_t netadress = 0x01) : port_(port), netadress_(netadress)
    {
        // Serial.print("\n  - Create rs485 \n");

        type_ = ILEVEL_SENSOR::RS485;
        level_start_ = MAX_ANALOGE_RS485_START;
    }

    void setNetadress(int netadress) override
    {
        netadress_ = netadress;
    }
    const int getNetadres() const override
    {
        return netadress_;
    }

    bool update() override
    {
        bool res = update_();
        // set_error_();
        return res;
    }

    const bool search() override
    {
        // Serial.print("\nSearch RS485\n");
        // flag_upgate_ = true;
        for (uint j = 0; j < 10; j++)
        {
            clearError();
            netadress_ = j;
            update_();
            if (doConnect_)
            {
                return true;
            }
        }
        netadress_ = 0xFF; // ошибка, ДУТ не найден
        error_ = error::NOT_FOUND;
        return false;
    }

    const bool searchLost() override
    {
        // Serial.print("\nSearch RS485\n");
        // flag_upgate_ = true;
        counter_errror_ = 0;
        for (uint j = 0; j < 10; j++)
        {
            update_();
            if (doConnect_)
            {
                clearError();
                return true;
            }
        }
        error_ = error::LOST;
        return false;
    }

    float getTarLevel()
    {
        std::map<uint16_t, float> tabl{{0, 0.0}, {8, 4.0}, {124, 40.0}, {298, 80.0}, {457, 129.0}, {634, 160.0}, {808, 200.0}, {979, 240.0}, {1142, 280.0}, {1230, 320.0}, {4000, 1000.0}};

        auto it_end = tabl.upper_bound(level_);
        auto it_begin = it_end;
        it_begin--;
        if (it_end == tabl.end())
            return -1.0;
        return map(it_begin->first, it_end->first, level_, it_begin->second, it_end->second);
    }

private:
    // функция вычисления контрольной суммы
    uint8_t crc8(uint8_t *addr, uint8_t len)
    {
        uint8_t crc = 0;
        while (len--)
            crc = pgm_read_byte(DSCRC_TABLE + (crc ^ *addr++));
        return crc;
    }
    // функция проверки целостности сообщения по контрольной сумме
    bool controlCrc8(std::vector<uint8_t> &buf)
    {
        uint8_t crc = 0;
        uint8_t len = buf.size();
        auto addr = buf.begin();
        while (len--)
            crc = pgm_read_byte(DSCRC_TABLE + (crc ^ *addr++));
        return crc;
    }

public:
    ~LS_RS485(){
        // Serial.print("\n  - Kill rs485\n");
    };
};
const uint8_t LS_RS485::DSCRC_TABLE[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65, 157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36, 248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205, 17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168, 116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};
