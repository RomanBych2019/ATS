#pragma once

#include <Arduino.h>
#include "LEVEL_SENSOR.h"

class LS_RS485 : public ILEVEL_SENSOR
{
private:
    int netadress_;
    String ttydata_;
    SoftwareSerial *port_;
    boolean doConnect_ = false;

    static const uint8_t PROGMEM DSCRC_TABLE[];

    // ошибки
    void set_error_()
    {
        if (netadress_ == 0xFF) // ДУТ не найден
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_SEARCH_ERROR)
                error_ = error::NOT_FOUND;
        }
        else if (!doConnect_) // ДУТ потерян
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_SEARCH_ERROR)
                error_ = error::LOST;
        }

        else if (level_ < MIN_DIGITAL_N)
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_SEARCH_ERROR)
                error_ = error::CLIFF;
        }
        else if (level_ > MAX_DIGITAL_N)
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_SEARCH_ERROR)
                error_ = error::CLOSURE; // показания датчика выше нормы (неисправность датчика)
        }
        else
        {
            counter_errror_ = 0;
            error_ = error::NO_ERROR;
        }
    }

public:
    LS_RS485() {}
    LS_RS485(SoftwareSerial *port, uint16_t netadress = 0x01) : netadress_(netadress)
    {
        // Serial.print("\n  - Create rs485");
        port_ = port;
        type_ = ILEVEL_SENSOR::RS485;
        level_start_ = MIN_ANALOGE_RS485_START;
    }

    void setNetadress(int netadress) override
    {
        netadress_ = netadress;
    }
    const int getNetadres() const override
    {
        return netadress_;
    }

    void update() override
    {
        if (netadress_ == 0xFF)
            return;
        flag_upgate_ = true;
        // Serial.printf("\nUpdate RS485 adr: %d", netadress_);
        byte bufferRead485[9] = {0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00};
        byte rs485TransmitArray[] = {0x31, 0x00, 0x06, 0x00};
        rs485TransmitArray[1] = netadress_;
        rs485TransmitArray[3] = crc8(rs485TransmitArray, 3);
        for (int i = 0; i < 2; i++)
        {
            port_->write(rs485TransmitArray, 4);
            delay(50);
            if (port_->available())
            {
                ttydata_ = port_->readString();
                for (int i = 0; i < 9; i++)
                {
                    bufferRead485[i] = ttydata_.charAt(i);
                }
                if (ttydata_.charAt(0) == 0x3E && ttydata_.charAt(1) == netadress_ && ttydata_.charAt(8) == crc8(bufferRead485, 8))
                {
                    level_ = ttydata_[5] << 8 | ttydata_[4];
                    setVLevel();
                    set_error_();
                    flag_upgate_ = false;
                    doConnect_ = true;
                    return;
                }
            }
            delay(100);
        }
        // level_ = 65535;
        doConnect_ = false;
        set_error_();
        flag_upgate_ = false;
    }

    const bool search() override
    {
        Serial.print("\nSearch RS485\n");

        for (uint j = 0; j < 10; j++)
        {
            netadress_ = j;
            update();
            if (doConnect_)
            {
                return true;
            }
        }
        netadress_ = 0xFF; // ошибка, ДУТ не найден
        error_ = error::NOT_FOUND;
        return {};
    }
    float getTarLevel()
    {
        if (level_ == 0)
            return 0;
        if (level_ < 8)
            return level_ * 0.31 + 1.51;
        else if (level_ < 124)
            return level_ * 0.23 + 11.49;
        else if (level_ < 298)
            return level_ * 0.25 + 5.03;
        else if (level_ < 457)
            return level_ * 0.23 + 16.72;
        else if (level_ < 634)
            return level_ * 0.23 + 14.25;
        else if (level_ < 808)
            return level_ * 0.23 + 10.99;
        else if (level_ < 979)
            return level_ * 0.25 - 0.24;
        else if (level_ < 1142)
            return level_ * 0.25 - 9.11;
        else if (level_ < 1300)
            return level_ * 0.25 - 7.40;
        return map(level_, 0, 4000, 0, 1000);
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

public:
    ~LS_RS485(){
        // Serial.print("\n  - Kill rs485");
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
