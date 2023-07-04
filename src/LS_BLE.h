#pragma once

#include "NimBLEDevice.h"
#include "LEVEL_SENSOR.h"

static String nameBLE_ls = {};
static bool doConnect_ = false;
static NimBLEAdvertisedDevice *llsDevice_;
const uint16_t scanTime_ = 5; // In seconds

class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
    void onResult(NimBLEAdvertisedDevice *advertisedDevice)
    {
        // Serial.printf("\nAdvertised Device found: %s", advertisedDevice->toString().c_str());
        if (advertisedDevice->getName() == nameBLE_ls.c_str())
        {
            doConnect_ = true;
            llsDevice_ = advertisedDevice;
            // Serial.printf("\n - Found our LLS: %s", llsDevice_->toString().c_str());
            NimBLEDevice::getScan()->stop();
        }
    };
};

class LS_BLE : public ILEVEL_SENSOR
{
public:
private:
    uint16_t dataBLE_[4] = {};
    uint16_t RSSI_ = {};
    BLEScan *BLEScan_;
    boolean _doConnect = false;
    String ManufacturerData = {};

    void buildData(uint8_t *source, uint8_t length)
    {
        if (length > 100)
            length = 100;
        if (length == 0)
        {
            return;
        }
        dataBLE_[0] = source[3];
        dataBLE_[0] |= source[4] << 8;
        dataBLE_[1] = source[5];
        dataBLE_[2] = source[6];
        dataBLE_[3] = source[13] ? 4095 : 1024;
    }

    // ошибки
    void set_error_()
    {
        if (!doConnect_)
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_SEARCH_ERROR) // ДУТ не найден
                error_ = error::LOST;
        }

        // if (level_ == 65535) // ДУТ потерян
        // {
        //     counter_errror_++;
        //     if (counter_errror_ > COUNT_SEARCH_ERROR)
        //         error_ = error::LOST;
        // }
        // else
        if (level_ < MIN_DIGITAL_B)
        {
            counter_errror_++;
            if (counter_errror_ > COUNT_SEARCH_ERROR)
                error_ = error::CLIFF;
        }
        else if (level_ > MAX_DIGITAL_B)
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
    LS_BLE()
    {
        // Serial.print("\n  - Create BLE");
        type_ = ILEVEL_SENSOR::BLE_ESKORT;
        level_start_ = MIN_ANALOGE_BLE_START;
        error_ = error::NO_ERROR;
        nameBLE_ls = "";

        //  --------------------BLE-------------------
        BLEDevice::init("");             // Инициализируем BLE-устройство:
        BLEScan_ = BLEDevice::getScan(); // create new scan
        BLEScan_->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
        BLEScan_->setActiveScan(true); // active scan uses more power, but get results faster
        BLEScan_->setInterval(100);
        BLEScan_->setWindow(99); // less or equal setInterval value
    }

    void setNameBLE(const String &name) override
    {
        nameBLE_ls = name;
    }

    const String getNameBLE() const override
    {
        return nameBLE_ls;
    }

    const int getNameBLE_int() const override
    {
        String result{};
        for (auto s : nameBLE_ls)
        {
            if (s >= 0x30 && s <= 0x39)
                result += s;
        }
        return result.toInt();
    }

    const uint16_t getRSSI() const override
    {
        return RSSI_;
    }

    const uint16_t getDataBLE(uint i) const override
    {
        return dataBLE_[i];
    }

    const bool search() override
    {
        error_ = error::NO_ERROR;
        update();
        if (!doConnect_)
            error_ = error::NOT_FOUND;
        return true;
    }

    void update() override
    {
        if (nameBLE_ls == "" || error_ == error::NOT_FOUND)
            return;

        if (flag_upgate_)
            return;

        flag_upgate_ = true;
        doConnect_ = false;
        BLEScanResults foundDevices = BLEScan_->start(scanTime_, false);

        if (doConnect_)
        {
            error_ = error::NO_ERROR;
            RSSI_ = llsDevice_->getRSSI();
            std::string DataBLE = llsDevice_->getManufacturerData();
            buildData((uint8_t *)DataBLE.data(), DataBLE.length());
            level_ = getDataBLE(0);
            setVLevel();
        }
        set_error_();
        BLEScan_->clearResults(); // delete results fromBLEScan buffer to release memory
        flag_upgate_ = false;
    }

    ~LS_BLE()
    {
        // Serial.print("\n  - Kill ble");
        BLEScan_->stop();
        BLEScan_->clearResults(); // delete results fromBLEScan buffer to release memory
    };
};
