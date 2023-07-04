#pragma once

#include <Arduino.h>
#include <chrono>
#include <iostream>
class TimingUtil
{
public:
    TimingUtil(std::string const &message) : start_(std::chrono::steady_clock::now()), message_(message)
    {
    }
    ~TimingUtil()
    {
        std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
        std::cout << "\n" << message_ <<  " took " << std::chrono::duration_cast<std::chrono::milliseconds>(finish - start_).count() << " mls." << std::endl;
    }

private:
    std::chrono::steady_clock::time_point start_;
    std::string message_;
};