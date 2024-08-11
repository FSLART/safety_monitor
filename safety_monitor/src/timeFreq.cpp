#include "Safety_monitor/timeFreq.h"

TimeFreq::TimeFreq(float frequency, std::chrono::time_point<std::chrono::system_clock> time){
    this->frequency = frequency;
    this->time = time;
}

void TimeFreq::setTime(std::chrono::time_point<std::chrono::system_clock> time){
    this->time = time;
}

float TimeFreq::getFrequency()
{
    return frequency;
}

std::chrono::time_point<std::chrono::system_clock> TimeFreq::getTime()
{
    return time;
}


