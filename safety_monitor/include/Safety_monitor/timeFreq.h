#ifndef TIMEFREQ_H_
#define TIMEFREQ_H_

#include <chrono>
#include <string>

class TimeFreq{
    public:
        TimeFreq(float frequency, std::chrono::time_point<std::chrono::system_clock> time);

        void setTime(std::chrono::time_point<std::chrono::system_clock> time);

        float getFrequency();
        std::chrono::time_point<std::chrono::system_clock> getTime();

    protected:
        float frequency;
        std::chrono::time_point<std::chrono::system_clock> time;

};


#endif
