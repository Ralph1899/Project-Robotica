#ifndef CLOCK_H
#define CLOCK_H

#include <chrono>
#include <thread>
#include <sys/time.h>

class clock
{
public:
    static void sleep_milliseconds(int milliseconds);
    static std::chrono::steady_clock::time_point current_time_ms();
    static std::chrono::milliseconds time_difference_ms(std::chrono::steady_clock::time_point old_time);
};

#endif // !CLOCK_H