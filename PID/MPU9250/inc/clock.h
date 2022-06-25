#ifndef CLOCK_H
#define CLOCK_H

#include <chrono>
#include <thread>
#include <sys/time.h>

class clock
{
public:
    static void sleep_milliseconds(int milliseconds);
    static double micros();
};

#endif // !CLOCK_H