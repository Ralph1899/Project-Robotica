#ifndef CLOCK_H
#define CLOCK_H

#include <chrono>
#include <thread>

class clock
{
public:
    static void sleep_milliseconds(int milliseconds);
};

#endif // !CLOCK_H