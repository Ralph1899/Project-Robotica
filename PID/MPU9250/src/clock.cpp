#include "../inc/clock.h"

void clock::sleep_milliseconds(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

std::chrono::steady_clock::time_point clock::current_time_ms()
{
    return std::chrono::steady_clock::time_point time_point = std::chrono::steady_clock::now();
}

std::chrono::milliseconds clock::time_difference_ms(std::chrono::steady_clock::time_point old_time)
{
    std::chrono::steady_clock::time_point current_time = current_time_ms();
    std::chrono::duration_cast<std::chrono::milliseconds>(current_time - old_time);
}