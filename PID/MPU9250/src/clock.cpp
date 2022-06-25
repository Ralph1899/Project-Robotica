#include "../inc/clock.h"

void clock::sleep_milliseconds(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

double clock::micros()
{
    // Function implements arduino style of micros() function
    // Source -> https://forums.raspberrypi.com/viewtopic.php?t=221829
   struct timeval tv;
   double t;

   gettimeofday(&tv, 0);

   t = (double)tv.tv_sec + (double)tv.tv_usec;

   return t;
}