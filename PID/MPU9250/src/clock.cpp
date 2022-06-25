#include "../inc/clock.h"

void clock::sleep_milliseconds(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

double clock::micros()
{
   struct timeval tv;
   double t;

   gettimeofday(&tv, 0);

   t = (double)tv.tv_sec + (double)tv.tv_usec;

   return t;
}