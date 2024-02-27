#ifndef RANDOM_H_INCLUDED
#define RANDOM_H_INCLUDED

#include <cstdlib>
#include <time.h>

#include "vector.hpp"

class Random{
public:
    inline static void init() {srand(time(NULL));}
    inline static int randRange(int min, int max) {return rand() % (max-min+1) + min; }
    inline static bool randBool() {return rand()&0x1;}
    inline static double randfactor() {return Random::randRange(0, 100000) * 0.00001;}

    inline static Coords randUnitVector() {
        return normalize(Coords(randfactor() * 2 - 1, randfactor() * 2 - 1));
    }
};

#endif // RANDOM_H_INCLUDED
