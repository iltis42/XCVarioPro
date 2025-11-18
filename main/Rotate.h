
#pragma once

#include <cstdint>

class Point {
public:
    int16_t x;
    int16_t y;
    constexpr Point( int16_t ax, int16_t ay ){
        x = ax;
        y = ay;
    };
    constexpr Point(){
        x = 0;
        y = 0;
    };
    Point rotate(const Point& center, float angle);
    void moveVertical(int16_t pixel);
};



