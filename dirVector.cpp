#include <iostream>
#include <vector>
#include <math.h>

#include "dirVector.h"

float degrees(float a) { return a*180.0/3.1415926; }
float radians(float a) { return a*3.1415926/180.0; }
dirVector vectorSum(dirVector& a, dirVector& b) { return dirVector(a.x+b.x, a.y+b.y, a.z+b.z); }
dirVector vectorSub(dirVector& a, dirVector& b) { return dirVector(a.x-b.x, a.y-b.y, a.z-b.z); }
dirVector vectorMulV(dirVector& a, dirVector& b) { return dirVector(a.x*b.x, a.y*b.y, a.z*b.z); }
dirVector vectorMulS(dirVector& a, float b) { return dirVector(a.x*b, a.y*b, a.z*b); }
float vectorMod(dirVector& a) { return round(sqrt(a.x*a.x+a.y*a.y)); }
inline float SafeAcos (float x) {
    if (x < -1.0)
        x = -1.0;
    else if (x > 1.0)
        x = 1.0;
    return acos (x);
}
float vectorMinAngle(dirVector& a, dirVector& b) {
    float scalar_product = a.x*b.x+a.y*b.y;
    float modulus_product = vectorMod(a)*vectorMod(b);
    if (modulus_product != 0)
        //return degrees(acos(scalar_product/modulus_product));
        return degrees(SafeAcos(scalar_product/modulus_product));
    else
        return 0;
}
float vectorAngle(dirVector& a, dirVector& b) {
    float sin_angle = sin(radians(-b.z));
    float cos_angle = cos(radians(-b.z));
    dirVector d(1, 0, 0), v(a.x*cos_angle-a.y*sin_angle, a.x*sin_angle+a.y*cos_angle, 0);
    if (v.y > 0)
        return vectorMinAngle(d, v);
    else
        return -vectorMinAngle(d, v);
}

