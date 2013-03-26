#include <math.h>

#include "dirVector.h"

int round(int n) { return n; }
int degrees(int a) { return a*180.0/3.1415926; }
int radians(int a) { return a*3.1415926/180.0; }
dirVector vectorSum(dirVector& a, dirVector& b) { return dirVector(a.x+b.x, a.y+b.y, a.z+b.z); }
dirVector vectorSub(dirVector& a, dirVector& b) { return dirVector(a.x-b.x, a.y-b.y, a.z-b.z); }
dirVector vectorMulV(dirVector& a, dirVector& b) { return dirVector(a.x*b.x, a.y*b.y, a.z*b.z); }
dirVector vectorMulS(dirVector& a, int b) { return dirVector(a.x*b, a.y*b, a.z*b); }
int vectorMod(dirVector& a) { return round(sqrt(a.x*a.x+a.y*a.y)); }
int vectorMinAngle(dirVector& a, dirVector& b) {
    int scalar_product = a.x*b.x+a.y*b.y;
    int modulus_product = vectorMod(a)*vectorMod(b);
    if (modulus_product != 0)
        return degrees(acos(scalar_product/modulus_product));
    else
        return 0;
}
int vectorAngle(dirVector& a, int b) {
    int sin_angle = sin(radians(b));
    int cos_angle = cos(radians(b));
    dirVector d(1, 0, 0), v(a.x*cos_angle-a.y*sin_angle, a.x*sin_angle+a.y*cos_angle, 0);
    if (v.y > 0)
        return vectorMinAngle(d, v);
    else
        return -vectorMinAngle(d, v);
}

