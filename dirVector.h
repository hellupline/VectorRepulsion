struct dirVector {
    float x, y, z;
    dirVector(float _x, float _y, float _z): x(_x), y(_y), z(_z) { }
    dirVector(): x(0), y(0), z(0) { }
};
float round(float n, float c=1);
float degrees(float a);
float radians(float a);
dirVector vectorSum(dirVector& a, dirVector& b);
dirVector vectorSub(dirVector& a, dirVector& b);
dirVector vectorMulV(dirVector& a, dirVector& b);
dirVector vectorMulS(dirVector& a, float b);
float vectorMod(dirVector& a);
float vectorMinAngle(dirVector& a, dirVector& b);
float vectorAngle(dirVector& a, dirVector& b);
