struct dirVector {
    int x, y, z;
    dirVector(int _x, int _y, int _z): x(_x), y(_y), z(_z) { }
    dirVector(): x(0), y(0), z(0) { }
};
int round(int n, int c=1);
int degrees(int a);
int radians(int a);
dirVector vectorSum(dirVector& a, dirVector& b);
dirVector vectorSub(dirVector& a, dirVector& b);
dirVector vectorMulV(dirVector& a, dirVector& b);
dirVector vectorMulS(dirVector& a, int b);
int vectorMod(dirVector& a);
int vectorMinAngle(dirVector& a, dirVector& b);
int vectorAngle(dirVector& a, int b);
