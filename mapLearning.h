class mapLearning {
    private:
        int MAP_SIZE, MEASURE_REDUCTION, HALF_SONAR, SONAR_NOISE_THRHOLD, EFFECT_RADIUS, EFFECT_NOISE_THRHOLD, K1, K2;
        quadTree *histogramMap;
    public:
        mapLearning(int _MAP_SIZE, int _MEASURE_REDUCTION, int _HALF_SONAR, int _SONAR_NOISE_THRHOLD, int _EFFECT_RADIUS, int _EFFECT_NOISE_THRHOLD, int _K1, int _K2);
        void render(dirVector pose, std::vector<int> sonar);
        dirVector vector(dirVector origin, dirVector dest);
};
