class mapLearning {
    private:
        float MAP_SIZE, MEASURE_REDUCTION, HALF_SONAR, SONAR_NOISE_THRHOLD, EFFECT_RADIUS, EFFECT_NOISE_THRHOLD, K1, K2;
        quadTree *histogramMap;
    public:
        mapLearning(float _MAP_SIZE, float _MEASURE_REDUCTION, float _HALF_SONAR, float _SONAR_NOISE_THRHOLD, float _EFFECT_RADIUS, float _EFFECT_NOISE_THRHOLD, float _K1, float _K2);
        void render(dirVector pose, std::vector<int> sonar);
        dirVector vector(dirVector origin, dirVector dest);
};
