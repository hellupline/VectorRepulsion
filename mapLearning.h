class mapLearning {
    private:
    public:
        float MEASURE_REDUCTION, MAP_SIZE, EFFECT_THRHOLD, SONAR_NOISE_THRHOLD, EFFECT_RADIUS, K1, K2, SONAR_HALF_POLAR_DISTANCE;
        quadTree *histogramMap;
        mapLearning(float, float, float, float, float, float, float, float);
        void renderPrecise(std::vector<xy>);
        void render(dirVector, std::vector<int>);
        dirVector vector(dirVector, dirVector);
};
