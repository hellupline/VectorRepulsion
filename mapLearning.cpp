#include <iostream>
#include <vector>
#include <math.h>

#include "quadTree.h"
#include "dirVector.h"
#include "mapLearning.h"

#define D_MEASURE_REDUCTION 50.0
#define D_MAP_SIZE 1000

#define D_EFFECT_THRHOLD 2
#define D_SONAR_NOISE_THRHOLD 3000
#define D_EFFECT_RADIUS 1500/D_MEASURE_REDUCTION

#define D_K1 25.0
#define D_K2 1.0

#define D_SONAR_HALF_POLAR_DISTANCE 10

mapLearning::mapLearning(float _MEASURE_REDUCTION, float _MAP_SIZE, float _EFFECT_THRHOLD, float _SONAR_NOISE_THRHOLD, float _EFFECT_RADIUS, float _K1, float _K2, float _SONAR_HALF_POLAR_DISTANCE) {
    MEASURE_REDUCTION = _MEASURE_REDUCTION;
    MAP_SIZE = _MAP_SIZE;
    EFFECT_THRHOLD = _EFFECT_THRHOLD;
    SONAR_NOISE_THRHOLD = _SONAR_NOISE_THRHOLD;
    EFFECT_RADIUS = _EFFECT_RADIUS;
    K1 = _K1;
    K2 = _K2;
    SONAR_HALF_POLAR_DISTANCE = _SONAR_HALF_POLAR_DISTANCE;

    histogramMap = new quadTree(rect(0, 0, MAP_SIZE, MAP_SIZE));
}

void mapLearning::renderPrecise(std::vector<xy> sonar) {
    for (unsigned int i=0; i < sonar.size(); i++)
        histogramMap->insert(sonar[i]);
    //CHEATER MODE :)
}

void mapLearning::render(dirVector pose, std::vector<int> sonar) {
    int angle[] = {90, 50, 30, 10, -10, -30, -50, -90};
    //int angle[] = {-90, -50, -30, -10, 10, 30, 50, 90};
    rect search(0, 0, 1, 1);
    std::vector<xy*> points;
    for(int i=0; i < 8; i++) {
        if (sonar[i] < SONAR_NOISE_THRHOLD/MEASURE_REDUCTION) {
            for(int j=pose.z+angle[i]-SONAR_HALF_POLAR_DISTANCE; j < pose.z+angle[i]+SONAR_HALF_POLAR_DISTANCE; j++) {
                search.x = pose.x + sonar[i]*cos(radians(j));
                search.y = pose.y + sonar[i]*sin(radians(j));
                points = histogramMap->queryRange(search);
                if (points.size() > 0) {
                    if (points[0]->value < 126)
                        points[0]->value++;
                } else {
                    histogramMap->insert(xy(search.x, search.y, 1));
                }
            }
        }
    }
}

dirVector mapLearning::vector(dirVector origin, dirVector dest) {
    rect search(origin.x-(EFFECT_RADIUS), origin.y-(EFFECT_RADIUS), EFFECT_RADIUS*2, EFFECT_RADIUS*2);
    std::vector<xy*> r_points= histogramMap->queryRange(search);
    std::vector<dirVector> r_vectors;

    dirVector v;
    int m;
    for(unsigned int i=0; i < r_points.size(); i++) {
        if (r_points[i]->value > EFFECT_THRHOLD) {
            v = dirVector(r_points[i]->x, r_points[i]->y, 0);
            v = vectorSub(origin, v);
            m = vectorMod(v);
            if (m > 0)
                r_vectors.push_back(vectorMulS(v, K1*exp(-m*K2)/m));
        }
    }

    v = vectorSub(dest, origin);
    v = vectorMulS(v, K1/vectorMod(v));
    for(unsigned int i=0; i < r_vectors.size(); i++)
        v = vectorSum(v, r_vectors[i]);
    return v;
}
