#include <iostream>
#include <vector>
#include <math.h>

#include "quadTree.h"
#include "dirVector.h"
#include "mapLearning.h"

mapLearning::mapLearning(float _MAP_SIZE, float _MEASURE_REDUCTION, float _HALF_SONAR, float _SONAR_NOISE_THRHOLD, float _EFFECT_RADIUS, float _EFFECT_NOISE_THRHOLD, float _K1, float _K2) {
    MAP_SIZE = _MAP_SIZE;
    MEASURE_REDUCTION = _MEASURE_REDUCTION;
    HALF_SONAR = _HALF_SONAR;
    SONAR_NOISE_THRHOLD = _SONAR_NOISE_THRHOLD;
    EFFECT_RADIUS = _EFFECT_RADIUS;
    EFFECT_NOISE_THRHOLD = _EFFECT_NOISE_THRHOLD;
    K1 = _K1;
    K2 = _K2;

    histogramMap = new quadTree(rect(0, 0, MAP_SIZE, MAP_SIZE));
}

void mapLearning::render(dirVector pose, std::vector<int> sonar) {
    int angle[] = {90, 50, 30, 10, -10, -30, -50, -90};
    //int angle[] = {-90, -50, -30, -10, 10, 30, 50, 90};
    rect search(0, 0, 1, 1);
    std::vector<xy*> points;
    for(int i=0; i < 8; i++) {
        if (sonar[i] < SONAR_NOISE_THRHOLD/MEASURE_REDUCTION) {
            for(int j=pose.z+angle[i]-HALF_SONAR; j < pose.z+angle[i]+HALF_SONAR; j++) {
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
    rect search(origin.x-(EFFECT_RADIUS/2), origin.y-(EFFECT_RADIUS/2), EFFECT_RADIUS, EFFECT_RADIUS);
    //rect search(0, 0, MAP_SIZE, MAP_SIZE);
    std::vector<xy*> r_points= histogramMap->queryRange(search);
    std::vector<dirVector> r_vectors;

    dirVector v;
    int m;
    for(unsigned int i=0; i < r_points.size(); i++) {
        if (r_points[i]->value > EFFECT_NOISE_THRHOLD) {
            v = dirVector(r_points[i]->x, r_points[i]->y, 0);
            v = vectorSub(origin, v);
            m = vectorMod(v);
            if (m > 0)
                r_vectors.push_back(vectorMulS(v, K1*exp(-m*K2)/m));
        }
    }

    v = vectorSub(dest, origin);
    v = vectorMulS(v, 20/vectorMod(v));
    for(unsigned int i=0; i < r_vectors.size(); i++)
        v = vectorSum(v, r_vectors[i]);
    return v;
}
