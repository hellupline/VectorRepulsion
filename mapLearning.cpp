#include <vector>
#include "math.h"

#include "quadTree.h"
#include "dirVector.h"
#include "mapLearning.h"

mapLearning::mapLearning(int _MAP_SIZE, int _HALF_SONAR, int _SONAR_NOISE_THRHOLD, int _EFFECT_RADIUS, int _EFFECT_NOISE_THRHOLD, int _K1, int _K2) {
    MAP_SIZE = _MAP_SIZE;
    HALF_SONAR = _HALF_SONAR;
    SONAR_NOISE_THRHOLD = _SONAR_NOISE_THRHOLD;
    EFFECT_RADIUS = _EFFECT_RADIUS;
    EFFECT_NOISE_THRHOLD = _EFFECT_NOISE_THRHOLD;
    K1 = _K1;
    K2 = _K2;

    histogramMap = new quadTree(rect(0, 0, MAP_SIZE, MAP_SIZE));
}

void mapLearning::render(dirVector pose, std::vector<int> sonar) {
    int angle[] = {-90, -50, -30, -10, 10, 30, 50, 90};
    rect search(0, 0, 1, 1);
    std::vector<xy*>* points;
    for(int i=0; i < 8; i++) {
        if (sonar[i] > SONAR_NOISE_THRHOLD) {
            for(int j=pose.z+angle[i]-HALF_SONAR; j < pose.z+angle[i]+HALF_SONAR; j++) {
                search.x = pose.x + sonar[i]*cos(radians(j));
                search.y = pose.y + sonar[i]*sin(radians(j));
                points = histogramMap->queryRange(search);
                if (points->size() > 0) {
                    if (points[0][0]->value < 126)
                        points[0][0]->value++;
                } else {
                    histogramMap->insert(xy(search.x, search.y, 1));
                }
            }
        }
    }
}

dirVector mapLearning::vector(dirVector origin, dirVector dest) {
    rect search(origin.x-(EFFECT_RADIUS>>1), origin.y-(EFFECT_RADIUS>>1), EFFECT_RADIUS, EFFECT_RADIUS);
    std::vector<xy*>* r_points= histogramMap->queryRange(search);
    std::vector<dirVector> r_vectors;

    dirVector v, w, d;
    int m;
    for(unsigned int i=0; i < r_points->size(); i++) {
        dirVector(r_points[0][i]->x, r_points[0][i]->y, 0);

        v = vectorSub(origin, w);
        m = vectorMod(v);

        if (m > 0)
            r_vectors.push_back(vectorMulS(v, K1*exp(-m*K2)/m));
    }

    d = vectorSub(dest, origin);
    d = vectorMulS(d, 20/vectorMod(d));
    for(unsigned int i=0; i < r_vectors.size(); i++)
        d = vectorSum(d, r_vectors[i]);

    return d;
}
