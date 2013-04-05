#include <iostream>
#include <vector>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "quadTree.h"
#include "dirVector.h"
#include "mapRender.h"

using namespace cv;

void hsv2rgb(int h, int *r, int *g, int *b) {
    float H, S, V, F, M, N, K;
    int   I;

    H = h/360.0;    /* Hue */
    S = 1;          /* Saturation */
    V = 1;          /* Value */

    H = (H >= 1.0) ? 1.0 : H*6;

    I = (int) H;    /* should be in the range 0..5 */
    F = H - I;      /* fractional part */

    M = V * (1 - S);
    N = V * (1 - S * F);
    K = V * (1 - S * (1 - F));

    if (I == 0) { *r = round(V*255.0); *g = round(K*255.0); *b = round(M*255.0);}
    if (I == 1) { *r = round(N*255.0); *g = round(V*255.0); *b = round(M*255.0);}
    if (I == 2) { *r = round(M*255.0); *g = round(V*255.0); *b = round(K*255.0);}
    if (I == 3) { *r = round(M*255.0); *g = round(N*255.0); *b = round(V*255.0);}
    if (I == 4) { *r = round(K*255.0); *g = round(M*255.0); *b = round(V*255.0);}
    if (I == 5) { *r = round(V*255.0); *g = round(M*255.0); *b = round(N*255.0);}
}
void render(quadTree& map, rect area, dirVector pose, dirVector vector, int radius) {
    Mat_<Vec3b> img(area.h, area.w, Vec3b(0,0,0));
    int r=0, g=0, b=0;

    namedWindow("mapRender", CV_WINDOW_KEEPRATIO);
    resizeWindow("mapRender", 500, 500);

    std::vector<xy*> particles = map.queryRange(area);
    for (unsigned int i=0; i < particles.size(); i++) {
        if (particles[i]->value == 0) {
            img(particles[i]->y-area.y, particles[i]->x-area.x) = Vec3b(0,0,0);
        } else {
            hsv2rgb(particles[i]->value, &r, &g, &b);
            img(particles[i]->y-area.y, particles[i]->x-area.x) = Vec3b(r,g,b);
        }
    }
    ellipse(img, Point(area.h/2, area.w/2), Size(radius*2, radius*2), 0, 0, 360, Scalar(0, 0, 255), 1, 8);
    line(img, Point(area.h/2, area.w/2), Point(vector.x+(area.w/2), vector.y+(area.h/2)), Scalar(0, 255, 0), 1, 8);

    flip(img, img, 0); transpose(img, img); flip(img, img, 0); // 1: fix orentation, 2-3, rotate 90 degress

    imshow("mapRender", img);
    waitKey(1);
    img.release();
}
