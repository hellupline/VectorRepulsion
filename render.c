#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>

#include <stdio.h>
#include <stdlib.h>

int round_n(float x) {
	return (int) x;
}

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

    if (I == 0) { *r = round_n(V*255.0); *g = round_n(K*255.0); *b = round_n(M*255.0);}
    if (I == 1) { *r = round_n(N*255.0); *g = round_n(V*255.0); *b = round_n(M*255.0);}
    if (I == 2) { *r = round_n(M*255.0); *g = round_n(V*255.0); *b = round_n(K*255.0);}
    if (I == 3) { *r = round_n(M*255.0); *g = round_n(N*255.0); *b = round_n(V*255.0);}
    if (I == 4) { *r = round_n(K*255.0); *g = round_n(M*255.0); *b = round_n(V*255.0);}
    if (I == 5) { *r = round_n(V*255.0); *g = round_n(M*255.0); *b = round_n(N*255.0);}
}

int main() {
    int x, y, z, i, width, height, points_count, radius, empty = 0;
    int r, g, b;
    IplImage *img = NULL;
    cvNamedWindow("render", 1);
    while (!feof(stdin)) {
        empty = scanf("%d %d", &width, &height);
        if (img == NULL)
            img = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
        for (x=0; x < width; x++) {
            for (y=0; y < width; y++) {
                empty = scanf("%d", &z);
                if (z == 0) {
                    cvSet2D(img, x, y, cvScalar(0,0,0,0));
                } else {
                    hsv2rgb(z*20, &r, &g, &b);
                    cvSet2D(img, x, y, cvScalar(r,g,b,0));
                }
            }
        }
        empty = scanf("%d %d %d %d %d", &x, &y, &z, &radius, &points_count);
        for (i=0; i < points_count; i++) {
            empty = scanf("%d %d %d", &x, &y, &z);
        }
        cvShowImage("render", img);
        cvWaitKey(1);
    }
    cvShowImage("render", img);
    cvWaitKey(0);
    return empty;
}
