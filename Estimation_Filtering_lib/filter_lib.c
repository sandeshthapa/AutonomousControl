//
// Created by sandesh.thapa on 9/25/2025.
//

#include "filter_lib.h"

#include <stdio.h>

lpf_t filter;

void lpf_init(lpf_t *lpf, float alpha,  float y0) {
    lpf->alpha = alpha;
    lpf->y = y0;
}

float  lpf_update(lpf_t *lpf,  float x) {
    lpf->y = (1.0f - lpf->alpha) * lpf->y + lpf->alpha * x;
    return lpf->y;
}
int main(void) {
    float sample[10] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    // float y0 = 0;
    float alpha0 = 0.01f;
    lpf_init(&filter, alpha0, sample[0]);
    int m = sizeof(sample) / sizeof(float);
    for (int i = 0; i < m; i++) {
        float out =  lpf_update(&filter, sample[i]);
        printf("sample=%.2f, lowpass_o=%.2f\n", sample[i], out);

    }

}