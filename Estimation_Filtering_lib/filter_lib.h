//
// Created by sandesh.thapa on 9/25/2025.
//

#ifndef DECODE_CAN_FILTER_LIB_H
#define DECODE_CAN_FILTER_LIB_H

typedef struct lpf_t {
    float alpha;
    float y;
} lpf_t;

void lpf_init(lpf_t *lpf, float alpha,  float y0) ;

float  lpf_update(lpf_t *lpf, float x) ;
#endif //DECODE_CAN_FILTER_LIB_H