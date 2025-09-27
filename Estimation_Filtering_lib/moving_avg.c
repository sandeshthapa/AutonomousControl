//
// Created by sandesh.thapa on 9/20/2025.
//
#include <stdio.h>

#define WINDOW_SIZE 5
#define DEBUG_EN 0
#define LPF_WINDOWS_SIZE 10

// state for moving average
static int avg_buffer[WINDOW_SIZE];
static int avg_curr_idx = 0;
static int avg_count = 0;
static int avg_curr_sum = 0;

static int lpf_buffer[LPF_WINDOWS_SIZE];
static float lpf_out[LPF_WINDOWS_SIZE];
static int lpf_curr_idx = 0;
static int lpf_sum = 0;
static int lpf_count = 0;

float get_moving_average(int y_sample) {
    // subtract oldest value from sum
    avg_curr_sum -= avg_buffer[avg_curr_idx];

    // add new sample
    avg_buffer[avg_curr_idx] = y_sample;
    avg_curr_sum += y_sample;

    // move index circularly
    avg_curr_idx = (avg_curr_idx + 1) % WINDOW_SIZE;

    if (DEBUG_EN) {
        printf("curr_idx = %d\n", avg_curr_idx);
    }

    if (avg_count < WINDOW_SIZE) {
        avg_count++;
    }

    return (float)avg_curr_sum / avg_count;
}

float simple_low_pass(int y_raw, float llp_alpha) {

    lpf_buffer[lpf_curr_idx] = y_raw;

    lpf_curr_idx = (lpf_curr_idx + 1) % LPF_WINDOWS_SIZE;

    // implement low pass filter
    // y(t_k) = (1- alpha)* y(t_k - 1) + alpha x(k) , x(k) is the input
     lpf_out[lpf_curr_idx] = (float) (1.0 - llp_alpha) * lpf_out[lpf_curr_idx -1] + llp_alpha * lpf_buffer[lpf_curr_idx];

    return lpf_out[lpf_curr_idx];
}

// read some files


int main() {
    int y_mes[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    int m = sizeof(y_mes) / sizeof(y_mes[0]);

    float llp_alpha = 0.05;
    float low_pass_array[m];

    if (m > 0) {
        for (int i = 0; i < m; i++) {
            low_pass_array[i] = simple_low_pass(y_mes[i], llp_alpha);
            printf("lowpass_out = %f\n", low_pass_array[i]);

            float average = get_moving_average(low_pass_array[i]);
            printf("Input: %d, Moving Average: %.2f\n", y_mes[i], average);
        }
    }
    else{
        printf("Array is empty, No moving average\n");
        }

    return 0;
}
