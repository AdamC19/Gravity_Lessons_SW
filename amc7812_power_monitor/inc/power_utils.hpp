#pragma once

#include <cstdio>
#include <cstdlib>
#include <string>

/* the same as what's in the driver code */
typedef union adc_sample_union {
    struct {
        unsigned char index;
        unsigned long long timestamp;
        unsigned short value;
    };
    unsigned char data[11];
}adc_sample_t;
