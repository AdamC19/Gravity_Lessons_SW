
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/ioctl.h>
#include <vector>

#include "filter.hpp"
#include "power_utils.hpp"

#define DAQ_FNAME       "/dev/amc7812"
#define DELIM_COUNT     4
#define CHANNEL_COUNT   16
#define BUF_SIZE        (DELIM_COUNT + (CHANNEL_COUNT*sizeof(adc_sample_t)))
#define DELIM_CHAR      0xA5

#define VOLT_SNS_CH     0
#define COMM_MODE_CH    1

/* This number is really only approximate (exact number would be 520.8333...)*/
#define SINGLE_CYCLE_SAMPLE_COUNT    521

static int filter_taps[FIR_FILTER_TAP_NUM] = {
    138,
    573,
    101,
    -1231,
    -1818,
    810,
    6507,
    11426,
    11426,
    6507,
    810,
    -1818,
    -1231,
    101,
    573,
    138
};



/**
 * Application entry point
*/
int main(int argc, char** argv){

    int daq_fd = open(DAQ_FNAME, 'r'); // open character device for reading
    if (daq_fd < 0){
        printf("Driver likely not loaded. Exiting...");
        return -1;
    }

    std::vector<adc_sample_t> samples;  // a raw sample for each channel
    std::vector<FirFilter> filters;     // filters for filtering the raw samples;
    std::vector<long long> rms_sums;    // RMS summation for each channel
    std::vector<long long> real_power_sums; // real power summation for each channel

    // create blank samples and blank filters
    for(int i = 0; i < CHANNEL_COUNT; i++){
        adc_sample_t tmp;
        tmp.index = 0;
        samples.push_back(tmp);
        filters.push_back( FirFilter(std::vector<int>(filter_taps, filter_taps + sizeof(filter_taps)/sizeof(int)) )   );
    }

    bool found_delim = false;
    uint8_t buffer[DELIM_COUNT];
    int delim_count = 0;
    auto channel_filter = filters.begin();
    auto sample = samples.begin();

    while(true){
        if(!found_delim){
            // read one byte at a time until we've gotten our delimiter
            read(daq_fd, buffer + delim_count, 1);

            if (buffer[delim_count] == DELIM_CHAR){
                delim_count++;
            }else{
                delim_count = 0;
            }
            found_delim = (delim_count == DELIM_COUNT);
        }else{
            // we have the delimiter, so we can proceed to read a channel
            // we'll do this op CHANNEL_COUNT-many times
            read(daq_fd, sample->data, sizeof(adc_sample_t));

            channel_filter->put((int)sample->value);

            if(sample->index == VOLT_SNS_CH){
                // treat appropriate for the voltage
            }else if(sample->index == COMM_MODE_CH){
                // common mode to subtract from everything else
            }

            
            sample++;
            if(sample == samples.end()){
                sample = samples.begin();
            }
            channel_filter++;
            if(channel_filter == filters.end()){
                channel_filter = filters.begin();
                found_delim = false;
            }
        }
    }

    return 0;
}