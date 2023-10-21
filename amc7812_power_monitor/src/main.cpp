
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
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


int filter_taps[FIR_FILTER_TAP_NUM] = {
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



void input_output(std::shared_ptr<std::vector<PowerDataclass>>& power_samples){

    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        for(int i = 0; i < CHANNEL_COUNT; i++){
            std::cout << power_samples->at(i).to_json() << std::endl;
        }
    }
}


void print_binary(uint8_t num){
    for(int i = 0; i < 8; i++){
        std::cout << ((num >> (7-i)) & 1);
    }
    std::cout << std::endl;
}

void dump_buffer(uint8_t* buffer, int len){
    int line_len = 33;
    int column = 0;
    for(int byte_ind = 0; byte_ind < len; byte_ind++){
        printf("%02X ", buffer[byte_ind]);
        column += 3;
        if(column >= line_len){
            printf("\r\n");
            column = 0;
        }
    }
    printf("\r\n");
}

/**
 * Application entry point
*/
int main(int argc, char** argv){

    int daq_fd = open(DAQ_FNAME, O_RDONLY); // open character device for reading
    if (daq_fd < 0){
        printf("Driver likely not loaded. Exiting...\r\n");
        return -1;
    }

    std::map<int, adc_sample_t> samples;
    // std::vector<adc_sample_t> samples;  // a raw sample for each channel
    std::vector<FirFilter> filters;     // filters for filtering the raw samples;
    auto power_samples = std::make_shared<std::vector<PowerDataclass>>(CHANNEL_COUNT);

    // create blank samples and blank filters
    for(int i = 0; i < CHANNEL_COUNT; i++){
        adc_sample_t tmp;
        tmp.index = i;
        samples[i] = tmp;
        filters.push_back( FirFilter(std::vector<int>(filter_taps, filter_taps + FIR_FILTER_TAP_NUM )));
        // power_samples->push_back(PowerDataclass(i, 0.0, 0.0));
    }

    // std::thread input_output_thread(input_output, std::ref(power_samples));

    uint8_t buffer[1024];
    int sample_count = 0;
    std::vector<int64_t> rms_sums(CHANNEL_COUNT);
    std::vector<int64_t> real_power_sums(CHANNEL_COUNT);
    int64_t v_sns = 0;
    int64_t v_comm = 2048;
    int cycles_per_output = 1;
    auto timestamp = std::chrono::high_resolution_clock::now();
    double avg_loop_time_us = 0.0;

    std::cout << "SIZEOF adc_sample_t: " << sizeof(adc_sample_t) << std::endl;
    while(true){
        int readnum = read(daq_fd, buffer, 1024);
        dump_buffer(buffer, readnum);
        auto read_complete_timestamp = std::chrono::high_resolution_clock::now();
        auto loop_time = read_complete_timestamp - timestamp;
        timestamp = read_complete_timestamp;
        avg_loop_time_us += std::chrono::duration_cast<std::chrono::microseconds>(loop_time).count();

        if (readnum == 0){
            printf("Timed out trying to get data from DAQ...\n");
        }else{
            int offset = 0;
            // auto sample = samples.begin();
            for(int i = 0; i < 16; i++){
                memcpy((uint8_t*)&(samples.at(i).index), buffer + offset, 1);
                offset += 1;
                memcpy((uint8_t*)&(samples.at(i).timestamp), buffer + offset, 8);
                offset += 8;
                memcpy((uint8_t*)&(samples.at(i).value), buffer + offset, 2);
                offset += 2;

                if (samples.at(i).index >= 0 && samples.at(i).index < CHANNEL_COUNT){
                    if (samples.at(i).index != COMM_MODE_CH){
                        filters.at(samples.at(i).index).put((int)(samples.at(i).value - v_comm)); // subtract common mode here
                    }else{
                        filters.at(samples.at(i).index).put((int)samples.at(i).value); // don't subtrace v_comm
                    }
                    
                    int64_t filt_sample = (int64_t)samples.at(i).value; //(int64_t)(filters.at(samples.at(i).index).get()); //

                    rms_sums[samples.at(i).index] += filt_sample*filt_sample;

                    if(samples.at(i).index == VOLT_SNS_CH){
                        v_sns = filt_sample;
                    }else if(samples.at(i).index == COMM_MODE_CH){
                        v_comm = filt_sample;
                    }else{
                        // treat appropriate for the current channels
                        real_power_sums[samples.at(i).index] += v_sns * filt_sample; 
                    }
                }else{
                    printf("Got %d sample index.\r\n", samples.at(i).index);
                }
            }
            sample_count++;
            
            if(sample_count >= cycles_per_output*SINGLE_CYCLE_SAMPLE_COUNT){
                avg_loop_time_us = avg_loop_time_us/sample_count;
                double rms_v = sqrtf64(1.0*rms_sums[VOLT_SNS_CH]/sample_count);
                std::cout << "V_COMM: " << v_comm << std::endl;
                for(int i = 0; i < CHANNEL_COUNT; i++){
                    double this_rms = sqrtf64(1.0*rms_sums[i]/sample_count);
                    power_samples->at(i).set_interval(avg_loop_time_us);
                    power_samples->at(i).set_readings(sample_count);
                    power_samples->at(i).set_index(i);
                    power_samples->at(i).set_real_power(1.0*real_power_sums[i]/sample_count);
                    power_samples->at(i).set_apparent_power(rms_v * this_rms);
                    // std::cout << power_samples->at(i).to_json() << std::endl;
                    real_power_sums[i] = 0;
                    rms_sums[i] = 0;
                }
                avg_loop_time_us = 0.0;
                sample_count = 0;
            }
        }
    }

    return 0;
}