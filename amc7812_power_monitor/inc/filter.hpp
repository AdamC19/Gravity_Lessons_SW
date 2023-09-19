#pragma once


#include <string>
#include <vector>

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 31250 Hz

fixed point precision: 16 bits

* 0 Hz - 3600 Hz
  gain = 1
  desired ripple = 0.2 dB
  actual ripple = 0.13

* 8000 Hz - 15600 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -41.22

*/

#define FIR_FILTER_TAP_NUM 16


class FirFilter {
private:
    std::vector<int> m_taps;
    std::vector<int> m_history;
    int m_last_index;
public:
    FirFilter(std::vector<int> taps);
    void put(int input);
    int get();
    int get_size(){
        return m_history.size();
    };
};

