

#include "filter.hpp"

FirFilter::FirFilter(std::vector<int> taps) : m_history(taps.size(), 0), m_taps(taps){
    // nothing to do here?
    m_last_index = 0;
}

int FirFilter::get(){
    long long acc = 0;
    int index = m_last_index;
    for(int i = 0; i < m_taps.size(); ++i) {
        index = index != 0 ? index - 1 : m_taps.size() - 1;
        acc += (long long)m_history[index--] * m_taps[i];
    };
    return acc >> 16;
}

void FirFilter::put(int input){
    m_history[m_last_index++] = input;
    if(m_last_index == m_taps.size()){
        m_last_index = 0;
    }
}
