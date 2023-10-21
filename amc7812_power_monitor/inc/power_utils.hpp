#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <mutex>
#include <string>

/* the same as what's in the driver code */
typedef struct adc_sample_union {
    uint8_t index;
    int64_t timestamp;
    int16_t value;
}adc_sample_t;


class PowerDataclass{
private:
    int m_index;
    int m_readings;
    double m_real_power;
    double m_apparent_power;
    double m_interval;
    std::mutex m_mtx;
public:
    PowerDataclass(int index, double real, double apparent): m_mtx(){
        m_index = index;
        m_real_power = real;
        m_apparent_power = apparent;
    };
    PowerDataclass(const PowerDataclass& og) :
        m_mtx(),
        m_index(og.get_index()), 
        m_real_power(og.get_real_power()),
        m_apparent_power(og.get_apparent_power()),
        m_readings(og.get_readings()),
        m_interval(og.get_interval())
    { };
    PowerDataclass():
        m_mtx(),
        m_index(0), 
        m_real_power(0.0),
        m_apparent_power(0.0)
    { };
    int get_index() const{return m_index;};
    int get_readings() const{return m_readings;};
    double get_interval() const{return m_interval;};
    double get_real_power() const{return m_real_power;};
    double get_apparent_power() const{return m_apparent_power;};
    void set_readings(int readings){
        const std::lock_guard<std::mutex> lock(m_mtx);
        m_readings = readings;
    };
    void set_index(int index){
        const std::lock_guard<std::mutex> lock(m_mtx);
        m_index = index;
    };
    void set_real_power(double real){
        const std::lock_guard<std::mutex> lock(m_mtx);
        m_real_power = real;
    };
    void set_apparent_power(double apparent){
        const std::lock_guard<std::mutex> lock(m_mtx);
        m_apparent_power = apparent;
    };
    void set_interval(double interval){
        const std::lock_guard<std::mutex> lock(m_mtx);
        m_interval = interval;
    };
    double get_power_factor() const{
        return m_real_power/m_apparent_power;
    };
    std::string to_json();
};