

#include "power_utils.hpp"
#include <iomanip>
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream



std::string PowerDataclass::to_json(){
    std::stringstream ss;

    ss.setf(std::ios::fixed);
    ss.precision(3);

    ss << "{";
    
    ss << "channel: " << m_index << ", ";
    ss << "real_power: " << m_real_power << ", ";
    ss << "apparent_power: " << m_apparent_power << ", ";
    ss << "interval: " << m_interval << ",";
    // ss << "power_factor: " << get_power_factor();

    ss << "}";

    return ss.str();
}