#include "Demodulator.h"
#include <iostream>
void Demodulator::init() { std::cout << "Demodulator initialized" << std::endl; }
std::vector<float> Demodulator::demodulate(const std::vector<float>& signal) { return signal; }