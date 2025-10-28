#include "FEC.h"
#include <iostream>
void FEC::init() { std::cout << "FEC module initialized" << std::endl; }
std::vector<float> FEC::correct(const std::vector<float>& signal) { return signal; }