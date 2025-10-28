#include "LNB.h"
#include <cmath>
#include <iostream>
void LNB::init() { std::cout << "LNB initialized" << std::endl; }
std::vector<float> LNB::sampleSignal() {
    std::vector<float> samples(1024);
    for (int i = 0; i < 1024; i++) {
        samples[i] = sin(2 * 3.14159 * i / 1024);
    }
    return samples;
}