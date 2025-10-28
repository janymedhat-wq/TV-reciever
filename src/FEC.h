#pragma once
#include <vector>
class FEC {
public:
    void init();
    std::vector<float> correct(const std::vector<float>& signal);
};