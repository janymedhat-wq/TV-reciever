#pragma once
#include <vector>
#include <string>
class Video {
public:
    void init();
    void displayFrame(const std::vector<std::string>& frame);
};