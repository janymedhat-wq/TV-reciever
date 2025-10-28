#include "Video.h"
#include <iostream>
void Video::init() { std::cout << "Video module initialized" << std::endl; }
void Video::displayFrame(const std::vector<std::string>& frame) {
    std::cout << "Displaying Video Frame: "; for (auto &f : frame) std::cout << f << " "; std::cout << std::endl;
}