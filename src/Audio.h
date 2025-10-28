#pragma once
#include <vector>
#include <string>
class Audio {
public:
    void init();
    void play(const std::vector<std::string>& audio);
};