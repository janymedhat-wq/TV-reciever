#pragma once
#include <vector>
#include <string>
class Decoder {
public:
    void init();
    std::vector<std::string> decodeVideo(const std::vector<float>& signal);
    std::vector<std::string> decodeAudio(const std::vector<float>& signal);
};