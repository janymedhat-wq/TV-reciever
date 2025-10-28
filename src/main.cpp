#include <iostream>
#include "LNB.h"
#include "Demodulator.h"
#include "FEC.h"
#include "Decoder.h"
#include "Video.h"
#include "Audio.h"

int main() {
    std::cout << "Digital TV Receiver Simulation" << std::endl;

    LNB lnb;
    Demodulator demod;
    FEC fec;
    Decoder decoder;
    Video video;
    Audio audio;

    // Initialize modules
    lnb.init();
    demod.init();
    fec.init();
    decoder.init();
    video.init();
    audio.init();

    while (true) {
        auto rawSignal = lnb.sampleSignal();
        auto demodSignal = demod.demodulate(rawSignal);
        auto correctedSignal = fec.correct(demodSignal);
        auto videoFrames = decoder.decodeVideo(correctedSignal);
        auto audioFrames = decoder.decodeAudio(correctedSignal);
        video.displayFrame(videoFrames);
        audio.play(audioFrames);
    }

    return 0;
}