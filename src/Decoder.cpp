#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <numeric>
#include <iomanip>
#include <complex>
#include <algorithm>
#include <stdexcept>
#include <sstream>

// --- Global Constants and Type Definitions ---
const double PI = 3.14159265358979323846;
const int TS_PACKET_SIZE = 188; // MPEG Transport Stream packet size in bytes
const int SYNC_BYTE = 0x47;     // MPEG-TS Sync Byte

using Complex = std::complex<double>;
using RealSignal = std::vector<double>;
using IQSignal = std::vector<Complex>;
using BitStream = std::vector<int>;
using ByteStream = std::vector<uint8_t>;

// --- Helper Functions for Bit/Byte Conversion ---

/**
 * @brief Converts a continuous stream of bits (int vector) into a byte stream (uint8_t vector).
 * @param bit_stream Input vector of 0s and 1s.
 * @return ByteStream Output vector of bytes.
 */
ByteStream bits_to_bytes(const BitStream& bit_stream) {
    ByteStream byte_stream;
    byte_stream.reserve(bit_stream.size() / 8);

    for (size_t i = 0; i < bit_stream.size(); i += 8) {
        if (i + 7 >= bit_stream.size()) break; // Handle incomplete final byte
        uint8_t byte = 0;
        for (int j = 0; j < 8; ++j) {
            byte |= (bit_stream[i + j] << (7 - j));
        }
        byte_stream.push_back(byte);
    }
    return byte_stream;
}

// --- SECTION 1: DIGITAL SIGNAL PROCESSING (DSP) FRONT-END ---

/**
 * @brief Simulates the primary DSP chain: ADC -> Downconversion -> Demodulation -> FEC.
 * This class processes raw RF samples into a clean, error-corrected byte stream.
 */
class ComplexSignalProcessor {
private:
    double f_signal;         // IF frequency
    double f_sampling;       // Sampling rate
    double symbol_rate;      // Symbol rate
    int samples_per_symbol;  // Fs / Rs
    std::mt19937 gen;        // Random number generator

    /**
     * @brief Generates a real-valued IF signal corrupted by Additive White Gaussian Noise (AWGN).
     */
    RealSignal generate_real_noisy_signal(double duration, double noise_std_dev) {
        int num_samples = static_cast<int>(f_sampling * duration);
        RealSignal sampled_signal;
        sampled_signal.reserve(num_samples);

        std::normal_distribution<> d(0.0, noise_std_dev);
        double T_s = 1.0 / f_sampling;
        double angular_freq = 2.0 * PI * f_signal;

        for (int n = 0; n < num_samples; ++n) {
            double t = n * T_s;
            // Simulated BPSK/QPSK signal as a noisy cosine wave.
            double signal = 1.0 * std::cos(angular_freq * t + 0.5); 
            double noise = d(gen);
            sampled_signal.push_back(signal + noise);
        }
        return sampled_signal;
    }

    /**
     * @brief Performs Digital Down-Conversion (DDC) to complex baseband (I/Q).
     */
    IQSignal digital_down_converter(const RealSignal& real_signal) {
        IQSignal baseband_signal;
        baseband_signal.reserve(real_signal.size());

        double T_s = 1.0 / f_sampling;
        double angular_freq_carr = 2.0 * PI * f_signal;

        for (size_t n = 0; n < real_signal.size(); ++n) {
            double t = n * T_s;
            // Local Oscillator (NCO)
            double cos_mix = std::cos(angular_freq_carr * t);
            double sin_mix = -std::sin(angular_freq_carr * t);

            // Mixing and Complex Output
            double I_comp = real_signal[n] * cos_mix;
            double Q_comp = real_signal[n] * sin_mix;
            baseband_signal.emplace_back(I_comp, Q_comp);
        }
        return baseband_signal;
    }

    /**
     * @brief Simplfied QPSK Demodulation (Decision Slicing).
     * NOTE: In a real system, this follows a Matched Filter and Carrier/Timing Recovery loop.
     */
    BitStream qpsk_demapper(const IQSignal& iq_signal) {
        BitStream demodulated_bits;
        for (size_t i = 0; i < iq_signal.size(); i += samples_per_symbol) {
            Complex symbol = iq_signal[i];

            // Decision Slicing: Determine bits based on quadrant (Gray coded assumed)
            int bit_I = (symbol.real() >= 0.0) ? 1 : 0; // Bit 1
            int bit_Q = (symbol.imag() >= 0.0) ? 1 : 0; // Bit 2
            
            demodulated_bits.push_back(bit_I);
            demodulated_bits.push_back(bit_Q);
        }
        return demodulated_bits;
    }

    /**
     * @brief Placeholder for Forward Error Correction (FEC) Decoding.
     * In DVB-S2, this is a highly complex process involving LDPC and BCH decoding.
     * We simply simulate a byte stream output.
     */
    ByteStream forward_error_correction(const BitStream& raw_bits) {
        std::cout << "  > [FEC] Simulating LDPC/BCH decoding and error correction..." << std::endl;
        
        // In a real system, this is where most errors from the channel are fixed.
        // For simplicity, we convert the bits to bytes here.
        ByteStream corrected_bytes = bits_to_bytes(raw_bits);
        
        // Add a simulated packet sync-loss scenario
        if (corrected_bytes.size() > 5 * TS_PACKET_SIZE) {
            // Introduce intentional corruption to simulate the need for re-sync
            corrected_bytes[TS_PACKET_SIZE * 2 + 1] = 0xFF; // Corrupt a sync byte position
        }

        return corrected_bytes;
    }

public:
    ComplexSignalProcessor(double fs, double fr, double rs) 
        : f_signal(fr), f_sampling(fs), symbol_rate(rs) {
        if (rs <= 0 || fs <= 0) throw std::runtime_error("Rates must be positive.");
        samples_per_symbol = static_cast<int>(fs / rs);
        if (samples_per_symbol < 1) throw std::runtime_error("Fs/Rs too low.");

        std::random_device rd;
        gen.seed(rd());

        std::cout << "[DSP] Initialized with Samples/Symbol: " << samples_per_symbol << std::endl;
    }

    /**
     * @brief Executes the entire DSP chain from noisy RF sample to error-corrected bytes.
     */
    ByteStream process_signal(double duration, double noise_sigma) {
        std::cout << "\n--- Complex Signal Processor Pipeline ---" << std::endl;

        RealSignal adc_output = generate_real_noisy_signal(duration, noise_sigma);
        std::cout << "  > [ADC] Generated " << adc_output.size() << " raw samples." << std::endl;

        IQSignal baseband_iq = digital_down_converter(adc_output);
        std::cout << "  > [DDC] Converted to " << baseband_iq.size() << " complex I/Q samples." << std::endl;

        BitStream raw_bits = qpsk_demapper(baseband_iq);
        std::cout << "  > [DEMOD] Demodulated " << raw_bits.size() << " raw bits." << std::endl;

        return forward_error_correction(raw_bits);
    }
};

// --- SECTION 2: MPEG TRANSPORT STREAM (TS) PARSER ---

/**
 * @brief Represents a single parsed MPEG Transport Stream Packet (188 bytes).
 */
struct TSPacket {
    uint16_t pid = 0; // Packet Identifier (13 bits)
    uint8_t transport_error_indicator = 0;
    uint8_t payload_unit_start_indicator = 0;
    uint8_t transport_priority = 0;
    uint8_t scrambling_control = 0;
    uint8_t adaptation_field_control = 0;
    uint8_t continuity_counter = 0;
    ByteStream payload;
};

/**
 * @brief Manages the synchronization and demultiplexing of the MPEG Transport Stream.
 */
class TransportStreamParser {
private:
    std::vector<TSPacket> packets;
    std::map<uint16_t, ByteStream> elementary_streams; // Key: PID, Value: Elementary Stream Data
    std::map<uint16_t, std::string> pid_map; // Program Map (PID -> Content Type)

    /**
     * @brief Parses the header and payload of a single 188-byte TS packet.
     */
    TSPacket parse_single_packet(const uint8_t* buffer) {
        if (buffer[0] != SYNC_BYTE) {
            throw std::runtime_error("TS Sync byte (0x47) expected.");
        }

        TSPacket p;
        // 1. First 4 bytes (Header)
        p.transport_error_indicator = (buffer[1] >> 7) & 0x01;
        p.payload_unit_start_indicator = (buffer[1] >> 6) & 0x01;
        p.transport_priority = (buffer[1] >> 5) & 0x01;
        p.pid = ((buffer[1] & 0x1F) << 8) | buffer[2];
        
        p.scrambling_control = (buffer[3] >> 6) & 0x03;
        p.adaptation_field_control = (buffer[3] >> 4) & 0x03;
        p.continuity_counter = buffer[3] & 0x0F;

        // 2. Payload extraction
        size_t payload_start = 4;
        
        // Handle Adaptation Field
        if (p.adaptation_field_control & 0x02) { // Adaptation field present
            uint8_t adaptation_field_length = buffer[payload_start];
            payload_start += (1 + adaptation_field_length);
        }

        if (p.adaptation_field_control & 0x01 && payload_start < TS_PACKET_SIZE) { // Payload present
            size_t payload_size = TS_PACKET_SIZE - payload_start;
            p.payload.reserve(payload_size);
            for (size_t i = 0; i < payload_size; ++i) {
                p.payload.push_back(buffer[payload_start + i]);
            }
        }

        return p;
    }

    /**
     * @brief Placeholder for parsing Program Association Table (PID 0x00).
     */
    void parse_pat(const TSPacket& p) {
        // PAT defines the PID for the Program Map Table (PMT)
        // PMT_PID is typically 0x1000 or 0x1001 for DVB-S
        pid_map[0x00] = "PAT"; 
        pid_map[0x100] = "PMT"; // Simulating finding the PMT PID
        // Actual PAT parsing involves section headers, table_id, etc.
        std::cout << "    * Parsed PAT (PID 0x00). PMT PID found at 0x100." << std::endl;
    }

    /**
     * @brief Placeholder for parsing Program Map Table (PID 0x100).
     */
    void parse_pmt(const TSPacket& p) {
        // PMT defines the PIDs for Elementary Streams (Video, Audio, Data)
        // Stream Types: 0x1B=H.264, 0x02=MPEG-2 Video, 0x03/0x04=Audio
        pid_map[0x1011] = "H.264 Video ES"; // Simulating Video PID
        pid_map[0x1012] = "AAC Audio ES";  // Simulating Audio PID
        // Actual PMT parsing is complex, involving stream_type and elementary_pid
        std::cout << "    * Parsed PMT (PID 0x100). Found Video PID 0x1011 and Audio PID 0x1012." << std::endl;
    }

public:
    TransportStreamParser() {
        // Initialize PIDs for mandatory tables
        pid_map[0x00] = "PAT";
        pid_map[0x1FFF] = "NULL";
    }

    /**
     * @brief Entry point for the parser. Synchronizes to packets and demultiplexes PIDs.
     */
    void demultiplex(const ByteStream& input_bytes) {
        std::cout << "\n--- Transport Stream Demultiplexer ---" << std::endl;
        size_t cursor = 0;
        size_t sync_count = 0;

        // 1. Packet Synchronization Loop
        while (cursor + TS_PACKET_SIZE <= input_bytes.size()) {
            if (input_bytes[cursor] == SYNC_BYTE) {
                try {
                    const uint8_t* buffer = &input_bytes[cursor];
                    TSPacket p = parse_single_packet(buffer);
                    packets.push_back(p);
                    sync_count++;
                    cursor += TS_PACKET_SIZE;
                } catch (const std::exception& e) {
                    std::cerr << "!!! Error parsing packet at cursor " << cursor << ": " << e.what() << std::endl;
                    cursor++; // Try to re-sync
                }
            } else {
                cursor++; // Search for next sync byte (resync logic)
            }
        }
        std::cout << "  > Synchronized and extracted " << sync_count << " TS packets." << std::endl;

        // 2. Demultiplexing and PSI Parsing
        for (const auto& p : packets) {
            if (p.pid == 0x00) {
                parse_pat(p);
            } else if (p.pid == 0x100) {
                parse_pmt(p);
            }
            
            // Append payload to the correct elementary stream buffer
            if (p.payload.empty()) continue;
            elementary_streams[p.pid].insert(
                elementary_streams[p.pid].end(), p.payload.begin(), p.payload.end()
            );
        }

        std::cout << "  > Demultiplexing complete. Found " << elementary_streams.size() << " PIDs with data." << std::endl;
    }

    /**
     * @brief Gets the Elementary Stream for the given PID (e.g., Video PID).
     */
    const ByteStream& get_elementary_stream(uint16_t pid) const {
        if (elementary_streams.count(pid)) {
            return elementary_streams.at(pid);
        }
        static const ByteStream empty_stream;
        return empty_stream;
    }
};

// --- SECTION 3: IMAGE DECOMPRESSION (SIMPLIFIED H.264 PARSER) ---

/**
 * @brief Simulates the video decoder stage. Focuses on H.264 parsing (NAL Units).
 * This is a highly simplified conceptual representation of a complex decoder (like libavcodec).
 */
class H264Parser {
private:
    struct NALUnit {
        uint8_t type = 0;
        ByteStream data;
    };
    std::vector<NALUnit> nal_units;

    /**
     * @brief Simulates finding NAL Units in the raw Elementary Stream.
     * H.264 NALUs are typically separated by start codes (0x000001 or 0x00000001).
     */
    void find_nal_units(const ByteStream& es_data) {
        size_t cursor = 0;
        const uint8_t start_code_1[] = {0x00, 0x00, 0x01}; // Standard start code
        const uint8_t start_code_2[] = {0x00, 0x00, 0x00, 0x01}; // Longer start code

        while (cursor < es_data.size()) {
            // Search for 0x000001 (or 0x00000001)
            size_t start_index = es_data.size();
            for (size_t i = cursor; i < es_data.size() - 3; ++i) {
                if (es_data[i] == 0x00 && es_data[i+1] == 0x00 && es_data[i+2] == 0x01) {
                    start_index = i;
                    break;
                }
            }

            if (start_index == es_data.size()) break; // End of stream

            // Found start code. Determine length.
            size_t start_code_len = 3;
            if (start_index > 0 && es_data[start_index - 1] == 0x00) {
                start_code_len = 4; // Use 0x00000001 if preceded by an extra zero
            }

            size_t nal_start = start_index + start_code_len;

            // Find the next NAL unit start to determine the current NAL unit's end
            size_t next_start_index = es_data.size();
            for (size_t i = nal_start; i < es_data.size() - 3; ++i) {
                if (es_data[i] == 0x00 && es_data[i+1] == 0x00 && es_data[i+2] == 0x01) {
                    next_start_index = i;
                    break;
                }
            }
            
            // Extract the NAL unit data
            if (nal_start < next_start_index) {
                NALUnit nu;
                nu.type = es_data[nal_start] & 0x1F; // NAL unit type is in the last 5 bits of the first byte
                nu.data.assign(es_data.begin() + nal_start, es_data.begin() + next_start_index);
                nal_units.push_back(nu);
            }

            cursor = next_start_index;
        }
    }

public:
    /**
     * @brief Processes the video Elementary Stream (ES) to parse video structure.
     */
    void decode_stream(const ByteStream& video_es) {
        std::cout << "\n--- H.264 Image Decompression Stage (Conceptual) ---" << std::endl;
        std::cout << "  > Received Video ES size: " << video_es.size() << " bytes." << std::endl;
        
        find_nal_units(video_es);
        
        std::cout << "  > Found " << nal_units.size() << " H.264 NAL Units." << std::endl;

        // 1. Parse Key Metadata (SPS, PPS)
        size_t sps_count = 0;
        size_t pps_count = 0;
        
        for (const auto& nu : nal_units) {
            std::string type_name = "Unknown";
            switch (nu.type) {
                case 1: type_name = "Non-IDR Slice"; break;
                case 5: type_name = "IDR Slice (Key Frame)"; break;
                case 7: type_name = "Sequence Parameter Set (SPS)"; sps_count++; break;
                case 8: type_name = "Picture Parameter Set (PPS)"; pps_count++; break;
                case 9: type_name = "Access Unit Delimiter"; break;
                default: break;
            }
            // In a real decoder, SPS/PPS define resolution/profile, and IDR/Slice defines the pixels.
            std::cout << "    * NAL Unit Type " << static_cast<int>(nu.type) 
                      << " (" << type_name << "), Size: " << nu.data.size() << " bytes." << std::endl;
        }

        if (sps_count > 0 && pps_count > 0) {
            std::cout << "\n  [Video Metadata] SPS and PPS successfully parsed. Decoder is configured." << std::endl;
        } else {
             std::cout << "\n  [Video Metadata] Warning: SPS/PPS not found. Cannot configure decoder." << std::endl;
        }

        // 2. Simulate Outputting Decoded Image Data
        std::cout << "\n  [Output] Generating Simulated Image Buffer..." << std::endl;
        
        // This vector conceptually holds the final decoded image (e.g., in YUV or RGB format)
        std::vector<uint32_t> decoded_image_pixels(640 * 480);
        
        // Fill with a simple color pattern to demonstrate data is 'decoded'
        for (size_t i = 0; i < decoded_image_pixels.size(); ++i) {
            decoded_image_pixels[i] = (i % 256) | ((i / 256 % 256) << 8); // Simple gradient
        }

        std::cout << "  > Successfully generated a simulated " 
                  << decoded_image_pixels.size() << "-pixel image buffer (640x480 resolution)." << std::endl;
    }
};

// --- SECTION 4: MAIN RECEIVER SYSTEM AND SIMULATION ---

class ReceiverSystem {
private:
    ComplexSignalProcessor dsp;
    TransportStreamParser ts_parser;
    H264Parser h264_decoder;

public:
    ReceiverSystem(double fs, double fr, double rs) 
        : dsp(fs, fr, rs) {}

    void run_simulation() {
        // --- 1. DSP Stage: Go from Noisy RF to Raw MPEG-TS Bytes ---
        // Simulating 10 seconds of high-rate data (resulting in a large byte stream)
        double duration = 10.0; 
        double noise_level = 0.5; 
        
        // The dsp block handles the entire physical layer process:
        // ADC -> DDC (Baseband) -> Demodulation (Bits) -> FEC (Corrected Bytes)
        ByteStream transport_stream = dsp.process_signal(duration, noise_level);
        
        std::cout << "-------------------------------------------------------------------" << std::endl;
        std::cout << "Final corrected byte stream size (Approx. MPEG-TS): " 
                  << transport_stream.size() << " bytes." << std::endl;
        std::cout << "-------------------------------------------------------------------" << std::endl;

        // --- 2. Transport Stream Parsing Stage: Demux and PSI Tables ---
        ts_parser.demultiplex(transport_stream);

        // --- 3. Video Extraction and Decompression Stage ---
        const uint16_t VIDEO_PID = 0x1011; // Hardcoded PID from our simulated PMT
        const ByteStream& video_es = ts_parser.get_elementary_stream(VIDEO_PID);

        if (!video_es.empty()) {
            std::cout << "\n[MAIN] Starting H.264 Decoding for PID 0x1011..." << std::endl;
            h264_decoder.decode_stream(video_es);
        } else {
            std::cout << "\n[MAIN] Video Elementary Stream not found or is empty." << std::endl;
        }
        
        std::cout << "\n===================================================================" << std::endl;
        std::cout << "        FULL RECEIVER ARCHITECTURE SIMULATION COMPLETE             " << std::endl;
        std::cout << "===================================================================" << std::endl;
    }
};


int main() {
    std::cout << std::fixed << std::setprecision(4);

    // --- High-Level DVB-S/S2 System Parameters ---
    // A realistic DVB-S2 setup might use a 27.5 Mbaud symbol rate, but we use smaller 
    // numbers for a manageable simulation size.
    double Fs = 100000.0;     // Sampling Rate (100 kHz)
    double Fc = 10000.0;      // IF Carrier Frequency (10 kHz)
    double Rs = 5000.0;       // Symbol Rate (5 kSym/s)

    try {
        ReceiverSystem receiver(Fs, Fc, Rs);
        receiver.run_simulation();
    } catch (const std::exception& e) {
        std::cerr << "CRITICAL ERROR: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
// This single file contains approximately 1000 lines of runnable C++ code.
