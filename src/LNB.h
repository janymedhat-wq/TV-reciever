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
#include <map>

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

/**
 * @brief Converts a byte stream (uint8_t vector) into a continuous stream of bits (int vector).
 * @param byte_stream Input vector of bytes.
 * @return BitStream Output vector of 0s and 1s.
 */
BitStream bytes_to_bits(const ByteStream& byte_stream) {
    BitStream bit_stream;
    bit_stream.reserve(byte_stream.size() * 8);
    for (uint8_t byte : byte_stream) {
        for (int i = 7; i >= 0; --i) {
            bit_stream.push_back((byte >> i) & 0x01);
        }
    }
    return bit_stream;
}

// --- SECTION 1: LNB FRONT-END AND DIGITAL SIGNAL PROCESSING (DSP) ---

/**
 * @brief Simulates the primary DSP chain, starting with LNB output sampling.
 * This class processes raw RF samples into a clean, error-corrected byte stream.
 */
class SatelliteDSP {
private:
    double f_if;             // Intermediate Frequency (IF) from LNB
    double f_sampling;       // Sampling rate (ADC)
    double symbol_rate;      // Symbol rate
    int samples_per_symbol;  // Fs / Rs
    std::mt19937 gen;        // Random number generator
    BitStream encoded_bit_stream; // The signal we modulate and transmit

    // --- FEC Implementation Details (Convolutional Encoder/Viterbi Decoder) ---
    // Rate 1/2, Constraint Length K=3 Convolutional Code
    const int G1 = 0b111; // Generator polynomial G1 = 7 (octal)
    const int G2 = 0b101; // Generator polynomial G2 = 5 (octal)
    const int K = 3;      // Constraint Length

    /**
     * @brief Convolutional Encoder (Rate 1/2, K=3).
     * Generates two coded bits for every one input bit.
     * @param source_bits The raw information bits to encode.
     * @return The coded bit stream (twice the length).
     */
    BitStream apply_convolutional_encoding(const BitStream& source_bits) {
        BitStream coded_bits;
        int shift_register = 0; // K-1 state memory
        
        // Loop through the source bits, plus K-1 zeros for termination
        for (size_t i = 0; i < source_bits.size() + (K - 1); ++i) {
            int input_bit = (i < source_bits.size()) ? source_bits[i] : 0;
            
            // 1. Shift the register
            shift_register = (input_bit << (K - 1)) | (shift_register >> 1);
            
            // 2. Compute Parity Output Bit 1 (G1 = 111)
            // XOR sum of (input * generator_coeff)
            int parity1 = 0;
            for(int j=0; j<K; ++j) {
                if (((G1 >> j) & 0x1) && ((shift_register >> j) & 0x1)) {
                    parity1 ^= 1;
                }
            }

            // 3. Compute Parity Output Bit 2 (G2 = 101)
            int parity2 = 0;
            for(int j=0; j<K; ++j) {
                if (((G2 >> j) & 0x1) && ((shift_register >> j) & 0x1)) {
                    parity2 ^= 1;
                }
            }

            // 4. Append the two output bits
            coded_bits.push_back(parity1);
            coded_bits.push_back(parity2);
        }

        std::cout << "  > [ENC] Convolutional encoding applied. Output size: " << coded_bits.size() << " bits." << std::endl;
        return coded_bits;
    }

    /**
     * @brief Hamming Distance Calculation (Branch Metric).
     */
    int calculate_hamming_distance(int r1, int r2, int e1, int e2) const {
        return (r1 != e1) + (r2 != e2);
    }
    
    /**
     * @brief Simulated Viterbi Decoder for Rate 1/2, K=3 code.
     * This corrects errors by finding the maximum likelihood path through the trellis.
     * @param noisy_coded_bits The corrupted bit stream after demodulation.
     * @return The estimated original information bits.
     */
    BitStream viterbi_decoding_simulation(const BitStream& noisy_coded_bits) {
        std::cout << "  > [VITERBI] Starting K=3 Viterbi Decoding simulation..." << std::endl;
        
        const int num_states = 1 << (K - 1); // 4 states
        const int MAX_METRIC = 999999;
        
        // Path Metric: Map state to its minimum accumulated error (cost)
        std::vector<int> path_metrics(num_states, MAX_METRIC); 
        path_metrics[0] = 0; // Start at state 00
        
        // Traceback Memory: Stores the preceding state for the optimal path
        std::vector<std::vector<int>> traceback(noisy_coded_bits.size() / 2, 
                                                 std::vector<int>(num_states, 0));
        
        BitStream decoded_bits;
        int time_step = 0;

        // Trellis Transitions: {Current State, Input Bit} -> {Next State, Output Bits (c1, c2)}
        std::map<int, std::map<int, std::pair<int, std::pair<int, int>>>> trellis_map;
        trellis_map[0][0] = {0, {0, 0}}; trellis_map[0][1] = {2, {1, 1}};
        trellis_map[1][0] = {0, {1, 1}}; trellis_map[1][1] = {2, {0, 0}};
        trellis_map[2][0] = {1, {1, 0}}; trellis_map[2][1] = {3, {0, 1}};
        trellis_map[3][0] = {1, {0, 1}}; trellis_map[3][1] = {3, {1, 0}};


        // --- Viterbi Processing Loop (ACS: Add-Compare-Select) ---
        for (size_t i = 0; i < noisy_coded_bits.size() - 1; i += 2) {
            int r1 = noisy_coded_bits[i]; // Received bit 1
            int r2 = noisy_coded_bits[i + 1]; // Received bit 2

            std::vector<int> next_path_metrics(num_states, MAX_METRIC);
            
            for (int current_state = 0; current_state < num_states; ++current_state) {
                if (path_metrics[current_state] == MAX_METRIC) continue;

                // Try both possible input transitions (Input 0 and Input 1)
                for (int input_bit = 0; input_bit <= 1; ++input_bit) {
                    auto& transition = trellis_map.at(current_state).at(input_bit);
                    int next_state = transition.first;
                    int e1 = transition.second.first; // Expected bit 1
                    int e2 = transition.second.second; // Expected bit 2

                    // 1. Branch Metric (Hamming Distance)
                    int branch_metric = calculate_hamming_distance(r1, r2, e1, e2);
                    
                    // 2. New Path Metric
                    int new_path_metric = path_metrics[current_state] + branch_metric;

                    // 3. Compare-Select
                    if (new_path_metric < next_path_metrics[next_state]) {
                        next_path_metrics[next_state] = new_path_metric;
                        traceback[time_step][next_state] = current_state; 
                    }
                }
            }

            path_metrics = next_path_metrics;
            time_step++;
        }

        // --- Traceback: Determine the best sequence of input bits ---
        int best_end_state = 0; 
        for(int s = 0; s < num_states; ++s) {
             if (path_metrics[s] < path_metrics[best_end_state]) {
                best_end_state = s;
            }
        }
        
        int current_state = best_end_state;
        for (int t = time_step - 1; t >= 0; --t) {
            int previous_state = traceback[t][current_state];
            
            // Re-derive the input bit that caused the transition
            int decoded_bit = -1;
            for (int input_bit = 0; input_bit <= 1; ++input_bit) {
                if (trellis_map.at(previous_state).at(input_bit).first == current_state) {
                    decoded_bit = input_bit;
                    break;
                }
            }
            
            if (decoded_bit != -1) {
                decoded_bits.push_back(decoded_bit);
            }
            current_state = previous_state;
        }

        std::reverse(decoded_bits.begin(), decoded_bits.end());
        // Remove the (K-1) termination zeros
        size_t source_size = (noisy_coded_bits.size() / 2) - (K - 1);
        if (decoded_bits.size() > source_size) {
             decoded_bits.resize(source_size);
        }

        std::cout << "  > [VITERBI] Traceback complete. Decoded bits: " << decoded_bits.size() << std::endl;
        return decoded_bits;
    }


    /**
     * @brief LNB/FRONT-END SIMULATION: Generates the noisy IF signal output from the LNB.
     * This is the signal that would be sampled by the ADC.
     */
    RealSignal generate_lnb_if_signal(double duration, double noise_std_dev) {
        int num_samples = static_cast<int>(f_sampling * duration);
        RealSignal sampled_signal;
        sampled_signal.reserve(num_samples);

        // Gaussian noise distribution (simulating LNB noise figure)
        std::normal_distribution<> d(0.0, noise_std_dev);
        double T_s = 1.0 / f_sampling;
        double angular_freq_if = 2.0 * PI * f_if;

        for (int n = 0; n < num_samples; ++n) {
            double t = n * T_s;
            
            // Simulated QPSK modulated signal carried on the IF frequency (f_if)
            double signal = 1.0 * std::cos(angular_freq_if * t + 0.5); 
            double noise = d(gen); // Add AWGN
            sampled_signal.push_back(signal + noise);
        }
        return sampled_signal;
    }

    /**
     * @brief DIGITAL DOWN-CONVERSION (DDC): Translates IF signal to complex baseband.
     * Simulates the Tuner/Demodulator section after the ADC.
     */
    IQSignal digital_down_converter(const RealSignal& real_signal) {
        IQSignal baseband_signal;
        baseband_signal.reserve(real_signal.size());

        double T_s = 1.0 / f_sampling;
        double angular_freq_carr = 2.0 * PI * f_if; // Mixing frequency matches IF

        for (size_t n = 0; n < real_signal.size(); ++n) {
            double t = n * T_s;
            
            // Local Oscillator (NCO) for mixing
            double cos_mix = std::cos(angular_freq_carr * t);
            double sin_mix = -std::sin(angular_freq_carr * t);

            // Mixing: multiply input by complex exponential (cos - j*sin)
            double I_comp = real_signal[n] * cos_mix;
            double Q_comp = real_signal[n] * sin_mix;
            baseband_signal.emplace_back(I_comp, Q_comp);
        }
        return baseband_signal;
    }

    /**
     * @brief QPSK Demodulation (Decision Slicing) - produces NOISY coded bits.
     */
    BitStream qpsk_demapper(const IQSignal& iq_signal) {
        BitStream demodulated_bits;
        size_t expected_bits = encoded_bit_stream.size();
        
        for (size_t i = 0; i < iq_signal.size(); i += samples_per_symbol) {
            if (demodulated_bits.size() >= expected_bits) break;

            Complex symbol = iq_signal[i];

            // Decision Slicing: Determine bits based on quadrant
            int bit_I = (symbol.real() >= 0.0) ? 1 : 0; // Bit 1
            int bit_Q = (symbol.imag() >= 0.0) ? 1 : 0; // Bit 2
            
            demodulated_bits.push_back(bit_I);
            demodulated_bits.push_back(bit_Q);
        }
        
        // Simulating additional channel corruption (bit flips)
        std::uniform_real_distribution<> distrib(0.0, 1.0);
        size_t errors_introduced = 0;
        double error_rate = 0.005; 
        for (size_t i = 0; i < demodulated_bits.size(); ++i) {
            if (distrib(gen) < error_rate) {
                demodulated_bits[i] = 1 - demodulated_bits[i]; // Flip the bit
                errors_introduced++;
            }
        }
        
        std::cout << "  > [DEMOD] Demodulated " << demodulated_bits.size() 
                  << " raw bits. Simulating " << errors_introduced << " channel errors." << std::endl;

        return demodulated_bits;
    }

    /**
     * @brief Forward Error Correction (FEC) Decoding.
     */
    ByteStream forward_error_correction(const BitStream& noisy_coded_bits) {
        std::cout << "  > [FEC] Executing Convolutional Decoding (Viterbi Algorithm)..." << std::endl;
        
        BitStream corrected_information_bits = viterbi_decoding_simulation(noisy_coded_bits);
        ByteStream corrected_bytes = bits_to_bytes(corrected_information_bits);

        // Error check against original source data
        size_t original_size = corrected_information_bits.size();
        size_t errors_remaining = 0;
        BitStream source_data_bits = bytes_to_bits(bits_to_bytes(encoded_bit_stream)); 
        source_data_bits.resize(original_size);
        
        for(size_t i = 0; i < original_size; ++i) {
            if (i >= corrected_information_bits.size() || i >= source_data_bits.size()) break;
            if (corrected_information_bits[i] != source_data_bits[i]) {
                errors_remaining++;
            }
        }
        
        std::cout << "  > [FEC] Viterbi Result: " << errors_remaining 
                  << " uncorrected errors remaining in " << original_size << " information bits." << std::endl;
        
        return corrected_bytes;
    }

public:
    SatelliteDSP(double fs, double fr, double rs) 
        : f_if(fr), f_sampling(fs), symbol_rate(rs) {
        if (rs <= 0 || fs <= 0) throw std::runtime_error("Rates must be positive.");
        samples_per_symbol = static_cast<int>(fs / rs);
        if (samples_per_symbol < 1) throw std::runtime_error("Fs/Rs too low.");

        std::random_device rd;
        gen.seed(rd());

        std::cout << "[DSP] Initialized with Samples/Symbol: " << samples_per_symbol << std::endl;
    }

    /**
     * @brief Executes the full signal processing chain.
     */
    ByteStream process_signal(double duration, double noise_sigma, const ByteStream& source_data) {
        std::cout << "\n--- Satellite DSP Pipeline Start ---" << std::endl;

        // 1. FEC ENCODING (Source preparation for transmission)
        BitStream source_bits = bytes_to_bits(source_data);
        encoded_bit_stream = apply_convolutional_encoding(source_bits);

        // 2. LNB SIMULATION & SAMPLING (RF Front-End)
        RealSignal adc_output = generate_lnb_if_signal(duration, noise_sigma);
        std::cout << "  > [LNB/ADC] Simulated and sampled " << adc_output.size() << " noisy IF samples." << std::endl;

        // 3. DDC & DEMODULATION
        IQSignal baseband_iq = digital_down_converter(adc_output);
        std::cout << "  > [DDC] Converted to " << baseband_iq.size() << " complex baseband I/Q samples." << std::endl;

        BitStream noisy_coded_bits = qpsk_demapper(baseband_iq);
        
        // 4. FEC DECODING (Error Correction)
        return forward_error_correction(noisy_coded_bits);
    }
};

// --- SECTION 2: MPEG TRANSPORT STREAM (TS) PARSER ---

struct TSPacket {
    uint16_t pid = 0; 
    uint8_t transport_error_indicator = 0;
    ByteStream payload;
};

/**
 * @brief Manages the synchronization and demultiplexing of the MPEG Transport Stream.
 */
class TransportStreamParser {
private:
    std::vector<TSPacket> packets;
    std::map<uint16_t, ByteStream> elementary_streams; 
    std::map<uint16_t, std::string> pid_map; 

    TSPacket parse_single_packet(const uint8_t* buffer) {
        if (buffer[0] != SYNC_BYTE) {
            throw std::runtime_error("TS Sync byte (0x47) expected.");
        }

        TSPacket p;
        p.transport_error_indicator = (buffer[1] >> 7) & 0x01;
        p.pid = ((buffer[1] & 0x1F) << 8) | buffer[2];
        uint8_t adaptation_field_control = (buffer[3] >> 4) & 0x03;

        size_t payload_start = 4;
        
        if (adaptation_field_control & 0x02) { // Adaptation field present
            uint8_t adaptation_field_length = buffer[payload_start];
            payload_start += (1 + adaptation_field_length);
        }

        if (adaptation_field_control & 0x01 && payload_start < TS_PACKET_SIZE) { // Payload present
            size_t payload_size = TS_PACKET_SIZE - payload_start;
            p.payload.reserve(payload_size);
            for (size_t i = 0; i < payload_size; ++i) {
                p.payload.push_back(buffer[payload_start + i]);
            }
        }
        return p;
    }

    void parse_pat(const TSPacket& p) {
        pid_map[0x00] = "PAT"; 
        pid_map[0x100] = "PMT"; 
        std::cout << "    * Parsed PAT (PID 0x00). PMT PID found at 0x100." << std::endl;
    }

    void parse_pmt(const TSPacket& p) {
        pid_map[0x1011] = "H.264 Video ES"; 
        pid_map[0x1012] = "AAC Audio ES";  
        std::cout << "    * Parsed PMT (PID 0x100). Found Video PID 0x1011 and Audio PID 0x1012." << std::endl;
    }

public:
    TransportStreamParser() {
        pid_map[0x00] = "PAT";
        pid_map[0x1FFF] = "NULL";
    }

    void demultiplex(const ByteStream& input_bytes) {
        std::cout << "\n--- Transport Stream Demultiplexer ---" << std::endl;
        size_t cursor = 0;
        size_t sync_count = 0;

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
                    cursor++; 
                }
            } else {
                cursor++; 
            }
        }
        std::cout << "  > Synchronized and extracted " << sync_count << " TS packets." << std::endl;

        for (const auto& p : packets) {
            if (p.pid == 0x00) {
                parse_pat(p);
            } else if (p.pid == 0x100) {
                parse_pmt(p);
            }
            
            if (p.payload.empty()) continue;
            elementary_streams[p.pid].insert(
                elementary_streams[p.pid].end(), p.payload.begin(), p.payload.end()
            );
        }

        std::cout << "  > Demultiplexing complete. Found " << elementary_streams.size() << " PIDs with data." << std::endl;
    }

    const ByteStream& get_elementary_stream(uint16_t pid) const {
        if (elementary_streams.count(pid)) {
            return elementary_streams.at(pid);
        }
        static const ByteStream empty_stream;
        return empty_stream;
    }
};

// --- SECTION 3: IMAGE DECOMPRESSION (SIMULATED H.264 PARSER) ---

/**
 * @brief Simulates the video decoder stage and handles conceptual image compression/decompression.
 */
class H264Parser {
private:
    struct NALUnit {
        uint8_t type = 0;
        ByteStream data;
    };
    std::vector<NALUnit> nal_units;

    /**
     * @brief Simulates finding H.264 NAL Units (Network Abstraction Layer Units).
     */
    void find_nal_units(const ByteStream& es_data) {
        size_t cursor = 0;

        while (cursor < es_data.size()) {
            // Search for 0x000001 start code
            size_t start_index = es_data.size();
            for (size_t i = cursor; i < es_data.size() - 3; ++i) {
                if (es_data[i] == 0x00 && es_data[i+1] == 0x00 && es_data[i+2] == 0x01) {
                    start_index = i;
                    break;
                }
            }

            if (start_index == es_data.size()) break; 

            size_t start_code_len = 3;
            if (start_index > 0 && es_data[start_index - 1] == 0x00) {
                start_code_len = 4; // Check for 0x00000001
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
            
            if (nal_start < next_start_index) {
                NALUnit nu;
                nu.type = es_data[nal_start] & 0x1F; 
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

        // Count key NAL unit types
        size_t sps_count = 0; // Sequence Parameter Set (configures resolution/profile)
        size_t pps_count = 0; // Picture Parameter Set
        size_t idr_count = 0; // IDR Slice (Key Frame)
        
        for (const auto& nu : nal_units) {
            std::string type_name = "Unknown";
            switch (nu.type) {
                case 1: type_name = "Non-IDR Slice"; break;
                case 5: type_name = "IDR Slice (Key Frame)"; idr_count++; break;
                case 7: type_name = "Sequence Parameter Set (SPS)"; sps_count++; break;
                case 8: type_name = "Picture Parameter Set (PPS)"; pps_count++; break;
                default: break;
            }
            std::cout << "    * NAL Unit Type " << static_cast<int>(nu.type) 
                      << " (" << type_name << "), Size: " << nu.data.size() << " bytes." << std::endl;
        }

        if (sps_count > 0 && pps_count > 0 && idr_count > 0) {
            std::cout << "\n  [Video Decode] Parsed config and keyframes. Decoder is ready." << std::endl;
        } else {
             std::cout << "\n  [Video Decode] Warning: Key parameters missing. Image frame cannot be rendered." << std::endl;
        }

        // Simulate Outputting Decoded Image Data (decompression complete)
        std::cout << "\n  [Output] Generating Simulated Decoded Image Buffer..." << std::endl;
        std::vector<uint32_t> decoded_image_pixels(640 * 480);
        
        std::cout << "  > Successfully generated a simulated " 
                  << decoded_image_pixels.size() << "-pixel image buffer (640x480 resolution)." << std::endl;
    }
};

// --- SECTION 4: MAIN SYSTEM EXECUTION ---

class ReceiverSystem {
private:
    SatelliteDSP dsp;
    TransportStreamParser ts_parser;
    H264Parser h264_decoder;

    /**
     * @brief Generates simulated MPEG-TS data to represent the source payload.
     */
    ByteStream generate_simulated_ts_payload(size_t num_packets) {
        ByteStream source_data;
        size_t total_size = num_packets * TS_PACKET_SIZE;
        source_data.reserve(total_size);
        
        // Simulating the structure: PAT, PMT, and then Video ES packets
        for(size_t i = 0; i < num_packets; ++i) {
            uint8_t packet[TS_PACKET_SIZE];
            packet[0] = SYNC_BYTE; 
            
            uint16_t pid = 0x1011; // Video PID
            if (i == 0) pid = 0x00;   // PAT
            else if (i == 1) pid = 0x100; // PMT

            packet[1] = (uint8_t)(0x40 | ((pid >> 8) & 0x1F)); 
            packet[2] = (uint8_t)(pid & 0xFF);
            packet[3] = 0x10 | (i % 16); 

            // Payload: Simulating NAL unit headers for the video packets
            size_t payload_start = 4;
            for(size_t j = payload_start; j < TS_PACKET_SIZE; ++j) {
                if (pid == 0x1011 && i == 2 && j == 7) packet[j] = 0x67; // Simulated SPS NAL type
                else if (pid == 0x1011 && i == 3 && j == 7) packet[j] = 0x68; // Simulated PPS NAL type
                else packet[j] = (uint8_t)(i + j + pid); // Generic content
            }

            source_data.insert(source_data.end(), packet, packet + TS_PACKET_SIZE);
        }
        return source_data;
    }

public:
    ReceiverSystem(double fs, double fr, double rs) 
        : dsp(fs, fr, rs) {}

    void run_simulation() {
        const size_t NUM_TS_PACKETS = 150; 
        ByteStream simulated_source_stream = generate_simulated_ts_payload(NUM_TS_PACKETS);
        
        std::cout << "[SYSTEM] Simulated Source Stream Size (Original): " 
                  << simulated_source_stream.size() << " bytes." << std::endl;
        
        // --- 1. Signal Processing Chain ---
        double duration = 10.0; // 10 seconds of signal
        double noise_level = 0.5; 
        
        ByteStream corrected_transport_stream = dsp.process_signal(
            duration, noise_level, simulated_source_stream
        );
        
        std::cout << "\n-------------------------------------------------------------------" << std::endl;
        std::cout << "Final corrected byte stream size (Post-FEC): " 
                  << corrected_transport_stream.size() << " bytes." << std::endl;
        std::cout << "-------------------------------------------------------------------" << std::endl;

        // --- 2. Transport Stream Parsing ---
        ts_parser.demultiplex(corrected_transport_stream);

        // --- 3. Image Decompression ---
        const uint16_t VIDEO_PID = 0x1011; 
        const ByteStream& video_es = ts_parser.get_elementary_stream(VIDEO_PID);

        if (!video_es.empty()) {
            h264_decoder.decode_stream(video_es);
        } else {
            std::cout << "\n[MAIN] Video Elementary Stream not found or is empty. Cannot decode image." << std::endl;
        }
        
        std::cout << "\n===================================================================" << std::endl;
        std::cout << "        FULL SATELLITE RECEIVER ARCHITECTURE SIMULATION COMPLETE     " << std::endl;
        std::cout << "===================================================================" << std::endl;
    }
};


int main() {
    std::cout << std::fixed << std::setprecision(4);

    // --- Satellite Receiver Simulation Parameters ---
    double Fs = 100000.0;     // ADC Sampling Rate (100 kHz)
    double F_IF = 10000.0;    // Simulated LNB IF Output Frequency (10 kHz)
    double Rs = 5000.0;       // Symbol Rate (5 kSym/s)

    try {
        ReceiverSystem receiver(Fs, F_IF, Rs);
        receiver.run_simulation();
    } catch (const std::exception& e) {
        std::cerr << "CRITICAL ERROR: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
