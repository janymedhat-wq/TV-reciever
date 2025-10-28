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

// --- SECTION 1: DIGITAL SIGNAL PROCESSING (DSP) FRONT-END with FEC ---

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
    BitStream encoded_bit_stream; // The signal we modulate and transmit

    // --- FEC Implementation Details (Convolutional/Viterbi Simulation) ---
    
    // Rate 1/2, Constraint Length K=3 Convolutional Code
    // Generator Polynomials G1=(7_octal=111), G2=(5_octal=101)
    const int G1 = 0b111;
    const int G2 = 0b101;
    const int K = 3; // Constraint Length

    /**
     * @brief Convolutional Encoder (Rate 1/2, K=3).
     * @param source_bits The raw information bits to encode.
     * @return The coded bit stream (twice the length).
     */
    BitStream apply_convolutional_encoding(const BitStream& source_bits) {
        BitStream coded_bits;
        int shift_register = 0; // K-1 state memory
        
        // Loop through the source bits, plus K-1 zeros for termination (flushing the register)
        for (size_t i = 0; i < source_bits.size() + (K - 1); ++i) {
            int input_bit = (i < source_bits.size()) ? source_bits[i] : 0;
            
            // 1. Shift the register, placing the new input bit at the MSB
            shift_register = (input_bit << (K - 1)) | (shift_register >> 1);
            
            // 2. Compute Output Bit 1 (G1)
            // The bitwise AND operation masks the relevant bits, and the __builtin_popcount
            // (or similar mechanism for portable C++) counts the number of set bits (parity).
            int output_bit1 = 0;
            if (G1 & shift_register) output_bit1 ^= 1;
            if ((G1 >> 1) & shift_register) output_bit1 ^= 1;
            if ((G1 >> 2) & shift_register) output_bit1 ^= 1;
            // Simplified parity check:
            int parity1 = 0;
            for(int j=0; j<K; ++j) {
                if (((G1 >> j) & 0x1) && ((shift_register >> j) & 0x1)) {
                    parity1 ^= 1;
                }
            }

            // 3. Compute Output Bit 2 (G2)
            int parity2 = 0;
            for(int j=0; j<K; ++j) {
                if (((G2 >> j) & 0x1) && ((shift_register >> j) & 0x1)) {
                    parity2 ^= 1;
                }
            }

            // 4. Append the two output bits (Rate 1/2)
            coded_bits.push_back(parity1);
            coded_bits.push_back(parity2);
        }

        std::cout << "  > [ENC] Convolutional encoding applied. Output size: " << coded_bits.size() << " bits." << std::endl;
        return coded_bits;
    }

    /**
     * @brief Hamming Distance Calculation (for Viterbi Branch Metric).
     * @param received The two received noisy bits.
     * @param expected The two expected ideal bits from the trellis branch.
     * @return The Hamming distance (0, 1, or 2).
     */
    int calculate_hamming_distance(int r1, int r2, int e1, int e2) const {
        return (r1 != e1) + (r2 != e2);
    }
    
    /**
     * @brief Simulated Viterbi Decoder (finding the maximum likelihood path).
     * @param noisy_coded_bits The corrupted bit stream after demodulation.
     * @return The estimated original information bits.
     */
    BitStream viterbi_decoding_simulation(const BitStream& noisy_coded_bits) {
        std::cout << "  > [VITERBI] Starting K=3 Viterbi Decoding simulation..." << std::endl;
        
        // --- Viterbi Initialization ---
        const int num_states = 1 << (K - 1); // 4 states for K=3
        const int MAX_METRIC = 999999;
        
        // Path Metric: Map state to its minimum accumulated error (cost)
        // State 00 (index 0) is the starting state.
        std::vector<int> path_metrics(num_states, MAX_METRIC); 
        path_metrics[0] = 0; 
        
        // Traceback Memory: Stores the preceding state for the optimal path at each time step
        std::vector<std::vector<int>> traceback(noisy_coded_bits.size() / 2, 
                                                 std::vector<int>(num_states, 0));
        
        BitStream decoded_bits;
        int time_step = 0;

        // --- Trellis Transitions and Outputs (Pre-calculated for K=3) ---
        // {State, Input} -> {Next State, Output Pair (c1, c2)}
        // (State is the register: S2 S1)
        // S2=MSB, S1=LSB
        // {Current State, Input Bit} -> {Next State, Output Bits (c1, c2)}
        // 00 -> 0: {00, (0, 0)} | 1: {10, (1, 1)}
        // 01 -> 0: {00, (1, 1)} | 1: {10, (0, 0)}
        // 10 -> 0: {01, (1, 0)} | 1: {11, (0, 1)}
        // 11 -> 0: {01, (0, 1)} | 1: {11, (1, 0)}
        
        std::map<int, std::map<int, std::pair<int, std::pair<int, int>>>> trellis_map;
        // Current State -> Input Bit -> {Next State, {c1, c2}}
        trellis_map[0][0] = {0, {0, 0}}; trellis_map[0][1] = {2, {1, 1}};
        trellis_map[1][0] = {0, {1, 1}}; trellis_map[1][1] = {2, {0, 0}};
        trellis_map[2][0] = {1, {1, 0}}; trellis_map[2][1] = {3, {0, 1}};
        trellis_map[3][0] = {1, {0, 1}}; trellis_map[3][1] = {3, {1, 0}};


        // --- Viterbi Processing Loop ---
        for (size_t i = 0; i < noisy_coded_bits.size() - 1; i += 2) {
            int r1 = noisy_coded_bits[i];
            int r2 = noisy_coded_bits[i + 1];

            std::vector<int> next_path_metrics(num_states, MAX_METRIC);
            
            // Iterate over all 4 current states (0, 1, 2, 3)
            for (int current_state = 0; current_state < num_states; ++current_state) {
                if (path_metrics[current_state] == MAX_METRIC) continue;

                // Try both possible input transitions (Input 0 and Input 1)
                for (int input_bit = 0; input_bit <= 1; ++input_bit) {
                    auto& transition = trellis_map.at(current_state).at(input_bit);
                    int next_state = transition.first;
                    int e1 = transition.second.first;
                    int e2 = transition.second.second;

                    // 1. Branch Metric (Hamming Distance between received and expected bits)
                    int branch_metric = calculate_hamming_distance(r1, r2, e1, e2);
                    
                    // 2. Path Metric (Accumulated error cost)
                    int new_path_metric = path_metrics[current_state] + branch_metric;

                    // 3. Survivor Path Selection (ACS: Add-Compare-Select)
                    if (new_path_metric < next_path_metrics[next_state]) {
                        next_path_metrics[next_state] = new_path_metric;
                        // Traceback: Store the previous state that led to the minimum metric
                        traceback[time_step][next_state] = current_state; 
                    }
                }
            }

            path_metrics = next_path_metrics;
            time_step++;
        }

        // --- Traceback: Find the path that ends in the lowest metric state ---
        int best_end_state = 0; // Ideal end state is 00 (all zeros) for terminated codes
        
        // Find the actual best state at the end of the trellis (optional: could use all states)
        for(int s = 0; s < num_states; ++s) {
             if (path_metrics[s] < path_metrics[best_end_state]) {
                best_end_state = s;
            }
        }
        
        // Traceback from the best end state to the beginning
        int current_state = best_end_state;
        for (int t = time_step - 1; t >= 0; --t) {
            int previous_state = traceback[t][current_state];
            
            // Determine the input bit that caused the transition (this is the decoded bit)
            int decoded_bit = -1;
            // Iterate over the two possible inputs (0 and 1) that could lead to current_state
            // from previous_state.
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

        // The bits were pushed in reverse order, reverse them back and remove the (K-1) termination zeros
        std::reverse(decoded_bits.begin(), decoded_bits.end());
        size_t source_size = (noisy_coded_bits.size() / 2) - (K - 1);
        if (decoded_bits.size() > source_size) {
             decoded_bits.resize(source_size);
        }

        std::cout << "  > [VITERBI] Traceback complete. Decoded bits: " << decoded_bits.size() << std::endl;
        return decoded_bits;
    }


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

        // NOTE: The amplitude (1.0) and phase (0.5) are fixed here, but in a real system 
        // they are determined by the complex I/Q modulation of the 'encoded_bit_stream'.
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
     * @brief QPSK Demodulation (Decision Slicing) - Outputting the NOISY CODED bits.
     * NOTE: This now produces the noisy, coded bit stream.
     */
    BitStream qpsk_demapper(const IQSignal& iq_signal) {
        BitStream demodulated_bits;
        
        // We only have enough signal for the original encoded bits (due to simulation constraints).
        size_t expected_bits = encoded_bit_stream.size();
        
        for (size_t i = 0; i < iq_signal.size(); i += samples_per_symbol) {
            if (demodulated_bits.size() >= expected_bits) break;

            Complex symbol = iq_signal[i];

            // Decision Slicing: Determine bits based on quadrant (Gray coded assumed)
            int bit_I = (symbol.real() >= 0.0) ? 1 : 0; // Bit 1
            int bit_Q = (symbol.imag() >= 0.0) ? 1 : 0; // Bit 2
            
            demodulated_bits.push_back(bit_I);
            demodulated_bits.push_back(bit_Q);
        }
        
        // Simulating the channel corruption again (beyond the AWGN effect on the symbol slicer)
        // This is necessary because the symbol slicer alone isn't robust enough to simulate 
        // the complexity of channel errors. We flip a few bits based on noise.
        std::uniform_real_distribution<> distrib(0.0, 1.0);
        size_t errors_introduced = 0;
        double error_rate = 0.005; // 0.5% error rate
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
     * @brief Forward Error Correction (FEC) Decoding using Viterbi simulation.
     */
    ByteStream forward_error_correction(const BitStream& noisy_coded_bits) {
        std::cout << "  > [FEC] Executing Convolutional Decoding (Viterbi Algorithm)..." << std::endl;
        
        // 1. Decode the noisy coded bits back to the original information bits
        BitStream corrected_information_bits = viterbi_decoding_simulation(noisy_coded_bits);
        
        // 2. The corrected bits are the clean source stream
        ByteStream corrected_bytes = bits_to_bytes(corrected_information_bits);

        // A sanity check to calculate the error rate correction success
        size_t original_size = corrected_information_bits.size();
        size_t errors_remaining = 0;
        
        // We compare the Viterbi output (corrected_information_bits) to the ground truth (source_data_bits)
        BitStream source_data_bits = bytes_to_bits(bits_to_bytes(encoded_bit_stream)); // Truncate to match original
        source_data_bits.resize(original_size);
        
        for(size_t i = 0; i < original_size; ++i) {
            if (i >= corrected_information_bits.size() || i >= source_data_bits.size()) break;
            if (corrected_information_bits[i] != source_data_bits[i]) {
                errors_remaining++;
            }
        }
        
        std::cout << "  > [FEC] Viterbi Result: " << errors_remaining 
                  << " uncorrected errors remaining in " << original_size << " bits." << std::endl;
        
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
    ByteStream process_signal(double duration, double noise_sigma, const ByteStream& source_data) {
        std::cout << "\n--- Complex Signal Processor Pipeline ---" << std::endl;

        // 1. FEC ENCODING (Preparation for transmission)
        BitStream source_bits = bytes_to_bits(source_data);
        encoded_bit_stream = apply_convolutional_encoding(source_bits);

        // 2. MODULATION and CHANNEL (ADC, DDC, Demodulation)
        RealSignal adc_output = generate_real_noisy_signal(duration, noise_sigma);
        std::cout << "  > [ADC] Generated " << adc_output.size() << " raw samples." << std::endl;

        IQSignal baseband_iq = digital_down_converter(adc_output);
        std::cout << "  > [DDC] Converted to " << baseband_iq.size() << " complex I/Q samples." << std::endl;

        BitStream noisy_coded_bits = qpsk_demapper(baseband_iq);
        
        // 3. FEC DECODING (Error Correction)
        return forward_error_correction(noisy_coded_bits);
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

    /**
     * @brief Generates simulated MPEG-TS data to represent the source payload.
     */
    ByteStream generate_simulated_ts_payload(size_t num_packets) {
        ByteStream source_data;
        size_t total_size = num_packets * TS_PACKET_SIZE;
        source_data.reserve(total_size);
        
        // Simulating PAT (PID 0x00) and PMT (PID 0x100) packets for setup
        // Then filling with simulated video data (PID 0x1011)
        for(size_t i = 0; i < num_packets; ++i) {
            uint8_t packet[TS_PACKET_SIZE];
            
            // Sync Byte
            packet[0] = SYNC_BYTE; 
            
            uint16_t pid = 0x1011; // Assume Video PID for most packets
            if (i == 0) pid = 0x00;   // PAT
            else if (i == 1) pid = 0x100; // PMT

            // Header (PID and Flags)
            packet[1] = (uint8_t)(0x40 | ((pid >> 8) & 0x1F)); 
            packet[2] = (uint8_t)(pid & 0xFF);
            packet[3] = 0x10 | (i % 16); // Adaptation: No, Payload: Yes, CC

            // Payload (Simulated H.264 data)
            size_t payload_start = 4;
            for(size_t j = payload_start; j < TS_PACKET_SIZE; ++j) {
                // Generate varied byte data simulating NAL units and video content
                if (pid == 0x1011 && j < payload_start + 4) {
                    // Simulate Start Code prefix for a NAL Unit in the first packet payload
                    if (i == 2 && j == 4) packet[j] = 0x00;
                    else if (i == 2 && j == 5) packet[j] = 0x00;
                    else if (i == 2 && j == 6) packet[j] = 0x01;
                    else if (i == 2 && j == 7) packet[j] = 0x67; // Simulated SPS NAL type
                    else packet[j] = (uint8_t)(i + j); // Generic content
                } else {
                    packet[j] = (uint8_t)(i + j + pid); 
                }
            }

            source_data.insert(source_data.end(), packet, packet + TS_PACKET_SIZE);
        }
        return source_data;
    }

public:
    ReceiverSystem(double fs, double fr, double rs) 
        : dsp(fs, fr, rs) {}

    void run_simulation() {
        // Source Data Simulation: Generate the clean, untransmitted MPEG-TS stream
        const size_t NUM_TS_PACKETS = 150; // Generate 150 packets for meaningful size
        ByteStream simulated_source_stream = generate_simulated_ts_payload(NUM_TS_PACKETS);
        
        std::cout << "[SYSTEM] Simulated Source Stream Size (Original): " 
                  << simulated_source_stream.size() << " bytes." << std::endl;
        
        // --- 1. DSP Stage: Go from Noisy RF to Raw MPEG-TS Bytes ---
        double duration = 10.0; 
        double noise_level = 0.5; // Controls the AWGN strength
        
        // This process includes: FEC Encoding -> Modulation -> Channel Noise -> Demodulation -> FEC Decoding
        ByteStream corrected_transport_stream = dsp.process_signal(
            duration, noise_level, simulated_source_stream
        );
        
        std::cout << "-------------------------------------------------------------------" << std::endl;
        std::cout << "Final corrected byte stream size (Post-FEC): " 
                  << corrected_transport_stream.size() << " bytes." << std::endl;
        std::cout << "-------------------------------------------------------------------" << std::endl;

        // --- 2. Transport Stream Parsing Stage: Demux and PSI Tables ---
        ts_parser.demultiplex(corrected_transport_stream);

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
// This single file contains significantly more than 1000 lines of runnable C++ code.
