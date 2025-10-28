#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <numeric>
#include <iomanip>
#include <complex> // Crucial for DSP, representing In-phase (I) and Quadrature (Q) components
#include <algorithm>
#include <stdexcept>

// Define constants
const double PI = 3.14159265358979323846;

// Type alias for convenience in DSP operations
using Complex = std::complex<double>;
using RealSignal = std::vector<double>;
using IQSignal = std::vector<Complex>;
using BitStream = std::vector<int>;

// --- SECTION 1: SIGNAL GENERATION AND NOISE MODELING (ADC Simulation) ---

/**
 * @brief Generates a discrete-time, real-valued signal simulating the output of an ADC
 * after the LNB IF stage. Includes the desired signal and additive Gaussian noise.
 *
 * @param signal_freq_hz The frequency of the desired IF signal (Hz).
 * @param sampling_rate_hz The rate at which the signal is sampled (Fs).
 * @param duration_seconds The total time length of the signal to generate.
 * @param noise_std_dev The standard deviation of the additive Gaussian noise (thermal noise).
 * @return RealSignal A vector containing the real-valued, noisy samples.
 */
RealSignal generate_real_noisy_signal(
    double signal_freq_hz,
    double sampling_rate_hz,
    double duration_seconds,
    double noise_std_dev)
{

    int num_samples = static_cast<int>(sampling_rate_hz * duration_seconds);
    RealSignal sampled_signal;
    sampled_signal.reserve(num_samples);

    // Setup Gaussian Noise Generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(0.0, noise_std_dev);

    double T_s = 1.0 / sampling_rate_hz;
    double angular_freq = 2.0 * PI * signal_freq_hz;

    std::cout << "\n[1. Signal Generation] Simulating ADC output (" << num_samples << " samples)..." << std::endl;

    for (int n = 0; n < num_samples; ++n)
    {
        double t = n * T_s;

        // Desired IF Signal (Amplitude = 1.0)
        double signal = 1.0 * std::cos(angular_freq * t + 0.5); // Add a small phase offset for realism

        // Additive Gaussian Noise
        double noise = d(gen);

        // ADC Output
        sampled_signal.push_back(signal + noise);
    }

    return sampled_signal;
}

// --- SECTION 2: DIGITAL DOWN-CONVERSION (DDC) AND BASEBAND FILTERING ---

/**
 * @brief Performs Digital Down-Conversion (DDC) on the real IF signal.
 * This mixes the IF signal down to complex baseband (I/Q).
 *
 * @param real_signal The input real-valued IF signal.
 * @param center_freq_hz The frequency (Fc) to mix the signal with (must match signal_freq_hz).
 * @param sampling_rate_hz The system sampling rate (Fs).
 * @return IQSignal The resulting complex baseband I/Q signal.
 */
IQSignal digital_down_converter(
    const RealSignal &real_signal,
    double center_freq_hz,
    double sampling_rate_hz)
{

    IQSignal baseband_signal;
    baseband_signal.reserve(real_signal.size());

    double T_s = 1.0 / sampling_rate_hz;
    double angular_freq_carr = 2.0 * PI * center_freq_hz;

    std::cout << "[2. Digital Down-Conversion] Mixing signal to baseband (Fc=" << center_freq_hz << " Hz)..." << std::endl;

    for (size_t n = 0; n < real_signal.size(); ++n)
    {
        double t = n * T_s;

        // Numerically Controlled Oscillator (NCO) for Mixing:
        // Local oscillator produces sine and cosine waves at the carrier frequency.
        double cos_mix = std::cos(angular_freq_carr * t);  // In-phase component mixer
        double sin_mix = -std::sin(angular_freq_carr * t); // Quadrature component mixer (90 deg shift)

        // Mixing (Multiplication)
        double I_comp = real_signal[n] * cos_mix;
        double Q_comp = real_signal[n] * sin_mix;

        // After mixing, the desired signal component is centered at 0 Hz (DC),
        // and the unwanted image component is at 2*Fc. A perfect DDC would
        // apply a low-pass filter here (decimation/matched filter).
        // For simplicity, we skip explicit filtering, assuming it is perfect.

        baseband_signal.emplace_back(I_comp, Q_comp);
    }

    return baseband_signal;
}

// --- SECTION 3: SYMBOL SYNCHRONIZATION AND QPSK DEMODULATION ---

/**
 * @brief Performs a simplified QPSK Demodulation (symbol decision) on the I/Q samples.
 * NOTE: This skips crucial Symbol Synchronization (timing recovery) and Matched Filtering.
 * We simply sample at the symbol boundaries (assuming ideal synchronization).
 *
 * @param iq_signal The input complex baseband signal.
 * @param samples_per_symbol The number of digital samples representing one modulated symbol.
 * @return BitStream The demodulated binary data stream (0s and 1s).
 */
BitStream qpsk_demapper(
    const IQSignal &iq_signal,
    int samples_per_symbol)
{

    if (samples_per_symbol <= 0)
    {
        throw std::runtime_error("Samples per symbol must be greater than zero.");
    }

    BitStream demodulated_bits;
    int symbols_processed = 0;

    std::cout << "[3. QPSK Demodulation] Demapping I/Q symbols ("
              << samples_per_symbol << " samples/symbol)..." << std::endl;

    // Simulate symbol-timing recovery by picking one sample per symbol interval
    for (size_t i = 0; i < iq_signal.size(); i += samples_per_symbol)
    {
        // This 'picked' sample is the ideal symbol location in time.
        Complex symbol = iq_signal[i];

        // --- Decision Region Slicing (Demapping) ---
        // For standard QPSK, the symbol decision is based on the sign of I and Q:
        //   Q > 0 (Top half) -> Bit 1 = 1, Q < 0 (Bottom half) -> Bit 1 = 0
        //   I > 0 (Right half) -> Bit 2 = 1, I < 0 (Left half) -> Bit 2 = 0

        int bit_I = (symbol.real() >= 0.0) ? 1 : 0; // Decide on In-phase bit
        int bit_Q = (symbol.imag() >= 0.0) ? 1 : 0; // Decide on Quadrature bit

        // QPSK transmits 2 bits per symbol
        demodulated_bits.push_back(bit_I);
        demodulated_bits.push_back(bit_Q);

        symbols_processed++;
    }

    std::cout << "Demodulated " << symbols_processed << " symbols, resulting in "
              << demodulated_bits.size() << " bits." << std::endl;

    return demodulated_bits;
}

// --- SECTION 4: MAIN SIMULATION EXECUTION ---

int main()
{
    // --- LNB Receiver Simulation Configuration ---
    // High-level parameters for the system:
    double f_signal = 10000.0;    // Target IF signal frequency (10 kHz)
    double f_sampling = 100000.0; // Sampling Rate (Fs = 100 kHz)
    double symbol_rate = 5000.0;  // Digital Symbol Rate (Rs = 5 kSym/s)
    double duration = 0.01;       // Generate 10 milliseconds of data
    double noise_sigma = 0.7;     // Noise level (high noise for a tough test)

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "===================================================================" << std::endl;
    std::cout << "       DVB-S Receiver DSP Front-End Simulation (QPSK)            " << std::endl;
    std::cout << "===================================================================" << std::endl;
    std::cout << "System Parameters:" << std::endl;
    std::cout << "  - Signal Carrier Frequency (Fc): " << f_signal << " Hz" << std::endl;
    std::cout << "  - Sampling Rate (Fs):          " << f_sampling << " Hz" << std::endl;
    std::cout << "  - Symbol Rate (Rs):            " << symbol_rate << " Sym/s" << std::endl;
    std::cout << "  - Bits per Symbol (QPSK):      2 bits" << std::endl;
    std::cout << "  - Expected Bit Rate:           " << symbol_rate * 2.0 << " bits/s" << std::endl;
    std::cout << "-------------------------------------------------------------------" << std::endl;

    // Calculation of a critical parameter for Demodulation
    int samples_per_symbol = static_cast<int>(f_sampling / symbol_rate);
    std::cout << "Calculated Samples per Symbol (Fs/Rs): " << samples_per_symbol << std::endl;
    if (samples_per_symbol < 1)
    {
        std::cerr << "ERROR: Symbol rate is too high for the sampling rate!" << std::endl;
        return 1;
    }
    std::cout << "-------------------------------------------------------------------" << std::endl;

    try
    {
        // --- STAGE 1: ADC Output Simulation (Real Signal) ---
        RealSignal adc_output = generate_real_noisy_signal(
            f_signal, f_sampling, duration, noise_sigma);

        // --- STAGE 2: Digital Down-Conversion (Real to Complex I/Q) ---
        // Note: For simplicity, we use the ideal center frequency (f_signal).
        // Real receivers must implement Carrier Frequency Offset (CFO) correction loops.
        IQSignal baseband_iq = digital_down_converter(
            adc_output, f_signal, f_sampling);

        // --- STAGE 3: QPSK Demodulation (Complex I/Q to Bit Stream) ---
        // Note: We skip the essential Matched Filtering/Timing Recovery blocks.
        BitStream demodulated_data = qpsk_demapper(
            baseband_iq, samples_per_symbol);

        // --- STAGE 4: Output Analysis (Verification) ---
        std::cout << "\n===================================================================" << std::endl;
        std::cout << "                           RECEIVER OUTPUT                         " << std::endl;
        std::cout << "===================================================================" << std::endl;

        // Displaying a small segment of the I/Q Baseband signal
        std::cout << "\n--- Baseband I/Q Samples (First 5 Symbols) ---" << std::endl;
        int max_iq_display = std::min((int)baseband_iq.size(), samples_per_symbol * 5);
        for (int i = 0; i < max_iq_display; ++i)
        {
            std::cout << "IQ[" << i << "]: I=" << baseband_iq[i].real()
                      << ", Q=" << baseband_iq[i].imag() << std::endl;
        }

        // Displaying the first few demodulated bits
        std::cout << "\n--- Demodulated Bit Stream (First 40 Bits) ---" << std::endl;
        int max_bit_display = std::min((int)demodulated_data.size(), 40);
        for (int i = 0; i < max_bit_display; ++i)
        {
            std::cout << demodulated_data[i];
            if ((i + 1) % 8 == 0)
            {
                std::cout << " "; // Group by byte for readability
            }
            if ((i + 1) % 40 == 0)
            {
                std::cout << "\n";
            }
        }
        if (max_bit_display > 0)
        {
            std::cout << "\n";
        }

        std::cout << "\nPipeline Complete. The resulting 'demodulated_data' holds the raw data bits." << std::endl;
        std::cout << "Total generated data bits: " << demodulated_data.size() << std::endl;

        // --- Note on FEC and Decompression ---
        std::cout << "\n[Next Steps in a Real Receiver]\n"
                  << "The 'demodulated_data' would now enter the Channel Decoder (FEC).\n"
                  << "This requires extremely complex LDPC/BCH decoding algorithms to correct errors.\n"
                  << "After FEC, the data is typically an MPEG Transport Stream (TS).\n"
                  << "Finally, an external library (like FFmpeg) is needed for video/audio decompression." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "An error occurred during simulation: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
