#ifndef NOTCH_FILTER_HPP
#define NOTCH_FILTER_HPP

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Single Biquad Notch Filter
class BiquadNotch {
public:
    BiquadNotch() { reset(); }

    void reset() {
        x1 = x2 = y1 = y2 = 0.0f;
    }

    // Update coefficients for a Notch at freq_hz
    // fs: Sampling Frequency (e.g. 6667 Hz)
    // q: Q Factor (Width). Higher = Narrower.
    void update(float freq_hz, float fs, float q) {
        // Safety Clamp: Don't filter below 80Hz (Control Bandwidth Protection)
        if (freq_hz < 80.0f) freq_hz = 80.0f;
        // Nyquist Clamp
        if (freq_hz > fs * 0.48f) freq_hz = fs * 0.48f;

        float omega = 2.0f * M_PI * freq_hz / fs;
        float alpha = sin(omega) / (2.0f * q);
        float cos_w = cos(omega);

        b0 = 1.0f;
        b1 = -2.0f * cos_w;
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_w;
        a2 = 1.0f - alpha;

        // Normalize
        float inv_a0 = 1.0f / a0;
        b0 *= inv_a0;
        b1 *= inv_a0;
        b2 *= inv_a0;
        a1 *= inv_a0;
        a2 *= inv_a0;
    }

    // Process one sample
    float process(float in) {
        float out = b0 * in + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        
        // Shift history
        x2 = x1;
        x1 = in;
        y2 = y1;
        y1 = out;

        return out;
    }

private:
    float b0, b1, b2, a0, a1, a2;
    float x1, x2, y1, y2;
};

// 6-Bank Notch Filter (One filter per Motor)
class NotchFilterBank {
public:
    // sample_rate: e.g. 6667.0
    NotchFilterBank(float sample_rate, float q_factor = 3.0f) 
        : fs_(sample_rate), q_(q_factor) {
        filters_.resize(6); // 6 Motors
    }

    // Update filters based on Motor RPMs
    void updateConfig(const std::array<float, 6>& motor_rpms) {
        for (size_t i = 0; i < 6; ++i) {
            float freq = motor_rpms[i] / 60.0f; // RPM to Hz
            // Apply clamps inside update()
            filters_[i].update(freq, fs_, q_);
        }
    }
    
    // Process a 3-axis vector (Apply same filters to X, Y, Z independently?)
    // Usually Structural noise is on all axes.
    // Ideally we need 3 Banks of 6 Filters? (18 filters total)
    // Yes. Each axis needs its own state.
    
    // So we need 3 * 6 = 18 Biquads.
    // Let's refactor.
    
    // Process single float
    float apply(float sample, int axis_idx) {
        float out = sample;
        // Cascade 6 filters for this axis
        for (auto& f : axis_banks_[axis_idx]) {
            out = f.process(out);
        }
        return out;
    }

    // Call this if using 3-axis filtering
    void init3Axis() {
        axis_banks_.resize(3);
        for(int i=0; i<3; i++) {
            axis_banks_[i].resize(6); // 6 Filters per axis
        }
    }
    
    // Update RPMs for all axes
    void updateConfig3Axis(const std::array<float, 6>& motor_rpms) {
        for (int axis = 0; axis < 3; axis++) {
             for (size_t i = 0; i < 6; ++i) {
                float freq = motor_rpms[i] / 60.0f;
                axis_banks_[axis][i].update(freq, fs_, q_);
            }
        }
    }

private:
    float fs_;
    float q_;
    std::vector<BiquadNotch> filters_; // Temp if single channel
    
    // 3 Channels (X, Y, Z), each has 6 Filters
    std::vector<std::vector<BiquadNotch>> axis_banks_;
};

#endif // NOTCH_FILTER_HPP
