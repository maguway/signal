#pragma once
#include "core/types.h"
#include <Eigen/Dense>
#include "kiss_fft.h"
#include <vector>
#include <map>
#include <deque>

enum class DOAMethod { PhaseDiff = 0, MUSIC, Capon, EV };

struct DOAConfig {
    float soundSpeed = 1500.0f;                     // скорость звука (м/с)
    float avgSensorSpacing = 0.2f;                  // среднее расстояние (м)
    float sensorSpacing[7] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f}; // расстояния (м)
    bool uniformSpacing = true;                     // одинаковые ли
    int numSensors = NUM_CHANNELS;
};

class DOAEngine {
public:
    DOAEngine();
    std::map<DOAMethod, BearingResult> process(const std::vector<kiss_fft_cpx>* specs, float fs, int peakBin);

    void setConfig(const DOAConfig& config) {
        cfg_ = config;
        // Пересчитываем среднее расстояние для MUSIC
        float sum = 0;
        for (int i = 0; i < NUM_CHANNELS - 1; i++) {
            sum += cfg_.sensorSpacing[i];
        }
        cfg_.avgSensorSpacing = sum / (NUM_CHANNELS - 1);
    }
    const DOAConfig& getConfig() const { return cfg_; }
    void setFFTSize(int fftSize) { fftSize_ = fftSize; }
    int getFFTSize() const { return fftSize_; }
private:
    int fftSize_ = FFT_SIZE;
    float phaseDifferenceMethod(const Eigen::VectorXcf& snapshot, float freq);
    float musicMethod(float freq);
    float caponMethod(float freq);
    float evMethod(float freq);

    void updateCovariance(const std::vector<kiss_fft_cpx>* specs, int peakBin, float fs);
    std::vector<float> scanAngles(const std::function<float(const Eigen::VectorXcf&)>& spectrumFunc, float freq);

    DOAConfig cfg_;
    std::deque<Eigen::VectorXcf> snapshotHistory_;
    Eigen::MatrixXcf covMatrix_;
    // static constexpr int HISTORY_SIZE = 32;  // Минимум 32 снапшота
    static constexpr int HISTORY_SIZE = 128;
    // static constexpr int FREQ_BINS_AROUND = 5;  // ±5 бинов вокруг пика
    static constexpr int FREQ_BINS_AROUND = 8;
};