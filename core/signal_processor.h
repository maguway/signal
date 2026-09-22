#pragma once
#include "types.h"
#include "kiss_fft.h"
#include <array>
#include <vector>
#include <mutex>
#include <chrono>
#include <fstream>

class SignalProcessor {
public:
    SignalProcessor();
    ~SignalProcessor();

    void setWindowType(WindowType type);
    void addPacket(const DataPacket& pkt, std::ofstream& logFile, bool isRecording);

    // Thread-safe getters for GUI (сигнатуры 1:1 с оригиналом)
    void getGraphData(float* outData[NUM_CHANNELS], int* outCounts);
    void getSpectrumData(std::vector<std::vector<float>>& outData);
    void getWeightedData(std::vector<std::vector<float>>& outData);
    void getFilteredData(float outData[BAND_COUNT][NUM_CHANNELS][FILTERED_RING_SIZE], int* outStartIndex, int* outSize);
    bool getAllComplexSpectra(std::vector<kiss_fft_cpx> outSpecs[NUM_CHANNELS]);

    int getTotalPackets();
    float getMeasuredSampleRate();

private:
    void generateWindows();
    void applyWindowAndFFT(int ch);
    void writeFFTBlockToLog(std::ofstream& logFile, int blockNum);

    std::mutex mtx_;
    std::vector<float> graphData_[NUM_CHANNELS];
    std::vector<float> fftBuffer_[NUM_CHANNELS];

    // Динамические массивы (вместо статических)
    std::vector<std::vector<float>> spectrumData_;           // [NUM_CHANNELS][FFT_SIZE/2]
    std::vector<std::vector<float>> weightedData_;           // [NUM_CHANNELS][FFT_SIZE]
    std::vector<std::vector<std::vector<float>>> filteredRingBuffer_;  // [BAND_COUNT][NUM_CHANNELS][FILTERED_RING_SIZE]

    int ringBufferIndex_ = 0;
    bool complexSpectrumReady_ = false;
    std::vector<kiss_fft_cpx> lastComplexSpectrum_[NUM_CHANNELS];

    std::vector<float> windowCoeffs_[4];
    WindowType currentWindow_ = WINDOW_HANN;
    kiss_fft_cfg cfg_fwd_ = nullptr;
    kiss_fft_cfg cfg_inv_ = nullptr;

    int currentFFTSize_ = FFT_SIZE; // Текущий размер FFT (изначально 256)

public:
    void setFFTSize(int newSize);
    int getFFTSize() { std::lock_guard<std::mutex> lock(mtx_); return currentFFTSize_; }

    int totalPacketsReceived_ = 0;
    std::chrono::steady_clock::time_point lastMeasureTime_;
    int packetsSinceLastMeasure_ = 0;
    float measuredSampleRate_ = 0.0f;
    int fftBlockCounter_ = 0;

    float runningAvg_[NUM_CHANNELS] = {0};
    static constexpr float ALPHA = 0.01f;
};