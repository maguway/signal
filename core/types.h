#pragma once
#include <cstdint>
#include <string>
#include <vector>

// Константы (идентичны main.cpp)
constexpr int MAX_SAMPLES = 1000;
constexpr int NUM_CHANNELS = 8;
constexpr float VISUAL_OFFSET = 400.0f;
constexpr float VISUAL_GAIN = 5.0f;

// FFT_SIZE теперь переменная величина, но пока оставляем 256 как значение по умолчанию
constexpr int FFT_SIZE_DEFAULT = 256;
constexpr int FFT_SIZE_MIN = 256;
constexpr int FFT_SIZE_MAX = 4096;

// Список поддерживаемых размеров FFT (степени двойки для kissfft)
constexpr int FFT_OPTIONS[] = {256, 512, 1024, 2048, 4096};
constexpr int FFT_OPTIONS_COUNT = 5;

// Пока оставляем как constexpr для обратной совместимости
// На Шаге 3 заменим на внешнюю переменную
constexpr int FFT_SIZE = FFT_SIZE_DEFAULT;

constexpr int BAND_COUNT = 12;
constexpr int FILTERED_RING_SIZE = 1024;

// Границы полос как ДОЛИ от частоты Найквиста (0.0 ... 1.0)
// 12 равномерных полос, покрывающих весь диапазон 0...Fs/2
constexpr float BAND_FRACTIONS_LOW[BAND_COUNT] = {
    0.000f, 0.083f, 0.167f, 0.250f, 0.333f, 0.417f, 0.500f, 0.583f, 0.667f, 0.750f, 0.833f, 0.917f
};
constexpr float BAND_FRACTIONS_HIGH[BAND_COUNT] = {
    0.083f, 0.167f, 0.250f, 0.333f, 0.417f, 0.500f, 0.583f, 0.667f, 0.750f, 0.833f, 0.917f, 1.000f
};

// Вспомогательная функция: переводит герцы в индекс бина FFT
inline int freqToBin(float freqHz, float sampleRate, int fftSize) {
    if (sampleRate <= 0.0f) return 0;
    int bin = static_cast<int>(freqHz * fftSize / sampleRate);
    if (bin < 0) bin = 0;
    if (bin >= fftSize / 2) bin = fftSize / 2 - 1;
    return bin;
}

// Вспомогательная функция: переводит долю в герцы
inline float fractionToHz(float fraction, float sampleRate) {
    return fraction * (sampleRate / 2.0f);
}

enum WindowType {
    WINDOW_RECTANGULAR = 0,
    WINDOW_HANN,
    WINDOW_HAMMING,
    WINDOW_BLACKMAN
};
extern const char* WINDOW_NAMES[];

enum FrequencyScale {
    SCALE_LINEAR = 0,
    SCALE_LOGARITHMIC
};

struct DataPacket {
    uint16_t values[NUM_CHANNELS];
    uint8_t checksum;
    bool isValid;
    std::string rawHex;
};

struct PeakInfo {
    float frequency;
    float power;
    int binIndex;
};

struct BearingResult {
    float azimuthDeg;
    float confidence;
    float peakFreq;
    bool isValid;
};