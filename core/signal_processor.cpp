#include "signal_processor.h"
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const char* WINDOW_NAMES[] = {"Rectangular", "Hann", "Hamming", "Blackman"};

SignalProcessor::SignalProcessor() : lastMeasureTime_(std::chrono::steady_clock::now()) {
    generateWindows();

    // Инициализация динамических массивов
    spectrumData_.resize(NUM_CHANNELS, std::vector<float>(FFT_SIZE/2, 0.0f));
    weightedData_.resize(NUM_CHANNELS, std::vector<float>(FFT_SIZE, 0.0f));
    filteredRingBuffer_.resize(BAND_COUNT,
        std::vector<std::vector<float>>(NUM_CHANNELS,
            std::vector<float>(FILTERED_RING_SIZE, 0.0f)));

    // Кэшируем конфигурации FFT один раз (оптимизация)
    cfg_fwd_ = kiss_fft_alloc(FFT_SIZE, 0, nullptr, nullptr);
    cfg_inv_ = kiss_fft_alloc(FFT_SIZE, 1, nullptr, nullptr);
}

void SignalProcessor::setFFTSize(int newSize) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (newSize == currentFFTSize_ || newSize < 64) return; // Игнорируем, если размер не изменился или слишком мал

    // 1. Освобождаем старые планы FFT
    if (cfg_fwd_) free(cfg_fwd_);
    if (cfg_inv_) free(cfg_inv_);

    // 2. Создаем новые планы
    cfg_fwd_ = kiss_fft_alloc(newSize, 0, nullptr, nullptr);
    cfg_inv_ = kiss_fft_alloc(newSize, 1, nullptr, nullptr);

    // 3. Пересчитываем оконные коэффициенты для нового размера
    for (int type = 0; type <= WINDOW_BLACKMAN; ++type) {
        windowCoeffs_[type].resize(newSize);
        for (int i = 0; i < newSize; ++i) {
            float n = static_cast<float>(i);
            float N = static_cast<float>(newSize - 1);
            switch (type) {
                case WINDOW_RECTANGULAR: windowCoeffs_[type][i] = 1.0f; break;
                case WINDOW_HANN: windowCoeffs_[type][i] = 0.5f * (1.0f - std::cos(2.0f * static_cast<float>(M_PI) * n / N)); break;
                case WINDOW_HAMMING: windowCoeffs_[type][i] = 0.54f - 0.46f * std::cos(2.0f * static_cast<float>(M_PI) * n / N); break;
                case WINDOW_BLACKMAN: windowCoeffs_[type][i] = 0.42f - 0.5f * std::cos(2.0f * static_cast<float>(M_PI) * n / N) +
                                      0.08f * std::cos(4.0f * static_cast<float>(M_PI) * n / N); break;
            }
        }
    }

    // 4. Ресайзим внутренние буферы
    // 4. Ресайзим внутренние буферы БЕЗОПАСНО
    try {
        spectrumData_.assign(NUM_CHANNELS, std::vector<float>(newSize / 2, 0.0f));
        weightedData_.assign(NUM_CHANNELS, std::vector<float>(newSize, 0.0f));

        // Для массива векторов используем явный цикл
        for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
            if (ch < NUM_CHANNELS) { // Дополнительная защита
                lastComplexSpectrum_[ch].assign(newSize, kiss_fft_cpx{0.0f, 0.0f});
            }
        }
    } catch (const std::bad_alloc&) {
        // Если не хватило памяти, откатываемся к старому размеру
        currentFFTSize_ = newSize; // Или оставляем старый, зависит от логики
        return;
    }

    // 5. Очищаем буфер сырых данных, чтобы не смешивать старые и новые размеры
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        fftBuffer_[ch].clear();
    }

    // 6. Сохраняем новый размер
    currentFFTSize_ = newSize;
}

SignalProcessor::~SignalProcessor() {
    if (cfg_fwd_) free(cfg_fwd_);
    if (cfg_inv_) free(cfg_inv_);
}

void SignalProcessor::generateWindows() {
    for (int type = 0; type <= WINDOW_BLACKMAN; ++type) {
        windowCoeffs_[type].resize(FFT_SIZE);
        for (int i = 0; i < FFT_SIZE; ++i) {
            float n = static_cast<float>(i);
            float N = static_cast<float>(FFT_SIZE - 1);
            switch (type) {
                case WINDOW_RECTANGULAR: windowCoeffs_[type][i] = 1.0f; break;
                case WINDOW_HANN: windowCoeffs_[type][i] = 0.5f * (1.0f - std::cos(2.0f * static_cast<float>(M_PI) * n / N)); break;
                case WINDOW_HAMMING: windowCoeffs_[type][i] = 0.54f - 0.46f * std::cos(2.0f * static_cast<float>(M_PI) * n / N); break;
                case WINDOW_BLACKMAN: windowCoeffs_[type][i] = 0.42f - 0.5f * std::cos(2.0f * static_cast<float>(M_PI) * n / N) +
                                                                 0.08f * std::cos(4.0f * static_cast<float>(M_PI) * n / N); break;
            }
        }
    }
}

void SignalProcessor::setWindowType(WindowType type) {
    std::lock_guard<std::mutex> lock(mtx_);
    currentWindow_ = type;
}

void SignalProcessor::addPacket(const DataPacket& pkt, std::ofstream& logFile, bool isRecording) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!pkt.isValid) return;

    if (isRecording && logFile.is_open()) {
        logFile << pkt.rawHex << std::endl;
        logFile << "       VAL_HEX: ";
        for (int i = 0; i < NUM_CHANNELS; ++i) logFile << std::hex << std::setw(4) << std::setfill('0') << (int)pkt.values[i] << " ";
        logFile << std::endl << "       VAL_DEC: ";
        for (int i = 0; i < NUM_CHANNELS; ++i) logFile << std::dec << std::setw(5) << std::setfill(' ') << (int)pkt.values[i] << " ";
        logFile << std::endl << "--------------------------------------------------" << std::endl;
    }

    packetsSinceLastMeasure_++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastMeasureTime_).count();
    if (elapsed >= 2000) {
        if (elapsed > 0) measuredSampleRate_ = (packetsSinceLastMeasure_ * 1000.0f) / elapsed;
        packetsSinceLastMeasure_ = 0;
        lastMeasureTime_ = now;
    }

    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        float rawVal = static_cast<float>(pkt.values[ch]);
        runningAvg_[ch] = (1.0f - ALPHA) * runningAvg_[ch] + ALPHA * rawVal;
        float centeredVal = rawVal - runningAvg_[ch];
        float amplifiedVal = centeredVal * VISUAL_GAIN;
        float visualVal = amplifiedVal + (ch * VISUAL_OFFSET);

        graphData_[ch].push_back(visualVal);
        if (graphData_[ch].size() > MAX_SAMPLES) graphData_[ch].erase(graphData_[ch].begin());

        fftBuffer_[ch].push_back(rawVal);
        if (fftBuffer_[ch].size() >= static_cast<size_t>(currentFFTSize_)) {
            applyWindowAndFFT(ch);
            fftBuffer_[ch].clear();
            if (ch == 0) {
                fftBlockCounter_++;
                if (isRecording && logFile.is_open()) writeFFTBlockToLog(logFile, fftBlockCounter_);
            }
        }
    }
    totalPacketsReceived_++;
}

void SignalProcessor::applyWindowAndFFT(int ch) {
    float mean = 0.0f;
    for (int i = 0; i < currentFFTSize_; ++i) mean += fftBuffer_[ch][i];
    mean /= currentFFTSize_;
    std::vector<kiss_fft_cpx> input(currentFFTSize_);
    for (int i = 0; i < currentFFTSize_; ++i) {
        float w = windowCoeffs_[currentWindow_][i];
        input[i].r = (fftBuffer_[ch][i] - mean) * w;
        input[i].i = 0.0f;
        weightedData_[ch][i] = input[i].r;
    }
    std::vector<kiss_fft_cpx> output(currentFFTSize_);
    kiss_fft(cfg_fwd_, input.data(), output.data());
    for (int i = 0; i < currentFFTSize_/2; ++i) {
        float power = output[i].r * output[i].r + output[i].i * output[i].i;
        spectrumData_[ch][i] = 10.0f * std::log10(power + 1e-10f);
    }
    if (lastComplexSpectrum_[ch].size() != static_cast<size_t>(currentFFTSize_))
        lastComplexSpectrum_[ch].resize(currentFFTSize_);
    for(int i=0; i<currentFFTSize_; ++i) lastComplexSpectrum_[ch][i] = output[i];
    for (int b = 0; b < BAND_COUNT; ++b) {
        std::vector<kiss_fft_cpx> maskedFFT(currentFFTSize_, {0.0f, 0.0f});

        // ИСПРАВЛЕНИЕ: Используем целевую Fs для определения границ фильтров
        float lowHz  = fractionToHz(BAND_FRACTIONS_LOW[b],  measuredSampleRate_);
        float highHz = fractionToHz(BAND_FRACTIONS_HIGH[b], measuredSampleRate_);

        // Важно: freqToBin всё равно использует measuredSampleRate_ для перевода Hz -> Bin,
        // потому что FFT рассчитан именно на измеренную частоту дискретизации.
        // Это гарантирует, что мы берем правильные бины из реального спектра,
        // но группируем их согласно целевым диапазонам.
        int low  = freqToBin(lowHz,  measuredSampleRate_, currentFFTSize_);
        int high = freqToBin(highHz, measuredSampleRate_, currentFFTSize_);

        if (low > high) high = low;
        for (int m = low; m <= high; ++m) {
            if (m < currentFFTSize_) {
                maskedFFT[m] = output[m];
                int mirrorIdx = currentFFTSize_ - m;
                if (mirrorIdx != m && mirrorIdx < currentFFTSize_)
                    maskedFFT[mirrorIdx] = output[mirrorIdx];
            }
        }
        std::vector<kiss_fft_cpx> timeDomain(currentFFTSize_);
        kiss_fft(cfg_inv_, maskedFFT.data(), timeDomain.data());
        for (int i = 0; i < currentFFTSize_; ++i) {
            int targetIdx = (ringBufferIndex_ + i) % FILTERED_RING_SIZE;
            filteredRingBuffer_[b][ch][targetIdx] = timeDomain[i].r;
        }
    }
    if (ch == 0) ringBufferIndex_ = (ringBufferIndex_ + currentFFTSize_) % FILTERED_RING_SIZE;
    complexSpectrumReady_ = true;
}

void SignalProcessor::writeFFTBlockToLog(std::ofstream& logFile, int blockNum) {
    logFile << "\n================================================================================\n";
    logFile << "=== FFT BLOCK #" << blockNum << " (Window: " << WINDOW_NAMES[currentWindow_] << ", Fs: "
            << std::fixed << std::setprecision(1) << measuredSampleRate_ << " Hz) ===\n";
    logFile << "================================================================================\n--- SPECTRUM (dB) ---\n";
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        logFile << "CH" << (ch + 1) << ": ";
        for (int i = 0; i < currentFFTSize_/2; ++i) logFile << std::fixed << std::setprecision(1) << spectrumData_[ch][i] << (i < currentFFTSize_/2-1 ? " " : "");
        logFile << "\n";
    }
    logFile << "--- WEIGHTED SIGNAL (Time Domain) ---\n";
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        logFile << "CH" << (ch + 1) << ": ";
        for (int i = 0; i < currentFFTSize_; ++i) logFile << std::fixed << std::setprecision(1) << weightedData_[ch][i] << (i < currentFFTSize_-1 ? " " : "");
        logFile << "\n";
    }
    logFile << "================================================================================\n";
    logFile.flush();
}

void SignalProcessor::getGraphData(float* outData[NUM_CHANNELS], int* outCounts) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        outData[ch] = graphData_[ch].data();
        outCounts[ch] = static_cast<int>(graphData_[ch].size());
    }
}

void SignalProcessor::getSpectrumData(std::vector<std::vector<float>>& outData) {
    std::lock_guard<std::mutex> lock(mtx_);
    outData = spectrumData_;
}

void SignalProcessor::getWeightedData(std::vector<std::vector<float>>& outData) {
    std::lock_guard<std::mutex> lock(mtx_);
    outData = weightedData_;
}

void SignalProcessor::getFilteredData(float outData[BAND_COUNT][NUM_CHANNELS][FILTERED_RING_SIZE], int* outStartIndex, int* outSize) {
    std::lock_guard<std::mutex> lock(mtx_);
    for(int b=0; b<BAND_COUNT; ++b)
        for(int ch=0; ch<NUM_CHANNELS; ++ch)
            for(int i=0; i<FILTERED_RING_SIZE; ++i) outData[b][ch][i] = filteredRingBuffer_[b][ch][i];
    *outStartIndex = ringBufferIndex_;
    *outSize = FILTERED_RING_SIZE;
}

bool SignalProcessor::getAllComplexSpectra(std::vector<kiss_fft_cpx> outSpecs[NUM_CHANNELS]) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!complexSpectrumReady_ || lastComplexSpectrum_[0].empty()) return false;
    for(int ch=0; ch<NUM_CHANNELS; ++ch) outSpecs[ch] = lastComplexSpectrum_[ch];
    return true;
}

int SignalProcessor::getTotalPackets() { std::lock_guard<std::mutex> lock(mtx_); return totalPacketsReceived_; }
float SignalProcessor::getMeasuredSampleRate() { std::lock_guard<std::mutex> lock(mtx_); return measuredSampleRate_; }