#include "core/types.h"
#include "core/signal_processor.h"
#include "io/serial_reader.h"
#include "algorithms/doa_engine.h"

#include <windows.h>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <float.h>
#include <complex>

// ImGui & GLFW headers
#include "GLFW/glfw3.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"



// --- Глобальные переменные приложения ---
std::atomic<bool> g_isRunning(false);
std::atomic<bool> g_isRecording(false);
SignalProcessor g_processor;
SerialReader g_serial;
DOAEngine g_doa;
char g_portNameBuf[64] = "\\\\.\\COM8";
std::string g_savedPortName = "\\\\.\\COM8";    // переменная для хранения в ini
std::ofstream g_logFile;


// Автоподбор baud rate на основе Fs
uint32_t g_currentBaudRate = 460800;  // текущий baud rate

// Fs теперь только измеренная, управление убрано
// Все расчёты используют g_processor.getMeasuredSampleRate()

// Текущий размер FFT (будет меняться адаптивно)
int g_currentFFTSize = FFT_SIZE;

// Функция автоподбора FFT_SIZE на основе Fs
// Цель: разрешение ~10 Гц/бин
int calculateFFTSize(float sampleRate) {
    if (sampleRate <= 0.0f) return FFT_SIZE;

    // Желаемое разрешение ~10 Гц/бин
    int targetFFT = static_cast<int>(sampleRate / 10.0f);

    // Выбираем ближайшую степень двойки из списка
    constexpr int FFT_OPTIONS[] = {256, 512, 1024, 2048, 4096};
    constexpr int FFT_OPTIONS_COUNT = 5;

    for (int i = 0; i < FFT_OPTIONS_COUNT; ++i) {
        if (FFT_OPTIONS[i] >= targetFFT) {
            return FFT_OPTIONS[i];
        }
    }
    // Если нужно больше максимума — возвращаем максимум
    return FFT_OPTIONS[FFT_OPTIONS_COUNT - 1];
}


const uint32_t BAUD_OPTIONS[] = {
    460800, 921600, 1000000, 1500000, 2000000, 3000000,
    4000000, 5000000, 6000000, 8000000, 10000000
};
const int BAUD_OPTIONS_COUNT = 11;

uint32_t calculateBaudRate(float sampleRate) {
    // Расчёт требуемой скорости: Fs * 19 байт * 10 бит/байт
    uint32_t requiredBaud = static_cast<uint32_t>(sampleRate * 19.0f * 10.0f);

    // Выбираем минимальное подходящее значение из списка
    for (int i = 0; i < BAUD_OPTIONS_COUNT; ++i) {
        if (BAUD_OPTIONS[i] >= requiredBaud) {
            return BAUD_OPTIONS[i];
        }
    }
    // Если ничего не подошло, возвращаем максимальное
    return BAUD_OPTIONS[BAUD_OPTIONS_COUNT - 1];
}


// Глобальная переменная — настройки берём из DOAConfig
DOAConfig g_doaConfig;  // ← используем ту же структуру, что и в DOAEngine

// Для передачи информации о текущей полосе на другие вкладки
int g_currentBand = -1;        // 0-11 (как domB)
std::string g_currentBandRange; // строка вида "Band 8 (297 - 352 Hz)"

// Обновление информации о текущей полосе на основе частоты
void UpdateCurrentBand(float freq, float sampleRate) {
    if (freq <= 0.0f || sampleRate <= 0.0f) {
        g_currentBand = -1;
        g_currentBandRange = "Waiting for signal...";
        return;
    }

    for (int b = 0; b < BAND_COUNT; ++b) {
        float lowFreq  = fractionToHz(BAND_FRACTIONS_LOW[b],  sampleRate);
        float highFreq = fractionToHz(BAND_FRACTIONS_HIGH[b], sampleRate);
        if (freq >= lowFreq && freq <= highFreq) {
            g_currentBand = b;
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "Band %d (%.0f - %.0f Hz)",
                     b + 1, lowFreq, highFreq);
            g_currentBandRange = buffer;
            return;
        }
    }

    // Частота не попала ни в одну полосу
    g_currentBand = -1;
    g_currentBandRange = "Band ? (outside range)";
}

void SaveDOASettingsToIni() {
    std::ofstream file("doa_config.ini", std::ios::out);
    if (!file.is_open()) return;

    file << "[DOA]" << std::endl;
    file << "soundSpeed=" << g_doaConfig.soundSpeed << std::endl;
    file << "uniformSpacing=" << (g_doaConfig.uniformSpacing ? 1 : 0) << std::endl;
    for (int i = 0; i < 7; i++) {
        file << "spacing" << i << "=" << g_doaConfig.sensorSpacing[i] << std::endl;
    }
    file << "comPort=" << g_portNameBuf << std::endl;
    file.close();
}

void LoadDOASettingsFromIni() {
    std::ifstream file("doa_config.ini");
    if (!file.is_open()) {
        // Файла нет — оставляем значения по умолчанию
        return;
    }

    std::string line;
    bool inDOASection = false;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        if (line[0] == '[') {
            inDOASection = (line == "[DOA]");
            continue;
        }
        if (!inDOASection) continue;

        size_t eqPos = line.find('=');
        if (eqPos == std::string::npos) continue;

        std::string key = line.substr(0, eqPos);
        std::string value = line.substr(eqPos + 1);

        if (key == "soundSpeed") {
            g_doaConfig.soundSpeed = std::stof(value);
        } else if (key == "uniformSpacing") {
            g_doaConfig.uniformSpacing = (std::stoi(value) != 0);
        } else if (key.substr(0, 7) == "spacing") {
            int idx = std::stoi(key.substr(7));
            if (idx >= 0 && idx < 7) {
                g_doaConfig.sensorSpacing[idx] = std::stof(value);
            }
        } else if (key == "comPort") {
            strncpy(g_portNameBuf, value.c_str(), sizeof(g_portNameBuf) - 1);
            g_portNameBuf[sizeof(g_portNameBuf) - 1] = '\0';
        }
    }
    file.close();
}

void onSerialPacket(const DataPacket& pkt) {
    g_processor.addPacket(pkt, g_logFile, g_isRecording);
}

// Параметры GUI
float g_spectrumGain = 1.0f;
int g_spectrumSmooth = 1;
int g_currentWindowIdx = 1;
int g_frequencyScale = SCALE_LINEAR;
float g_peakSensitivity = 50.0f;
bool g_autoScaleOscilloscope = true;
int g_filterViewMode = 0;

// --- Вспомогательные функции алгоритмов ---
void smoothArray(float* data, int size, int windowSize) {
    if (windowSize <= 1) return;
    std::vector<float> temp(size);
    int halfWindow = windowSize / 2;
    for (int i = 0; i < size; ++i) {
        float sum = 0.0f; int count = 0;
        for (int j = -halfWindow; j <= halfWindow; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < size) { sum += data[idx]; count++; }
        }
        temp[i] = sum / count;
    }
    for (int i = 0; i < size; ++i) data[i] = temp[i];
}

std::vector<PeakInfo> findSignificantPeaks(const float* spectrum, int size, float sampleRate, int fftSize, float sensitivityPercent) {
    std::vector<PeakInfo> peaks;
    if (size < 3) return peaks;
    std::vector<float> sortedSpectrum(spectrum, spectrum + size);
    std::sort(sortedSpectrum.begin(), sortedSpectrum.end());
    int noiseCount = static_cast<int>(size * 0.7f);
    if (noiseCount < 1) noiseCount = 1;
    float noiseSum = 0.0f;
    for (int i = 0; i < noiseCount; ++i) noiseSum += sortedSpectrum[i];
    float noiseFloor = noiseSum / noiseCount;
    float maxPower = spectrum[0];
    for (int i = 1; i < size; ++i) if (spectrum[i] > maxPower) maxPower = spectrum[i];
    float dynamicRange = maxPower - noiseFloor;
    if (dynamicRange < 1.0f) dynamicRange = 1.0f;
    float threshold = noiseFloor + (sensitivityPercent / 100.0f) * dynamicRange;
    if (sensitivityPercent > 80.0f) threshold = maxPower - ((100.0f - sensitivityPercent) / 20.0f) * 5.0f;
    if (threshold < noiseFloor + 3.0f) threshold = noiseFloor + 3.0f;
    const int CLUSTER_MERGE_DISTANCE = 3;
    std::vector<bool> visited(size, false);
    for (int i = 1; i < size - 1; ++i) {
        if (visited[i] || spectrum[i] < threshold) continue;
        int startBin = i, endBin = i;
        while (startBin > 0 && spectrum[startBin - 1] >= threshold && (endBin - startBin) < 10) startBin--;
        while (endBin < size - 1 && spectrum[endBin + 1] >= threshold && (endBin - startBin) < 10) endBin++;
        float localMaxPower = -1000.0f; int localMaxBin = startBin;
        for (int k = startBin; k <= endBin; ++k) {
            if (spectrum[k] > localMaxPower) { localMaxPower = spectrum[k]; localMaxBin = k; }
            visited[k] = true;
        }
        bool isValidPeak = true;
        if (startBin > 0 && spectrum[startBin - 1] > localMaxPower) isValidPeak = false;
        if (endBin < size - 1 && spectrum[endBin + 1] > localMaxPower) isValidPeak = false;
        if (isValidPeak) peaks.push_back({static_cast<float>(localMaxBin) * sampleRate / fftSize, localMaxPower, localMaxBin});
    }
    std::sort(peaks.begin(), peaks.end(), [](const PeakInfo& a, const PeakInfo& b) { return a.power > b.power; });
    std::vector<PeakInfo> finalPeaks;
    for (const auto& p : peaks) {
        bool tooClose = false;
        for (const auto& fp : finalPeaks) if (abs(p.binIndex - fp.binIndex) < CLUSTER_MERGE_DISTANCE) { tooClose = true; break; }
        if (!tooClose) finalPeaks.push_back(p);
    }
    std::sort(finalPeaks.begin(), finalPeaks.end(), [](const PeakInfo& a, const PeakInfo& b) { return a.frequency < b.frequency; });
    return finalPeaks;
}

// --- Главная функция ---
int main() {
    if (!glfwInit()) return -1;
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    GLFWwindow* window = glfwCreateWindow(1400, 900, "Hydroacoustic Monitor", NULL, NULL);
    if (window == NULL) return -1;
    glfwMakeContextCurrent(window); glfwSwapInterval(1);
    IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImPlot::CreateContext();
    ImGui::StyleColorsDark(); ImPlot::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    LoadDOASettingsFromIni();
    g_doa.setConfig(g_doaConfig);

    ImVec4 clear_color = ImVec4(0.1f, 0.1f, 0.1f, 1.00f);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
        ImGui::Begin("Main Interface", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_MenuBar);

        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Exit")) glfwSetWindowShouldClose(window, true);
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }


        // === ПАНЕЛЬ УПРАВЛЕНИЯ ===
        float sampleRate = g_processor.getMeasuredSampleRate();

        // Используем таблицу с 3 колонками, выравнивание по верху
        if (ImGui::BeginTable("ControlPanel", 3, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoBordersInBody)) {
            ImGui::TableSetupColumn("Controls", ImGuiTableColumnFlags_WidthFixed, 450.0f);
            ImGui::TableSetupColumn("LeftData", ImGuiTableColumnFlags_WidthFixed, 250.0f);
            ImGui::TableSetupColumn("RightData", ImGuiTableColumnFlags_WidthFixed, 250.0f);
            ImGui::TableNextRow();

            // === КОЛОНКА 1: Порт + кнопки ===
            ImGui::TableNextColumn();
            ImGui::BeginGroup();
            ImGui::Text("📡 Port: "); ImGui::SameLine();
            ImGui::SetNextItemWidth(150);
            ImGui::InputText("##PortName", g_portNameBuf, IM_ARRAYSIZE(g_portNameBuf));

            // Строка с кнопками
            if (!g_isRunning) {
                if (ImGui::Button("▶ START", ImVec2(100, 35))) {
                    g_isRunning = true;
                    // Используем текущее измеренное значение, если есть, иначе 2000
                    float currentFs = g_processor.getMeasuredSampleRate();
                    if (currentFs < 100.0f) currentFs = 2000.0f;  // fallback
                    g_currentBaudRate = calculateBaudRate(currentFs);
                    int newFFTSize = calculateFFTSize(currentFs);
                    g_processor.setFFTSize(newFFTSize);
                    g_currentFFTSize = newFFTSize;
                    g_serial.start(g_portNameBuf, g_currentBaudRate, onSerialPacket);
                }
            } else {
                if (ImGui::Button("⏹ STOP", ImVec2(100, 35))) {
                    g_isRunning = false;
                    g_serial.stop();
                }
            }
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();

            if (g_isRunning) {
                if (!g_isRecording) {
                    if (ImGui::Button("⏺ RECORD", ImVec2(120, 35))) {
                        g_isRecording = true;
                        auto now = std::chrono::system_clock::now();
                        auto time_t_now = std::chrono::system_clock::to_time_t(now);
                        std::string filename = "log_" + std::to_string(time_t_now) + ".txt";
                        g_logFile.open(filename);
                    }
                } else {
                    if (ImGui::Button("⏹ STOP REC", ImVec2(120, 35))) {
                        g_isRecording = false;
                        if (g_logFile.is_open()) g_logFile.close();
                    }
                }
            } else {
                ImGui::TextDisabled("Start to enable recording");
            }
            ImGui::EndGroup();

            // === КОЛОНКА 2: Левые данные ===
            ImGui::TableNextColumn();
            ImGui::BeginGroup();
            ImGui::Text("📡 Measured Fs: %.1f Hz", sampleRate);
            ImGui::Text("📦 Packets: %d", g_processor.getTotalPackets());
            ImGui::EndGroup();

            // === КОЛОНКА 3: Правые данные ===
            // === КОЛОНКА 3: Правые данные ===
            ImGui::TableNextColumn();
            ImGui::BeginGroup();

            // ✅ ДИНАМИЧЕСКИЙ ПЕРЕСЧЁТ BAUD RATE
            if (sampleRate > 100.0f) {
                uint32_t calculatedBaud = calculateBaudRate(sampleRate);
                if (calculatedBaud != g_currentBaudRate) {
                    g_currentBaudRate = calculatedBaud;
                    if (g_isRunning) {
                        g_serial.stop();
                        g_serial.start(g_portNameBuf, g_currentBaudRate, onSerialPacket);
                    }
                }
            }

            ImGui::Text("⚡ Baud Rate: %d", g_currentBaudRate);
            ImGui::Text("📐 FFT: %d (auto)", g_currentFFTSize);

            float nyquist = sampleRate / 2.0f;
            ImGui::Text("📡 Nyquist: %.1f Hz", nyquist);

            // Диагностика потерь пакетов
            float expectedFs = g_currentBaudRate / (19.0f * 10.0f);
            if (expectedFs > 100.0f && sampleRate > 100.0f) {
                float lossPercent = (1.0f - sampleRate / expectedFs) * 100.0f;
                if (lossPercent > 5.0f) {
                    ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "⚠ Packet loss: %.1f%%", lossPercent);
                } else {
                    ImGui::Text("📊 Packet loss: %.1f%%", lossPercent);
                }
            }

            ImGui::EndGroup();

            ImGui::EndTable();
        }
        ImGui::Separator();

        // === ВКЛАДКИ ===
        if (ImGui::BeginTabBar("MainTabs")) {
            // 1. Oscilloscope
            if (ImGui::BeginTabItem("📈 Oscilloscope")) {
                int totalPackets = g_processor.getTotalPackets();
                ImGui::Checkbox("Auto Scale Y", &g_autoScaleOscilloscope);
                ImGui::SameLine();
                ImGui::TextDisabled("(Uncheck to zoom/pan manually)");

                if (totalPackets == 0) {
                    ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "Waiting for data...");
                } else {
                    float *dataPtrs[NUM_CHANNELS];
                    int counts[NUM_CHANNELS];
                    g_processor.getGraphData(dataPtrs, counts);

                    const char *channelNames[] = {"CH1", "CH2", "CH3", "CH4", "CH5", "CH6", "CH7", "CH8"};

                    // ✅ ОГРАНИЧИВАЕМ КОЛИЧЕСТВО ТОЧЕК
                    int maxPoints = 1024;
                    int plotPoints = counts[0];
                    if (plotPoints > maxPoints) plotPoints = maxPoints;

                    // Находим min/max ТОЛЬКО среди отображаемых точек
                    float globalMin = FLT_MAX, globalMax = -FLT_MAX;
                    bool hasData = false;
                    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
                        if (counts[ch] > 0) {
                            hasData = true;
                            for (int i = 0; i < plotPoints && i < counts[ch]; ++i) {
                                if (dataPtrs[ch][i] < globalMin) globalMin = dataPtrs[ch][i];
                                if (dataPtrs[ch][i] > globalMax) globalMax = dataPtrs[ch][i];
                            }
                        }
                    }

                    float range = globalMax - globalMin;
                    float padding = (range > 1.0f) ? range * 0.1f : 50.0f;
                    float plotMinY = globalMin - padding;
                    float plotMaxY = globalMax + padding;

                    if (!hasData || (plotMaxY - plotMinY) < 10.0f) {
                        plotMinY = -100.0f;
                        plotMaxY = (7 * VISUAL_OFFSET) + 100.0f;
                    }

                    // ✅ ФЛАГИ ДЛЯ УМЕНЬШЕНИЯ НАГРУЗКИ
                    if (ImPlot::BeginPlot("8-Channel Stacked View", ImVec2(-1, 600),
                                          ImPlotFlags_NoBoxSelect | ImPlotFlags_NoMouseText)) {
                        ImPlot::SetupAxes("Time (samples)", "Amplitude");
                        ImPlot::SetupAxisLimits(ImAxis_X1, 0, MAX_SAMPLES, ImGuiCond_Once);
                        if (g_autoScaleOscilloscope) {
                            ImPlot::SetupAxisLimits(ImAxis_Y1, plotMinY, plotMaxY, ImGuiCond_Always);
                        }

                        for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
                            if (counts[ch] >= 2) {
                                ImPlot::PlotLine(channelNames[ch], dataPtrs[ch], plotPoints);
                            }
                        }
                        ImPlot::EndPlot();
                    }
                }
                ImGui::EndTabItem();
            }
            // 2. Spectrum Analyzer
            if (ImGui::BeginTabItem(" Spectrum Analyzer")) {
                ImGui::BeginGroup(); ImGui::Text("Settings:"); ImGui::Text("Window:");
                if(ImGui::Combo("##Window", &g_currentWindowIdx, WINDOW_NAMES, 4)) g_processor.setWindowType(static_cast<WindowType>(g_currentWindowIdx));
                ImGui::SliderFloat("Gain (dB)", &g_spectrumGain, 0.1f, 5.0f, "%.1f"); ImGui::SliderInt("Smooth", &g_spectrumSmooth, 1, 21, "%d");
                ImGui::RadioButton("Linear", &g_frequencyScale, SCALE_LINEAR); ImGui::RadioButton("Log", &g_frequencyScale, SCALE_LOGARITHMIC);
                ImGui::SliderFloat("Sensitivity", &g_peakSensitivity, 0.0f, 100.0f, "%.0f%%"); ImGui::EndGroup();
                ImGui::SameLine(); ImGui::TextDisabled("|"); ImGui::SameLine(); ImGui::Text("Fs: %.1f Hz", sampleRate);

                // Получаем спектр через новый getter (возвращает vector актуального размера)
                std::vector<std::vector<float>> spectrumData;
                g_processor.getSpectrumData(spectrumData);
                if(!spectrumData.empty() && !spectrumData[0].empty()){
                    int fftSize = g_processor.getFFTSize();
                    int halfFFT = fftSize / 2;
                    const char* chNames[]={"CH1","CH2","CH3","CH4","CH5","CH6","CH7","CH8"};

                    // Применяем усиление и сглаживание
                    std::vector<std::vector<float>> displayData(NUM_CHANNELS, std::vector<float>(halfFFT));
                    for(int ch=0; ch<NUM_CHANNELS; ++ch)
                        for(int i=0; i<halfFFT; ++i)
                            displayData[ch][i] = spectrumData[ch][i] * g_spectrumGain;
                    for(int ch=0; ch<NUM_CHANNELS; ++ch)
                        smoothArray(displayData[ch].data(), halfFFT, g_spectrumSmooth);

                    if(ImPlot::BeginPlot("Power Spectrum (dB)", ImVec2(-1,600))){
                        ImPlot::SetupAxes("Frequency (Hz)", "Power (dB)");
                        float maxFreq = sampleRate / 2.0f;
                        if(g_frequencyScale == SCALE_LOGARITHMIC){
                            ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);
                            ImPlot::SetupAxisLimits(ImAxis_X1, 10.0f, maxFreq, ImGuiCond_Always);
                        } else {
                            ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Linear);
                            ImPlot::SetupAxisLimits(ImAxis_X1, 0, maxFreq, ImGuiCond_Always);
                        }
                        ImPlot::SetupAxisLimits(ImAxis_Y1, -20, 80, ImGuiCond_Once);

                        // Массив частот для оси X
                        std::vector<float> freqVals(halfFFT);
                        for(int i=0; i<halfFFT; ++i)
                            freqVals[i] = static_cast<float>(i) * sampleRate / fftSize;

                        for(int ch=0; ch<NUM_CHANNELS; ++ch)
                            ImPlot::PlotLine(chNames[ch], freqVals.data(), displayData[ch].data(), halfFFT);

                        auto peaks = findSignificantPeaks(displayData[0].data(), halfFFT, sampleRate, fftSize, g_peakSensitivity);
                        for(const auto& p : peaks) {
                            char lbl[64];
                            snprintf(lbl, sizeof(lbl), "%.0f Hz\n%.1f dB", p.frequency, p.power);
                            ImPlot::PushStyleColor(ImPlotCol_InlayText, ImVec4(1,1,0,1));
                            ImPlot::PlotText(lbl, p.frequency, p.power + 2.0f, ImVec2(0.5f, 0.0f));
                            ImPlot::PopStyleColor();
                        }
                        ImPlot::EndPlot();
                    }
                } else {
                    ImGui::TextColored(ImVec4(1,0.5,0,1), "Accumulating data...");
                }
                ImGui::EndTabItem();
            }
            // 3. Weighted Signals
            if (ImGui::BeginTabItem(" Weighted Signals")) {
                ImGui::Text("Windowed Time Domain");
                if(ImGui::Combo("##WinW", &g_currentWindowIdx, WINDOW_NAMES, 4))
                    g_processor.setWindowType(static_cast<WindowType>(g_currentWindowIdx));

                std::vector<std::vector<float>> wd;
                g_processor.getWeightedData(wd);
                if(!wd.empty() && !wd[0].empty()){
                    int fftSize = g_processor.getFFTSize();
                    const char* chN[]={"CH1","CH2","CH3","CH4","CH5","CH6","CH7","CH8"};
                    if(ImPlot::BeginPlot("Weighted Block", ImVec2(-1,600))){
                        ImPlot::SetupAxes("Sample", "Amplitude");
                        ImPlot::SetupAxisLimits(ImAxis_X1, 0, fftSize, ImGuiCond_Always);
                        ImPlot::SetupAxisLimits(ImAxis_Y1, -500, 4500, ImGuiCond_Once);
                        for(int ch=0; ch<NUM_CHANNELS; ++ch)
                            ImPlot::PlotLine(chN[ch], wd[ch].data(), fftSize);
                        ImPlot::EndPlot();
                    }
                } else {
                    ImGui::TextColored(ImVec4(1,0.5,0,1), "Waiting...");
                }
                ImGui::EndTabItem();
            }

            // 4. Band Filters
            if (ImGui::BeginTabItem(" Band Filters")) {
                ImGui::RadioButton("By Bands (12)", &g_filterViewMode, 0); ImGui::SameLine();
                ImGui::RadioButton("By Channels (8)", &g_filterViewMode, 1);
                static bool autoScaleF = true; ImGui::SameLine(); ImGui::Checkbox("Auto Y", &autoScaleF);

                float fd[BAND_COUNT][NUM_CHANNELS][FILTERED_RING_SIZE];
                int si, ds;
                g_processor.getFilteredData(fd, &si, &ds);

                // === ВЫЧИСЛЯЕМ domB ОДИН РАЗ ДЛЯ ОБОИХ РЕЖИМОВ ===
                int domB = -1;
                float maxR = 0.0f;
                const int DEC = 8;
                const int MAX_DP = FILTERED_RING_SIZE / DEC;
                for(int b=0; b<BAND_COUNT; ++b) {
                    float mn=FLT_MAX, mx=-FLT_MAX;
                    for(int ch=0; ch<NUM_CHANNELS; ++ch) {
                        for(int i=0; i<MAX_DP; ++i) {
                            float v = fd[b][ch][(si + i*DEC) % FILTERED_RING_SIZE];
                            if(v < mn) mn = v;
                            if(v > mx) mx = v;
                        }
                    }
                    float r = mx - mn;
                    if(r > maxR) { maxR = r; domB = b; }
                }
                // ====================================================

                // Используем TARGET Fs для заголовков
                float displayFs = sampleRate;  // используем измеренную Fs

                const char* chN[]={"CH1","CH2","CH3","CH4","CH5","CH6","CH7","CH8"};
                const int DEC2 = 8;
                const int MAX_DP2 = FILTERED_RING_SIZE / DEC2;

                static float xV[MAX_DP2];
                for(int i=0; i<MAX_DP2; ++i) xV[i] = (float)(i * DEC2);

                float aw = ImGui::GetContentRegionAvail().x - 20.0f;
                float ph = 180.0f;

                // --- Режим: По Полосам (12 графиков) ---
                if (g_filterViewMode == 0) {
                    float pw = (aw - 40.0f) / 3.0f;
                    ImGui::Text("12 Bands x 8 Channels (Target Fs=%.0f Hz)", displayFs);

                    for(int b=0; b<BAND_COUNT; ++b) {
                        ImGui::PushID(b);

                        float lowHz  = fractionToHz(BAND_FRACTIONS_LOW[b],  displayFs);
                        float highHz = fractionToHz(BAND_FRACTIONS_HIGH[b], displayFs);

                        char t[128];
                        snprintf(t, sizeof(t), "Band %d (%.0f-%.0f Hz)", b+1, lowHz, highHz);

                        float plotData[NUM_CHANNELS][MAX_DP2];
                        float mn = FLT_MAX, mx = -FLT_MAX;
                        for(int ch=0; ch<NUM_CHANNELS; ++ch) {
                            for(int i=0; i<MAX_DP2; ++i) {
                                int idx = (si + i * DEC2) % FILTERED_RING_SIZE;
                                float v = fd[b][ch][idx];
                                plotData[ch][i] = v;
                                if(v < mn) mn = v;
                                if(v > mx) mx = v;
                            }
                        }

                        float rng = mx - mn;
                        float pad = (rng > 1.0f) ? rng * 0.1f : 50.0f;
                        bool dom = (b == domB);

                        if(ImPlot::BeginPlot(t, ImVec2(pw, ph))) {
                            ImPlot::SetupAxes("Time", "Amp");
                            ImPlot::SetupAxisLimits(ImAxis_X1, 0, FILTERED_RING_SIZE, ImGuiCond_Always);
                            if(autoScaleF) ImPlot::SetupAxisLimits(ImAxis_Y1, mn-pad, mx+pad, ImGuiCond_Always);

                            if(dom) {
                                ImPlot::PushStyleColor(ImPlotCol_FrameBg, ImVec4(0.8f, 0.7f, 0.1f, 0.3f));
                                ImPlot::PushStyleColor(ImPlotCol_TitleText, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
                                ImPlot::PushStyleColor(ImPlotCol_AxisText, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
                                ImPlot::PushStyleColor(ImPlotCol_AxisGrid, ImVec4(1.0f, 1.0f, 0.0f, 0.5f));
                            }

                            for(int ch=0; ch<NUM_CHANNELS; ++ch) {
                                ImPlot::PlotLine(chN[ch], xV, plotData[ch], MAX_DP2);
                            }

                            if(dom) {
                                ImPlot::PopStyleColor(4);
                            }
                            ImPlot::EndPlot();
                        }
                        ImGui::PopID();

                        if((b+1)%3 != 0) ImGui::SameLine();
                        else ImGui::NewLine();
                    }
                }
                    // --- Режим: По Каналам (8 графиков) ---
                else {
                    float pw = (aw - 20.0f) / 2.0f;
                    ImGui::Text("8 Channels x 12 Bands");

                    if (domB >= 0) {
                        float lowHz  = fractionToHz(BAND_FRACTIONS_LOW[domB], displayFs);
                        float highHz = fractionToHz(BAND_FRACTIONS_HIGH[domB], displayFs);
                        ImGui::TextColored(ImVec4(1,1,0,1), "Active: Band %d (%.0f-%.0f Hz)", domB+1, lowHz, highHz);
                    }

                    for(int ch=0; ch<NUM_CHANNELS; ++ch) {
                        ImGui::PushID(ch + 100);
                        char t[64]; snprintf(t, sizeof(t), "Channel %d", ch+1);

                        float plotData[BAND_COUNT][MAX_DP2];
                        float mn = FLT_MAX, mx = -FLT_MAX;

                        for(int b=0; b<BAND_COUNT; ++b) {
                            for(int i=0; i<MAX_DP2; ++i) {
                                int idx = (si + i * DEC2) % FILTERED_RING_SIZE;
                                float v = fd[b][ch][idx];
                                plotData[b][i] = v;
                                if(v < mn) mn = v;
                                if(v > mx) mx = v;
                            }
                        }

                        float rng = mx - mn;
                        float pad = (rng > 1.0f) ? rng * 0.1f : 50.0f;

                        if(ImPlot::BeginPlot(t, ImVec2(pw, ph))) {
                            ImPlot::SetupAxes("Time", "Amp");
                            ImPlot::SetupAxisLimits(ImAxis_X1, 0, FILTERED_RING_SIZE, ImGuiCond_Always);
                            if(autoScaleF) ImPlot::SetupAxisLimits(ImAxis_Y1, mn-pad, mx+pad, ImGuiCond_Always);

                            for(int b=0; b<BAND_COUNT; ++b) {
                                char bl[64];
                                if (b == domB) {
                                    snprintf(bl, sizeof(bl), "*B%d", b+1);
                                } else {
                                    snprintf(bl, sizeof(bl), "B%d", b+1);
                                }
                                ImPlot::PlotLine(bl, xV, plotData[b], MAX_DP2);
                            }
                            ImPlot::EndPlot();
                        }
                        ImGui::PopID();

                        if((ch+1)%2 != 0) ImGui::SameLine();
                        else ImGui::NewLine();
                    }
                }
                ImGui::EndTabItem();
            }

            // 5. Direction Finding (4 метода)
            if (ImGui::BeginTabItem("🧭 Direction Finding")) {
                ImGui::Text("=== DOA Comparison (PhaseDiff / MUSIC / Capon / EV) ==="); ImGui::Separator();


                ImGui::Separator();
                ImGui::Text("📐 Calibration Settings");

                // Переключатель среды
                if (ImGui::RadioButton("Air (330 m/s)", g_doaConfig.soundSpeed == 330.0f)) {
                    g_doaConfig.soundSpeed = 330.0f;
                    g_doa.setConfig(g_doaConfig);
                }
                ImGui::SameLine();
                if (ImGui::RadioButton("Water (1500 m/s)", g_doaConfig.soundSpeed == 1500.0f)) {
                    g_doaConfig.soundSpeed = 1500.0f;
                    g_doa.setConfig(g_doaConfig);
                }

                // Переключатель режима расстояний
                if (ImGui::Button(g_doaConfig.uniformSpacing ? "Uniform spacing" : "Non-uniform spacing")) {
                    g_doaConfig.uniformSpacing = !g_doaConfig.uniformSpacing;
                    g_doa.setConfig(g_doaConfig);
                }

                // Ползунок для расстояния (см)
                float spacingCm = g_doaConfig.sensorSpacing[0] * 100.0f;
                if (ImGui::SliderFloat("Spacing (cm)", &spacingCm, 1.0f, 50.0f, "%.1f")) {
                    for (int i = 0; i < 7; i++) {
                        g_doaConfig.sensorSpacing[i] = spacingCm / 100.0f;
                    }
                    g_doa.setConfig(g_doaConfig);
                }

                if (!g_doaConfig.uniformSpacing) {
                    ImGui::Indent();
                    ImGui::Text("Individual spacings (cm):");
                    for (int i = 0; i < 7; i++) {
                        char label[32];
                        snprintf(label, sizeof(label), "  %d-%d", i+1, i+2);
                        float spacingCm_i = g_doaConfig.sensorSpacing[i] * 100.0f;
                        if (ImGui::SliderFloat(label, &spacingCm_i, 1.0f, 50.0f, "%.1f")) {
                            g_doaConfig.sensorSpacing[i] = spacingCm_i / 100.0f;
                            g_doa.setConfig(g_doaConfig);
                        }
                    }
                    ImGui::Unindent();
                }

                ImGui::Separator();


                static std::map<DOAMethod, BearingResult> doaResults;
                std::vector<kiss_fft_cpx> specs[NUM_CHANNELS];
                if (g_processor.getAllComplexSpectra(specs)) {
                    float fs = g_processor.getMeasuredSampleRate();

                    int peakBin = 0; float maxP = 0;
                    // Сняли ограничение поиска пика разумным диапазоном (было до 500 Гц, сейчас нет...)
                    int fftSize = g_processor.getFFTSize();
                    int maxFreqBin = fftSize/2 - 5;

                    for(int i=5; i<maxFreqBin; ++i) {
                        float p = 0;
                        for(int ch = 0; ch < NUM_CHANNELS; ++ch) {
                            p += specs[ch][i].r * specs[ch][i].r + specs[ch][i].i * specs[ch][i].i;
                        }
                        if(p > maxP) { maxP = p; peakBin = i; }
                    }

                    // ДИАГНОСТИКА: выводим выбранный peakBin
                    static bool firstPeak = true;
                    if(firstPeak) {
                        std::cout << "Selected peakBin = " << peakBin << ", freq = " << (peakBin * fs / fftSize) << " Hz" << std::endl;
                        firstPeak = false;
                    }

                    if(maxP > 100.0f && fs > 0) {
                        g_doa.setFFTSize(g_processor.getFFTSize());     // !!! 1 из 3 при исправлении азимута - добавить
                        doaResults = g_doa.process(specs, fs, peakBin);

                        //// для обновления данных о текущей полосе на основе частоты
                        auto it = doaResults.find(DOAMethod::PhaseDiff);
                        if (it != doaResults.end() && it->second.isValid) {
                            UpdateCurrentBand(it->second.peakFreq, fs);
                        } ////
                    }
                }
                const char* names[] = {"Phase Diff", "MUSIC", "Capon", "EV"};
                ImVec4 colors[] = {ImVec4(0.2,0.8,0.2,1), ImVec4(0.2,0.4,1.0,1), ImVec4(1.0,0.6,0.0,1), ImVec4(1.0,0.2,0.8,1)};
                ImGui::BeginGroup();
                for(int i=0; i<4; ++i) {
                    DOAMethod m = static_cast<DOAMethod>(i);
                    auto it = doaResults.find(m); bool valid = (it != doaResults.end() && it->second.isValid);
                    ImGui::PushStyleColor(ImGuiCol_Text, valid ? colors[i] : ImVec4(0.5,0.5,0.5,0.5));
                    ImGui::Text("%s:", names[i]); ImGui::SameLine();
                    if(valid) ImGui::Text("%.1f° (Conf: %.0f%%) | %.1f Hz", it->second.azimuthDeg, it->second.confidence*100, it->second.peakFreq);
                    else ImGui::TextDisabled("---");
                    ImGui::PopStyleColor();
                }
                ImGui::EndGroup(); ImGui::SameLine(); ImGui::Dummy(ImVec2(50,0));

                //// Вывод полосы на экран
                ImGui::Separator();
                if (g_currentBand >= 0) {
                    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "🎵 %s", g_currentBandRange.c_str());
                } else {
                    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "🎵 %s", g_currentBandRange.c_str());
                }
                ImGui::Separator(); ////

                ImVec2 cc = ImGui::GetCursorScreenPos(); cc.x+=100; cc.y+=100; float rad=80.0f;
                ImDrawList* dl = ImGui::GetWindowDrawList(); dl->AddCircle(cc, rad, IM_COL32(100,100,100,255), 32, 2.0f);
                dl->AddText(ImVec2(cc.x-10,cc.y-rad-20), IM_COL32_WHITE, "0°"); dl->AddText(ImVec2(cc.x+rad+10,cc.y-5), IM_COL32_WHITE, "+90°"); dl->AddText(ImVec2(cc.x-30,cc.y+rad+5), IM_COL32_WHITE, "-90°");
                for(int i=0; i<4; ++i) {
                    DOAMethod m = static_cast<DOAMethod>(i); auto it = doaResults.find(m);
                    if(it != doaResults.end() && it->second.isValid && it->second.confidence > 0.2f) {
                        float aRad = it->second.azimuthDeg * M_PI / 180.0f;
                        ImVec2 tip; tip.x=cc.x+rad*sinf(aRad); tip.y=cc.y-rad*cosf(aRad);
                        ImU32 arrowCol = ImColor(colors[i]);
                        dl->AddLine(cc, tip, arrowCol, 2.0f); dl->AddCircleFilled(tip, 4.0f, arrowCol);
                    }
                }
                ImGui::Dummy(ImVec2(220,220)); ImGui::Separator(); ImGui::TextDisabled("Covariance matrix: 8x8 | Temporal avg: 8 blocks | Uses ALL channels");
                ImGui::EndTabItem();
            }
            // 6. Info
            if (ImGui::BeginTabItem("ℹ️ Info")) {
                ImGui::Text("Hydroacoustic Monitor v5.0 (Multi-DOA)"); ImGui::Text("Raw Range: 0-4095"); ImGui::Text("FFT: %d (auto)", g_processor.getFFTSize()); ImGui::Text("Window: %s", WINDOW_NAMES[g_currentWindowIdx]);
                if(sampleRate>0){float nyq = sampleRate/2.0f;ImGui::Text("Fs: %.1f Hz | Nyquist: %.1f Hz | Res: %.2f Hz", sampleRate, nyq);
                    for(int b=0;b<BAND_COUNT;++b){float lowHz  = fractionToHz(BAND_FRACTIONS_LOW[b],  sampleRate);
                        float highHz = fractionToHz(BAND_FRACTIONS_HIGH[b], sampleRate);
                        ImGui::Text("  Band %d: %.1f-%.1f Hz", b+1, lowHz, highHz);
                    }
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar(); // ✅ СТРОГО ВНУТРИ БЛОКА BeginTabBar
        }

        ImGui::End(); // Закрывает "Main Interface"
        ImGui::Render();
        int dw, dh; glfwGetFramebufferSize(window, &dw, &dh); glViewport(0,0,dw,dh);
        glClearColor(clear_color.x*clear_color.w, clear_color.y*clear_color.w, clear_color.z*clear_color.w, clear_color.w); glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData()); glfwSwapBuffers(window);
    }

    if(g_isRunning){g_isRunning=false; g_serial.stop();}
    if(g_logFile.is_open())g_logFile.close();
    SaveDOASettingsToIni();
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImPlot::DestroyContext(); ImGui::DestroyContext(); glfwTerminate();
    g_doa.setConfig(g_doaConfig);
    return 0;
}