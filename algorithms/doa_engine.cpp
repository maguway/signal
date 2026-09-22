#include "doa_engine.h"
#include <cmath>
#include <algorithm>
#include <complex>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

DOAEngine::DOAEngine() : covMatrix_(cfg_.numSensors, cfg_.numSensors) {
    covMatrix_.setZero();
}

void DOAEngine::updateCovariance(const std::vector<kiss_fft_cpx>* specs, int peakBin, float fs) {
    int startBin = std::max(0, peakBin - FREQ_BINS_AROUND);
//    int endBin = std::min(FFT_SIZE/2 - 1, peakBin + FREQ_BINS_AROUND);
    int endBin = std::min(fftSize_/2 - 1, peakBin + FREQ_BINS_AROUND);  // !!! 1 из 3 при исправлении азимута - заменить

    for(int bin = startBin; bin <= endBin; ++bin) {
        Eigen::VectorXcf snapshot(cfg_.numSensors);
        float power = 0.0f;
        for(int ch = 0; ch < cfg_.numSensors; ++ch) {
            // Комплексное значение FFT
            snapshot(ch) = std::complex<float>(specs[ch][bin].r, specs[ch][bin].i);
            power += std::norm(snapshot(ch));
        }
        if(power > 10.0f) {
            snapshotHistory_.push_back(snapshot);
        }
    }

    while(snapshotHistory_.size() > HISTORY_SIZE) {
        snapshotHistory_.pop_front();
    }

    // Комплексная ковариационная матрица
    covMatrix_.setZero();
    for(const auto& s : snapshotHistory_) {
        covMatrix_ += s * s.adjoint();
    }
    covMatrix_ /= (float)snapshotHistory_.size();

    // Диагональная нагрузка (сохраняем комплексность)
    float trace = std::real(covMatrix_.trace());
//    float loading = 0.1f * trace / cfg_.numSensors;
    float loading = 0.3f * trace / cfg_.numSensors;
    covMatrix_.diagonal().array() += loading;
}

float DOAEngine::phaseDifferenceMethod(const Eigen::VectorXcf& snapshot, float freq) {
    // Все операции с комплексными числами
    std::vector<float> phases(cfg_.numSensors);
    std::vector<float> powers(cfg_.numSensors);

    for(int i = 0; i < cfg_.numSensors; ++i) {
        phases[i] = std::arg(snapshot(i));      // аргумент комплексного
        powers[i] = std::norm(snapshot(i));     // квадрат модуля комплексного
    }

    float weightedDelta = 0.0f;
    float totalWeight = 0.0f;

    for(int i = 0; i < cfg_.numSensors - 1; ++i) {
        float delta = phases[i+1] - phases[i];
        while(delta > M_PI) delta -= 2.0f * M_PI;
        while(delta < -M_PI) delta += 2.0f * M_PI;

        float weight = std::sqrt(powers[i] * powers[i+1]);
        weightedDelta += delta * weight;
        totalWeight += weight;
    }

    if(totalWeight < 1e-9f) return 0.0f;

    float avgDelta = weightedDelta / totalWeight;

    // ИСПРАВЛЕНО: используем sensorSpacing[i] для КАЖДОЙ пары?
    // Нет — для Phase Diff нужно ЕДИНОЕ расстояние, берём среднее
    float avgSpacing = 0.0f;
    for(int i = 0; i < cfg_.numSensors - 1; ++i) {
        avgSpacing += cfg_.sensorSpacing[i];
    }
    avgSpacing /= (cfg_.numSensors - 1);

    float denominator = 2.0f * M_PI * freq * avgSpacing;
    float sinTheta = (avgDelta * cfg_.soundSpeed) / denominator;
    sinTheta = std::clamp(sinTheta, -1.0f, 1.0f);

    return std::asin(sinTheta) * 180.0f / M_PI;
}

std::vector<float> DOAEngine::scanAngles(const std::function<float(const Eigen::VectorXcf&)>& spectrumFunc, float freq) {
    const int ANGLE_STEPS = 181;
    std::vector<float> out(ANGLE_STEPS, 0.0f);

    float k = 2.0f * M_PI * freq / cfg_.soundSpeed;

    auto steeringVector = [&](float thetaDeg) {
        float thetaRad = thetaDeg * M_PI / 180.0f;
        float sinTheta = std::sin(thetaRad);
        Eigen::VectorXcf a(cfg_.numSensors);
        for(int i = 0; i < cfg_.numSensors; ++i) {
            // Используем avgSensorSpacing (среднее расстояние) для MUSIC/Capon/EV
            float phase = k * cfg_.avgSensorSpacing * i * sinTheta;
            a(i) = std::exp(std::complex<float>(0.0f, phase));
        }
        return a.normalized();
    };

    float maxVal = -1e9f;
    int maxIdx = 0;
    float minVal = 1e9f;
    int minIdx = 0;

    for(int i = 0; i < ANGLE_STEPS; ++i) {
        float theta = -90.0f + i;
        Eigen::VectorXcf a = steeringVector(theta);
        out[i] = spectrumFunc(a);
        if(out[i] > maxVal) {
            maxVal = out[i];
            maxIdx = i;
        }
        if(out[i] < minVal) {
            minVal = out[i];
            minIdx = i;
        }
    }

    // ДИАГНОСТИКА (только первые 5 вызовов)
    static int callCount = 0;
    if(callCount++ < 5) {
        std::cout << "=== DIAGNOSTIC ===" << std::endl;
        std::cout << "freq = " << freq << " Hz" << std::endl;
        std::cout << "MAX at angle = " << (-90.0f + maxIdx) << "°, value = " << maxVal << std::endl;
        std::cout << "MIN at angle = " << (-90.0f + minIdx) << "°, value = " << minVal << std::endl;
        std::cout << "maxVal/minVal ratio = " << (maxVal / minVal) << std::endl;
    }

    if(maxVal > 1e-9f) {
        for(float& v : out) v /= maxVal;
    }

    return out;
}

float DOAEngine::musicMethod(float freq) {
    if(snapshotHistory_.size() < cfg_.numSensors * 2) return 0.0f;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXcf> solver(covMatrix_);
    if(solver.info() != Eigen::Success) return 0.0f;

    int numSources = 1;
    int noiseDim = cfg_.numSensors - numSources;
    if(noiseDim <= 0) return 0.0f;

    // Берём векторы с НАИБОЛЬШИМИ собственными значениями (пробуем так)
    Eigen::MatrixXcf En = solver.eigenvectors().leftCols(noiseDim);

    auto spectrumFunc = [&](const Eigen::VectorXcf& a) -> float {
        Eigen::MatrixXcf temp = a.adjoint() * En * En.adjoint() * a;
        float projection = std::real(temp.coeff(0,0));
        if(projection < 1e-12f) return 0.0f;
        return 1.0f / projection;
    };

    auto spectrum = scanAngles(spectrumFunc, freq);
    int bestIdx = std::distance(spectrum.begin(), std::max_element(spectrum.begin(), spectrum.end()));
    return -90.0f + bestIdx;
}

float DOAEngine::caponMethod(float freq) {
    if(snapshotHistory_.size() < cfg_.numSensors * 2) return 0.0f;

    // Добавляем небольшую регуляризацию для устойчивости
    Eigen::MatrixXcf R_reg = covMatrix_ + 1e-3f * Eigen::MatrixXcf::Identity(cfg_.numSensors, cfg_.numSensors);
    Eigen::MatrixXcf R_inv = R_reg.inverse();

    auto spectrumFunc = [&](const Eigen::VectorXcf& a) -> float {
        std::complex<float> temp = (a.adjoint() * R_inv * a).coeff(0,0);
        float denominator = std::real(temp);
        return 1.0f / (denominator + 1e-6f);
    };

    auto spectrum = scanAngles(spectrumFunc, freq);
    int bestIdx = std::distance(spectrum.begin(), std::max_element(spectrum.begin(), spectrum.end()));
    return -90.0f + bestIdx;
}

float DOAEngine::evMethod(float freq) {
    if(snapshotHistory_.size() < cfg_.numSensors * 2) return 0.0f;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXcf> solver(covMatrix_);
    if(solver.info() != Eigen::Success) return 0.0f;

    int numSources = 1;
    int noiseDim = cfg_.numSensors - numSources;
    if(noiseDim <= 0) return 0.0f;

    // EV метод использует шумовое подпространство (наименьшие с.з.)
    Eigen::MatrixXcf En = solver.eigenvectors().rightCols(noiseDim);

    auto spectrumFunc = [&](const Eigen::VectorXcf& a) -> float {
        float sum = 0.0f;
        for(int j = 0; j < noiseDim; ++j) {
            float lambda = std::real(solver.eigenvalues()(j + numSources));
            if(lambda > 1e-9f) {
                std::complex<float> proj = (a.adjoint() * En.col(j)).coeff(0,0);
                sum += std::norm(proj) / lambda;
            }
        }
        return 1.0f / (sum + 1e-6f);
    };

    auto spectrum = scanAngles(spectrumFunc, freq);
    int bestIdx = std::distance(spectrum.begin(), std::max_element(spectrum.begin(), spectrum.end()));
    return -90.0f + bestIdx;
}

std::map<DOAMethod, BearingResult> DOAEngine::process(const std::vector<kiss_fft_cpx>* specs, float fs, int peakBin) {
    std::map<DOAMethod, BearingResult> results;

    // Текущий снапшот — вектор комплексных чисел (8 каналов)
    Eigen::VectorXcf snapshot(cfg_.numSensors);
    float maxPower = 0.0f;
    for(int ch = 0; ch < cfg_.numSensors; ++ch) {
        snapshot(ch) = std::complex<float>(specs[ch][peakBin].r, specs[ch][peakBin].i);
        maxPower += std::norm(snapshot(ch));
    }

    if(maxPower < 100.0f) return results;

//    float freq = peakBin * fs / (float)FFT_SIZE;
    float freq = peakBin * fs / (float)fftSize_;    // !!! 1 из 3 при исправлении азимута - заменить

    // Обновление ковариационной матрицы (комплексная)
    updateCovariance(specs, peakBin, fs);

    // Phase Diff (использует комплексный снапшот)
    float pdAngle = phaseDifferenceMethod(snapshot, freq);

    bool enoughData = (snapshotHistory_.size() >= cfg_.numSensors * 2);

    BearingResult pdRes;
    pdRes.isValid = true;
    pdRes.azimuthDeg = pdAngle;
    pdRes.confidence = std::min(1.0f, std::sqrt(maxPower) / 1000.0f);
    pdRes.peakFreq = freq;
    results[DOAMethod::PhaseDiff] = pdRes;

    if(enoughData) {
        float musicAngle = musicMethod(freq);
        float caponAngle = caponMethod(freq);
        float evAngle = evMethod(freq);

        auto validate = [&](float angle) -> float {
            if(std::isnan(angle) || std::abs(angle) > 90.1f) return pdAngle;
            return angle;
        };

        // Правильный порядок: azimuthDeg, confidence, peakFreq, isValid
        results[DOAMethod::MUSIC] = {validate(musicAngle), 0.9f, freq, true};
        results[DOAMethod::Capon] = {validate(caponAngle), 0.9f, freq, true};
        results[DOAMethod::EV] = {validate(evAngle), 0.9f, freq, true};
    } else {
        // Правильный порядок: azimuthDeg, confidence, peakFreq, isValid
        BearingResult fallback = {pdAngle, 0.3f, freq, true};
        results[DOAMethod::MUSIC] = fallback;
        results[DOAMethod::Capon] = fallback;
        results[DOAMethod::EV] = fallback;
    }

    return results;
}