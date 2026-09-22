#include "serial_reader.h"
#include <windows.h>
#include <iostream>
#include <sstream>
#include <iomanip>

SerialReader::SerialReader() {}
SerialReader::~SerialReader() { stop(); }

bool SerialReader::start(const std::string& portName, uint32_t baudRate, PacketCallback callback) {
    if (running_) return false;
    portName_ = portName;
    baudRate_ = baudRate;
    onPacket_ = callback;
    running_ = true;
    worker_ = std::thread(&SerialReader::runLoop, this);
    return true;
}

void SerialReader::stop() {
    if (!running_) return;
    running_ = false;
    if (worker_.joinable()) worker_.join();
}

void SerialReader::runLoop() {
    HANDLE hSerial = CreateFileA(portName_.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "[IO] Failed to open port: " << portName_ << std::endl;
        running_ = false;
        return;
    }

    DCB dcb = { sizeof(DCB) };
    if (!GetCommState(hSerial, &dcb)) { CloseHandle(hSerial); running_ = false; return; }
    dcb.BaudRate = baudRate_;
    dcb.ByteSize = 8; dcb.StopBits = ONESTOPBIT; dcb.Parity = NOPARITY;
    dcb.fBinary = TRUE; dcb.fDtrControl = DTR_CONTROL_ENABLE; dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(hSerial, &dcb)) { CloseHandle(hSerial); running_ = false; return; }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 10;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.ReadTotalTimeoutConstant = 10;
    SetCommTimeouts(hSerial, &timeouts);

    std::cout << "[IO] Connected to " << portName_ << " at " << baudRate_ << " baud" << std::endl;

    while (running_) {
    // uint8_t buffer[4096];  // Большой буфер для пачки
    uint8_t buffer[8192];
    DWORD bytesRead = 0;

    if (!ReadFile(hSerial, buffer, sizeof(buffer), &bytesRead, nullptr) || bytesRead == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
    }

    // ✅ ДИАГНОСТИКА: первые 20 байт из буфера (каждые 50 итераций)
    static int dumpCounter = 0;
    if (dumpCounter++ % 50 == 0 && bytesRead > 0) {
        std::cout << "[DIAG] bytesRead=" << bytesRead << " | HEX: ";
        for (DWORD i = 0; i < std::min(bytesRead, (DWORD)20); ++i) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i] << " ";
        }
        std::cout << std::dec << std::endl;
    }

    // Обрабатываем все байты из пачки
    for (DWORD i = 0; i < bytesRead; ++i) {
        uint8_t byte = buffer[i];

        // Ищем начало пакета: '#'
        // if (byte != 0x23) continue;
        if (byte != 0xFF) continue;     // Ищем начало пакета: 'FF'

        // Проверяем второй байт '#'
        if (i + 1 >= bytesRead) {
            // Если '#', но второй байт в конце буфера — читаем его
            uint8_t next = 0;
            DWORD read = 0;
            // if (!ReadFile(hSerial, &next, 1, &read, nullptr) || read == 0 || next != 0x23) { continue; }
            if (!ReadFile(hSerial, &next, 1, &read, nullptr) || read == 0 || next != 0xFF) { continue; }
            i += 1;  // пропускаем обработанный байт
        } else {
            // if (buffer[i + 1] != 0x23) continue;
            if (buffer[i + 1] != 0xFF) continue;
            i += 1;  // пропускаем второй '#'
        }

        // Теперь читаем 17 байт данных.
        // ВАЖНО: они могут быть как в буфере, так и за его пределами.
        uint8_t raw[17];
        DWORD total = 0;

        // Сначала берём из буфера
        while (total < 17 && (i + 1 + total) < bytesRead) {
            raw[total] = buffer[i + 1 + total];
            total++;
        }

        // Если остались байты — читаем из порта
        while (total < 17 && running_) {
            DWORD r = 0;
            if (!ReadFile(hSerial, raw + total, 17 - total, &r, nullptr)) break;
            total += r;
        }

        if (total != 17) continue;

        // --- Формируем пакет ---
        DataPacket pkt;
        pkt.isValid = true;
        uint8_t cs = 0;
        std::ostringstream hex;
        hex << "## ";
        for (int j = 0; j < 16; ++j) {
            cs += raw[j];
            hex << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)raw[j] << " ";
        }
        pkt.checksum = raw[16];
        hex << "| CS: " << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)pkt.checksum;
        pkt.rawHex = hex.str();

        if ((cs & 0xFF) != pkt.checksum) {
            pkt.isValid = false;
        } else {
            for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
                pkt.values[ch] = (static_cast<uint16_t>(raw[ch * 2]) << 8) | raw[ch * 2 + 1];
            }
        }

        if (onPacket_) onPacket_(pkt);
    }
}
    CloseHandle(hSerial);
    std::cout << "[IO] Port closed." << std::endl;
}