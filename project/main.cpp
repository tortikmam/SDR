// GUI добавлен вокруг исходного алгоритма main.cpp.
// Передающий формирующий фильтр, RX-свёртка и офлайн-Гарднер сохранены.
#include <atomic>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <deque>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>

#include "SoapySDR/Device.h"
#include "SoapySDR/Formats.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <complex>
#include <new>
#include <math.h>

using Complex = std::complex<double>;

#define N 10
#define L 10

#define H_SIZE L
#define UPS_SIZE ((N - 1) * L + 1)
#define Y_SIZE (UPS_SIZE + H_SIZE - 1)

// Частоты для разных режимов
#define TX_FREQUENCY 800e6
#define RX_FREQUENCY 840e6
#define SAMPLE_RATE  1e6


// ==================== СОСТОЯНИЕ UI И SDR ====================
struct Status {
    std::atomic<bool> running{true};
    std::atomic<bool> device_connected{false};
    std::atomic<bool> is_tx{false};
    std::atomic<double> rx_gain{40.0};
    std::atomic<double> tx_gain{-10.0};
    std::atomic<double> tx_amplitude{3200.0};
    std::atomic<bool> gardner_enabled{true};
    std::atomic<size_t> rx_packets{0};
    std::atomic<size_t> tx_packets{0};
    std::mutex status_mutex;
    std::string message = "Starting PlutoSDR...";
    std::deque<std::string> log;
};
static Status g_status;

static void set_status(const std::string& value) {
    std::lock_guard<std::mutex> lock(g_status.status_mutex);
    g_status.message = value;
    g_status.log.push_back(value);
    while (g_status.log.size() > 100) g_status.log.pop_front();
}
static void on_sigint(int) { g_status.running = false; }

static void apply_live_gain(SoapySDRDevice* sdr, int direction, double& applied) {
    const double requested = direction == SOAPY_SDR_TX ? g_status.tx_gain.load() : g_status.rx_gain.load();
    if (std::fabs(requested - applied) > 0.05) {
        int rc = SoapySDRDevice_setGain(sdr, direction, 0, requested);
        if (rc == 0) applied = requested;
        else set_status(std::string("setGain failed: ") + SoapySDRDevice_lastError());
    }
}

// ==================== LIVE DSP ДЛЯ ГРАФИКОВ ====================
// Прямоугольный согласованный фильтр RX повторяет алгоритм старой свёртки,
// но реализован поточно (сохраняет 10 последних отсчётов между пакетами).
// Отдельный исходный офлайн-алгоритм ниже по-прежнему пишет symbols.txt.
static constexpr int kFftSize = 256;
static constexpr int kWaterfallRows = 128;
static constexpr int kSymbolCapacity = 1024;
static constexpr int kFilterRing = 128;

static void fft_radix2(std::array<Complex, kFftSize>& a) {
    for (int i = 1, j = 0; i < kFftSize; i++) {
        int bit = kFftSize >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    const double pi = 3.14159265358979323846;
    for (int len = 2; len <= kFftSize; len <<= 1) {
        double angle = -2.0 * pi / len;
        Complex wlen(std::cos(angle), std::sin(angle));
        for (int i = 0; i < kFftSize; i += len) {
            Complex w(1.0, 0.0);
            for (int j = 0; j < len / 2; j++) {
                Complex u = a[i + j], v = a[i + j + len / 2] * w;
                a[i + j] = u + v;
                a[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

struct PlotSnapshot {
    std::vector<double> ms, raw_i, raw_q, filt_i, filt_q;
    std::vector<double> sym_i, sym_q;
    std::vector<float> waterfall_db;
    double peak = 0.0, rms = 0.0, timing_error = 0.0, strobe_phase = 0.0;
    size_t received_samples = 0, symbols = 0;
    int waterfall_filled = 0;
    bool has_rx = false;
};

class StreamMonitor {
public:
    StreamMonitor() : waterfall_(kWaterfallRows * kFftSize, -120.0f) {}

    void on_samples(const int16_t* data, size_t n, bool is_rx) {
        if (n == 0) return;
        std::vector<double> x_ms, ri, rq, fi, fq;
        x_ms.reserve(n); ri.reserve(n); rq.reserve(n); fi.reserve(n); fq.reserve(n);
        double pmax = 0, power = 0;
        std::vector<Complex> new_symbols;
        new_symbols.reserve(n / L + 4);
        const double norm = 32768.0;

        // Поточный RX-фильтр + те же три точки Гарднера (I-channel), что и в main.cpp.
        for (size_t i = 0; i < n; ++i) {
            Complex raw((double)data[2 * i], (double)data[2 * i + 1]);
            x_ms.push_back((double)i * 1e3 / SAMPLE_RATE);
            ri.push_back(raw.real()); rq.push_back(raw.imag());
            pmax = std::max(pmax, std::max(std::fabs(raw.real()), std::fabs(raw.imag())));
            power += std::norm(raw);

            Complex filt = raw;
            if (is_rx) {
                size_t hist_index = sample_index_ % L;
                filter_sum_ += raw - filter_hist_[hist_index];
                filter_hist_[hist_index] = raw;
                filt = filter_sum_;
                filter_ring_[sample_index_ % kFilterRing] = filt;
                // После поступления поздней точки можно завершить очередной символ.
                while (sample_index_ >= 2 * L &&
                       next_symbol_ + (L / 2.0) + 1.0 <= (double)sample_index_) {
                    const Complex early = interpolate(next_symbol_ - L / 2.0);
                    const Complex center = interpolate(next_symbol_);
                    const Complex late = interpolate(next_symbol_ + L / 2.0);
                    const double e = (center.real() / norm) *
                                     ((late.real() - early.real()) / norm);
                    latest_error_ = e;
                    new_symbols.push_back(center / norm);
                    symbol_count_++;
                    next_symbol_ += L - (g_status.gardner_enabled.load() ? 0.02 * e : 0.0);
                    // Clamp extreme time corrections caused by clipped/noisy data.
                    if (next_symbol_ < (double)sample_index_ - 3.0 * L)
                        next_symbol_ = (double)sample_index_ - L;
                }
                sample_index_++;
            }
            // На графике обе ветки приведены к близкому масштабу: RX filter / L.
            fi.push_back(filt.real() / (is_rx ? L : 1));
            fq.push_back(filt.imag() / (is_rx ? L : 1));
        }

        std::lock_guard<std::mutex> lock(mutex_);
        view_.ms = std::move(x_ms);
        view_.raw_i = std::move(ri);
        view_.raw_q = std::move(rq);
        view_.filt_i = std::move(fi);
        view_.filt_q = std::move(fq);
        view_.peak = pmax;
        view_.rms = std::sqrt(power / (double)n);
        view_.has_rx = is_rx;
        if (is_rx) {
            view_.received_samples += n;
            view_.timing_error = latest_error_;
            view_.strobe_phase = std::fmod(next_symbol_, L);
            view_.symbols = symbol_count_;
            for (const auto& symbol : new_symbols) {
                sym_i_.push_back(symbol.real());
                sym_q_.push_back(symbol.imag());
            }
            while ((int)sym_i_.size() > kSymbolCapacity) {
                sym_i_.pop_front(); sym_q_.pop_front();
            }
        } else {
            // Чтобы полярный график оставался полезным и в режиме TX.
            for (size_t i = 0; i < n; i += L) {
                sym_i_.push_back((double)data[2*i] / 32768.0);
                sym_q_.push_back((double)data[2*i+1] / 32768.0);
            }
            while ((int)sym_i_.size() > kSymbolCapacity) {
                sym_i_.pop_front(); sym_q_.pop_front();
            }
        }
        view_.sym_i.assign(sym_i_.begin(), sym_i_.end());
        view_.sym_q.assign(sym_q_.begin(), sym_q_.end());

        // Waterfall: STFT FFT по 256 IQ-сэмплам, каждые 8 буферов.
        // Отдельную кривую спектра не строим (как просил пользователь).
        if (++fft_buffer_counter_ % 8 == 0 && n >= kFftSize) {
            std::array<Complex, kFftSize> spectrum{};
            const double pi = 3.14159265358979323846;
            for (int i = 0; i < kFftSize; i++) {
                double hann = 0.5 - 0.5 * std::cos(2.0 * pi * i / (kFftSize - 1));
                spectrum[i] = Complex(data[2*i], data[2*i+1]) * (hann / 32768.0);
            }
            fft_radix2(spectrum);
            for (int j = 0; j < kFftSize; j++) {
                int centered = (j + kFftSize/2) % kFftSize;
                double magnitude = std::abs(spectrum[centered]) / kFftSize;
                waterfall_[(waterfall_head_ % kWaterfallRows) * kFftSize + j] =
                    (float)(20.0 * std::log10(magnitude + 1e-12));
            }
            waterfall_head_++;
            view_.waterfall_filled = std::min(waterfall_head_, kWaterfallRows);
        }
    }

    PlotSnapshot snapshot() {
        std::lock_guard<std::mutex> lock(mutex_);
        PlotSnapshot cp = view_;
        // Chronological rows; newest at the bottom.
        cp.waterfall_db.resize(kWaterfallRows * kFftSize);
        const int first = waterfall_head_ >= kWaterfallRows ?
                waterfall_head_ % kWaterfallRows : 0;
        for (int r = 0; r < kWaterfallRows; ++r) {
            int row = (first + r) % kWaterfallRows;
            std::copy_n(waterfall_.data() + row*kFftSize, kFftSize,
                        cp.waterfall_db.data() + r*kFftSize);
        }
        return cp;
    }

private:
    Complex interpolate(double t) const {
        if (t < 0) return Complex(0, 0);
        const size_t first = (size_t)std::floor(t);
        const double f = t - (double)first;
        return filter_ring_[first % kFilterRing] * (1.0-f) +
               filter_ring_[(first+1) % kFilterRing] * f;
    }
    std::mutex mutex_;
    PlotSnapshot view_;
    std::deque<double> sym_i_, sym_q_;
    std::array<Complex, L> filter_hist_{};
    std::array<Complex, kFilterRing> filter_ring_{};
    Complex filter_sum_{0,0};
    size_t sample_index_ = 0;
    double next_symbol_ = L / 2.0;
    double latest_error_ = 0.0;
    size_t symbol_count_ = 0;
    std::vector<float> waterfall_;
    int waterfall_head_ = 0;
    int fft_buffer_counter_ = 0;
};
static StreamMonitor g_monitor;

static int sdr_main(int argc, char *argv[])
{
    // --- Проверка аргумента командной строки ---
    if (argc < 2)
    {
        printf("Usage: %s rx\n", argv[0]);
        printf("  tx - передатчик (%.0f MHz)\n", TX_FREQUENCY / 1e6);
        printf("  rx - приёмник   (%.0f MHz)\n", RX_FREQUENCY / 1e6);
        return EXIT_FAILURE;
    }

    int is_tx;
    if (strcmp(argv[1], "tx") == 0)
    {
        is_tx = 1;
        g_status.is_tx = true;
        printf("Mode: TRANSMITTER, freq = %.0f MHz\n", TX_FREQUENCY / 1e6);
    }
    else if (strcmp(argv[1], "rx") == 0)
    {
        is_tx = 0;
        g_status.is_tx = false;
        printf("Mode: RECEIVER, freq = %.0f MHz\n", RX_FREQUENCY / 1e6);
    }
    else
    {
        printf("Unknown mode '%s'. Use 'tx' or 'rx'.\n", argv[1]);
        return EXIT_FAILURE;
    }

    // ==================== ОБЩАЯ ИНИЦИАЛИЗАЦИЯ ====================
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");

    if (1) {
        SoapySDRKwargs_set(&args, "uri", "usb:");
    } else {
        SoapySDRKwargs_set(&args, "uri", "ip:192.168.2.1");
    }

    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRKwargs_set(&args, "loopback", "0");

    set_status("Connecting to PlutoSDR...");
    SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (sdr == NULL)
    {
        printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        set_status(std::string("Connection failed: ") + SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }

    // Направление: TX или RX
    int direction = is_tx ? SOAPY_SDR_TX : SOAPY_SDR_RX;
    double freq = is_tx ? TX_FREQUENCY : RX_FREQUENCY;

    // Настройка частоты и sample rate
    if (SoapySDRDevice_setSampleRate(sdr, direction, 0, SAMPLE_RATE) != 0)
    {
        printf("setSampleRate fail: %s\n", SoapySDRDevice_lastError());
    }

    if (SoapySDRDevice_setFrequency(sdr, direction, 0, freq, NULL) != 0)
    {
        printf("setFrequency fail: %s\n", SoapySDRDevice_lastError());
    }

    printf("Frequency: %lf Hz\n",
           SoapySDRDevice_getFrequency(sdr, direction, 0));

    // Усиление
    double gain_val = is_tx ? -10.0 : 40.0;

    if (SoapySDRDevice_setGain(sdr, direction, 0, gain_val) != 0)
    {
        printf("setGain fail: %s\n", SoapySDRDevice_lastError());
    }

    // Создание потока
    size_t channels[] = {0};
    size_t channel_count = sizeof(channels) / sizeof(channels[0]);

    SoapySDRStream *stream = SoapySDRDevice_setupStream(
        sdr, direction, SOAPY_SDR_CS16,
        channels, channel_count, NULL
    );

    if (stream == NULL)
    {
        printf("setupStream fail: %s\n", SoapySDRDevice_lastError());
        set_status(std::string("Stream setup failed: ") + SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    size_t mtu = 960;
    printf("MTU buffer size: %lu samples\n", mtu);

    SoapySDRDevice_activateStream(sdr, stream, 0, 0, 0);
    set_status("PlutoSDR connected: streaming");
    g_status.device_connected = true;

    const long timeoutUs = 400000;

        // ==================== РЕЖИМ ПЕРЕДАТЧИКА ====================
    if (is_tx)
    {
        printf("Генерация сигнала...\n");
        
        // 1. Исходные данные (BPSK: 1.0 или -1.0)
        Complex arr[N] = {
            Complex(1.0, 0.0), Complex(0.0, 0.0), Complex(0.0, 0.0), Complex(1.0, 0.0),
            Complex(1.0, 0.0), Complex(0.0, 0.0), Complex(0.0, 0.0), Complex(1.0, 0.0),
            Complex(1.0, 0.0), Complex(1.0, 0.0)
        };

        Complex arr_mod[N] = {0};
        for (int i = 0; i < N; i++) {
            arr_mod[i] = (std::real(arr[i]) > 0.5) ? 1.0 : -1.0; // Корректная BPSK модуляция
        }

        // 2. Вставка нулей (Upsampling)
        Complex upsampled[UPS_SIZE] = {0};
        for (int i = 0; i < N; i++) {
            upsampled[i * L] = arr_mod[i];
        }

        // 3. Прямоугольный формирующий фильтр
        Complex h[H_SIZE];
        for (int


i = 0; i < H_SIZE; i++) {
            h[i] = Complex(1.0, 0.0);
        }

        // 4. Свёртка
        Complex y[Y_SIZE] = {0};
        for (int n = 0; n < Y_SIZE; n++) {
            for (int k = 0; k < UPS_SIZE; k++) {
                int j = n - k;
                if (j >= 0 && j < H_SIZE) {
                    y[n] += upsampled[k] * h[j];
                }
            }
        }

        int16_t tx_buff[1920]; // 960 комплексных отсчетов (I и Q)
        void *tx_buffs[] = {tx_buff};

        // Масштабирование для максимального использования ЦАП PlutoSDR (макс ~32000)
        // Максимальное значение y[n] здесь равно 10.0. 10.0 * 3200 = 32000.
        // Изменяемый через ImGui коэффициент амплитуды, начальное значение прежнее (3200).
        double scale_factor = g_status.tx_amplitude.load(); 

        size_t pos = 0;
        long long current_time_ns = 0;
        // Время одного пакета в наносекундах: (кол-во отсчетов / частота дискретизации) * 1e9
        long long packet_time_ns = (long long)((double)mtu / SAMPLE_RATE * 1e9);

        printf("Начало передачи (500 пакетов)...\n");

        for (int packet = 0; g_status.running.load(); packet++)
        {
            scale_factor = g_status.tx_amplitude.load();
            apply_live_gain(sdr, direction, gain_val);
            // Заполняем ВЕСЬ буфер непрерывно, без пропусков на "тишину"
            for (size_t i = 0; i < mtu; i++)
            {
                tx_buff[2 * i]     = (int16_t)(std::real(y[pos]) * scale_factor);
                tx_buff[2 * i + 1] = (int16_t)(std::imag(y[pos]) * scale_factor);

                pos++;
                if (pos >= Y_SIZE) {
                    pos = 0; // Плавный циклический повтор сигнала
                }
            }

            // Правильная отправка с таймстампом (как в коде одногруппника)
            g_monitor.on_samples(tx_buff, mtu, false);
            g_status.tx_packets++;

            int flags = SOAPY_SDR_HAS_TIME;
            long long tx_time = current_time_ns + (4 * 1000 * 1000); // Задержка 4 мс для стабильности

            int st = SoapySDRDevice_writeStream(
                sdr, stream,
                (const void * const *)tx_buffs,
                mtu, &flags, tx_time, timeoutUs
            );

            if (st < 0)
            {
                printf("TX error: %s\n", SoapySDR_errToStr(st));
                set_status(std::string("TX stream error: ") + SoapySDR_errToStr(st));
                break;
            }

            // Сдвигаем время на длину текущего пакета для следующего цикла
            current_time_ns += packet_time_ns;

            if (packet % 50 == 0) {
                printf("Отправлено пакетов: %d\n", packet);
            }
        }
        printf("Передача завершена.\n");
    }

    // ==================== РЕЖИМ ПРИЁМНИКА ====================
    else
    {
        int16_t *rx_buffer =
            (int16_t *)malloc(2 * mtu * sizeof(int16_t));

        if (!rx_buffer)
        {
            printf("Memory allocation failed\n");
            SoapySDRDevice_deactivateStream(sdr, stream, 0, 0);
            SoapySDRDevice_closeStream(sdr, stream);
            SoapySDRDevice_unmake(sdr);
            return EXIT_FAILURE;
        }

        FILE *fp_out = fopen("rx_recv.pcm", "wb");

        if (!fp_out)
        {
            printf("Error opening file for writing\n");
            free(rx_buffer);
            SoapySDRDevice_deactivateStream(sdr, stream, 0, 0);
            SoapySDRDevice_closeStream(sdr, stream);
            SoapySDRDevice_unmake(sdr);
            return EXIT_FAILURE;
        }

        printf("Streaming started. Waiting for signal...\n");

        int flags;
        long long timeNs;
        size_t total_samples = 0;
        const size_t iteration_limit = 5000;

        for (size_t i = 0; g_status.running.load(); i++)
        {
            void *buffs[] = {rx_buffer};

            int sr = SoapySDRDevice_readStream(
                sdr, stream, buffs, mtu,
                &flags, &timeNs, timeoutUs
            );

            if (sr >= 0)
            {
                apply_live_gain(sdr, direction, gain_val);
                if (i < iteration_limit) {
                size_t written = fwrite(
                    rx_buffer,
                    2 * sizeof(int16_t),
                    sr,
                    fp_out
                );

                if (written != (size_t)sr)
                {
                    printf("File write error\n");
                    break;
                }

                total_samples += sr;
                } // После первоначальных 5000 буферов останавливаем только запись, но не live RX.
                if (sr > 0) g_monitor.on_samples(rx_buffer, (size_t)sr, true);
                g_status.rx_packets++;

                if (i % 50 == 0)
                {


printf("Packet #%zu/%zu | Samples: %d | Total: %zu\n",
                           i + 1, iteration_limit, sr, total_samples);
                }
            }
            else if (sr == SOAPY_SDR_TIMEOUT)
            {
                continue;
            }
            else
            {
                printf("Read error code: %d (%s)\n",
                       sr, SoapySDR_errToStr(sr));
                set_status(std::string("RX stream error: ") + SoapySDR_errToStr(sr));
                break;
            }
        }

        fclose(fp_out);
        set_status("RX stopped; offline convolution and symbol synchronization...");

        printf("Reception finished. Saved %zu complex samples to 'rx_recv.pcm'\n",
               total_samples);

        free(rx_buffer);

        // ================= СИМВОЛЬНАЯ СИНХРОНИЗАЦИЯ =================

        FILE *fp = fopen("rx_recv.pcm", "rb");

        if (fp == NULL)
        {
            printf("Error opening RX file\n");
        }
        else
        {
            int16_t *samples =
                (int16_t *)malloc(2 * total_samples * sizeof(int16_t));

            if (samples == NULL)
            {
                printf("Memory allocation failed\n");
                fclose(fp);
            }
            else
            {
                fread(samples, 2 * sizeof(int16_t),
                      total_samples, fp);

                fclose(fp);

                // ================= СВЁРТКА НА ПРИЁМНИКЕ =================
                // Такой же прямоугольный фильтр, как на передатчике.
                Complex h_rx[H_SIZE];
                for (int i = 0; i < H_SIZE; i++) {
                    h_rx[i] = Complex(1.0, 0.0);
                }

                // Свёртка комплексного принятого сигнала с h_rx.
                size_t rx_y_size = total_samples + H_SIZE - 1;
                Complex *y_rx = new (std::nothrow) Complex[rx_y_size]();
                if (y_rx == NULL)
                {
                    printf("Memory allocation failed (RX convolution)\n");
                    free(samples);
                    SoapySDRDevice_deactivateStream(sdr, stream, 0, 0);
                    SoapySDRDevice_closeStream(sdr, stream);
                    SoapySDRDevice_unmake(sdr);
                    return EXIT_FAILURE;
                }

                for (size_t n = 0; n < rx_y_size; n++) {
                    for (int k = 0; k < H_SIZE; k++) {
                        if (n >= (size_t)k && n - (size_t)k < total_samples) {
                            size_t j = n - (size_t)k;
                            Complex x((double)samples[2 * j],
                                      (double)samples[2 * j + 1]);
                            y_rx[n] += x * h_rx[k];
                        }
                    }
                }

                int L_sync = L;
                double mu = 0.02;

                // Начальный момент выборки
                double t = L_sync / 2.0;

                FILE *fp_sync = fopen("symbols.txt", "w");

                if (fp_sync == NULL)
                {
                    printf("Error creating symbols file\n");
                }
                else
                {
                    int symbol_count = 0;

                    while (t + L_sync / 2.0 + 1 < total_samples)
                    {
                        // Индексы трёх точек
                        int ic = (int)t;
                        int ie = (int)(t - L_sync / 2.0);
                        int il = (int)(t + L_sync / 2.0);

                        // Дробные части времени
                        double fc = t - ic;
                        double fe = t - L_sync / 2.0 - ie;
                        double fl = t + L_sync / 2.0 - il;

                        // Центральная точка
                        double center =
                            (1.0 - fc) * std::real(y_rx[ic]) +
                            fc * std::real(y_rx[ic + 1]);

                        // Ранняя точка
                        double early =
                            (1.0 - fe) * std::real(y_rx[ie]) +
                            fe * std::real(y_rx[ie + 1]);

                        // Поздняя точка
                        double late =
                            (1.0 - fl) * std::real(y_rx[il]) +
                            fl * std::real(y_rx[il + 1]);

                        // Нормировка
                        center /= 32768.0;
                        early /= 32768.0;
                        late /= 32768.0;

                        // Ошибка синхронизации
                        double e = center * (late - early);

                        // Запись синхронизированного символа
                        if (fabs(center) <= 0.003) {
                            fprintf(fp_sync, "%lf\n", center);
                        }

                        // Коррекция момента выборки
                        t += L_sync - mu * e;

                        symbol_count++;
                    }

                    fclose(fp_sync);

                    printf("Symbol synchronization finished!\n");
                    printf("Recovered symbols: %d\n", symbol_count);
                    printf("Saved to symbols.txt\n");
                }

                delete[] y_rx;
                free(samples);
            }
        }
    }

    // ==================== ЗАВЕРШЕНИЕ ====================
    SoapySDRDevice_deactivateStream(sdr, stream, 0, 0);
    SoapySDRDevice_closeStream(sdr, stream);
    SoapySDRDevice_unmake(sdr);

    g_status.device_connected = false;
    set_status("SDR finished");
    printf("Done!\n");

    return EXIT_SUCCESS;
}
// ==================== ПАНЕЛИ ImGui / ImPlot ====================
static void configure_theme() {
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 10.0f;
    style.ChildRounding = 10.0f;
    style.FrameRounding = 6.0f;
    style.GrabRounding = 5.0f;
    style.WindowPadding = ImVec2(14, 12);
    style.FramePadding = ImVec2(9, 6);
    style.ItemSpacing = ImVec2(10, 9);
    ImVec4* c = style.Colors;
    c[ImGuiCol_WindowBg]   = ImVec4(0.050f, 0.063f, 0.088f, 1.0f);
    c[ImGuiCol_ChildBg]    = ImVec4(0.083f, 0.107f, 0.147f, 1.0f);
    c[ImGuiCol_FrameBg]    = ImVec4(0.14f, 0.17f, 0.22f, 1.0f);
    c[ImGuiCol_Button]     = ImVec4(0.11f, 0.35f, 0.43f, 1.0f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.13f, 0.53f, 0.62f, 1.0f);
    c[ImGuiCol_CheckMark] = ImVec4(0.20f, 0.83f, 0.82f, 1.0f);
    c[ImGuiCol_Header]     = ImVec4(0.11f, 0.35f, 0.43f, 1.0f);
}

static void draw_time_plot(const PlotSnapshot& s, bool show_matched) {
    ImGui::TextUnformatted("СИГНАЛ ВО ВРЕМЕНИ");
    ImGui::TextDisabled("Комплексные IQ-отсчёты, один последний буфер. Фильтр /10 только для масштаба.");
    if (ImPlot::BeginPlot("##time", ImVec2(-1, -1))) {
        ImPlot::SetupAxes("Время, мс", "Амплитуда, ADU");
        if (!s.ms.empty()) {
            ImPlot::PlotLine("I", s.ms.data(), s.raw_i.data(), (int)s.ms.size());
            ImPlot::PlotLine("Q", s.ms.data(), s.raw_q.data(), (int)s.ms.size());
            if (show_matched && s.has_rx) {
                ImPlot::PlotLine("I (matched /10)", s.ms.data(), s.filt_i.data(), (int)s.ms.size());
                ImPlot::PlotLine("Q (matched /10)", s.ms.data(), s.filt_q.data(), (int)s.ms.size());
            }
        }
        ImPlot::EndPlot();
    }
}

static void draw_polar_plot(const PlotSnapshot& s, double radius) {
    ImGui::TextUnformatted("ПОЛЯРНАЯ ДИАГРАММА IQ");
    ImGui::TextDisabled("Радиус = |IQ|, угол = arg(I+jQ). RX: после Гарднера, TX: передаваемые IQ.");
    if (ImPlot::BeginPlot("##polar", ImVec2(-1, -1), ImPlotFlags_Equal)) {
        ImPlot::SetupAxes("I", "Q");
        ImPlot::SetupAxesLimits(-radius, radius, -radius, radius, ImGuiCond_Always);
        constexpr int circle_n = 96;
        double angles_x[circle_n+1], angles_y[circle_n+1];
        const double pi = 3.14159265358979323846;
        for (int ring = 1; ring <= 3; ring++) {
            for (int j = 0; j <= circle_n; j++) {
                double a = 2.0*pi*j/circle_n;
                angles_x[j] = (radius*ring/3.0)*std::cos(a);
                angles_y[j] = (radius*ring/3.0)*std::sin(a);
            }
            if (ring == 1) ImPlot::PlotLine("##ring1", angles_x, angles_y, circle_n+1);
            if (ring == 2) ImPlot::PlotLine("##ring2", angles_x, angles_y, circle_n+1);
            if (ring == 3) ImPlot::PlotLine("##ring3", angles_x, angles_y, circle_n+1);
        }
        double hz[2] = {-radius, radius};
        double zero[2] = {0,0};
        ImPlot::PlotLine("##real_axis", hz, zero, 2);
        ImPlot::PlotLine("##imag_axis", zero, hz, 2);
        if (!s.sym_i.empty())
            ImPlot::PlotScatter("Символы", s.sym_i.data(), s.sym_q.data(), (int)s.sym_i.size());
        ImPlot::EndPlot();
    }
}

static void draw_waterfall(const PlotSnapshot& s, int palette, double gamma, float db_min, float db_max) {
    ImGui::TextUnformatted("WATERFALL (STFT)");
    ImGui::TextDisabled("Частота по горизонтали, история по вертикали (новые строки снизу).");
    static const ImPlotColormap colormaps[] = {
        ImPlotColormap_Viridis, ImPlotColormap_Plasma,
        ImPlotColormap_Hot, ImPlotColormap_Jet,
        ImPlotColormap_Cool, ImPlotColormap_Greys
    };
    palette = std::max(0, std::min(palette, 5));
    std::vector<double> pixels(s.waterfall_db.size(), 0.0);
    const double denom = std::max(1.0, (double)db_max - (double)db_min);
    for (size_t i = 0; i < pixels.size(); ++i) {
        double value = ((double)s.waterfall_db[i] - db_min) / denom;
        value = std::max(0.0, std::min(1.0, value));
        pixels[i] = std::pow(value, gamma); // Пользовательская гамма отображения.
    }
    ImPlot::PushColormap(colormaps[palette]);
    if (ImPlot::BeginPlot("##waterfall", ImVec2(-1, -1))) {
        ImPlot::SetupAxes("Частота относительно LO, МГц", "Время / строка");
        ImPlot::SetupAxesLimits(-0.5, 0.5, 0, kWaterfallRows, ImGuiCond_Always);
        if (pixels.size() == (size_t)(kWaterfallRows*kFftSize))
            ImPlot::PlotHeatmap("##wf_cells", pixels.data(), kWaterfallRows,
                kFftSize, 0.0, 1.0, nullptr,
                ImPlotPoint(-0.5, 0), ImPlotPoint(0.5, kWaterfallRows));
        ImPlot::EndPlot();
    }
    ImPlot::PopColormap();
}

static void draw_debug(const PlotSnapshot& s, const char* mode) {
    ImGui::TextUnformatted("ОТЛАДКА / СТАТУС SDR");
    ImGui::Separator();
    std::string status;
    std::deque<std::string> logs;
    { std::lock_guard<std::mutex> lock(g_status.status_mutex); status = g_status.message; logs = g_status.log; }
    ImGui::TextWrapped("SDR: %s", status.c_str());
    ImGui::Text("Режим: %s | Fs: %.0f kS/s | SPS: %d", mode, SAMPLE_RATE/1000.0, L);
    ImGui::Text("TX: %.0f МГц | RX: %.0f МГц", TX_FREQUENCY/1e6, RX_FREQUENCY/1e6);
    ImGui::Text("Пакеты TX / RX: %zu / %zu",
        g_status.tx_packets.load(), g_status.rx_packets.load());
    ImGui::Text("IQ всего (live): %zu | Максимум: %.0f ADU | RMS: %.1f ADU",
        s.received_samples, s.peak, s.rms);
    ImGui::Text("Выбрано символов (live): %zu", s.symbols);
    ImGui::Text("Gardner e: %+.6f | strobe: %.4f / %d", s.timing_error, s.strobe_phase, L);
    ImGui::Text("Waterfall: %d / %d строк", s.waterfall_filled, kWaterfallRows);
    ImGui::Text("GUI FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Separator();
    ImGui::TextWrapped("Оригинальные файлы RX: rx_recv.pcm (первые 5000 буферов), "
                       "symbols.txt (создаётся при завершении RX после исходной свёртки и синхронизации).");
    ImGui::TextDisabled("Спектр отдельным графиком отложен.");
    ImGui::Separator();
    ImGui::TextUnformatted("Журнал событий:");
    ImGui::BeginChild("##EventLog", ImVec2(0, 90), true);
    for (const std::string& line : logs) ImGui::TextWrapped("%s", line.c_str());
    ImGui::EndChild();
}

// Новый main: OpenGL/GLFW живёт только в UI-потоке, SoapySDR только в рабочем потоке.
int main(int argc, char* argv[]) {
    if (argc < 2 || (strcmp(argv[1], "tx") != 0 && strcmp(argv[1], "rx") != 0)) {
        printf("Usage: %s tx|rx\n", argv[0]);
        return EXIT_FAILURE;
    }
    const bool is_tx = strcmp(argv[1], "tx") == 0;
    g_status.is_tx = is_tx;
    signal(SIGINT, on_sigint);
    if (!glfwInit()) {
        fprintf(stderr, "Cannot initialize GLFW. Check DISPLAY/Wayland/X11.\n");
        return EXIT_FAILURE;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    GLFWwindow* window = glfwCreateWindow(1450, 900, "PlutoSDR | Live Signal Dashboard", nullptr, nullptr);
    if (!window) {
        fprintf(stderr, "Cannot create GLFW window.\n");
        glfwTerminate();
        return EXIT_FAILURE;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    // Кириллический шрифт для подписей на русском; запасной — встроенный ImGui.
    const char* cyrillic_font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";
    FILE* f = fopen(cyrillic_font, "rb");
    if (f) {
        fclose(f);
        io.Fonts->AddFontFromFileTTF(cyrillic_font, 15.0f, nullptr, io.Fonts->GetGlyphRangesCyrillic());
    }
    configure_theme();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    std::thread worker([&] { sdr_main(argc, argv); });

    bool show_matched = true;
    int palette = 0;
    const char* palette_names[] = {"Viridis", "Plasma", "Hot", "Jet", "Cool", "Greys"};
    float gamma = 1.0f;
    float db_min = -100.0f, db_max = -25.0f;
    float polar_range = 1.0f;
    bool running_ui = true;

    while (!glfwWindowShouldClose(window) && running_ui) {
        glfwPollEvents();
        if (!g_status.running.load()) {
            // Ctrl+C/Stop SDR завершает SDR, но UI ещё позволяет просмотреть последние данные.
            // Закрывается кнопкой «Закрыть окно» или системным крестиком.
        }
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        const PlotSnapshot s = g_monitor.snapshot();

        int width, height;
        glfwGetWindowSize(window, &width, &height);
        ImGui::SetNextWindowPos(ImVec2(0,0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2((float)width, (float)height), ImGuiCond_Always);
        const ImGuiWindowFlags app_flags = ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBringToFrontOnFocus;
        ImGui::Begin("##Dashboard", nullptr, app_flags);
        ImGui::TextColored(ImVec4(0.27f, 0.85f, 0.84f, 1.0f), "PLUTO SDR");
        ImGui::SameLine();
        ImGui::Text("  /   LIVE SIGNAL DASHBOARD");
        ImGui::SameLine();
        ImGui::TextDisabled("     %s  |  %s", is_tx ? "TX 840 MHz" : "RX 800 MHz",
                            g_status.device_connected.load() ? "CONNECTED" : "DISCONNECTED");
        ImGui::Separator();

        const float left_width = 310.0f;
        ImGui::BeginChild("##Controls", ImVec2(left_width, 0), true);
        ImGui::TextUnformatted("УПРАВЛЕНИЕ");
        ImGui::Separator();
        ImGui::Text("Режим: %s", is_tx ? "Передатчик" : "Приёмник");
        ImGui::Text("Формат: CS16  |  Fs: 1 MHz");
        ImGui::Spacing();
        if (ImGui::CollapsingHeader("RF / мощность / усиление", ImGuiTreeNodeFlags_DefaultOpen)) {
            float tx_gain = (float)g_status.tx_gain.load();
            float rx_gain = (float)g_status.rx_gain.load();
            float amp = (float)g_status.tx_amplitude.load();
            ImGui::BeginDisabled(!is_tx);
            if (ImGui::SliderFloat("TX gain, dB", &tx_gain, -89.0f, 0.0f, "%.1f"))
                g_status.tx_gain = tx_gain;
            if (ImGui::SliderFloat("TX amplitude", &amp, 0.0f, 3200.0f, "%.0f"))
                g_status.tx_amplitude = amp;
            ImGui::EndDisabled();
            ImGui::BeginDisabled(is_tx);
            if (ImGui::SliderFloat("RX gain, dB", &rx_gain, 0.0f, 70.0f, "%.1f"))
                g_status.rx_gain = rx_gain;
            ImGui::EndDisabled();
            ImGui::TextDisabled("TX amplitude: цифровой масштаб, не dBm.");
            ImGui::TextDisabled("Частоты неизменны, как в исходнике.");
        }
        if (ImGui::CollapsingHeader("Символьная синхронизация", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool enabled = g_status.gardner_enabled.load();
            if (ImGui::Checkbox("Gardner TED (live)", &enabled))
                g_status.gardner_enabled = enabled;
            ImGui::Text("SPS = %d | mu = 0.02", L);
            ImGui::Text("Строб: %.4f", s.strobe_phase);
            ImGui::Text("Ошибка: %+.6f", s.timing_error);
            ImGui::TextDisabled("Исходный офлайн-алгоритм сохранён.");
        }
        if (ImGui::CollapsingHeader("Графики", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Показывать согласованный фильтр", &show_matched);
            ImGui::SliderFloat("Полярный масштаб", &polar_range, 0.15f, 5.0f, "%.2f");
        }
        if (ImGui::CollapsingHeader("Waterfall: палитра и гамма", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Combo("Палитра", &palette, palette_names, IM_ARRAYSIZE(palette_names));
            ImGui::SliderFloat("Гамма", &gamma, 0.2f, 3.0f, "%.2f");
            ImGui::SliderFloat("Нижний dB", &db_min, -150.0f, -20.0f, "%.0f");
            ImGui::SliderFloat("Верхний dB", &db_max, -90.0f, 0.0f, "%.0f");
            if (db_max < db_min + 5.0f) db_max = db_min + 5.0f;
        }
        ImGui::Spacing();
        ImGui::Separator();
        if (g_status.running.load()) {
            if (ImGui::Button("Остановить SDR", ImVec2(-1, 36))) g_status.running = false;
        } else {
            ImGui::TextColored(ImVec4(1,0.69f,0.38f,1), "SDR остановлен");
        }
        if (ImGui::Button("Закрыть окно", ImVec2(-1, 32))) running_ui = false;
        ImGui::EndChild();
        ImGui::SameLine();

        ImGui::BeginChild("##Charts", ImVec2(0,0), false);
        float avail_w = ImGui::GetContentRegionAvail().x;
        float avail_h = ImGui::GetContentRegionAvail().y;
        float top_h = std::max(250.0f, avail_h * 0.42f);
        ImGui::BeginChild("##TimeCard", ImVec2(avail_w*0.52f - 5.0f, top_h), true);
        draw_time_plot(s, show_matched);
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("##PolarCard", ImVec2(0, top_h), true);
        draw_polar_plot(s, polar_range);
        ImGui::EndChild();
        float bottom_h = std::max(220.0f, avail_h - top_h - ImGui::GetStyle().ItemSpacing.y);
        ImGui::BeginChild("##WaterfallCard", ImVec2(avail_w*0.62f - 5.0f, bottom_h), true);
        draw_waterfall(s, palette, gamma, db_min, db_max);
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("##DebugCard", ImVec2(0, bottom_h), true);
        draw_debug(s, is_tx ? "TX" : "RX");
        ImGui::EndChild();
        ImGui::EndChild();
        ImGui::End();

        ImGui::Render();
        int fw, fh;
        glfwGetFramebufferSize(window, &fw, &fh);
        glViewport(0, 0, fw, fh);
        glClearColor(0.050f, 0.063f, 0.088f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
    g_status.running = false;
    if (worker.joinable()) worker.join();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}