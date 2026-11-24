
#include "../libs/SoapySDR/include/SoapySDR/Device.h"
#include "../libs/SoapySDR/include/SoapySDR/Formats.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <complex.h>
#include <math.h>

#define N 10
#define L 10

#define H_SIZE L
#define UPS_SIZE ((N - 1) * L + 1)
#define Y_SIZE (UPS_SIZE + H_SIZE - 1)

// Частоты для разных режимов
#define TX_FREQUENCY 800e6
#define RX_FREQUENCY 840e6
#define SAMPLE_RATE  1e6

int main(int argc, char *argv[])
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
        printf("Mode: TRANSMITTER, freq = %.0f MHz\n", TX_FREQUENCY / 1e6);
    }
    else if (strcmp(argv[1], "rx") == 0)
    {
        is_tx = 0;
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

    SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (sdr == NULL)
    {
        printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
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
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    size_t mtu = 960;
    printf("MTU buffer size: %lu samples\n", mtu);

    SoapySDRDevice_activateStream(sdr, stream, 0, 0, 0);

    const long timeoutUs = 400000;

        // ==================== РЕЖИМ ПЕРЕДАТЧИКА ====================
    if (is_tx)
    {
        printf("Генерация сигнала...\n");
        
        // 1. Исходные данные (BPSK: 1.0 или -1.0)
        double complex arr[N] = {
            1.0 + 0.0*I, 0.0 + 0.0*I, 0.0 + 0.0*I, 1.0 + 0.0*I,
            1.0 + 0.0*I, 0.0 + 0.0*I, 0.0 + 0.0*I, 1.0 + 0.0*I,
            1.0 + 0.0*I, 1.0 + 0.0*I
        };

        double complex arr_mod[N] = {0};
        for (int i = 0; i < N; i++) {
            arr_mod[i] = (creal(arr[i]) > 0.5) ? 1.0 : -1.0;
        }

        // 2. Вставка нулей (Upsampling)
        double complex upsampled[UPS_SIZE] = {0};
        for (int i = 0; i < N; i++) {
            upsampled[i * L] = arr_mod[i];
        }

        // 3. Прямоугольный формирующий фильтр
        double complex h[H_SIZE];
        for (int i = 0; i < H_SIZE; i++) {
            h[i] = 1.0 + 0.0 * I;
        }

        // 4. Свёртка
        double complex y[Y_SIZE] = {0};
        for (int n = 0; n < Y_SIZE; n++) {
            for (int k = 0; k < UPS_SIZE; k++) {
                int j = n - k;
                if (j >= 0 && j < H_SIZE) {
                    y[n] += upsampled[k] * h[j];
                }
            }
        }

        int16_t tx_buff[1920]; 
        void *tx_buffs[] = {tx_buff};

        const double scale_factor = 3200.0; 

        size_t pos = 0;
        long long current_time_ns = 0;

        long long packet_time_ns = (long long)((double)mtu / SAMPLE_RATE * 1e9);

        printf("Начало передачи (500 пакетов)...\n");

        for (int packet = 0; packet < 500; packet++)
        {
            for (size_t i = 0; i < mtu; i++)
            {
                tx_buff[2 * i]     = (int16_t)(creal(y[pos]) * scale_factor);
                tx_buff[2 * i + 1] = (int16_t)(cimag(y[pos]) * scale_factor);

                pos++;
                if (pos >= Y_SIZE) {
                    pos = 0; 
                }
            }

            int flags = SOAPY_SDR_HAS_TIME;
            long long tx_time = current_time_ns + (4 * 1000 * 1000);

            int st = SoapySDRDevice_writeStream(
                sdr, stream,
                (const void * const *)tx_buffs,
                mtu, &flags, tx_time, timeoutUs
            );

            if (st < 0)
            {
                printf("TX error: %s\n", SoapySDR_errToStr(st));
                break;
            }

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

        for (size_t i = 0; i < iteration_limit; i++)
        {
            void *buffs[] = {rx_buffer};

            int sr = SoapySDRDevice_readStream(
                sdr, stream, buffs, mtu,
                &flags, &timeNs, timeoutUs
            );

            if (sr >= 0)
            {
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
                break;
            }
        }

        fclose(fp_out);

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
                            (1.0 - fc) * samples[2 * ic] +
                            fc * samples[2 * (ic + 1)];

                        // Ранняя точка
                        double early =
                            (1.0 - fe) * samples[2 * ie] +
                            fe * samples[2 * (ie + 1)];

                        // Поздняя точка
                        double late =
                            (1.0 - fl) * samples[2 * il] +
                            fl * samples[2 * (il + 1)];

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

                free(samples);
            }
        }
    }

    // ==================== ЗАВЕРШЕНИЕ ====================
    SoapySDRDevice_deactivateStream(sdr, stream, 0, 0);
    SoapySDRDevice_closeStream(sdr, stream);
    SoapySDRDevice_unmake(sdr);

    printf("Done!\n");

    return EXIT_SUCCESS;
}

