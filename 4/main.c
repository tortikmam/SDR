#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h> //printf
#include <stdlib.h> //free
#include <stdint.h>
#include <complex.h>
int main(void)
{
    // 1. Инициализация и подключение к ADALM-Pluto
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");
    
    // В зависимости от подключения: USB или IP
    if (1) {
        SoapySDRKwargs_set(&args, "uri", "usb:");
    } else {
        SoapySDRKwargs_set(&args, "uri", "ip:192.168.3.1");
    }
    
    SoapySDRKwargs_set(&args, "direct", "1");
    SoapySDRDevice *sdr = SoapySDRDevice_make(&args);
    SoapySDRKwargs_clear(&args);

    if (sdr == NULL)
    {
        printf("SoapySDRDevice_make fail: %s\n", SoapySDRDevice_lastError());
        return EXIT_FAILURE;
    }

    // 2. Настройка радиочастотных параметров TX (Частота и Частота дискретизации)
    // ВАЖНО: Частота (800 МГц) и Sample Rate (1 МГц) должны В ТОЧНОСТИ совпадать с настройками RX у одногруппника!
    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, 1e6) != 0)
    {
        printf("setSampleRate tx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, 800e6, NULL) != 0)
    {
        printf("setFrequency tx fail: %s\n", SoapySDRDevice_lastError());
    }
    
    printf("TX Frequency: %lf Hz\n", SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_TX, 0));

    // 3. Создание TX-потока (Stream)
    size_t channels[] = {0};
    size_t channel_count = sizeof(channels) / sizeof(channels[0]);

    if (SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, -10.0) != 0) { // Gain поднят до -10.0 dB (ранее -40.0 dB был слишком тихим)
        printf("setGain tx fail: %s\n", SoapySDRDevice_lastError());
    }

    SoapySDRStream *txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (txStream == NULL)
    {
        printf("setupStream tx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    // 4. Получение размера буфера (MTU) и создание TX-буфера
    size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);
    printf("TX MTU buffer size: %lu samples\n", tx_mtu);

    // В формате CS16 каждый сэмпл содержит 2 элемента по 16 бит: I (In-phase) и Q (Quadrature)
    int16_t tx_buff[2 * tx_mtu];

    // 5. Заполнение буфера тестовым сигналом (ступенчатая амплитуда)
    const int steps = 8;            
    const int step_len = tx_mtu / steps;
    const int amp_min = 500;         
    const int amp_max = 8000;

    for (int i = 0; i < 2 * tx_mtu; i += 2)
    {
        int step = (i / 2) / step_len;
        int amp = amp_min + (amp_max - amp_min) * step / (steps - 1);

        tx_buff[i]     = amp; // I
        tx_buff[i + 1] = amp; // Q
    }

    // Маркеры начала/конца кадра
    tx_buff[0] = 0x7fff;
    tx_buff[1] = 0x7fff;

    // 6. Активация передачи
    SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0);

// 7. Цикл непрерывной передачи данных
    printf("Start transmitting...\n");
    const long timeoutUs = 1000000; // Увеличиваем таймаут до 1 секунды (1 000 000 мкс)
    void *tx_buffs[] = {tx_buff};

    for (size_t buffers_sent = 0; buffers_sent < 500; buffers_sent++)
    {
        int flags = 0; // Обычная передача без таймштампов
        
        // Отправляем буфер в PlutoSDR
        int st = SoapySDRDevice_writeStream(sdr, txStream, (const void * const*)tx_buffs, tx_mtu, &flags, 0, timeoutUs);
        
        if (st < 0)
        {
            printf("TX Error, status: %i (%s)\n", st, SoapySDRDevice_lastError());
        }
        else if ((size_t)st != tx_mtu)
        {
            printf("TX Warning: sent %i of %lu samples\n", st, tx_mtu);
        }
        
        if (buffers_sent % 50 == 0)
        {
            printf("Transmitted buffer %zu\n", buffers_sent);
        }
    }

    // 8. Корректное завершение работы
    printf("Stopping transmission...\n");
    SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);
    SoapySDRDevice_closeStream(sdr, txStream);
    SoapySDRDevice_unmake(sdr);

    printf("Test complete!\n");
    return EXIT_SUCCESS;
}