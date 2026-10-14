#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>

int main(void)
{
    SoapySDRKwargs args = {};
    SoapySDRKwargs_set(&args, "driver", "plutosdr");

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

    if (SoapySDRDevice_setSampleRate(sdr, SOAPY_SDR_TX, 0, 1e6) != 0)
    {
        printf("setSampleRate tx fail: %s\n", SoapySDRDevice_lastError());
    }
    if (SoapySDRDevice_setFrequency(sdr, SOAPY_SDR_TX, 0, 800e6, NULL) != 0)
    {
        printf("setFrequency tx fail: %s\n", SoapySDRDevice_lastError());
    }

    printf("TX Frequency: %lf Hz\n", SoapySDRDevice_getFrequency(sdr, SOAPY_SDR_TX, 0));

    size_t channels[] = {0};
    size_t channel_count = sizeof(channels) / sizeof(channels[0]);

    if (SoapySDRDevice_setGain(sdr, SOAPY_SDR_TX, 0, -10.0) != 0) {
        printf("setGain tx fail: %s\n", SoapySDRDevice_lastError());
    }

    SoapySDRStream *txStream = SoapySDRDevice_setupStream(sdr, SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, channel_count, NULL);
    if (txStream == NULL)
    {
        printf("setupStream tx fail: %s\n", SoapySDRDevice_lastError());
        SoapySDRDevice_unmake(sdr);
        return EXIT_FAILURE;
    }

    size_t tx_mtu = SoapySDRDevice_getStreamMTU(sdr, txStream);
    printf("TX MTU buffer size: %lu samples\n", tx_mtu);

    int16_t tx_buff[2 * tx_mtu];

    const int steps = 8;
    const int step_len = tx_mtu / steps;
    const int amp_min = 500;
    const int amp_max = 8000;

    for (int i = 0; i < 2 * tx_mtu; i += 2)
    {
        int step = (i / 2) / step_len;
        int amp = amp_min + (amp_max - amp_min) * step / (steps - 1);

        tx_buff[i]     = amp;
        tx_buff[i + 1] = amp;
    }

    tx_buff[0] = 0x7fff;
    tx_buff[1] = 0x7fff;

    SoapySDRDevice_activateStream(sdr, txStream, 0, 0, 0);

    printf("Start transmitting...\n");
    const long timeoutUs = 1000000;
    void *tx_buffs[] = {tx_buff};

    for (size_t buffers_sent = 0; buffers_sent < 500; buffers_sent++)
    {
        int flags = 0;

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

    printf("Stopping transmission...\n");
    SoapySDRDevice_deactivateStream(sdr, txStream, 0, 0);
    SoapySDRDevice_closeStream(sdr, txStream);
    SoapySDRDevice_unmake(sdr);

    printf("Test complete!\n");
    return EXIT_SUCCESS;
}