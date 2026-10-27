
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <complex.h>

#define N 5
#define L 10

#define H_SIZE L
#define UPS_SIZE ((N - 1) * L + 1)
#define Y_SIZE (UPS_SIZE + H_SIZE - 1)

int main(void)
{
    srand((unsigned int)time(NULL));

    // Генератор битов
    double complex arr[N] = {0};

    for (int i = 0; i < N; i++) {
        int bit = rand() % 2;
        arr[i] = bit + 0.0 * I;
    }

    // Модуляция BPSK
    double complex arr_mod[N] = {0};

    for (int i = 0; i < N; i++) {
        arr_mod[i] = 2.0 * creal(arr[i]) - 1.0;
    }

    // вставка нулей между символами
    double complex upsampled[UPS_SIZE] = {0};

    for (int i = 0; i < N; i++) {
        upsampled[i * L] = arr_mod[i];
    }

    // Прямоугольный фильтр 
    double complex h[H_SIZE];

    for (int i = 0; i < H_SIZE; i++) {
        h[i] = 1.0 + 0.0 * I;
    }

    // свёртка
    double complex y[Y_SIZE] = {0};

    for (int n = 0; n < Y_SIZE; n++) {
        for (int k = 0; k < UPS_SIZE; k++) {
            int j = n - k;

            if (j >= 0 && j < H_SIZE) {
                y[n] += upsampled[k] * h[j];
            }
        }
    }

    FILE *file = fopen("signals.txt", "w");

    if (file == NULL) {
        printf("Ошибка открытия файла!\n");
        return 1;
    }

    fprintf(file, "n\tbit\tbpsk\tupsampled\ty\n");

    for (int n = 0; n < Y_SIZE; n++) {

        fprintf(file, "%d\t", n);

        if (n % L == 0 && n / L < N) {
            fprintf(file, "%.0f\t%.0f\t",
                    creal(arr[n / L]),
                    creal(arr_mod[n / L]));
        } else {
            fprintf(file, "-\t-\t");
        }

        if (n < UPS_SIZE) {
            fprintf(file, "%.1f\t", creal(upsampled[n]));
        } else {
            fprintf(file, "-\t");
        }

        fprintf(file, "%.1f\n", creal(y[n]));
    }

    fclose(file);

    printf("Bits: ");
    for (int i = 0; i < N; i++) {
        printf("%.0f ", creal(arr[i]));
    }
    return 0;
}
