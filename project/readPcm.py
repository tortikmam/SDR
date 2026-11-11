
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

SENT_FILE = sys.argv[1] if len(sys.argv) > 1 else "build/rx_recv.pcm"
RECV_FILE = sys.argv[2] if len(sys.argv) > 2 else "build/rx_recv.pcm"

SAMPLE_RATE = 1e6
SHOW = 50000       # отсчетов во времени
BLOCK = 1920       # размер блока
SPS = 10           # отсчетов на символ
EST_LEN = 50000    # отсчетов для сдвига частоты


def read_iq(name):
    if not os.path.exists(name):
        print(f"Ошибка: файл '{name}' не найден")
        sys.exit(1)

    raw = np.fromfile(name, dtype=np.int16)
    raw = raw[: len(raw) // 2 * 2].astype(np.float32)

    return raw[0::2], raw[1::2]



tx_i, tx_q = read_iq(SENT_FILE)
rx_i, rx_q = read_iq(RECV_FILE)

rx = rx_i + 1j * rx_q


n_blocks = max(len(rx) // BLOCK, 1)

# Рассчитываем мощность каждого блока
power = np.array([
    np.mean(np.abs(rx[k * BLOCK:(k + 1) * BLOCK]) ** 2)
    for k in range(n_blocks)
])

sorted_power = np.sort(power)

noise_blocks = max(1, len(sorted_power) // 3)
noise_floor = np.median(sorted_power[:noise_blocks])

THRESHOLD = max(noise_floor * 5.0, np.finfo(float).eps)

active = power > THRESHOLD

MIN_ACTIVE_BLOCKS = 3

start_block = None

for k in range(len(active) - MIN_ACTIVE_BLOCKS + 1):

    if np.all(active[k:k + MIN_ACTIVE_BLOCKS]):
        start_block = k
        break

if start_block is None:
    print("Внимание: устойчивый сигнал не найден!")
    print("Используется блок с максимальной мощностью.")
    start_block = int(np.argmax(power))

start = start_block * BLOCK

start = min(start, max(len(rx) - SHOW, 0))

seg = slice(start, start + SHOW)

print("\n========== ПОИСК СИГНАЛА ==========")
print(f"Всего блоков: {n_blocks}")
print(f"Уровень шума: {noise_floor:.2f}")
print(f"Порог обнаружения: {THRESHOLD:.2f}")
print(f"Найденный блок: {start_block}")
print(f"Начальный отсчет: {start}")
print(f"Мощность выбранного блока: {power[start_block]:.2f}")
print("===================================\n")


h = [1.0] * SPS

len_y = len(rx) + len(h) - 1
y = [0.0 + 0.0j] * len_y

for n in range(len_y):

    acc = 0.0 + 0.0j

    for k in range(len(h)):

        if 0 <= n - k < len(rx):
            acc += rx[n - k] * h[k]

    y[n] = acc

y = np.asarray(y)


plt.figure(figsize=(6, 6))

plt.scatter(
    rx_i[seg],
    rx_q[seg],
    color="blue",
    alpha=0.6,
    label="I/Q"
)

plt.title("I/Q Созвездие (до фильтра)")
plt.xlabel("I")
plt.ylabel("Q")

plt.axhline(0, color="black", linewidth=0.8, linestyle="--")
plt.axvline(0, color="black", linewidth=0.8, linestyle="--")

plt.grid(True)
plt.legend(loc="upper right")
plt.tight_layout()



plt.figure(figsize=(6, 6))

plt.scatter(
    y[seg].real,
    y[seg].imag,
    color="blue",
    alpha=0.6,
    label="Filtered I/Q"
)

plt.title("I/Q Созвездие (после свертки)")
plt.xlabel("I")
plt.ylabel("Q")

plt.axhline(0, color="black", linewidth=0.8, linestyle="--")
plt.axvline(0, color="black", linewidth=0.8, linestyle="--")

plt.grid(True)
plt.legend(loc="upper right")
plt.tight_layout()


y_eval = y[start : start + SHOW]

fig, axes = plt.subplots(
    2, 5,
    figsize=(18, 7),
    sharex=True,
    sharey=True
)

axes = axes.flatten()

for phase in range(SPS):

    z_phase = y_eval[phase::SPS]

    ax = axes[phase]

    ax.scatter(
        z_phase.real,
        z_phase.imag,
        color="crimson",
        alpha=0.6,
        s=15
    )

    ax.set_title(f"Фаза: {phase}")
    ax.set_xlabel("I")
    ax.set_ylabel("Q")

    ax.axhline(0, color="black", linewidth=0.8, linestyle="--")
    ax.axvline(0, color="black", linewidth=0.8, linestyle="--")

    ax.grid(True)

plt.suptitle("Созвездия по фазам", fontsize=14)
plt.tight_layout()

plt.show()
