
import pandas as pd
import matplotlib.pyplot as plt

# Читаем данные из TXT
df = pd.read_csv("signals.txt", sep="\t")

# Маркеры отсутствующих символов заменяем на NaN
df["bit"] = df["bit"].replace(-1, float("nan"))
df["bpsk_real"] = df["bpsk_real"].replace(-999, float("nan"))

# Создаём четыре графика
fig, axes = plt.subplots(4, 1, figsize=(12, 12))

# 1. Исходные биты
bits = df.dropna(subset=["bit"])

axes[0].stem(
    bits["n"],
    bits["bit"],
    basefmt=" "
)
axes[0].set_title("1. Генератор битов")
axes[0].set_ylabel("Бит")
axes[0].set_yticks([0, 1])
axes[0].grid(True)

# 2. BPSK
symbols = df.dropna(subset=["bpsk_real"])

axes[1].stem(
    symbols["n"],
    symbols["bpsk_real"],
    basefmt=" "
)
axes[1].set_title("2. Модуляция BPSK")
axes[1].set_ylabel("Амплитуда")
axes[1].set_yticks([-1, 1])
axes[1].grid(True)

# 3. Upsampling
axes[2].stem(
    df["n"],
    df["upsampled_real"],
    basefmt=" "
)
axes[2].set_title("3. Upsampling")
axes[2].set_ylabel("Амплитуда")
axes[2].grid(True)

# 4. Результат свёртки
axes[3].plot(
    df["n"],
    df["y_real"],
    drawstyle="steps-post",
    marker="."
)
axes[3].set_title("4. После свёртки с прямоугольным фильтром")
axes[3].set_xlabel("Номер отсчёта")
axes[3].set_ylabel("Амплитуда")
axes[3].grid(True)

plt.tight_layout()

# Сохраняем графики в файл
plt.savefig("signals.png", dpi=150)

# Показываем графики на экране
plt.show()
