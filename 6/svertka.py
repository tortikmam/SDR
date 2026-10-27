import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("signals.txt", sep="\t", na_values="-")

fig, ax = plt.subplots(4, 1, figsize=(10, 10))

ax[0].stem(df["n"], df["bit"])
ax[0].set_title("Исходные биты")

ax[1].stem(df["n"], df["bpsk"])
ax[1].set_title("BPSK")

ax[2].stem(df["n"], df["upsampled"])
ax[2].set_title("Upsampling")

ax[3].plot(df["n"], df["y"], drawstyle="steps-post")
ax[3].set_title("После свёртки")

for a in ax:
    a.grid(True)
    a.set_xlabel("Номер отсчёта")

plt.tight_layout()
plt.savefig("signals.png")
plt.show()