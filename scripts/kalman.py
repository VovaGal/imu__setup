import numpy as np
from scipy.signal import freqz, butter
from matplotlib import pyplot as plt

x = 0  # начальное состояние
P = 0  # начальная оценка ошибки состояния
Q = 0.0001  # шум процесса (чем ближе к 0, тем плавнее)
R = 0.1  # шум измерения (чем больше, тем плавнее)

def kalman_filter(z):
    global x, P

    # Prediction step (предсказание)
    x = x  # Предполагаем новое состояние (обычно это x = F * x + B * u, но здесь для простоты предлагается оставить значение неизменным)
    P = P + Q  # Предполагаем ошибку оценки

    # Update step (обновление)
    K = P / (P + R)  # Коэффициент Калмана
    x = x + K * (z - x)  # Обновляем состояние на основе измерения
    P = (1 - K) * P  # Обновляем оценку ошибки

    return x

data = list()
for n in range(1000):
    data.append(np.sin(0.02*n) + 0.2*np.sin(n))
plt.plot(data)


filt_data = list()
for i in range(1000):
    filt_val = kalman_filter(data[i])

    filt_data.append(filt_val)
plt.plot(filt_data)


plt.show()
