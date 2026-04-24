import numpy as np
import matplotlib.pyplot as plt

# Small file to design IIR filter ot block DC component

def filter_z(z, beta=0.9):
    return (1-z**(-1))/(1-beta*z**(-1))

def filter_gain_compensation(z, beta=0.9):
    return (1+beta)/2*filter_z(z, beta)

fs = 125

f = np.linspace(0, fs/2, 1000)
omega = 2*np.pi*f/fs

H1 = filter_z(np.exp(1j*omega))
H2 = filter_gain_compensation(np.exp(1j*omega))

H3 = filter_z(np.exp(1j*omega), beta=0.95)
H4 = filter_gain_compensation(np.exp(1j*omega), beta=0.95)

H5 = filter_z(np.exp(1j*omega), beta=0.99)
H6 = filter_gain_compensation(np.exp(1j*omega), beta=0.99)

plt.plot(f, 20*np.log10(np.abs(H1)), label='Without Gain Compensation, beta=0.9')
plt.plot(f, 20*np.log10(np.abs(H2)), '--', label='With Gain Compensation, beta=0.9')
plt.plot(f, 20*np.log10(np.abs(H3)), label='Without Gain Compensation, beta=0.95')
plt.plot(f, 20*np.log10(np.abs(H4)), '--', label='With Gain Compensation, beta=0.95')
plt.plot(f, 20*np.log10(np.abs(H5)), label='Without Gain Compensation, beta=0.99')
plt.plot(f, 20*np.log10(np.abs(H6)), '--', label='With Gain Compensation, beta=0.99')
plt.xlabel('Frequency (Hz)')
plt.xscale('log')
plt.ylabel('Magnitude (dB)')
plt.title('Magnitude Response of the Filter')
plt.legend()
plt.grid()
plt.show()