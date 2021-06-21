import numpy as np
import math
from icecream import ic


def generate_noise(t):
    v = np.zeros((4,1))

    # noise parameters
    acc_noise_freq = np.array([2., 5.])
    acc_noise_mag  = np.array([0.1, 0.1])
    acc_noise_offset = np.array([0., 0.])

    # continuous noise
    v[1] = acc_noise_mag[0]*math.sin(acc_noise_freq[0]*t) + acc_noise_offset[0]
    v[3] = acc_noise_mag[0]*math.sin(acc_noise_freq[1]*t) + acc_noise_offset[1]

    return v


if __name__ == '__main__':

    ic(generate_noise(0.))
    ic(generate_noise(1.))
    ic(generate_noise(10.))
