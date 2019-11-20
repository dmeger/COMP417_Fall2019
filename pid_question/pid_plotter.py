__author__ = "Travis Manderson"
__copyright__ = "Copyright 2018, Travis Manderson"

import matplotlib.pyplot as plt
import numpy as np


def plot_matplotlib(t_series, target_pos, y_series, real_pos, fan_series):
    fig, (ax1) = plt.subplots(1, 1, sharey=False)
    ax1.plot(t_series, fan_series, 'b-', linewidth=0.5, label='fan rpm')
    ax1.legend(loc=2)
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('fan rpm [RPM]', color='b')
    ax1.tick_params('y', colors='b')

    max_y = np.max(real_pos)
    ax12 = ax1.twinx()
    ax12.plot(t_series, real_pos, 'r-', linewidth=0.5, label='real position')
    ax12.plot(t_series, target_pos, 'g--', linewidth=0.5, label='target position')
    ax12.set_ylabel('position [m]', color='r')
    ax12.tick_params('y', colors='r')
    ax12.set_ylim([0, max_y + 0.2])
    ax12.legend(loc=1)

    #error = np.abs(np.array(real_pos) - np.array(y_series))
    #ax2.plot(t_series, error, 'r-', linewidth=0.5, label='real vs. detected position')
    #max_y = np.max(error)
    #ax2.set_ylabel('detection error [m]', color='r')
    #ax2.tick_params('y', colors='r')
    #if max_y < 0.1:
    #    ax2.set_ylim([0, 0.2])
    #ax2.legend()

    fig.tight_layout()
    fig.set_size_inches(18, 9)
    plt.savefig('pid performance.png', dpi=100)
    plt.show()
    return

