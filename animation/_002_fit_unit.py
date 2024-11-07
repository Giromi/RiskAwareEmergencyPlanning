import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Iterable
from matplotlib.artist import Artist

def update(frame, ax, t, x, y) -> Iterable[Artist]:
    """ Clear """
    for cur_ax in ax:
        cur_ax.cla()
    """ Draw """
    ax[0].plot(t, x, 'r')
    ax[0].plot(t[frame], x[frame], 'ro')

    ax[1].plot(t, y, 'b')  
    ax[1].plot(t[frame], y[frame], 'ro')  # 움직이는 점

    ax[2].plot(x, y, 'g')
    ax[2].plot(x[frame], y[frame], 'ro')

    """ Set """
    # ax[0].set_aspect('equal', 'box')
    ax[0].set_title(f'Frame: {frame}')
    ax[0].grid(True)
    ax[0].set_xlabel('Time')#, fontsize=16)
    ax[0].set_ylabel('Distance')#, fontsize=16)

    # ax[1].set_aspect('equal', 'box')
    ax[1].set_title(f'Frame: {frame}')
    ax[1].grid(True)
    ax[1].set_xlabel('Time')#, fontsize=16)
    ax[1].set_ylabel('Distance')#, fontsize=16)

    ax[2].set_title(f'Frame: {frame}')
    ax[2].grid(True)
    ax[2].xlim = (0, 10)
    ax[2].ylim = (0, 10)
    ax[2].set_xlabel('x')
    ax[2].set_ylabel('y')
    return ax[1].lines + ax[2].lines

def main():

##################################################
    SPEED = 36      # [km/h] (36km/h = 10m/s)
    END_TIME = 5    # [s]
    FRAME = 100     # [Hz]   
    THETA = 0       # [rad]
##################################################

    v = SPEED / 3.6 # [km/h] -> [m/s]
    t = np.linspace(0, END_TIME, FRAME)
    x = v * np.cos(THETA) * t
    y = v * np.sin(THETA) * t
    print(f'x: {x}')

    frames = len(t)
    interval = (END_TIME / frames) * 1000  # Becuase of [ms]

    # fig1, (ax1, ax2) = plt.subplots(figsize=(7, 7))
    # => Need nrows=2, ncols=1
    fig1, ax = plt.subplots(nrows=1, ncols=3, figsize=(10, 6))

    ani = FuncAnimation(
            fig1, 
            update, 
            fargs=(ax, t, x, y),  # ax1, ax2를 fargs로 전달
            frames=frames, 
            repeat=True,
            interval=interval
            # blit=True,          # Draw only changing parts not all plot.
    )
    plt.show()

if __name__ == '__main__':
    main()

# ani.save('animation.mp4', writer='ffmpeg', fps=30)
