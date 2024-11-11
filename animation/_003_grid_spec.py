import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Iterable
from matplotlib.artist import Artist
from matplotlib.gridspec import GridSpec
import os


def on_key_event(event):
    if event.key == 'escape':
        plt.close(event.canvas.figure)

def update(frame, ax, t, x, y, theta) -> Iterable[Artist]:
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

    ax[2].set_title(f'Theta: {np.rad2deg(theta)}°')
    ax[2].grid(True)
    ax[2].set_xlabel('x')
    ax[2].set_ylabel('y')
    if (x[-1] + 10) > 10:
        ax[2].set_xlim((0, x[-1] + 10))
        ax[2].set_xticks(np.arange(0, x[-1] + 10, 10))
    if (y[-1] + 10) > 10:
        ax[2].set_ylim((0, y[-1] + 10))
        ax[2].set_yticks(np.arange(0, y[-1] + 10, 10))
    ax_lines = [line for cur_ax in ax for line in cur_ax.lines]
    return ax_lines


def main(current_file_name: str) -> None:

##################################################
    SPEED = 108             # [km/h] (108km/h = 30m/s)
    END_TIME = 5            # [s]
    THETA = 0               # [deg]
    FPS = 30                # [Hz]
##################################################
    frames = END_TIME * FPS
    # interval = 1000 * (END_TIME / frames)
    interval = 1000 / FPS
    print(f'interval: {interval}')


    v = SPEED / 3.6 # [km/h] -> [m/s]
    t = np.linspace(0, END_TIME, frames)
    x = v * np.cos(THETA) * t
    y = v * np.sin(THETA) * t
    theta = np.deg2rad(THETA)
    print(f'x: {x}')


    '''
    fig, ax = plt.subplots(2, 2, figsize=(12, 8), gridspec_kw={
    'width_ratios': [1, 1],
    'height_ratios': [1, 2],
    'hspace': 0.4,
    'wspace': 0.3
    })
    ''' # Impossible to use all Columns
    fig = plt.figure(figsize=(10, 6))
    fig.canvas.mpl_connect('key_press_event', on_key_event)
    gs = GridSpec(2, 2, figure=fig)


    ax = [
        fig.add_subplot(gs[0, 0]),   # Left plot
        fig.add_subplot(gs[0, 1]),   # Right plot
        fig.add_subplot(gs[1, :])   # Large plot spanning the width
    ]

    plt.subplots_adjust(wspace=0.3, hspace=0.5)  # wspace: 가로 간격, hspace: 세로 간격

    ani = FuncAnimation(
            fig, 
            update, 
            fargs=(ax, t, x, y, theta),  # fargs로 전달
            frames=frames, 
            repeat=True,
            interval=interval
            # blit=True,          # Draw only changing parts not all plot.
    )
    current_file_name = os.path.splitext(current_file_name)[0]  # 정확히 .py 제거
    print(f'current_file_name: {current_file_name}')
    ani.save(current_file_name + '.mp4', writer='ffmpeg', fps=FPS)
    plt.show()

if __name__ == '__main__':
    current_file_path = __file__
    current_file_name = os.path.basename(current_file_path)
    main(current_file_name)

