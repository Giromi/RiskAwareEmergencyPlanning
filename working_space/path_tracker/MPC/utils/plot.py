import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from capdilib.convention import Waypoint
import json
import math
from capdilib.convention import X, Y, YAW

def plot_arrow_big(ax, x, y, yaw, arrow_length=1.0,
               origin_point_plot_style="xr",
               head_width=0.1, fc="r", ec="k", **kwargs):
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw, head_width=head_width,
                       fc=fc, ec=ec, **kwargs)
    else:
        ax.arrow(x, y,
                  arrow_length * math.cos(yaw),
                  arrow_length * math.sin(yaw),
                  head_width=head_width,
                  fc=fc, ec=ec,
                  **kwargs)
        if origin_point_plot_style is not None:
            ax.plot(x, y, origin_point_plot_style)
# # load data.json
# with open('data.json') as json_file:
#     data = json.load(json_file)
#
# # JSON 데이터를 파이썬 딕셔너리로 로드
# json_data = {
#     "building_dict": data["building_dict"],
#     "trafficlight_dict": data["trafficlight_dict"],
#     "streetlight_dict": data["streetlight_dict"],
#     "tree_dict": data["tree_dict"],
#     "car_dict": data["car_dict"]
# }
#
# def plot_arrow_2(ax, x, y, yaw, arrow_length=1.0,
#                            origin_point_plot_style="xr",
#                            head_width=3.0, fc="r", ec="k", **kwargs):
#     if not isinstance(x, float):
#         arrows = []
#         for (i_x, i_y, i_yaw) in zip(x, y, yaw):
#             arrow = plot_arrow(ax, i_x, i_y, i_yaw, head_width=head_width,
#                            fc=fc, ec=ec, **kwargs)
#             arrows.append(arrow)
#             return arrows
#     else:
#         # Add an arrow to the axis
#         arrow = mpatches.FancyArrow(x, y,
#                                     arrow_length * math.cos(yaw),
#                                     arrow_length * math.sin(yaw),
#                                     head_width=head_width,
#                                     facecolor=fc,
#                                     edgecolor=ec,
#                                     **kwargs)
#         ax.add_patch(arrow)
#
#         # Optionally plot the origin point style
#         if origin_point_plot_style is not None:
#             ax.plot(x, y, origin_point_plot_style)
#
#         # Recalculate the axis limits and rescale view to include new arrow
#         ax.relim()  # Recalculate limits
#         ax.autoscale_view()  # Adjust the view based on the new limits
#
#         return arrow
#

def plot_curvature(ax, x_list, y_list, heading_list, curvature,
                   k=0.01, c="-c", label="Curvature"):
    cx = [x + d * k * np.cos(yaw - np.pi / 2.0) for x, y, yaw, d in
          zip(x_list, y_list, heading_list, curvature)]
    cy = [y + d * k * np.sin(yaw - np.pi / 2.0) for x, y, yaw, d in
          zip(x_list, y_list, heading_list, curvature)]

    ax.plot(cx, cy, c)
    for ix, iy, icx, icy in zip(x_list, y_list, cx, cy):
        ax.plot([ix, icx], [iy, icy], c)

def plot_button(button_info) -> plt.axes:
    if button_info.size != 4:
        ValueError("button_info must have 4 elements.")
    return plt.axes(tuple(button_info))


def plot_building(ax, pos, size):
    # 건물을 사각형으로 시각화
    ax.add_patch(plt.Rectangle((pos[0] - size[0]/2, pos[1] - size[1]/2), size[0], size[1], color='gray', alpha=0.5))

def plot_trafficlight(ax, pos, radius):
    # 신호등을 작은 원으로 시각화
    ax.add_patch(plt.Circle((pos[0], pos[1]), radius, color='red', alpha=0.8))

def plot_streetlight(ax, pos, radius):
    # 가로등을 작은 원으로 시각화
    ax.add_patch(plt.Circle((pos[0], pos[1]), radius, color='yellow', alpha=0.8))

def plot_tree(ax, pos, radius):
    # 나무를 작은 원으로 시각화
    ax.add_patch(plt.Circle((pos[0], pos[1]), radius, color='green', alpha=0.8))

def plot_car(ax, pos, size):
    # 자동차를 사각형으로 시각화
    ax.add_patch(plt.Rectangle((pos[0] - size[0]/2, pos[1] - size[1]/2), size[0], size[1], color='blue', alpha=0.5))


def make_plot(ax, path_x, path_y, path_yaw, path_list=None, arrow_list=None):
    # 건물 시각화
    for building, info in data["building_dict"].items():
        plot_building(ax, info["pos"], info["size"])
    
    # 신호등 시각화
    for trafficlight, info in data["trafficlight_dict"].items():
        plot_trafficlight(ax, info["pos"], info["radius"])
    
    # 가로등 시각화
    for streetlight, info in data["streetlight_dict"].items():
        plot_streetlight(ax, info["pos"], info["radius"])
    
    # 나무 시각화
    for tree, info in data["tree_dict"].items():
        plot_tree(ax, info["pos"], info["radius"])
    
    # 자동차 시각화
    for car, info in data["car_dict"].items():
        plot_car(ax, info["pos"], info["size"])



    # 만약 기존의 선과 화살표가 존재한다면, 제거합니다.

    # if path_list:
    #     for path in path_list:
    #         path.remove()
    # if arrow_list:
    #     for arrow in arrow_list:
    #         arrow.remove()
    

    # 새로운 경로를 그립니다.
    # path_list = []
    # arrow_list = []

    # 경로를 그리며 새로운 선을 저장합니다.
    ax.plot(path_x, path_y, color='b', lw=2)
    # path_list.append(line)

    start_arrow = plot_arrow(ax, path_x[0], path_y[0], path_yaw[0])
    end_arrow = plot_arrow(ax, path_x[-1], path_y[-1], path_yaw[-1])
    for i, (x, y, yaw) in enumerate(zip(path_x, path_y, path_yaw)):
        # 화살표를 그리며 새로운 화살표를 저장합니다.
        print(f"i: {i}, x: {x}, y: {y}, yaw: {yaw}")
        if np.isnan(x) and np.isnan(y) and np.isnan(yaw):
            plot_arrow(ax, path_x[i - 1], path_y[i - 1], path_yaw[i - 1])
            # arrow_list.append(arrow)

    # 시작점과 끝점에 화살표를 추가합니다.
    # arrow_list.extend([start_arrow, end_arrow])

    # 축의 한계를 다시 설정하여 업데이트합니다.
    ax.relim()  # 데이터 한계를 재설정
    ax.autoscale_view()  # 스케일을 다시 조정

    # 업데이트된 플롯 요소를 반환합니다.
    return path_list, arrow_list

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

