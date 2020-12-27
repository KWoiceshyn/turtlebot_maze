#! /usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm


class LineParams:
    def __init__(self, angle, radius):
        self.a = angle
        self.r = radius
        self.p0 = np.array([0.0, 0.0])
        self.p1 = np.array([0.0, 0.0])


def polar_params(r_arr, t_arr):
    n = len(r_arr)
    a_num1 = a_den1 = 0
    a_num2 = a_den2 = 0
    for j, (pj, tj) in enumerate(zip(r_arr, t_arr)):
        a_num1 += pj * pj * np.sin(2 * tj)
        a_den1 += pj * pj * np.cos(2 * tj)
        for k in range(j):
            a_num2 += pj * r_arr[k] * np.sin(tj + t_arr[k])
            a_den2 += pj * r_arr[k] * np.cos(tj + t_arr[k])
    a = 0.5 * np.arctan2((1 - n) * a_num1 / n + (2 / n) * a_num2, (1 - n) * a_den1 / n + (2 / n) * a_den2)
    r = 0
    for pi, ti in zip(r_arr, t_arr):
        r += pi * np.cos(ti - a)
    r /= n
    return LineParams(a, r)


def cartesian_params(x_arr, y_arr):
    n = len(x_arr)
    x_bar = np.sum(x_arr) / n
    y_bar = np.sum(y_arr) / n
    num = den = 0
    for xi, yi in zip(x_arr, y_arr):
        num += (x_bar - xi) * (y_bar - yi)
        den += (y_bar - yi) ** 2 - (x_bar - xi) ** 2
    a = np.arctan2(-2 * num, den)
    r = x_bar * np.cos(a) + y_bar * np.sin(a)
    return LineParams(a, r)


def hough_lines(r_arr, t_arr):
    x_arr = np.array(r_arr * np.cos(t_arr))
    y_arr = np.array(r_arr * np.sin(t_arr))
    theta_inc = 2
    r_max = 4.0  # max expected scan range
    detect_thresh = 40
    grid_size = int(180 / theta_inc)
    r_inc = r_max / grid_size
    h_array = np.zeros((grid_size, grid_size))
    for xi, yi in zip(x_arr, y_arr):
        for theta in range(0, 180, theta_inc):
            r = np.abs(xi * np.cos(np.radians(theta)) + yi * np.sin(np.radians(theta)))
            theta_idx = int(theta / theta_inc)
            r_idx = int(r / r_inc)
            h_array[theta_idx, r_idx] += 1

    # loop through accumulator and find peaks
    line_list = []
    m = h_array.shape[0] - 1
    n = h_array.shape[1] - 1
    for i in range(0, m + 1):
        for j in range(0, n + 1):
            if (h_array[i, j] >= detect_thresh and
                    h_array[i, j] >= h_array[max(i - 1, 0), j] and
                    h_array[i, j] >= h_array[min(i + 1, m), j] and
                    h_array[i, j] >= h_array[i, max(j - 1, 0)] and
                    h_array[i, j] >= h_array[i, min(j + 1, n)] and
                    h_array[i, j] >= h_array[max(i - 1, 0), max(j - 1, 0)] and
                    h_array[i, j] >= h_array[max(i - 1, 0), min(j + 1, n)] and
                    h_array[i, j] >= h_array[min(i + 1, m), max(j - 1, 0)] and
                    h_array[i, j] >= h_array[min(i + 1, m), min(j + 1, n)]):
                line_list.append(LineParams(i * theta_inc, j * r_inc))

    # walk along found lines and find end points
    max_deviation = 0.12
    for line in line_list:
        a_rad = np.radians(line.a)
        idx = (np.abs(t_arr - a_rad)).argmin()
        # walk down the scan
        flag = False
        for i in range(idx - 1, -1, -1):
            error = np.abs(r_arr[i] * np.cos(t_arr[i] - a_rad) - line.r)
            if error > max_deviation:
                new_error = r_arr[i + 1] * np.cos(t_arr[i + 1] - a_rad) - line.r # force endpoint to be on model
                print('Error ' + str(new_error) + ' r ' + str(line.r) + ' a ' + str(line.a))
                line.p0[0] = r_arr[i + 1] * np.cos(t_arr[i + 1])
                line.p0[1] = r_arr[i + 1] * np.sin(t_arr[i + 1])
                print('p0b ' + str(line.p0[0]) + ',' + str(line.p0[1]))
                line.p0[0] -= new_error * np.cos(a_rad)
                line.p0[1] -= new_error * np.sin(a_rad)
                print('p0 ' + str(line.p0[0]) + ',' + str(line.p0[1]))
                flag = True
                break
        if not flag:
            new_error = r_arr[0] * np.cos(t_arr[0] - a_rad) - line.r  # force endpoint to be on model
            line.p0[0] = r_arr[0] * np.cos(t_arr[0]) - new_error * np.cos(a_rad)
            line.p0[1] = r_arr[0] * np.sin(t_arr[0]) - new_error * np.sin(a_rad)
        # walk up the scan
        flag = False
        for i in range(idx + 1, len(t_arr), 1):
            error = np.abs(r_arr[i] * np.cos(t_arr[i] - a_rad) - line.r)
            if error > max_deviation:
                new_error = r_arr[i - 1] * np.cos(t_arr[i - 1] - a_rad) - line.r  # force endpoint to be on model
                print('Error ' + str(new_error) + ' r ' + str(line.r) + ' a ' + str(line.a))
                line.p1[0] = r_arr[i - 1] * np.cos(t_arr[i - 1])
                line.p1[1] = r_arr[i - 1] * np.sin(t_arr[i - 1])
                print('p1b ' + str(line.p1[0]) + ',' + str(line.p1[1]))
                line.p1[0] -= new_error * np.cos(a_rad)
                line.p1[1] -= new_error * np.sin(a_rad)
                print('p1 ' + str(line.p1[0]) + ',' + str(line.p1[1]))
                flag = True
                break
        if not flag:
            new_error = r_arr[-1] * np.cos(t_arr[-1] - a_rad) - line.r  # force endpoint to be on model
            line.p1[0] = r_arr[-1] * np.cos(t_arr[-1]) - new_error * np.cos(a_rad)
            line.p1[1] = r_arr[-1] * np.sin(t_arr[-1]) - new_error * np.sin(a_rad)
    return h_array, line_list

'''
t1 = np.array([0, 0.297533154, 0.595066309, 0.892599463, 1.19013262])
p1 = np.array([0.752899766, 0.787183166, 0.877166092, 1.1679759, 1.94579315])
xy1 = np.array([p1 * np.cos(t1), p1 * np.sin(t1)])

t2 = np.array([0.446, 0.5863, 0.726, 0.8663, 1.006])
p2 = np.array([2.591, 2.013, 1.663, 1.444, 1.286])
xy2 = np.array([p2 * np.cos(t2), p2 * np.sin(t2)])

t3 = np.array([1.22513652, 1.30389524, 1.38265407, 1.46141291, 1.54017162])
p3 = np.array([3.60409117, 3.4892211, 3.4386394, 3.39564037, 3.35781503])
xy3 = np.array([p3 * np.cos(t3), p3 * np.sin(t3)])

t4 = np.array([2.0943951024, 2.3481733814, 2.6019516604, 2.8557299384, 3.1095082224])
p4 = np.array([1.33735152, 0.996890783, 0.822020888, 0.707503974, 0.700269401])
xy4 = np.array([p4 * np.cos(t4), p4 * np.sin(t4)])
xy = np.concatenate((xy1, xy2, xy3, xy4), axis=1)
'''

th, rh = np.loadtxt('sampleScan.txt', delimiter=' ', unpack=True)
xy = np.array([rh * np.cos(th), rh * np.sin(th)])

H, lines = hough_lines(rh, th)

plt.figure(1)
plt.scatter(xy[0], xy[1])

colors = ['red', 'green', 'blue']
i = 0
for line in lines:
    plt.scatter(line.p0[0], line.p0[1], c=colors[i], marker='X')
    plt.scatter(line.p1[0], line.p1[1], c=colors[i], marker='X')
    i+=1

plt.axis('equal')
plt.show()

'''
fig2 = plt.figure(2)
ax = fig2.gca(projection='3d')
T = np.arange(0, 180, 2)
R = np.arange(0, 4, 4/90)
R, T = np.meshgrid(R, T)
surf = ax.plot_surface(R, T, H, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
fig2.colorbar(surf, shrink=0.5, aspect=5)
'''

plt.figure(2)
plt.imshow(H)
plt.show()

#res_p = polar_params(rh, th)
#res_cart = cartesian_params(xy[0], xy[1])
