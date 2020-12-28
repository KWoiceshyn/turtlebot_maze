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
    x_arr = np.array(r_arr * np.cos(t_arr - np.pi/2))
    y_arr = np.array(r_arr * np.sin(t_arr - np.pi/2))
    theta_inc = 2
    r_max = 10.0  # max expected scan range
    detect_thresh = 80
    grid_size = int(360 / theta_inc)
    r_inc = 2 * r_max / grid_size
    h_array = np.zeros((grid_size, grid_size))
    half_grid = int(grid_size/2)
    for xi, yi in zip(x_arr, y_arr):
        for j in range(-half_grid, half_grid, 1):
            r = xi * np.cos(np.radians(theta_inc*j)) + yi * np.sin(np.radians(theta_inc*j))
            r_idx = int(r / r_inc)
            h_array[j+half_grid, r_idx+half_grid] += 1

    # loop through accumulator and find peaks
    line_list = []
    m = h_array.shape[0] - 1
    n = h_array.shape[1] - 1
    count = 0
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
                    h_array[i, j] >= h_array[min(i + 1, m), min(j + 1, n)] and
                    j >= half_grid): # only want positive range values
                if count < 2:
                    line_list.append(LineParams((i-half_grid) * theta_inc, (j-half_grid) * r_inc))
                    if count == 1 and np.abs(line_list[1].a + 90) < np.abs(line_list[0].a + 90):
                        tmp = line_list[1]
                        line_list[1] = line_list[0]
                        line_list[0] = tmp
                    count += 1
                else:
                    if np.abs((i-half_grid)*theta_inc + 90) < np.abs(line_list[0].a + 90):
                        line_list[0].a = (i-half_grid)*theta_inc
                        line_list[0].r = (j-half_grid)*r_inc
                    if np.abs((i - half_grid) * theta_inc - 90) < np.abs(line_list[1].a - 90):
                        line_list[1].a = (i - half_grid) * theta_inc
                        line_list[1].r = (j - half_grid) * r_inc


    # walk along found lines and find end points
    max_deviation = 0.12
    count = 0
    for line in line_list:
        a_rad = np.radians(line.a + 90)
        idx = (np.abs(t_arr - a_rad)).argmin()
        print('idx ' + str(idx))
        if count == 0:
            end_idx = len(t_arr) # walk up scan for first wall
            incr = 1
        else:
            end_idx = -1 # walk down scan for second wall
            incr = -1
        flag = False
        for i in range(idx+incr, end_idx, incr):
            error = np.abs(r_arr[i] * np.cos(t_arr[i] - a_rad) - line.r)
            if error > max_deviation or np.abs(r_arr[i] - r_arr[i-incr]) > 0.3:
                new_error = r_arr[i-incr] * np.cos(t_arr[i-incr] - a_rad) - line.r # force endpoint to be on model
                print('Error ' + str(new_error) + ' r ' + str(line.r) + ' a ' + str(line.a))
                line.p1[0] = r_arr[i-incr] * np.cos(t_arr[i-incr] - np.pi/2)
                line.p1[1] = r_arr[i-incr] * np.sin(t_arr[i-incr] - np.pi/2)
                print('p1b ' + str(line.p1[0]) + ',' + str(line.p1[1]))
                line.p1[0] -= new_error * np.cos(a_rad)
                line.p1[1] -= new_error * np.sin(a_rad)
                print('p1 ' + str(line.p1[0]) + ',' + str(line.p1[1]))
                flag = True
                break
        if not flag:
            new_error = r_arr[end_idx-incr] * np.cos(t_arr[end_idx-incr] - a_rad) - line.r  # force endpoint to be on model
            line.p1[0] = r_arr[end_idx-incr] * np.cos(t_arr[end_idx-incr]- np.pi/2) - new_error * np.cos(a_rad)
            line.p1[1] = r_arr[end_idx-incr] * np.sin(t_arr[end_idx-incr]- np.pi/2) - new_error * np.sin(a_rad)
        count += 1
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
#th = np.array([0, 0.1745329252, 0.3490658504, 0.5235987756, 0.6981317008, 0.872664626])
#rh = np.array([1, 1.1305158748, 1.3472963553, 1.7320508076, 2.5320888862, 4.987241533])

xy = np.array([rh * np.cos(th), rh * np.sin(th)])

H, lines = hough_lines(rh, th)

plt.figure(1)
plt.scatter(xy[0], xy[1])


colors = ['red', 'blue', 'green']
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
