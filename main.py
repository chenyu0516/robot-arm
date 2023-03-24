import vpython as vp
import numpy as math


# parameter
floor_depth = 0.1
robot_arm1_length = 2
robot_arm2_length = 2
original_point = vp.vec(0, 0, 0)
#initial coordinate
a = robot_arm2_length
b = robot_arm1_length
c = 0
a_n = 1
b_n = 1.6
c_n = 2
current_robot_arm1_axis = vp.vec(0, robot_arm1_length, 0)
current_robot_arm2_axis = vp.vec(robot_arm2_length, 0, 0)
rotate_speed = 0.01

scene = vp.canvas(title='robot arm', width=800, height=600, x=0, y=0, center=vp.vec(0, 0.1, 0),
                  background=vp.vec(0, 0.6, 0.6))
floor = vp.box(pos=vp.vec(0, floor_depth/2, 0), size=vp.vec(5, floor_depth, 5), color=vp.color.black)
joint01 = vp.sphere(pos=original_point, radius=0.2, color=vp.color.blue)
#joint type is sphere
joint02 = vp.sphere(pos=vp.vec(0, robot_arm1_length, 0), radius=0.2, color=vp.color.blue)
#joint type is ratation
joint03 = vp.sphere(pos=vp.vec(a, b, c), radius=0.16, color=vp.color.blue)

robot_arm1 = vp.cylinder(pos=original_point, radius=0.08, axis=vp.vec(0, 1, 0), length=robot_arm1_length,
                         color=vp.color.cyan)
robot_arm2 = vp.cylinder(pos=vp.vec(0, robot_arm1_length, 0), radius=0.08, axis=vp.vec(1, 0, 0),
                         length=robot_arm2_length, color=vp.color.cyan)



def theta_calculation(x, y, n):
    if a != 0 and b != 0:
        cos_theta = x / math.sqrt(x ** 2 + y ** 2)
        sin_theta = y / math.sqrt(x ** 2 + y ** 2)
        if n == 1:
            return cos_theta
        elif n == 0:
            return sin_theta


def psi_calculation(x, z, y, n):
    cos_psi = (robot_arm2_length**2+robot_arm1_length**2-(x**2+y**2+z**2))/(2*robot_arm2_length*robot_arm1_length)
    sin_psi = math.sqrt(1-cos_psi**2)
    if n == 1:
        return cos_psi
    elif n == 0:
        return sin_psi


def phi_calculation(x, z, y, n):
    sin_phi0 = z/math.sqrt(x**2+y**2+z**2)
    cos_phi0 = math.sqrt(1-sin_phi0**2)
    sin_phi1 = robot_arm2_length*psi_calculation(x, z, y, 0)/math.sqrt(x**2+y**2+z**2)
    cos_phi1 = math.sqrt(1-sin_phi1**2)
    cos_phi = cos_phi1*cos_phi0-sin_phi1*sin_phi0
    sin_phi = sin_phi1*cos_phi0+sin_phi0*cos_phi1
    if n == 1:
        return cos_phi
    elif n == 0:
        return sin_phi


def delta_degree_cal(x, z, y, x_n, z_n, y_n, n):
    if n == 1:
        sin_delta_degree = theta_calculation(x_n, y_n, 0)*theta_calculation(x, y, 1) - \
                           theta_calculation(x_n, y_n, 1)*theta_calculation(x, y, 0)
    elif n == 2:
        sin_delta_degree = phi_calculation(x_n, z_n, y_n, 0) * phi_calculation(x, z, y, 1) - \
                           phi_calculation(x_n, z_n, y_n, 1) * phi_calculation(x, z, y, 0)
    elif n == 3:
        sin_delta_degree = psi_calculation(x_n, z_n, y_n, 0) * psi_calculation(x, z, y, 1) - \
                           psi_calculation(x_n, z_n, y_n, 1) * psi_calculation(x, z, y, 0)
    else:
        sin_delta_degree = 0
    return math.arcsin(sin_delta_degree)


def rotate_matrix(x, z, y, theta1, theta2, n):
    global current_robot_arm1_axis
    matrix_a = math.array([
                    [math.cos(theta1), -math.sin(theta1), 0],
                    [math.sin(theta1), math.cos(theta1), 0],
                    [0, 0, 1]
    ])
    axis1 = math.array([x, y, 0])
    axis2 = math.array([0, 0, 1], dtype=float)
    if x == 0 and y == 0:
        rotate_axis = math.array([0, 1, 0])
        mag_rotate_axis = 1
    else:
        rotate_axis = math.cross(axis1, axis2)
        mag_rotate_axis = math.sqrt(rotate_axis.item(0)**2+rotate_axis.item(1)**2+1)
    e_x = rotate_axis.item(0)/mag_rotate_axis
    e_y = rotate_axis.item(1)/mag_rotate_axis
    matrix_b = math.array([
                           [(math.cos(theta2)+(1-math.cos(theta2)))*e_x**2, (1-math.cos(theta2))*e_x*e_y, math.sin(theta2)*e_y],
                           [(1-math.cos(theta2))*e_x*e_y, (math.cos(theta2)+(1-math.cos(theta2)))*e_y**2, -math.sin(theta2)*e_x],
                           [-math.sin(theta2)*e_y, math.sin(theta2)*e_x, 0]
    ])
    axis3_matrix = math.array([x, y, z])
    temp = math.dot(matrix_b, axis3_matrix)
    result = math.dot(matrix_a, temp)
    if n == 1:
        current_robot_arm1_axis.x = result.item(0)
        current_robot_arm1_axis.y = result.item(1)
        current_robot_arm1_axis.z = result.item(2)
    elif n == 2:
        current_robot_arm2_axis.x = result.item(0)
        current_robot_arm2_axis.y = result.item(1)
        current_robot_arm2_axis.z = result.item(2)


delta_theta = delta_degree_cal(a, c, b, a_n, c_n, b_n, 1)
delta_phi = delta_degree_cal(a, c, b, a_n, c_n, b_n, 2)
delta_psi = delta_degree_cal(a, c, b, a_n, c_n, b_n, 3)
rotate_matrix(current_robot_arm1_axis.x, current_robot_arm1_axis.z, current_robot_arm1_axis.y, delta_theta, delta_phi, 1)
rotate_matrix(current_robot_arm2_axis.x, current_robot_arm2_axis.z, current_robot_arm2_axis.y, delta_theta, delta_psi, 2)

robot_arm1.axis = current_robot_arm1_axis
joint02.pos = current_robot_arm1_axis
joint03.pos = vp.vec(a_n, c_n, b_n)
robot_arm2.pos = current_robot_arm1_axis
robot_arm2.axis = current_robot_arm2_axis



'''

while delta_theta != 0 or delta_phi != 0 or delta_psi != 0:
    vp.rate(1000)
    if delta_theta != 0 and delta_theta>0:
        rotate_2d(rotate_speed)
        delta_theta -= rotate_speed
    elif delta_theta != 0 and delta_theta<0:
        rotate_2d(-rotate_speed)
        delta_theta += rotate_speed
    if delta_phi != 0 and delta_phi>0:
        rotate_3d(rotate_speed, current_robot_arm1_axis.x, current_robot_arm1_axis.y, current_robot_arm1_axis.z)
        delta_phi -= rotate_speed
        current_robot_arm1_axis = temp_vec
    elif delta_phi != 0 and delta_phi<0:
        rotate_3d(-rotate_speed, current_robot_arm1_axis.x, current_robot_arm1_axis.y, current_robot_arm1_axis.z)
        delta_phi += rotate_speed
        current_robot_arm1_axis = temp_vec
    if delta_psi != 0 and delta_psi>0:
        rotate_3d(rotate_speed, current_robot_arm2_axis.x, current_robot_arm2_axis.y, current_robot_arm2_axis.z)
        delta_psi -= rotate_speed
        current_robot_arm2_axis = temp_vec
    elif delta_psi != 0 and delta_psi<0:
        rotate_3d(-rotate_speed, current_robot_arm2_axis.x, current_robot_arm2_axis.y, current_robot_arm2_axis.z)
        delta_psi += rotate_speed
        current_robot_arm2_axis = temp_vec
    t=t+1
    print(t)
    robot_arm1.axis = current_robot_arm1_axis
    robot_arm2.axis = current_robot_arm2_axis
    joint02.pos = current_robot_arm1_axis + vp.vec(0, floor_depth/2, 0)
    joint02.pos = joint02.pos + current_robot_arm2_axis

'''
