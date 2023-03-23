import vpython as vp
import math


# parameter
floor_depth = 0.1
robot_arm1_length = 2
robot_arm2_length = 2
original_point = vp.vec(0, floor_depth/2, 0)
a = robot_arm2_length
b = robot_arm1_length+floor_depth/2
c = 0
a_n = 0.3
b_n = 0.4
c_n = 0.5
current_robot_arm1_axis = vp.vec(0, robot_arm1_length, 0)
current_robot_arm2_axis = vp.vec(robot_arm2_length, 0, 0)
rotate_speed = 0.01

scene = vp.canvas(title='robot arm', width=800, height=600, x=0, y=0, center=vp.vec(0, 0.1, 0),
                  background=vp.vec(0, 0.6, 0.6))
floor = vp.box(pos=vp.vec(0, 0, 0), size=vp.vec(5, floor_depth, 5), color=vp.color.black)
joint01 = vp.sphere(pos=original_point, radius=0.2, color=vp.color.blue)
#joint type is sphere
joint02 = vp.sphere(pos=vp.vec(0, robot_arm1_length+floor_depth/2, 0), radius=0.2, color=vp.color.blue)
#joint type is ratation
joint03 = vp.sphere(pos=vp.vec(a, b, c), radius=0.16, color=vp.color.blue)

robot_arm1 = vp.cylinder(pos=original_point, radius=0.08, axis=vp.vec(0, 1, 0), length=robot_arm1_length,
                         color=vp.color.cyan)
robot_arm2 = vp.cylinder(pos=vp.vec(0, robot_arm1_length+floor_depth/2, 0), radius=0.08, axis=vp.vec(1, 0, 0),
                         length=robot_arm2_length, color=vp.color.cyan)



def theta_calculation(x, y, n):
    if a != 0 and b != 0:
        cos_theta = x / math.sqrt(a ** 2 + b ** 2)
        sin_theta = y / math.sqrt(a ** 2 + b ** 2)
        if n == 1:
            return cos_theta
        elif n == 0:
            return sin_theta


def psi_calculation(x, y, z, n):
    if robot_arm2_length != 0 and robot_arm2_length != 0:
        cos_psi = (robot_arm2_length**2+robot_arm1_length**2-(x**2+y**2+z**2))/(2*robot_arm2_length*robot_arm1_length)
        sin_psi = math.sqrt(1-cos_psi**2)
        if n == 1:
            return cos_psi
        elif n == 0:
            return sin_psi


def phi_calculation(x, y, z, n):
    sin_phi0 = c/math.sqrt(x**2+y**2+z**2)
    cos_phi0 = math.sqrt(1-sin_phi0**2)
    sin_phi1 = robot_arm2_length*psi_calculation(x, y, z, 0)/math.sqrt(x**2+y**2+z**2)
    cos_phi1 = math.sqrt(1-sin_phi1**2)
    cos_phi = cos_phi1*cos_phi0-sin_phi1*sin_phi0
    sin_phi = sin_phi1*cos_phi0+sin_phi0*cos_phi1
    if n == 1:
        return cos_phi
    elif n == 0:
        return sin_phi


def delta_theta_cal(x, y, x_n, y_n):
    sin_delta_theta = theta_calculation(x_n, y_n, 0)*theta_calculation(x, y, 1)-theta_calculation(x_n, y_n, 1)*theta_calculation(x, y, 0)
    result = math.asin(sin_delta_theta)
    return result


def delta_phi_cal(x, y, z, x_n, y_n, z_n):
    sin_delta_phi = phi_calculation(x_n, y_n, z_n, 0) * phi_calculation(x, y, z, 1) - \
                    phi_calculation(x_n, y_n, z_n, 1) * phi_calculation(x, y, z, 0)
    result = math.asin(sin_delta_phi)
    return result


def delta_psi_cal(x, y, z, x_n, y_n, z_n):
    sin_delta_psi = psi_calculation(x_n, y_n, z_n, 0) * psi_calculation(x, y, z, 1) - \
                    psi_calculation(x_n, y_n, z_n, 1) * psi_calculation(x, y, z, 0)
    result = math.asin(sin_delta_psi)
    return result

temp_vec = vp.vec(0,0,0)
def rotate_2d(degree):
    x_0 = current_robot_arm1_axis.x
    y_0 = current_robot_arm1_axis.y
    z_0 = current_robot_arm1_axis.z
    current_robot_arm1_axis.x = x_0*math.cos(degree)-y_0*math.sin(degree)
    current_robot_arm1_axis.y = x_0*math.sin(degree)+y_0*math.cos(degree)
    current_robot_arm1_axis.z = z_0

def rotate_3d(degree, x_0, y_0, z_0):
    vec_a = vp.vec(x_0, 0, y_0)
    vec_b = vp.vec(0, z_0, 0)
    unit_revolve_vec = vp.cross(vec_a, vec_b)/(vec_b.mag*vec_a.mag)
    temp_vec.x = x_0*(math.cos(degree)+(1-math.cos(degree))*unit_revolve_vec.x**2)+\
                                y_0*((1-math.cos(degree))*unit_revolve_vec.x*unit_revolve_vec.y-math.sin(degree)*unit_revolve_vec.z)+\
                                z_0*((1-math.cos(degree))*unit_revolve_vec.x*unit_revolve_vec.y+math.sin(degree)*unit_revolve_vec.y)
    temp_vec.y = x_0*((1-math.cos(degree))*unit_revolve_vec.x*unit_revolve_vec.y+math.sin(degree)*unit_revolve_vec.z)+\
                                y_0*(math.cos(degree)+(1-math.cos(degree))*unit_revolve_vec.y**2)+\
                                z_0*((1-math.cos(degree))*unit_revolve_vec.z*unit_revolve_vec.y+math.sin(degree)*unit_revolve_vec.x)
    temp_vec.z = x_0*((1-math.cos(degree))*unit_revolve_vec.x*unit_revolve_vec.z-math.sin(degree)*unit_revolve_vec.y)+\
                                y_0*((1-math.cos(degree))*unit_revolve_vec.y*unit_revolve_vec.z+math.sin(degree)*unit_revolve_vec.x)+\
                                z_0*(math.cos(degree)+(1-math.cos(degree))*unit_revolve_vec.z**2)


delta_theta = delta_theta_cal(a, b, a_n, b_n)
delta_phi = delta_phi_cal(a, b, c, a_n, b_n, c_n)
delta_psi = delta_psi_cal(a, b, c, a_n, b_n, c_n)
t = 0
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


