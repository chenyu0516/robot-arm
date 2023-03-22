import vpython as vp
import pylab as pl

# parameter
floor_depth = 0.1
robot_arm1_length = 2
robot_arm2_length = 2
original_point = vp.vec(0, floor_depth/2, 0)
a = robot_arm2_length
b = robot_arm1_length+floor_depth/2
c = 0

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


def theta_calculation(n):
    if a != 0 and b != 0:
        cos_theta = a / pl.sqrt(a ** 2 + b ** 2)
        sin_theta = b / pl.sqrt(a ** 2 + b ** 2)
        if n == 1:
            return cos_theta
        elif n == 0:
            return sin_theta


def psi_calculation(n):
    if robot_arm2_length != 0 and robot_arm2_length != 0:
        cos_psi = (robot_arm2_length**2+robot_arm1_length**2-(a**2+b**2+c**2))/(2*robot_arm2_length*robot_arm1_length)
        sin_psi = pl.sqrt(1-cos_psi**2)
        if n == 1:
            return cos_psi
        elif n == 0:
            return sin_psi


def phi_calculation(n):
    sin_phi0 = c/pl.sqrt(a**2+b**2+c**2)
    cos_phi0 = pl.sqrt(1-sin_phi0**2)
    sin_phi1 = robot_arm2_length*psi_calculation(0)/pl.sqrt(a**2+b**2+c**2)
    cos_phi1 = pl.sqrt(1-sin_phi1**2)
    cos_phi = cos_phi1*cos_phi0-sin_phi1*sin_phi0
    sin_phi = sin_phi1*cos_phi0+sin_phi0*cos_phi1
    if n == 1:
        return cos_phi
    elif n == 0:
        return sin_phi


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
