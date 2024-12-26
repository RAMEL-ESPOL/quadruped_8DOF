import math
import numpy as np


# URDF GO2 real values
HIP_LENGTH = 0.0955
THIGH_LENGTH = 0.213
CALF_LENGTH = 0.2135

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def set_from_axis_angle(self, n, o):
        s = o / 2
        c = math.sin(s)
        self.x = n.x * c
        self.y = n.y * c
        self.z = n.z * c
        self.w = math.cos(s)
        return self.x, self.y, self.z, self.w
    
    def invert(self):
        self.x *= -1
        self.y *= -1
        self.z *= -1
        return self.x, self.y, self.z, self.w


class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def add(self, n):
        self.x += n.x
        self.y += n.y
        self.z += n.z
        return self.x, self.y, self.z
    
    def clone(self):
        new_vector = Vector3(self.x, self.y, self.z)
        return new_vector
      
    def apply_quaternion(self, quaternion):
        o = self.x
        s = self.y
        c = self.z
        u = quaternion.x
        l = quaternion.y
        f = quaternion.z
        _ = quaternion.w
        g = _ * o + l * c - f * s
        v = _ * s + f * o - u * c
        T = _ * c + u * s - l * o
        E = -u * o - l * s - f * c

        self.x = g * _ + E * -u + v * -f - T * -l
        self.y = v * _ + E * -l + T * -u - g * -f
        self.z = T * _ + E * -f + g * -l - v * -u
        return self.x, self.y, self.z
    
    def negate(self): 
        self.x = -self.x
        self.y = -self.y
        self.z = -self.z
        return self.x, self.y, self.z
    
    def distance_to(self, n):
        return math.sqrt(self.distance_to_squared(n))
      
    def distance_to_squared(self, n):
        o = self.x - n.x
        s = self.y - n.y
        c = self.z - n.z
        return o * o + s * s + c * c
      
    def apply_axis_angle(self, n, o):
        quaternion = Quaternion(0, 0, 0, 1)
        return self.apply_quaternion(quaternion.set_from_axis_angle(n, o))


def get_robot_joints(footPositionValue, foot_num):
    foot_position = Vector3(footPositionValue[0], footPositionValue[1], footPositionValue[2])

    base_tf_offset_hip_joint = Vector3(0.1934, 0.0465, 0)
    if foot_num > 1:
        base_tf_offset_hip_joint.x = -base_tf_offset_hip_joint.x
    if foot_num % 2 == 1:
        base_tf_offset_hip_joint.y = -base_tf_offset_hip_joint.y
    
    foot_position_distance = foot_position.distance_to(base_tf_offset_hip_joint)
    
    # Check for invalid distance
    if foot_position_distance == 0:
        return 0.0, 0.0

    # cos theory
    E = foot_position_distance  # Distancia directa desde la base a la posición del pie
    try:
        y = np.arccos((E ** 2 + THIGH_LENGTH ** 2 - CALF_LENGTH ** 2) / (2 * E * THIGH_LENGTH))
        S = np.arccos((CALF_LENGTH ** 2 + THIGH_LENGTH ** 2 - E ** 2) / (2 * CALF_LENGTH * THIGH_LENGTH)) - np.pi
    except ValueError:
        # If arccos input is out of range, return zero angles
        return 0.0, 0.0

    # Coordenadas de compensación
    C = foot_position.x - base_tf_offset_hip_joint.x
    R = foot_position.y - base_tf_offset_hip_joint.y

    if foot_position.z < 0:
        A = np.arcsin(-C / E) + y
    else:
        A = -np.pi + np.arcsin(C / E) + y

    # Longitud del vector en el plano XY
    O = np.sqrt(foot_position_distance ** 2 - C ** 2)
    
    try:
        L = np.arcsin(R / O)
    except ValueError:
        # If arcsin input is out of range, return zero angles
        return 0.0, 0.0

    # Determinar la orientación del pie
    if foot_position.z > 0:
        P = -1
    else:
        P = 1

    try:
        # Calcular el ángulo de la cadera en función de la paridad del número de la pierna
        if foot_num % 2 == 0:
            J = P * (L - np.arcsin(HIP_LENGTH / O))
        else:
            J = P * (L + np.arcsin(HIP_LENGTH / O)) 
    except ValueError:
        # If arcsin input is out of range, return zero angles
        return 0.0, 0.0

    if math.isnan(A + S):
        return 0.0, 0.0

    return A, S  # Solo devolvemos los ángulos para las dos articulaciones


def generar_trayectoria_cicloide(punto_inicial, R, num_puntos=100):
    # Punto inicial
    x0, y0 = punto_inicial

    # Parámetro t
    t = np.linspace(0, 2 * np.pi, num_puntos)
    
    # Ecuaciones paramétricas de la cicloide
    x = x0 + R * (t - np.sin(t))
    y = y0 + R * (1 - np.cos(t))
    
    return x, y


# Definir longitudes de los segmentos de la pierna
L1 = 0.21  # Longitud del primer segmento (muslo)
L2 = 0.21  # Longitud del segundo segmento (pantorrilla)

def cinemática_directa(theta1, theta2):
    """Calcula la posición (x, y) del extremo del brazo a partir de los ángulos de las articulaciones."""
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    return x, y

def cinemática_inversa(x, y):
    """Calcula los ángulos de las articulaciones a partir de la posición (x, y) del extremo del brazo."""
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if np.abs(D) > 1:
        raise ValueError("No es posible alcanzar la posición dada con los longitudes de los segmentos proporcionados")
    
    theta2 = np.arctan2(np.sqrt(1 - D**2), D)
    theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    return theta1, theta2
