import numpy as np
from config import L0, A, L1, L2, DELTA_0, THETA_MESA_INICIAL, BASE_CENTRO, LIMITES_SERVOS, Z_MIN_SEGURIDAD


def dh_matrix(theta_deg, d, a, alpha_deg):
    """Construye la matriz de transformación homogénea estándar D-H."""
    th = np.radians(theta_deg)
    al = np.radians(alpha_deg)
    return np.array([
        [np.cos(th), -np.sin(th)*np.cos(al),  np.sin(th)*np.sin(al), a*np.cos(th)],
        [np.sin(th),  np.cos(th)*np.cos(al), -np.cos(th)*np.sin(al), a*np.sin(th)],
        [0.0,         np.sin(al),             np.cos(al),            d],
        [0.0,         0.0,                    0.0,                   1.0]
    ])


def servos_a_dh(s_base, s0, s1):
    """Convierte comandos físicos de servos a variables articulares D-H (q1, q2, q3)."""
    s1_ini = LIMITES_SERVOS["s1"]["inicial"]
    q1 = s_base - BASE_CENTRO
    q2 = s0 - DELTA_0
    alpha_abs_2 = THETA_MESA_INICIAL + (s1_ini - s1)
    q3 = alpha_abs_2 - q2
    return q1, q2, q3


def dh_a_servos(q1, q2, q3):
    """Convierte variables articulares D-H (q1, q2, q3) a comandos de servos y valida rangos."""
    s1_ini = LIMITES_SERVOS["s1"]["inicial"]
    s_base = round(float(q1 + BASE_CENTRO), 2)
    s0 = round(float(q2 + DELTA_0), 2)
    s1 = round(float(THETA_MESA_INICIAL + s1_ini - (q2 + q3)), 2)

    if not (LIMITES_SERVOS["base"]["min"] <= s_base <= LIMITES_SERVOS["base"]["max"]):
        raise ValueError(f"Servo Base fuera de rango: {s_base:.1f}°")
    if not (LIMITES_SERVOS["s0"]["min"] <= s0 <= LIMITES_SERVOS["s0"]["max"]):
        raise ValueError(
            f"Servo 0 (Hombro) fuera de rango: {s0:.1f}° "
            f"(permitido: {LIMITES_SERVOS['s0']['min']}°-{LIMITES_SERVOS['s0']['max']}°)"
        )
    if not (LIMITES_SERVOS["s1"]["min"] <= s1 <= LIMITES_SERVOS["s1"]["max"]):
        raise ValueError(
            f"Servo 1 (Antebrazo) fuera de rango: {s1:.1f}° "
            f"(permitido: {LIMITES_SERVOS['s1']['min']}°-{LIMITES_SERVOS['s1']['max']}°)"
        )

    return s_base, s0, s1

def forward_kinematics(q1, q2, q3):
    """
    Cinemática Directa (FK) de 3 GDL con separación horizontal A en la base.

    1. Formulación Matricial Denavit-Hartenberg (Estándar):
       Tabla D-H [theta_i, d_i, a_i, alpha_i]:
         - Eslabón 1 (Base -> Hombro): [q1, L0,  A, 90°]
         - Eslabón 2 (Hombro -> Codo): [q2,  0, L1,  0°]
         - Eslabón 3 (Codo -> Punta):  [q3,  0, L2,  0°]
       T03 = A1(q1) @ A2(q2) @ A3(q3)

    2. Ecuaciones Analíticas Cerradas resultantes:
       r = A + L1 * cos(q2) + L2 * cos(q2 + q3)
       x = r * cos(q1)
       y = r * sin(q1)
       z = L0 + L1 * sin(q2) + L2 * sin(q2 + q3)
    """
    # --- Método 1: Cadena de Matrices Homogéneas D-H ---
    A1 = dh_matrix(q1, L0, A, 90.0)
    A2 = dh_matrix(q2, 0.0, L1, 0.0)
    A3 = dh_matrix(q3, 0.0, L2, 0.0)

    T01 = A1
    T02 = T01 @ A2
    T03 = T02 @ A3

    # --- Método 2: Ecuaciones Analíticas Explícitas ---
    q1_r, q2_r, q3_r = np.radians(q1), np.radians(q2), np.radians(q3)
    r = A + L1 * np.cos(q2_r) + L2 * np.cos(q2_r + q3_r)

    x = r * np.cos(q1_r)
    y = r * np.sin(q1_r)
    z = L0 + L1 * np.sin(q2_r) + L2 * np.sin(q2_r + q3_r)

    puntos = {
        "base":    np.array([0.0, 0.0, 0.0]),
        "hombro":  T01[:3, 3],
        "codo":    T02[:3, 3],
        "efector": np.array([x, y, z])  # Idéntico a T03[:3, 3]
    }
    return T03, puntos


def inverse_kinematics(x, y, z, codo_arriba=True):
    """
    Cinemática Inversa Analítica (IK) para punto objetivo (x, y, z).

    Ecuaciones geométricas cerradas:
      1. Giro de la base:
         q1 = atan2(y, x)
      2. Reducción al plano vertical del brazo (descontando A y L0):
         r_total = sqrt(x^2 + y^2)
         r = r_total - A
         z' = z - L0
      3. Ángulo del codo (Ley de Cosenos):
         D = cos(q3) = (r^2 + z'^2 - L1^2 - L2^2) / (2 * L1 * L2)
         q3 = atan2(±sqrt(1 - D^2), D)   (- para codo arriba)
      4. Ángulo del hombro:
         q2 = atan2(z', r) - atan2(L2 * sin(q3), L1 + L2 * cos(q3))
    """
    if z < Z_MIN_SEGURIDAD:
        raise ValueError(
            f"SEGURIDAD MESA: Z={z:.2f} cm es menor al límite mínimo ({Z_MIN_SEGURIDAD} cm por longitud de pinza)."
        )
    # 1. Ángulo de la base
    q1_rad = np.arctan2(y, x)

    # 2. Coordenadas en el plano vertical desde el eje del hombro
    r_total = np.hypot(x, y)
    r = r_total - A
    z_prima = z - L0

    # 3. Ángulo del codo por Ley de Cosenos
    D = (r**2 + z_prima**2 - L1**2 - L2**2) / (2.0 * L1 * L2)
    if abs(D) > 1.0 + 1e-6:
        raise ValueError(f"Punto ({x}, {y}, {z}) fuera del alcance geométrico (|D|={abs(D):.3f}).")

    D = np.clip(D, -1.0, 1.0)
    signo_codo = -1.0 if codo_arriba else 1.0
    q3_rad = np.arctan2(signo_codo * np.sqrt(1.0 - D**2), D)

    # 4. Ángulo del hombro
    q2_rad = np.arctan2(z_prima, r) - np.arctan2(L2 * np.sin(q3_rad), L1 + L2 * np.cos(q3_rad))

    return np.degrees(q1_rad), np.degrees(q2_rad), np.degrees(q3_rad)

def linear_trajectory(p_inicio, p_fin, num_pasos=20):
    """
    Interpola una línea recta en el espacio cartesiano (X, Y, Z) y calcula
    la secuencia completa de ángulos de servo mediante Cinemática Inversa.
    """
    p0 = np.array(p_inicio, dtype=float)
    p1 = np.array(p_fin, dtype=float)

    secuencia_servos = []
    puntos_intermedios = []

    for i in range(1, num_pasos + 1):
        t = i / num_pasos
        punto_t = (1.0 - t) * p0 + t * p1
        x, y, z = punto_t

        try:
            q1, q2, q3 = inverse_kinematics(x, y, z, codo_arriba=True)
            s_base, s0, s1 = dh_a_servos(q1, q2, q3)
        except ValueError as e:
            raise ValueError(
                f"Trayectoria abortada en paso {i}/{num_pasos} "
                f"({x:.2f}, {y:.2f}, {z:.2f}): {e}"
            )

        secuencia_servos.append((s_base, s0, s1))
        puntos_intermedios.append(punto_t)

    return secuencia_servos, np.array(puntos_intermedios)


def get_jacobian(q1_deg, q2_deg, q3_deg):
    """
    Matriz Jacobiana Analítica (3x3) con desplazamiento A en la base.
    Relaciona velocidades articulares [dq1, dq2, dq3] (rad/s)
    con velocidades lineales cartesianas [vx, vy, vz] (cm/s).
    """
    q1 = np.radians(q1_deg)
    q2 = np.radians(q2_deg)
    q3 = np.radians(q3_deg)

    s1, c1 = np.sin(q1), np.cos(q1)
    s2, c2 = np.sin(q2), np.cos(q2)
    s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)

    r_brazo = L1 * c2 + L2 * c23
    r_total = A + r_brazo
    z_prima = L1 * s2 + L2 * s23

    # Fila 1: dx/dq1, dx/dq2, dx/dq3
    j11 = -r_total * s1
    j12 = -z_prima * c1
    j13 = -L2 * s23 * c1

    # Fila 2: dy/dq1, dy/dq2, dy/dq3
    j21 = r_total * c1
    j22 = -z_prima * s1
    j23 = -L2 * s23 * s1

    # Fila 3: dz/dq1, dz/dq2, dz/dq3
    j31 = 0.0
    j32 = r_brazo
    j33 = L2 * c23

    return np.array([
        [j11, j12, j13],
        [j21, j22, j23],
        [j31, j32, j33]
    ])


def get_singularities(q2_deg, q3_deg):
    """
    Calcula el determinante analítico del Jacobiano 3x3:
    det(J) = -r_total * L1 * L2 * sin(q3)
    """
    q2 = np.radians(q2_deg)
    q3 = np.radians(q3_deg)

    r_total = A + L1 * np.cos(q2) + L2 * np.cos(q2 + q3)
    det_plano = L1 * L2 * np.sin(q3)
    determinant = -r_total * det_plano

    is_singular = abs(determinant) < 1e-3
    return is_singular, determinant


def get_inverse_jacobian(q1_deg, q2_deg, q3_deg):
    """
    Matriz Jacobiana Inversa Analítica (3x3) con desplazamiento A en la base.
    Permite calcular [dq1, dq2, dq3] (rad/s) a partir de [vx, vy, vz] (cm/s).
    """
    is_singular, det_3x3 = get_singularities(q2_deg, q3_deg)
    if is_singular:
        raise ValueError(f"El robot está en una singularidad (det(J) = {det_3x3:.5f}).")

    q1 = np.radians(q1_deg)
    q2 = np.radians(q2_deg)
    q3 = np.radians(q3_deg)

    s1, c1 = np.sin(q1), np.cos(q1)
    s2, c2 = np.sin(q2), np.cos(q2)
    s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)

    r_brazo = L1 * c2 + L2 * c23
    r_total = A + r_brazo
    z_prima = L1 * s2 + L2 * s23
    det_plano = L1 * L2 * np.sin(q3)

    # Fila 1: dq1/dx, dq1/dy, dq1/dz (Giro de la base)
    j_inv_11 = -s1 / r_total
    j_inv_12 = c1 / r_total
    j_inv_13 = 0.0

    # Fila 2: dq2/dx, dq2/dy, dq2/dz (Hombro)
    j_inv_21 = (L2 * c23 * c1) / det_plano
    j_inv_22 = (L2 * c23 * s1) / det_plano
    j_inv_23 = (L2 * s23) / det_plano

    # Fila 3: dq3/dx, dq3/dy, dq3/dz (Codo)
    j_inv_31 = -(r_brazo * c1) / det_plano
    j_inv_32 = -(r_brazo * s1) / det_plano
    j_inv_33 = -z_prima / det_plano

    return np.array([
        [j_inv_11, j_inv_12, j_inv_13],
        [j_inv_21, j_inv_22, j_inv_23],
        [j_inv_31, j_inv_32, j_inv_33]
    ])