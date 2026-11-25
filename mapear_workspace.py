import numpy as np
from config import LIMITES_SERVOS, Z_MIN_SEGURIDAD, PINZA_OFFSET_X, BASE_CENTRO
from kinematics import servos_a_dh, dh_a_servos, forward_kinematics, inverse_kinematics

b_min, b_max = LIMITES_SERVOS["base"]["min"], LIMITES_SERVOS["base"]["max"]
s0_min, s0_max = LIMITES_SERVOS["s0"]["min"], LIMITES_SERVOS["s0"]["max"]
s1_min, s1_max = LIMITES_SERVOS["s1"]["min"], LIMITES_SERVOS["s1"]["max"]

# 1. Barrido de (s0, s1) para obtener todos los (r, z) alcanzables con Z >= Z_MIN_SEGURIDAD
r_vals, z_vals = [], []
for s0 in np.arange(s0_min, s0_max + 0.1, 1.0):
    for s1 in np.arange(s1_min, s1_max + 0.1, 1.0):
        _, q2, q3 = servos_a_dh(BASE_CENTRO, s0, s1)
        _, pts = forward_kinematics(0.0, q2, q3)
        r, z = pts["efector"][0], pts["efector"][2]
        if z >= Z_MIN_SEGURIDAD - 0.01:
            r_vals.append(r)
            z_vals.append(z)

r_vals = np.array(r_vals)
z_vals = np.array(z_vals)

# 2. Proyectar con el rango real de giro de la base (q1)
q1_min, _, _ = servos_a_dh(b_min, s0_min, s1_min)
q1_max, _, _ = servos_a_dh(b_max, s0_min, s1_min)
ang_q1 = np.radians(np.linspace(q1_min, q1_max, 181))

x_all = np.outer(r_vals, np.cos(ang_q1))
y_all = np.outer(r_vals, np.sin(ang_q1))

print("=== ESPACIO DE TRABAJO REAL 3D (MUÑECA L2) ===")
print(f"Giro base (q1) : {q1_min:+.1f}° a {q1_max:+.1f}°")
print(f"Radio (R)      : {r_vals.min():.2f} cm a {r_vals.max():.2f} cm")
print(f"Eje X          : {x_all.min():.2f} cm a {x_all.max():.2f} cm")
print(f"Eje Y          : {y_all.min():.2f} cm a {y_all.max():.2f} cm")
print(f"Eje Z          : {z_vals.min():.2f} cm a {z_vals.max():.2f} cm")

# 3. Límites exactos en el plano de Pick & Place (Z = Z_MIN_SEGURIDAD)
r_pp = []
for r_test in np.arange(1.0, 35.0, 0.1):
    try:
        q1, q2, q3 = inverse_kinematics(r_test, 0.0, Z_MIN_SEGURIDAD, codo_arriba=True)
        dh_a_servos(q1, q2, q3)
        r_pp.append(r_test)
    except ValueError:
        pass

if r_pp:
    r_l2_min, r_l2_max = min(r_pp), max(r_pp)
    r_obj_min, r_obj_max = r_l2_min + PINZA_OFFSET_X, r_l2_max + PINZA_OFFSET_X

    print(f"\n=== PLANO PICK & PLACE (Z = {Z_MIN_SEGURIDAD:.1f} cm) ===")
    print(f"Muñeca L2 (Radio R)      : {r_l2_min:.2f} cm a {r_l2_max:.2f} cm")
    print(f"Objeto/Pinza (R + {PINZA_OFFSET_X:.1f}cm): {r_obj_min:.2f} cm a {r_obj_max:.2f} cm")
    print(f"Ancho máximo en Y (obj)  : {r_obj_max * np.sin(np.radians(q1_min)):+.2f} cm a {r_obj_max * np.sin(np.radians(q1_max)):+.2f} cm")