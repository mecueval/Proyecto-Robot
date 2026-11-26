# Longitudes de eslabones medidas entre centros de giro (cm)
L0 = 10.0     # Altura mesa -> eje del hombro
L1 = 14.5    # Hombro -> codo
L2 = 16.0    # Codo -> punta
A = 1.0
# Offsets de calibración en postura inicial (s0 = 90°, s1 = 20°)
BASE_CENTRO = 102
DELTA_0 = 0              # El brazo 1 está a 99.52° (retrocede 2.40 cm en X y sube 14.3 cm en Z)
THETA_MESA_INICIAL = -30.0 # El brazo 2 baja 8.3 cm (de 21.0 a 12.7 cm) y avanza 13.68 cm en X
ESCALA_BASE_FK = 0.75   # Cinemática directa (1.0 = sin escala / 1:1)
ESCALA_BASE_IK = 0.75  # Cinemática inversa
ESCALA_BASE = ESCALA_BASE_IK  # Respaldo por si algún otro archivo aún importa este nombre

LIMITES_SERVOS = {
    "base":  {"canal": 0, "min": 52,   "max": 152,   "inicial": 102},
    "s0":    {"canal": 1, "min": 72.0, "max": 100.0, "inicial": 90.0},
    "s1":    {"canal": 2, "min": 65.0, "max": 110.0, "inicial": 90.0},
    "pinza": {"canal": 3, "min": 40.0, "max": 90.0,  "inicial": 40.0},
}

Z_MIN_SEGURIDAD = 16.0
R_MIN_SEGURIDAD = 12.5
R_MAX_SEGURIDAD = 19.5  # Bloquea posturas con el brazo muy estirado (como R = 21.96 cm)

PINZA_ABIERTA = 40.0
PINZA_AGARRE  = 80.0
PINZA_CERRADA = 88.0
PINZA_OFFSET_X = 4.6