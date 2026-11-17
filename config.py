# Dimensiones físicas del robot (en centímetros o milímetros, sé consistente)
L0 = 10.0   # d1: Altura desde la mesa hasta el eje del hombro (Servo 0)
L1 = 12.0   # a2: Longitud del Brazo 1 (hombro -> codo)
L2 = 12.0   # a3: Longitud del Brazo 2 (codo -> punta de la garra)

# Offsets de calibración física (medidos con transportador/regla)
DELTA_0 = 10.0          # Offset del hombro: q2 = s0 - DELTA_0 (cuando s0=90°, q2=80°)
THETA_MESA_INICIAL = 0.0  # Ángulo del Brazo 2 respecto a la mesa cuando s1=20°
BASE_CENTRO = 90.0      # Ángulo del servo de base cuando q1 = 0° (apuntando en +X)

# Límites estrictos de hardware para evitar colisiones o atascos
LIMITES_SERVOS = {
    "base": {"canal": 2, "min": 30, "max": 150, "inicial": 90},
    "s0":   {"canal": 0, "min": 50, "max": 90,  "inicial": 90},
    "s1":   {"canal": 1, "min": 0,  "max": 20,  "inicial": 20},
}