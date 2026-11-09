# Longitudes de eslabones medidas entre centros de giro (cm)
L0 = 9.5     # Altura mesa -> eje del hombro
L1 = 14.5    # Hombro -> codo
L2 = 16.0    # Codo -> punta

# Offsets de calibración en postura inicial (s0 = 90°, s1 = 20°)
BASE_CENTRO = 97.5
DELTA_0 = 0              # El brazo 1 está a 99.52° (retrocede 2.40 cm en X y sube 14.3 cm en Z)
THETA_MESA_INICIAL = -25.94 # El brazo 2 baja 8.3 cm (de 21.0 a 12.7 cm) y avanza 13.68 cm en X

LIMITES_SERVOS = {
    "base": {"canal": 0, "min": 37.5, "max": 157.5, "inicial": 97.5}, 
    "s0":   {"canal": 1, "min": 50.0, "max": 110.0, "inicial": 90.0},
    "s1":   {"canal": 2, "min": 50.0, "max": 120.0, "inicial": 90.0}
}