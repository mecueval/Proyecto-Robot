import time
from config import LIMITES_SERVOS

try:
    from adafruit_servokit import ServoKit
    HARDWARE_DISPONIBLE = True
except (ImportError, NotImplementedError, RuntimeError):
    HARDWARE_DISPONIBLE = False


class RobotController:
    def __init__(self, simular=False):
        self.simular = simular or not HARDWARE_DISPONIBLE
        self.angulos_actuales = {}

        if not self.simular:
            try:
                self.kit = ServoKit(channels=16)
                for nombre, cfg in LIMITES_SERVOS.items():
                    ch = cfg["canal"]
                    self.kit.servo[ch].set_pulse_width_range(500, 2500)
                print("[HW] PCA9685 conectado correctamente.")
            except Exception as e:
                print(f"[HW] No se detectó I2C/PCA9685 ({e}). Cambiando a MODO SIMULACIÓN.")
                self.simular = True
        else:
            print("[SIM] Ejecutando en MODO SIMULACIÓN (sin hardware físico).")

        self.ir_a_inicio(suave=False)

    def _validar_limites(self, s_base, s0, s1):
        objetivos = {"base": s_base, "s0": s0, "s1": s1}
        for nombre, ang in objetivos.items():
            min_a = LIMITES_SERVOS[nombre]["min"]
            max_a = LIMITES_SERVOS[nombre]["max"]
            if ang < min_a or ang > max_a:
                raise ValueError(
                    f"SEGURIDAD: '{nombre}' en {ang:.1f}° viola el rango permitido [{min_a}°, {max_a}°]."
                )

    def _aplicar_angulo(self, nombre, angulo):
        self.angulos_actuales[nombre] = round(float(angulo), 2)
        if not self.simular:
            ch = LIMITES_SERVOS[nombre]["canal"]
            self.kit.servo[ch].angle = self.angulos_actuales[nombre]

    def ir_a_inicio(self, suave=True):
        """Lleva todos los servos a su posición inicial calibrada (Base: 90°, s0: 90°, s1: 20°)."""
        s_base_ini = LIMITES_SERVOS["base"]["inicial"]
        s0_ini = LIMITES_SERVOS["s0"]["inicial"]
        s1_ini = LIMITES_SERVOS["s1"]["inicial"]

        if not suave or not self.angulos_actuales:
            self._validar_limites(s_base_ini, s0_ini, s1_ini)
            self._aplicar_angulo("base", s_base_ini)
            self._aplicar_angulo("s0", s0_ini)
            self._aplicar_angulo("s1", s1_ini)
        else:
            self.mover_servos(s_base_ini, s0_ini, s1_ini)

    def mover_servos(self, s_base, s0, s1, paso_deg=2.0, pausa=0.025):
        """Mueve los 3 servos de forma interpolada y segura hasta los ángulos objetivo."""
        self._validar_limites(s_base, s0, s1)

        destinos = {"base": float(s_base), "s0": float(s0), "s1": float(s1)}
        inicios = self.angulos_actuales.copy()

        # Calcula el número de pasos según la articulación que deba recorrer más grados
        max_delta = max(abs(destinos[k] - inicios[k]) for k in destinos)
        num_pasos = max(1, int(max_delta / paso_deg))

        for i in range(1, num_pasos + 1):
            t = i / num_pasos
            for nombre in destinos:
                ang_intermedio = inicios[nombre] + t * (destinos[nombre] - inicios[nombre])
                self._aplicar_angulo(nombre, ang_intermedio)
            if not self.simular:
                time.sleep(pausa)

    def ejecutar_trayectoria(self, secuencia_servos, pausa_paso=0.04):
        """Ejecuta una lista pre-validada de comandos (s_base, s0, s1) en secuencia."""
        # Validación preventiva total
        for s_base, s0, s1 in secuencia_servos:
            self._validar_limites(s_base, s0, s1)

        for s_base, s0, s1 in secuencia_servos:
            self._aplicar_angulo("base", s_base)
            self._aplicar_angulo("s0", s0)
            self._aplicar_angulo("s1", s1)
            if not self.simular:
                time.sleep(pausa_paso)