import math
import time
from config import LIMITES_SERVOS, PINZA_ABIERTA, PINZA_AGARRE, PINZA_CERRADA
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

    def _enviar_pwm(self, s_base, s0, s1):
        """Paso elemental que aplica los 3 ángulos redondeados al PCA9685."""
        self._aplicar_angulo("base", s_base)
        self._aplicar_angulo("s0", s0)
        self._aplicar_angulo("s1", s1)

    def ir_a_inicio(self, suave=True):
        """Lleva todos los servos a su posición inicial calibrada (Base: 97.5°, s0: 90°, s1: 90°)."""
        s_base_ini = LIMITES_SERVOS["base"]["inicial"]
        s0_ini = LIMITES_SERVOS["s0"]["inicial"]
        s1_ini = LIMITES_SERVOS["s1"]["inicial"]
        pinza_ini = LIMITES_SERVOS["pinza"]["inicial"]

        if not suave or not self.angulos_actuales:
            self._validar_limites(s_base_ini, s0_ini, s1_ini)
            self._enviar_pwm(s_base_ini, s0_ini, s1_ini)
            self._aplicar_angulo("pinza", pinza_ini)
        else:
            self.mover_servos(s_base_ini, s0_ini, s1_ini, suave=True)
            self.mover_pinza(pinza_ini)

    def mover_servos(self, s_base, s0, s1, suave=True, pasos=40, pausa=0.025):
        """Valida límites y mueve los 3 servos simultáneamente con perfil suave cosenoidal."""
        self._validar_limites(s_base, s0, s1)

        if not suave or self.simular:
            self._enviar_pwm(s_base, s0, s1)
            return

        b_ini = self.angulos_actuales["base"]
        s0_ini = self.angulos_actuales["s0"]
        s1_ini = self.angulos_actuales["s1"]

        if max(abs(s_base - b_ini), abs(s0 - s0_ini), abs(s1 - s1_ini)) < 1.0:
            self._enviar_pwm(s_base, s0, s1)
            return

        for i in range(1, pasos + 1):
            t = 0.5 * (1.0 - math.cos(math.pi * i / pasos))
            b_act = b_ini + (s_base - b_ini) * t
            s0_act = s0_ini + (s0 - s0_ini) * t
            s1_act = s1_ini + (s1 - s1_ini) * t
            self._enviar_pwm(b_act, s0_act, s1_act)
            time.sleep(pausa)

    def ejecutar_trayectoria(self, secuencia_servos, pausa_paso=0.04):
        """Ejecuta una lista pre-validada de comandos (s_base, s0, s1) en secuencia."""
        for s_base, s0, s1 in secuencia_servos:
            self._validar_limites(s_base, s0, s1)

        for s_base, s0, s1 in secuencia_servos:
            self._enviar_pwm(s_base, s0, s1)
            if not self.simular:
                time.sleep(pausa_paso)

    def mover_pinza(self, angulo_dest, pasos=20, pausa=0.02):
        """Mueve la pinza suavemente verificando sus límites [40°, 90°]."""
        min_p = LIMITES_SERVOS["pinza"]["min"]
        max_p = LIMITES_SERVOS["pinza"]["max"]
        if angulo_dest < min_p or angulo_dest > max_p:
            raise ValueError(
                f"SEGURIDAD: 'pinza' en {angulo_dest:.1f}° viola el rango [{min_p}°, {max_p}°]."
            )

        ang_ini = self.angulos_actuales.get("pinza", LIMITES_SERVOS["pinza"]["inicial"])
        if self.simular or abs(angulo_dest - ang_ini) < 1.0:
            self._aplicar_angulo("pinza", angulo_dest)
            return

        for i in range(1, pasos + 1):
            t = 0.5 * (1.0 - math.cos(math.pi * i / pasos))
            ang_act = ang_ini + (angulo_dest - ang_ini) * t
            self._aplicar_angulo("pinza", ang_act)
            time.sleep(pausa)

    def abrir_pinza(self):
        self.mover_pinza(PINZA_ABIERTA)

    def agarrar_objeto(self):
        self.mover_pinza(PINZA_AGARRE)

    def cerrar_pinza(self):
        self.mover_pinza(PINZA_CERRADA)