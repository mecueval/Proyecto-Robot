import time
import numpy as np
from kinematics import (
    servos_a_dh,
    dh_a_servos,
    forward_kinematics,
    inverse_kinematics,
    linear_trajectory,
)
from hardware_controller import RobotController
from config import LIMITES_SERVOS, Z_MIN_SEGURIDAD, PINZA_OFFSET_X


def obtener_estado_actual(robot):
    s_base = robot.angulos_actuales["base"]
    s0 = robot.angulos_actuales["s0"]
    s1 = robot.angulos_actuales["s1"]
    q1, q2, q3 = servos_a_dh(s_base, s0, s1)
    _, puntos = forward_kinematics(q1, q2, q3)
    return puntos["efector"], (q1, q2, q3)


def mostrar_estado_actual(robot):
    s_base = robot.angulos_actuales["base"]
    s0 = robot.angulos_actuales["s0"]
    s1 = robot.angulos_actuales["s1"]
    pinza = robot.angulos_actuales.get("pinza", LIMITES_SERVOS["pinza"]["inicial"])
    ef, (q1, q2, q3) = obtener_estado_actual(robot)

    print("\n--- ESTADO ACTUAL DEL ROBOT ---")
    print(f"Servos físicos : Base = {s_base:.1f}° | s0 = {s0:.1f}° | s1 = {s1:.1f}° | Pinza = {pinza:.1f}°")
    print(f"Variables D-H  : q1 = {q1:.2f}°   | q2 = {q2:.2f}° | q3 = {q3:.2f}°")
    print(f"Posición (FK)  : X = {ef[0]:.2f} | Y = {ef[1]:.2f} | Z = {ef[2]:.2f} (Z_min = {Z_MIN_SEGURIDAD} cm)")
    print("-------------------------------\n")


def modo_fk(robot):
    """Evalúa Cinemática Directa desde ángulos de servo y mueve el robot."""
    print("\n[1. CINEMÁTICA DIRECTA (FK)]")
    try:
        b_min, b_max = LIMITES_SERVOS["base"]["min"], LIMITES_SERVOS["base"]["max"]
        s0_min, s0_max = LIMITES_SERVOS["s0"]["min"], LIMITES_SERVOS["s0"]["max"]
        s1_min, s1_max = LIMITES_SERVOS["s1"]["min"], LIMITES_SERVOS["s1"]["max"]

        s_base = float(input(f"  Servo Base ({b_min}° a {b_max}°) [{robot.angulos_actuales['base']:.1f}]: ") or robot.angulos_actuales["base"])
        s0 = float(input(f"  Servo 0 Hombro ({s0_min}° a {s0_max}°) [{robot.angulos_actuales['s0']:.1f}]: ") or robot.angulos_actuales["s0"])
        s1 = float(input(f"  Servo 1 Antebrazo ({s1_min}° a {s1_max}°) [{robot.angulos_actuales['s1']:.1f}]: ") or robot.angulos_actuales["s1"])

        robot._validar_limites(s_base, s0, s1)

        q1, q2, q3 = servos_a_dh(s_base, s0, s1)
        _, puntos = forward_kinematics(q1, q2, q3)
        ef = puntos["efector"]

        if ef[2] < Z_MIN_SEGURIDAD - 0.01:
            raise ValueError(
                f"SEGURIDAD MESA: Esta postura baja la muñeca a Z={ef[2]:.2f} cm (< {Z_MIN_SEGURIDAD} cm)."
            )

        print(f"\n-> Predicción D-H: X = {ef[0]:.2f}, Y = {ef[1]:.2f}, Z = {ef[2]:.2f}")
        robot.mover_servos(s_base, s0, s1)
        mostrar_estado_actual(robot)

    except ValueError as e:
        print(f"Error: {e}\n")


def modo_ik_lineal(robot):
    """Resuelve Cinemática Inversa interpolando una trayectoria lineal desde la posición actual."""
    pos_actual, _ = obtener_estado_actual(robot)
    print("\n[2. CINEMÁTICA INVERSA CON TRAYECTORIA LINEAL (IK)]")
    print(f"  Origen actual P0 = ({pos_actual[0]:.2f}, {pos_actual[1]:.2f}, {pos_actual[2]:.2f})")
    print(f"  Nota: Z mínimo permitido = {Z_MIN_SEGURIDAD} cm")

    try:
        x_fin = float(input("  Destino X: "))
        y_fin = float(input("  Destino Y: "))
        z_fin = float(input("  Destino Z: "))

        q1_fin, q2_fin, q3_fin = inverse_kinematics(x_fin, y_fin, z_fin, codo_arriba=True)
        sb_fin, s0_fin, s1_fin = dh_a_servos(q1_fin, q2_fin, q3_fin)
        print(f"\n-> Solución D-H destino : q1 = {q1_fin:.2f}°, q2 = {q2_fin:.2f}°, q3 = {q3_fin:.2f}°")
        print(f"-> Servos destino       : Base = {sb_fin}°, s0 = {s0_fin}°, s1 = {s1_fin}°")

        secuencia, _ = linear_trajectory(pos_actual, (x_fin, y_fin, z_fin), num_pasos=25)
        print(f"-> Trayectoria lineal validada ({len(secuencia)} puntos). Ejecutando...")

        robot.ejecutar_trayectoria(secuencia, pausa_paso=0.035)
        mostrar_estado_actual(robot)

    except ValueError as e:
        print(f"Movimiento rechazado: {e}\n")


def modo_pinza_manual(robot):
    """Permite abrir, agarrar o cerrar la pinza manualmente."""
    print("\n[3. CONTROL DE PINZA]")
    print("  a -> Abrir (40°)")
    print("  g -> Agarrar objeto (70°)")
    print("  c -> Cerrar en vacío (88°)")
    print("  o escribe un ángulo entre 40 y 90")
    cmd = input("  Acción pinza > ").strip().lower()

    try:
        if cmd == "a":
            robot.abrir_pinza()
        elif cmd == "g":
            robot.agarrar_objeto()
        elif cmd == "c":
            robot.cerrar_pinza()
        else:
            ang = float(cmd)
            robot.mover_pinza(ang)
        print(f"-> Pinza en {robot.angulos_actuales['pinza']:.1f}°\n")
    except ValueError as e:
        print(f"Error: {e}\n")


def modo_pick_and_place(robot):
    """
    Pick & Place en el plano (X, Y) a altura fija Z = Z_MIN_SEGURIDAD (15.5 cm),
    compensando radialmente PINZA_OFFSET_X para que la punta de la pinza caiga
    sobre las coordenadas reales del objeto.
    """
    Z_TRABAJO = Z_MIN_SEGURIDAD
    DELTA_LEVANTE = 10.0  # 90° -> 80° al levantar el antebrazo
    s1_min = LIMITES_SERVOS["s1"]["min"]

    def compensar_pinza(x_obj, y_obj):
        r_obj = np.hypot(x_obj, y_obj)
        if r_obj <= PINZA_OFFSET_X:
            raise ValueError(
                f"Objeto ({x_obj}, {y_obj}) demasiado cerca del origen "
                f"(radio {r_obj:.2f} cm <= offset de pinza {PINZA_OFFSET_X} cm)."
            )
        escala = (r_obj - PINZA_OFFSET_X) / r_obj
        return x_obj * escala, y_obj * escala

    print("\n[4. SECUENCIA AUTOMÁTICA PICK & PLACE]")
    print(f"  Altura fija: Z = {Z_TRABAJO:.1f} cm | Offset radial pinza: {PINZA_OFFSET_X:.1f} cm")

    try:
        x_obj_ini = float(input("  Objeto origen  X (recoger) [19.0]: ") or 19.0)
        y_obj_ini = float(input("  Objeto origen  Y (recoger) [ 4.0]: ") or 4.0)
        x_obj_fin = float(input("  Objeto destino X (dejar)   [19.0]: ") or 19.0)
        y_obj_fin = float(input("  Objeto destino Y (dejar)   [-4.0]: ") or -4.0)

        # 1. Compensar radialmente el largo de la pinza para obtener la posición de la muñeca (L2)
        x_l2_ini, y_l2_ini = compensar_pinza(x_obj_ini, y_obj_ini)
        x_l2_fin, y_l2_fin = compensar_pinza(x_obj_fin, y_obj_fin)

        # 2. Calcular IK exacta para origen y destino en Z = 15.5 cm
        q_ini = inverse_kinematics(x_l2_ini, y_l2_ini, Z_TRABAJO, codo_arriba=True)
        sb_ini, s0_ini, s1_ini = dh_a_servos(*q_ini)

        q_fin = inverse_kinematics(x_l2_fin, y_l2_fin, Z_TRABAJO, codo_arriba=True)
        sb_fin, s0_fin, s1_fin = dh_a_servos(*q_fin)

        # 3. Ángulos de levante del segundo brazo (s1 - 10°, respetando límite mínimo)
        s1_lev_ini = max(s1_ini - DELTA_LEVANTE, s1_min)
        s1_lev_fin = max(s1_fin - DELTA_LEVANTE, s1_min)

        # 4. Validar todos los estados antes de mover el hardware
        robot._validar_limites(sb_ini, s0_ini, s1_lev_ini)
        robot._validar_limites(sb_ini, s0_ini, s1_ini)
        robot._validar_limites(sb_fin, s0_fin, s1_lev_fin)
        robot._validar_limites(sb_fin, s0_fin, s1_fin)

        print(f"\n-> Origen  Obj({x_obj_ini:.1f}, {y_obj_ini:.1f}) -> L2({x_l2_ini:.2f}, {y_l2_ini:.2f}, {Z_TRABAJO}) | Base={sb_ini}°, s0={s0_ini}°, s1={s1_ini}° (levante: {s1_lev_ini}°)")
        print(f"-> Destino Obj({x_obj_fin:.1f}, {y_obj_fin:.1f}) -> L2({x_l2_fin:.2f}, {y_l2_fin:.2f}, {Z_TRABAJO}) | Base={sb_fin}°, s0={s0_fin}°, s1={s1_fin}° (levante: {s1_lev_fin}°)")

        # --- EJECUCIÓN DE LA SECUENCIA ---
        print("\n-> [1/8] Abriendo pinza...")
        robot.abrir_pinza()

        print("-> [2/8] Ubicando sobre el origen y bajando a la pieza...")
        robot.mover_servos(sb_ini, s0_ini, s1_lev_ini, suave=True)
        robot.mover_servos(sb_ini, s0_ini, s1_ini, suave=True)

        print("-> [3/8] Cerrando garra para sujetar la pieza...")
        robot.agarrar_objeto()
        time.sleep(0.3)

        print(f"-> [4/8] Levantando segundo brazo (s1 -> {s1_lev_ini:.1f}°)...")
        robot.mover_servos(sb_ini, s0_ini, s1_lev_ini, suave=True)

        print("-> [5/8] Trasladando pieza hacia el destino (brazo levantado)...")
        robot.mover_servos(sb_fin, s0_fin, s1_lev_fin, suave=True)

        print(f"-> [6/8] Bajando pieza al destino exacto (s1 -> {s1_fin:.1f}°)...")
        robot.mover_servos(sb_fin, s0_fin, s1_fin, suave=True)

        print("-> [7/8] Abriendo garra para soltar la pieza...")
        robot.abrir_pinza()
        time.sleep(0.3)

        print(f"-> [8/8] Levantando segundo brazo (s1 -> {s1_lev_fin:.1f}°) y regresando a inicio...")
        robot.mover_servos(sb_fin, s0_fin, s1_lev_fin, suave=True)
        robot.ir_a_inicio(suave=True)
        mostrar_estado_actual(robot)

    except ValueError as e:
        print(f"\nSecuencia abortada por seguridad: {e}\n")


def main():
    print("==========================================================")
    print("  CONTROL CINEMÁTICO D-H DEL MANIPULADOR 3-DOF (RRR)")
    print("==========================================================")
    robot = RobotController(simular=False)
    mostrar_estado_actual(robot)

    while True:
        print("Menú:")
        print("  1. Cinemática Directa (Ángulos de servos)")
        print("  2. Cinemática Inversa (XYZ)")
        print("  3. Control Manual de Pinza (Abrir / Agarrar / Cerrar)")
        print("  4. Secuencia Pick & Place")
        print("  5. Regresar a Posición Inicial")
        print("  6. Salir")

        opcion = input("Opción > ").strip()

        if opcion == "1":
            modo_fk(robot)
        elif opcion == "2":
            modo_ik_lineal(robot)
        elif opcion == "3":
            modo_pinza_manual(robot)
        elif opcion == "4":
            modo_pick_and_place(robot)
        elif opcion == "5":
            robot.ir_a_inicio(suave=True)
            mostrar_estado_actual(robot)
        elif opcion == "6":
            print("Regresando a posición inicial antes de salir...")
            robot.ir_a_inicio(suave=True)
            break
        else:
            print("Opción no válida.\n")


if __name__ == "__main__":
    main()