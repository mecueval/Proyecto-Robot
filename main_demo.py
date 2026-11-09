import numpy as np
from kinematics import (
    servos_a_dh,
    dh_a_servos,
    forward_kinematics,
    inverse_kinematics,
    linear_trajectory,
    get_jacobian,
    get_singularities,
    get_inverse_jacobian,
)
from hardware_controller import RobotController
import numpy as np
from config import LIMITES_SERVOS

# Formato limpio para todas las matrices (evita "0.", "1." y notación científica)
np.set_printoptions(
    precision=3,
    suppress=True,
    floatmode="fixed",
    formatter={"float_kind": lambda x: f"{0.0 if abs(x) < 1e-9 else x:8.3f}"}
)


def obtener_estado_actual(robot):
    s_base = robot.angulos_actuales["base"]
    s0 = robot.angulos_actuales["s0"]
    s1 = robot.angulos_actuales["s1"]
    q1, q2, q3 = servos_a_dh(s_base, s0, s1)
    T03, puntos = forward_kinematics(q1, q2, q3)
    return T03, puntos["efector"], (q1, q2, q3)


def mostrar_estado_actual(robot):
    s_base = robot.angulos_actuales["base"]
    s0 = robot.angulos_actuales["s0"]
    s1 = robot.angulos_actuales["s1"]
    T03, ef, (q1, q2, q3) = obtener_estado_actual(robot)

    J = get_jacobian(q1, q2, q3)
    es_sing, det_J = get_singularities(q2, q3)

    print("\n--- ESTADO ACTUAL DEL ROBOT ---")
    print(f"Servos físicos : Base = {s_base:.1f}° | s0 (Hombro) = {s0:.1f}° | s1 (Antebrazo) = {s1:.1f}°")
    print(f"Variables D-H  : q1 = {q1:.2f}°   | q2 = {q2:.2f}°          | q3 = {q3:.2f}°")
    print(f"Posición (FK)  : X = {ef[0]:.2f} | Y = {ef[1]:.2f} | Z = {ef[2]:.2f}")
    print("\nMatriz Homogénea T0^3:")
    print(np.round(T03, 3))
    print(f"\nDeterminante J : {det_J:.2f} | Singularidad: {'SÍ' if es_sing else 'NO'}")
    print("Matriz Jacobiana J(q):")
    print(np.round(J, 3))

    if not es_sing:
        J_inv = get_inverse_jacobian(q1, q2, q3)
        print("Matriz Jacobiana Inversa J^-1(q):")
        print(np.round(J_inv, 4))
    else:
        print("Matriz Jacobiana Inversa: No invertible (en singularidad).")
    print("-------------------------------\n")


def modo_fk(robot):
    """Evalúa Cinemática Directa desde ángulos articulares de servo y mueve el robot."""
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

        print(f"\n-> Predicción D-H: X = {ef[0]:.2f}, Y = {ef[1]:.2f}, Z = {ef[2]:.2f}")
        robot.mover_servos(s_base, s0, s1)
        mostrar_estado_actual(robot)

    except ValueError as e:
        print(f"Error: {e}\n")


def modo_ik_lineal(robot):
    """Resuelve Cinemática Inversa interpolando una trayectoria lineal desde la posición actual."""
    _, pos_actual, _ = obtener_estado_actual(robot)
    print("\n[2. CINEMÁTICA INVERSA CON TRAYECTORIA LINEAL (IK)]")
    print(f"  Origen actual P0 = ({pos_actual[0]:.2f}, {pos_actual[1]:.2f}, {pos_actual[2]:.2f})")

    try:
        x_fin = float(input("  Destino X: "))
        y_fin = float(input("  Destino Y: "))
        z_fin = float(input("  Destino Z: "))

        # Verifica solución final y muestra ángulos D-H objetivo
        q1_fin, q2_fin, q3_fin = inverse_kinematics(x_fin, y_fin, z_fin, codo_arriba=True)
        sb_fin, s0_fin, s1_fin = dh_a_servos(q1_fin, q2_fin, q3_fin)
        print(f"\n-> Solución D-H destino : q1 = {q1_fin:.2f}°, q2 = {q2_fin:.2f}°, q3 = {q3_fin:.2f}°")
        print(f"-> Servos destino       : Base = {sb_fin}°, s0 = {s0_fin}°, s1 = {s1_fin}°")

        # Interpola y valida toda la recta en el espacio cartesiano (25 pasos)
        secuencia, _ = linear_trajectory(
            pos_actual, (x_fin, y_fin, z_fin), num_pasos=25
        )
        print(f"-> Trayectoria lineal validada ({len(secuencia)} puntos). Ejecutando...")

        robot.ejecutar_trayectoria(secuencia, pausa_paso=0.035)
        mostrar_estado_actual(robot)

    except ValueError as e:
        print(f"Movimiento rechazado: {e}\n")


def test_consistencia_matematica():
    """Verifica analíticamente que IK(FK(q)) == q y que J @ J^-1 == I."""
    print("\n[3. TEST DE CONSISTENCIA D-H (FK <-> IK) Y DIFERENCIAL (J @ J^-1 == I)]")
    casos_prueba = [
        (97.5, 90.0, 90.0),   # Nueva posición inicial
        (97.5, 70.0, 80.0),   # Brazo extendido hacia adelante
        (67.5, 65.0, 100.0),  # Base girada + antebrazo más abajo
    ]
    for s_base, s0, s1 in casos_prueba:
        q1, q2, q3 = servos_a_dh(s_base, s0, s1)
        _, puntos = forward_kinematics(q1, q2, q3)
        x, y, z = puntos["efector"]

        # 1. Consistencia posicional (FK -> IK)
        q1_ik, q2_ik, q3_ik = inverse_kinematics(x, y, z)
        sb_ik, s0_ik, s1_ik = dh_a_servos(q1_ik, q2_ik, q3_ik)
        error_ang = max(abs(s_base - sb_ik), abs(s0 - s0_ik), abs(s1 - s1_ik))

        # 2. Consistencia diferencial (J * J^-1 = I)
        J = get_jacobian(q1, q2, q3)
        J_inv = get_inverse_jacobian(q1, q2, q3)
        _, det_J = get_singularities(q2, q3)
        error_jac = np.max(np.abs((J @ J_inv) - np.eye(3)))

        print(
            f"  Servos ({s_base}°, {s0}°, {s1}°) -> XYZ=({x:.2f}, {y:.2f}, {z:.2f}) "
            f"| Error IK: {error_ang:.4f}° | det(J): {det_J:.1f} | Error J*J^-1: {error_jac:.6f}"
        )
    print()


def main():
    print("==========================================================")
    print("  VALIDACIÓN CINEMÁTICA D-H Y CONTROL DE SERVOS DEL ROBOT")
    print("==========================================================")
    robot = RobotController(simular=False)
    mostrar_estado_actual(robot)

    while True:
        print("Menú de pruebas:")
        print("  1. Cinemática Directa (Ángulos de servos -> XYZ y Jacobiana)")
        print("  2. Cinemática Inversa (XYZ -> Trayectoria lineal)")
        print("  3. Test de Consistencia Matemática (FK <-> IK y J @ J^-1)")
        print("  4. Regresar a Posición Inicial")
        print("  5. Salir")

        opcion = input("Opción > ").strip()

        if opcion == "1":
            modo_fk(robot)
        elif opcion == "2":
            modo_ik_lineal(robot)
        elif opcion == "3":
            test_consistencia_matematica()
        elif opcion == "4":
            robot.ir_a_inicio(suave=True)
            mostrar_estado_actual(robot)
        elif opcion == "5":
            print("Regresando a posición inicial antes de salir...")
            robot.ir_a_inicio(suave=True)
            break
        else:
            print("Opción no válida.\n")


if __name__ == "__main__":
    main()