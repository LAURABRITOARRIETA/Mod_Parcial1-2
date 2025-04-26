import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button, RadioButtons

# ==================== MODELOS CINEMÁTICOS Y DINÁMICOS ====================
# El modelo cinemático de un robot de tracción diferencial se basa en las siguientes ecuaciones:
# - v = (v_derecha + v_izquierda) / 2       → velocidad lineal
# - ω = (v_derecha - v_izquierda) / L       → velocidad angular
# - x_dot = v * cos(θ)                      → derivada de posición en X
# - y_dot = v * sin(θ)                      → derivada de posición en Y
# - θ_dot = ω                               → derivada de orientación

# Donde:
#   - v_derecha, v_izquierda: velocidades de cada rueda (rad/s * r)
#   - L: distancia entre ruedas
#   - θ: ángulo de orientación del robot

# El modelo dinámico se aproxima considerando:
#   - Torque de motor T = Kt * I
#   - Velocidad del eje ω = (V - I*R) / Ke
#   - Se desprecia masa, fricción y otras fuerzas externas para esta simulación simple

class MotorDC:
    def __init__(self):
        self.R = 5.45
        self.Ke = 0.0095
        self.Kt = 0.0095
        self.voltaje_max = 12.0
        self.corriente_max = 2.2
        self.velocidad_sin_carga = 350
        self.RPM_A_RADPS = 2 * np.pi / 60
        self.RADPS_A_RPM = 60 / (2 * np.pi)

    def obtener_velocidad(self, voltaje, torque_carga=0):
        voltaje = min(max(voltaje, -self.voltaje_max), self.voltaje_max)
        if voltaje == 0:
            return 0
        rpm = (self.velocidad_sin_carga / self.voltaje_max) * voltaje
        torque_parada = 0.2
        factor_carga = max(0, 1 - (torque_carga / torque_parada))
        rpm *= factor_carga
        return rpm

class RobotTraccionDiferencial:
    def __init__(self):
        self.radio_rueda = 0.05
        self.distancia_ruedas = 0.2
        self.longitud_robot = 0.3
        self.ancho_robot = 0.2
        self.motor_izquierdo = MotorDC()
        self.motor_derecho = MotorDC()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.rpm_izquierdo = 0.0
        self.rpm_derecho = 0.0

    def actualizar(self, voltaje_izquierdo, voltaje_derecho, dt):
        self.rpm_izquierdo = self.motor_izquierdo.obtener_velocidad(voltaje_izquierdo)
        self.rpm_derecho = self.motor_derecho.obtener_velocidad(voltaje_derecho)
        radps_izquierdo = self.rpm_izquierdo * self.motor_izquierdo.RPM_A_RADPS
        radps_derecho = self.rpm_derecho * self.motor_derecho.RPM_A_RADPS
        v_izquierda = radps_izquierdo * self.radio_rueda
        v_derecha = radps_derecho * self.radio_rueda
        v = (v_derecha + v_izquierda) / 2
        omega = (v_derecha - v_izquierda) / self.distancia_ruedas
        self.x += v * dt * np.cos(self.theta)
        self.y += v * dt * np.sin(self.theta)
        self.theta += omega * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        return self.x, self.y, self.theta, v, omega

def ejecutar_simulacion():
    global tiempo_actual
    robot = RobotTraccionDiferencial()
    fig, ax = plt.subplots(figsize=(12, 8))
    plt.subplots_adjust(left=0.1, bottom=0.35)
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title('Simulación de Robot con Tracción Diferencial')
    cuerpo_robot = plt.Rectangle((0, 0), robot.longitud_robot, robot.ancho_robot, angle=0, fc='blue', alpha=0.7)
    ax.add_patch(cuerpo_robot)
    path_x, path_y = [robot.x], [robot.y]
    linea_trayectoria, = ax.plot(path_x, path_y, 'g-', alpha=0.7, linewidth=2)
    flecha_patch = None

    ax_voltaje_izq = plt.axes([0.1, 0.25, 0.35, 0.03])
    ax_voltaje_der = plt.axes([0.1, 0.2, 0.35, 0.03])
    ax_reset = plt.axes([0.55, 0.25, 0.1, 0.05])
    ax_borrar = plt.axes([0.7, 0.25, 0.1, 0.05])
    ax_modos = plt.axes([0.55, 0.05, 0.25, 0.15])

    slider_izq = Slider(ax_voltaje_izq, 'Voltaje Izq (V)', -12, 12, valinit=0, color='lightblue')
    slider_der = Slider(ax_voltaje_der, 'Voltaje Der (V)', -12, 12, valinit=0, color='lightgreen')
    boton_reset = Button(ax_reset, 'Reiniciar', color='lightcoral')
    boton_borrar = Button(ax_borrar, 'Borrar Trayectoria', color='lightblue')
    modos = ('Manual', 'Adelante', 'Círculo', 'Zigzag')
    radio_modos = RadioButtons(ax_modos, modos)

    panel_info = plt.axes([0.85, 0.5, 0.13, 0.4])
    panel_info.axis('off')
    texto_info = panel_info.text(0.05, 0.95, '', transform=panel_info.transAxes, fontsize=10, verticalalignment='top')

    dt = 0.05
    tiempo_actual = 0
    valores_actuales = {'voltaje_izquierdo': 0, 'voltaje_derecho': 0, 'modo': 'Manual'}

    def actualizar_modo(modo):
        valores_actuales['modo'] = modo
        if modo == 'Adelante':
            slider_izq.set_val(6)
            slider_der.set_val(6)

    radio_modos.on_clicked(actualizar_modo)

    def reiniciar(event):
        global tiempo_actual
        robot.x = 0
        robot.y = 0
        robot.theta = 0
        path_x.clear()
        path_y.clear()
        path_x.append(robot.x)
        path_y.append(robot.y)
        linea_trayectoria.set_data(path_x, path_y)
        tiempo_actual = 0
        fig.canvas.draw_idle()

    boton_reset.on_clicked(reiniciar)

    def borrar_trayectoria(event):
        path_x.clear()
        path_y.clear()
        path_x.append(robot.x)
        path_y.append(robot.y)
        linea_trayectoria.set_data(path_x, path_y)
        fig.canvas.draw_idle()

    boton_borrar.on_clicked(borrar_trayectoria)

    def actualizar_izquierdo(val):
        valores_actuales['voltaje_izquierdo'] = val

    def actualizar_derecho(val):
        valores_actuales['voltaje_derecho'] = val

    slider_izq.on_changed(actualizar_izquierdo)
    slider_der.on_changed(actualizar_derecho)

    def actualizar(frame):
        nonlocal flecha_patch
        global tiempo_actual
        tiempo_actual += dt
        t = tiempo_actual

        modo = valores_actuales['modo']
        if modo == 'Manual':
            voltaje_izquierdo = valores_actuales['voltaje_izquierdo']
            voltaje_derecho = valores_actuales['voltaje_derecho']
        elif modo == 'Adelante':
            voltaje_izquierdo = voltaje_derecho = 6
        elif modo == 'Círculo':
            voltaje_izquierdo = 6 + 3 * np.sin(t * 0.3)
            voltaje_derecho = 6 - 3 * np.sin(t * 0.3)
            slider_izq.set_val(voltaje_izquierdo)
            slider_der.set_val(voltaje_derecho)
        elif modo == 'Zigzag':
            voltaje_izquierdo = 6 + 4 * np.sin(t * 0.7)
            voltaje_derecho = 6 - 4 * np.sin(t * 0.7)
            slider_izq.set_val(voltaje_izquierdo)
            slider_der.set_val(voltaje_derecho)

        x, y, theta, v, omega = robot.actualizar(voltaje_izquierdo, voltaje_derecho, dt)

        if 'tiempos' not in valores_actuales:
            valores_actuales['tiempos'] = []
            valores_actuales['velocidades'] = []
            valores_actuales['omegas'] = []

        valores_actuales['tiempos'].append(t)
        valores_actuales['velocidades'].append(v)
        valores_actuales['omegas'].append(omega)

        if x < -3 or x > 3:
            robot.theta = np.pi - robot.theta
        if y < -3 or y > 3:
            robot.theta = -robot.theta
        robot.x = np.clip(robot.x, -3, 3)
        robot.y = np.clip(robot.y, -3, 3)

        cuerpo_robot.set_xy((robot.x - robot.longitud_robot/2, robot.y - robot.ancho_robot/2))
        cuerpo_robot.angle = np.degrees(robot.theta)

        if flecha_patch:
            flecha_patch.remove()

        flecha_patch = ax.add_patch(plt.Arrow(robot.x, robot.y, 0.2*np.cos(robot.theta), 0.2*np.sin(robot.theta), width=0.1, fc='red'))
        path_x.append(robot.x)
        path_y.append(robot.y)
        linea_trayectoria.set_data(path_x, path_y)

        texto_info.set_text(
            f"Tiempo: {t:.1f}s\n\n"
            f"X: {robot.x:.2f} m\nY: {robot.y:.2f} m\n\n"
            f"θ: {np.degrees(robot.theta):.1f}°\n\n"
            f"Izq: {voltaje_izquierdo:.1f}V ({robot.rpm_izquierdo:.0f} RPM)\n"
            f"Der: {voltaje_derecho:.1f}V ({robot.rpm_derecho:.0f} RPM)\n\n"
            f"Modo: {modo}")

        return cuerpo_robot, linea_trayectoria

    ani = FuncAnimation(fig, actualizar, interval=50, blit=False)
    plt.show()

    tiempos = valores_actuales.get('tiempos', [])
    velocidades = valores_actuales.get('velocidades', [])
    omegas = valores_actuales.get('omegas', [])

    if tiempos:
        plt.figure(figsize=(10, 4))
        plt.subplot(1, 2, 1)
        plt.plot(tiempos, velocidades, label='Velocidad lineal (m/s)')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('v (m/s)')
        plt.title('Velocidad Lineal')
        plt.grid(True)
        plt.legend()

        plt.subplot(1, 2, 2)
        plt.plot(tiempos, omegas, label='Velocidad angular (rad/s)', color='orange')
        plt.xlabel('Tiempo (s)')
        plt.ylabel('ω (rad/s)')
        plt.title('Velocidad Angular')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    ejecutar_simulacion()
