import time
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crtp import init_drivers
from cflib.positioning.motion_commander import MotionCommander

# Parâmetros físicos
g = 9.81
mass = 0.035

# Ganhos
Kp = np.diag([6, 6, 6])
Kv = np.diag([6, 6, 8])

# Saturações
def saturate(vec, max_vals):
    return np.clip(vec, -np.abs(max_vals), np.abs(max_vals))

# Conversão de aceleração para roll/pitch/thrust
def acc_to_cmd(u_a):
    # Assumindo yaw = 0
    thrust = mass * np.linalg.norm(u_a)
    pitch = np.arcsin(np.clip(u_a[0] / g, -0.4, 0.4))
    roll  = -np.arcsin(np.clip(u_a[1] / g, -0.4, 0.4))
    return roll, pitch, thrust

# Trajetória desejada
def trajectory(t):
    if t < 3.0:
        pd = np.array([0.0, 0.0, 2.0 * t / 3])
        vd = np.array([0.0, 0.0, 2.0 / 3])
        ad = np.array([0.0, 0.0, 0.0])
    else:
        t2 = t - 3.0
        r = 0.5
        omega = 0.6
        pd = np.array([r * np.sin(omega * t2),
                       r * np.cos(omega * t2),
                       2.0])
        vd = np.array([r * omega * np.cos(omega * t2),
                      -r * omega * np.sin(omega * t2),
                       0.0])
        ad = np.array([-r * omega**2 * np.sin(omega * t2),
                      -r * omega**2 * np.cos(omega * t2),
                       0.0])
    return pd, vd, ad

# Callback: envia roll, pitch, thrust
def control_loop(cf, duration=10, dt=0.02):
    p = np.array([0.0, 0.0, 0.0])
    v = np.array([0.0, 0.0, 0.0])
    max_acc = np.array([4.0, 4.0, 3.0])

    start_time = time.time()
    while time.time() - start_time < duration:
        t = time.time() - start_time
        pd, vd, ad = trajectory(t)

        e_p = pd - p
        e_v = vd - v

        u_a = ad + Kv @ e_v + Kp @ e_p + np.array([0, 0, g])
        u_a = saturate(u_a, max_acc)

        roll, pitch, thrust = acc_to_cmd(u_a)

        cf.commander.send_setpoint(roll * 180 / np.pi,
                                   pitch * 180 / np.pi,
                                   0,  # yaw rate
                                   int(thrust / g * 60000))  # aproximação de thrust

        # Aqui usarias feedback real do estado com logs (não simulado)
        # p = ...
        # v = ...
        time.sleep(dt)

    # Parar o drone
    cf.commander.send_setpoint(0, 0, 0, 0)

# Ligação e execução
def main():
    init_drivers(enable_debug_driver=False)
    uri = 'radio://0/80/2M/E7E7E7E7E7'  # mudar se necessário

    cf = Crazyflie(rw_cache='./cache')
    cf.connected.add_callback(lambda uri: control_loop(cf))
    cf.open_link(uri)
    time.sleep(12)
    cf.close_link()

if __name__ == '__main__':
    main()
