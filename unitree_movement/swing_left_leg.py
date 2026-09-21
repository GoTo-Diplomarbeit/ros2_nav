"""
swing_left_legs.py — turn/pedal the LEFT legs of the Go2 forever.

1. Start the simulator:  cd ~/unitree_mujoco/simulate_python && python3 unitree_mujoco.py
2. Run this script (PyCharm with the WSL venv interpreter, or any WSL terminal).
3. Ctrl+C to stop.
"""
import math
import sys
import time

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# ----------------------------- tune me -----------------------------
DDS_DOMAIN = 1            # must match the simulator (it uses 1)
DDS_NETIF  = "lo"         # loopback -> talks to the local sim only

MOVE = {"FL": True, "RL": True}     # front-left / rear-left leg
LEFT_JOINTS = {                     # (hip, thigh, calf) motor indices
    "FL": (3, 4, 5),
    "RL": (9, 10, 11),
}

FREQ_HZ = 0.8                                             # pedalling speed
AMP     = {"hip": 0.30, "thigh": 0.60, "calf": 1.00}      # radians
PHASE   = {"hip": 0.0, "thigh": 0.0, "calf": math.pi / 2} # knee lags -> circles

KP, KD = 40.0, 3.0      # sim-safe PD gains
DT     = 0.002          # 500 Hz command rate
RAMP_S = 1.0            # seconds to fade the gains in
# -------------------------------------------------------------------

low_state = None

def LowStateHandler(msg: LowState_):
    global low_state
    low_state = msg

def main():
    global low_state
    ChannelFactoryInitialize(DDS_DOMAIN, DDS_NETIF)

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(LowStateHandler, 10)

    # wait for the first robot state (5 s timeout)
    t0 = time.monotonic()
    while low_state is None:
        if time.monotonic() - t0 > 5.0:
            sys.exit("No rt/lowstate received — is the simulator running?")
        time.sleep(0.01)

    # current joint angles = centre of the motion (no jump at start)
    q0 = [low_state.motor_state[i].q for i in range(12)]
    print("Start joint angles:", [round(q, 3) for q in q0])

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01        # PMSM servo mode
        cmd.motor_cmd[i].q = cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kp = cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    crc = CRC()
    w = 2.0 * math.pi * FREQ_HZ
    t = 0.0
    print("Pedalling the left legs forever — Ctrl+C to stop.")

    while True:
        step_start = time.perf_counter()
        t += DT
        ramp = min(1.0, t / RAMP_S)         # gentle gain fade-in

        # hold every joint at its start angle...
        for i in range(12):
            m = cmd.motor_cmd[i]
            m.q, m.dq, m.tau = q0[i], 0.0, 0.0
            m.kp, m.kd = KP * ramp, KD * ramp

        # ...then overlay the pedalling sine on the left legs
        s = w * t
        for leg, (hip, thigh, calf) in LEFT_JOINTS.items():
            if not MOVE[leg]:
                continue
            cmd.motor_cmd[hip].q   = q0[hip]   + AMP["hip"]   * math.sin(s + PHASE["hip"])
            cmd.motor_cmd[thigh].q = q0[thigh] + AMP["thigh"] * math.sin(s + PHASE["thigh"])
            cmd.motor_cmd[calf].q  = q0[calf]  + AMP["calf"]  * math.sin(s + PHASE["calf"])

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_left = DT - (time.perf_counter() - step_start)
        if time_left > 0:
            time.sleep(time_left)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")