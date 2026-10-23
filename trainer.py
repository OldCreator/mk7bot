"""MK7 Bot trainer (single file): Citra I/O + Gymnasium env + PPO training + export.
Usage:
  python trainer.py bin2json F   # convert a plugin-learned line_XXXX.bin -> line.json
  python trainer.py record       # OR drive one clean lap in Citra by hand -> line.json
  python trainer.py train        # PPO training (Citra focused, race savestate in the load slot)
  python trainer.py export       # -> policy.bin + line.bin for SD:/MK7Bot/
Needs: pip install gymnasium numpy stable-baselines3 torch vgamepad pynput
Observation/action layout MUST match plugin.cpp. UNTESTED against a live Citra.
"""
import json
import math
import socket
import struct
import sys
import time

try:
    import gymnasium as gym
    import numpy as np
    _Base = gym.Env
except ImportError:          # record/bin2json/export still need numpy-free paths only
    gym = np = None
    _Base = object


# ===================== Citra I/O =====================
# ===== TODO: same placeholder addresses as plugin.cpp (Citra uses the same virtual addresses) =====
ADDR = dict(x=0x0, z=0x0, yaw=0x0, speed=0x0, rank=0x0, item=0x0, in_race=0x0)


class CitraMem:
    def __init__(self, host="127.0.0.1", port=45987):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.settimeout(1.0)
        self.addr = (host, port)
        self.rid = 0

    def read(self, address, size=4):
        self.rid += 1
        pkt = struct.pack("<IIII", 1, self.rid, 1, 8) + struct.pack("<II", address, size)
        self.s.sendto(pkt, self.addr)
        data, _ = self.s.recvfrom(4096)
        _, _, _, sz = struct.unpack("<IIII", data[:16])
        return data[16:16 + sz]

    def f32(self, a):
        return struct.unpack("<f", self.read(a, 4))[0]

    def u32(self, a):
        return struct.unpack("<I", self.read(a, 4))[0]

    def kart(self):
        return dict(x=self.f32(ADDR["x"]), z=self.f32(ADDR["z"]), yaw=self.f32(ADDR["yaw"]),
                    speed=self.f32(ADDR["speed"]), rank=self.u32(ADDR["rank"]),
                    item=self.u32(ADDR["item"]), in_race=self.u32(ADDR["in_race"]))


class Pad:
    """Virtual Xbox pad (Windows, `pip install vgamepad`). In Citra: map Circle Pad to the
    left stick, A to the pad's A, and item (L) to the left shoulder. Analog steering needs this.
    On Linux, swap this class for a python-evdev/uinput gamepad."""

    def __init__(self):
        import vgamepad as vg
        self.vg = vg
        self.g = vg.VX360Gamepad()

    def set(self, steer, accel, item):
        vg = self.vg
        self.g.left_joystick_float(x_value_float=float(max(-1, min(1, steer))), y_value_float=0.0)
        for pressed, btn in ((accel, vg.XUSB_BUTTON.XUSB_GAMEPAD_A),
                             (item, vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)):
            (self.g.press_button if pressed else self.g.release_button)(button=btn)
        self.g.update()

    def release(self):
        self.set(0, False, False)


def load_state(keys=("ctrl", "l")):
    """Press Citra's 'Load from Slot' hotkey. Set it in Citra's hotkey settings, match here."""
    from pynput.keyboard import Controller, Key
    kb = Controller()
    mods = [getattr(Key, k) for k in keys[:-1]]
    for m in mods:
        kb.press(m)
    kb.press(keys[-1]); kb.release(keys[-1])
    for m in mods:
        kb.release(m)
    time.sleep(2.0)


# ===================== Gymnasium env =====================
LOOK = 3
MAX_SPEED = 100.0      # TODO: real max speed units (match kMaxSpeed in plugin.cpp)
RADIUS = 40.0          # match kPolicyRadius
STEP_HZ = 15
LAPS = 3
TIME_LIMIT = 150.0     # seconds per episode


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MK7Env(_Base):
    def __init__(self, line_path="line.json"):
        self.line = [tuple(p) for p in json.load(open(line_path))]
        self.n = len(self.line)
        self.mem, self.pad = CitraMem(), Pad()
        self.observation_space = gym.spaces.Box(-1.0, 1.0, (8,), np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (3,), np.float32)   # steer, accel, item

    def _obs(self, k):
        t1, t2 = self.line[self.idx % self.n], self.line[(self.idx + LOOK) % self.n]
        e1 = wrap(math.atan2(t1[0] - k["x"], t1[1] - k["z"]) - k["yaw"])
        e2 = wrap(math.atan2(t2[0] - k["x"], t2[1] - k["z"]) - k["yaw"])
        d = math.hypot(t1[0] - k["x"], t1[1] - k["z"])
        return np.array([math.sin(e1), math.cos(e1), math.sin(e2), math.cos(e2),
                         max(-1, min(1, k["speed"] / MAX_SPEED)), min(d / 200, 1),
                         k["rank"] / 8, 1.0 if k["item"] else 0.0], dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.pad.release()
        load_state()                       # savestate taken right after the race starts
        k = self.mem.kart()
        self.idx = min(range(self.n), key=lambda i: (self.line[i][0] - k["x"]) ** 2 + (self.line[i][1] - k["z"]) ** 2)
        self.start_idx, self.prev_speed, self.prev_rank = self.idx, k["speed"], k["rank"]
        self.t0, self.stuck_since = time.time(), None
        return self._obs(k), {}

    def step(self, a):
        self.pad.set(a[0], a[1] > 0, a[2] > 0.5)
        time.sleep(1 / STEP_HZ)            # NOTE: wall-clock pacing; keep Citra at 100% speed
        k = self.mem.kart()
        reward = 0.0
        while math.hypot(self.line[self.idx % self.n][0] - k["x"], self.line[self.idx % self.n][1] - k["z"]) < RADIUS:
            self.idx += 1
            reward += 1.0                  # waypoint progress
        reward += 0.01 * max(k["speed"], 0) / MAX_SPEED
        if self.prev_speed - k["speed"] > 0.3 * MAX_SPEED:
            reward -= 0.5                  # sudden speed loss = wall / hit
        reward += 0.3 * (self.prev_rank - k["rank"])   # moved up in rank
        self.prev_speed, self.prev_rank = k["speed"], k["rank"]

        now = time.time()
        if k["speed"] < 0.05 * MAX_SPEED:
            self.stuck_since = self.stuck_since or now
        else:
            self.stuck_since = None
        stuck = self.stuck_since is not None and now - self.stuck_since > 3
        finished = (self.idx - self.start_idx) >= LAPS * self.n
        timeout = now - self.t0 > TIME_LIMIT
        if stuck:
            reward -= 5
        if finished:
            reward += 50 + 20 * max(0, 9 - k["rank"])
        return self._obs(k), reward, finished, (stuck or timeout), {}


# ===================== Commands =====================
def record():
    from citra_io import CitraMem
    m, pts = CitraMem(), []
    print("Recording. Drive a lap, Ctrl+C when done.")
    try:
        while True:
            k = m.kart()
            if k["in_race"]:
                pts.append((k["x"], k["z"]))
            time.sleep(0.5)
    except KeyboardInterrupt:
        json.dump(pts, open("line.json", "w"))
        print(f"saved {len(pts)} points")


def bin2json(path):
    data = open(path, "rb").read()
    vals = struct.unpack("<%df" % (len(data) // 4), data)
    pts = [(vals[i], vals[i + 1]) for i in range(0, len(vals) - 1, 2)]
    json.dump(pts, open("line.json", "w"))
    print(f"wrote line.json with {len(pts)} points")


def train():
    import torch.nn as nn
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    
    env = MK7Env()
    model = PPO("MlpPolicy", env, n_steps=1024, batch_size=128, learning_rate=3e-4, verbose=1,
                policy_kwargs=dict(net_arch=dict(pi=[32, 32], vf=[64, 64]), activation_fn=nn.Tanh))
    model.learn(total_timesteps=2_000_000,
                callback=CheckpointCallback(20_000, "checkpoints", name_prefix="mk7"))
    model.save("mk7_ppo")


def export(model_path="mk7_ppo"):
    from stable_baselines3 import PPO
    p = PPO.load(model_path).policy
    layers = [p.mlp_extractor.policy_net[0], p.mlp_extractor.policy_net[2], p.action_net]
    blob = b""
    for l in layers:        # PyTorch weight shape is [out][in], matches plugin.cpp Dense()
        blob += l.weight.detach().cpu().numpy().astype("<f4").tobytes()
        blob += l.bias.detach().cpu().numpy().astype("<f4").tobytes()
    open("policy.bin", "wb").write(blob)
    pts = json.load(open("line.json"))
    open("line.bin", "wb").write(b"".join(struct.pack("<ff", *p) for p in pts))
    print("wrote policy.bin, line.bin -> copy to SD:/MK7Bot/")


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "bin2json":
        bin2json(sys.argv[2])
    else:
        {"record": record, "train": train, "export": export}[cmd]()
