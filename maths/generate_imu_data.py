import argparse
import json
import math
import random
import sys
from dataclasses import dataclass

# ============================================================
# CONFIG
# ============================================================

SEED = 42
DT = 0.010                    # 10 ms packet interval
SEGMENT_LENGTH_M = 10.0       # each segment is 10 metres
NUM_SEGMENTS = 20             # how many direction changes
SPEED_MPS = 1.25              # constant walking speed
TURN_STD_DEG = 35.0           # how much heading can change per segment
PITCH_STD_DEG = 6.0           # cave up/down tilt variation
START_SEPARATION_M = 1.5      # initial separation between the two devices
NOISE_POS_STD = 0.02          # optional measurement noise in metres
NOISE_YAW_STD_DEG = 0.8       # optional yaw noise
OUTPUT_FILE = "imu_cave_packets.jsonl"

random.seed(SEED)


# ============================================================
# MATH HELPERS
# ============================================================

def deg(rad: float) -> float:
    return math.degrees(rad)

def rad(deg_val: float) -> float:
    return math.radians(deg_val)

def clamp_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle

def vec_len(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)

def normalize(x: float, y: float, z: float):
    length = vec_len(x, y, z)
    if length < 1e-9:
        return 0.0, 0.0, 0.0
    return x / length, y / length, z / length

def direction_from_yaw_pitch(yaw_deg: float, pitch_deg: float):
    """
    yaw: heading in XY plane
    pitch: tilt up/down
    """
    yaw_r = rad(yaw_deg)
    pitch_r = rad(pitch_deg)

    x = math.cos(pitch_r) * math.cos(yaw_r)
    y = math.cos(pitch_r) * math.sin(yaw_r)
    z = math.sin(pitch_r)
    return normalize(x, y, z)

def bearing_world_deg(dx: float, dy: float) -> float:
    return deg(math.atan2(dy, dx))

def bearing_local_deg(self_yaw_deg: float, dx: float, dy: float) -> float:
    world_bearing = bearing_world_deg(dx, dy)
    return clamp_angle_deg(world_bearing - self_yaw_deg)


# ============================================================
# DATA TYPES
# ============================================================

@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def copy(self):
        return Vec3(self.x, self.y, self.z)

@dataclass
class SharedFrame:
    initial_yaw_locked: int
    initial_yaw_deg: float
    shared_frame_locked: int
    shared_yaw_offset_deg: float

@dataclass
class PositionVelocity:
    raw_pos: Vec3
    raw_vel: Vec3
    corr_offset: Vec3
    clamped_pos: Vec3
    clamped_vel: Vec3

@dataclass
class PeerState:
    peer_id: int
    peer_t_us: int
    peer_pos: Vec3
    peer_yaw_deg: float
    peer_init_yaw_deg: float
    peer_speed_mps: float

@dataclass
class RelativeToPeer:
    delta_xyz: Vec3
    distance_m: float
    bearing_world_deg: float
    bearing_local_deg: float


# ============================================================
# SIMULATED DEVICE
# ============================================================

class SimDevice:
    def __init__(self, device_id: int, start_pos: Vec3, yaw_deg: float):
        self.device_id = device_id
        self.true_pos = start_pos.copy()
        self.raw_pos = start_pos.copy()
        self.clamped_pos = start_pos.copy()

        self.vel = Vec3(0.0, 0.0, 0.0)
        self.clamped_vel = Vec3(0.0, 0.0, 0.0)
        self.corr_offset = Vec3(0.0, 0.0, 0.0)

        self.yaw_deg = yaw_deg
        self.pitch_deg = 0.0
        self.initial_yaw_deg = yaw_deg

        self.segment_dir = direction_from_yaw_pitch(self.yaw_deg, self.pitch_deg)
        self.distance_in_segment = 0.0
        self.speed_mps = SPEED_MPS

    def choose_new_segment(self, base_yaw_deg=None):
        """
        Pick a new 10 m segment heading.
        base_yaw_deg lets device 2 roughly follow device 1 but not exactly.
        """
        if base_yaw_deg is None:
            self.yaw_deg = clamp_angle_deg(self.yaw_deg + random.gauss(0.0, TURN_STD_DEG))
        else:
            self.yaw_deg = clamp_angle_deg(base_yaw_deg + random.gauss(0.0, 12.0))

        self.pitch_deg = max(-20.0, min(20.0, random.gauss(0.0, PITCH_STD_DEG)))
        self.segment_dir = direction_from_yaw_pitch(self.yaw_deg, self.pitch_deg)
        self.distance_in_segment = 0.0

    def step(self, dt: float):
        dx = self.segment_dir[0] * self.speed_mps * dt
        dy = self.segment_dir[1] * self.speed_mps * dt
        dz = self.segment_dir[2] * self.speed_mps * dt

        self.true_pos.x += dx
        self.true_pos.y += dy
        self.true_pos.z += dz

        self.vel = Vec3(
            self.segment_dir[0] * self.speed_mps,
            self.segment_dir[1] * self.speed_mps,
            self.segment_dir[2] * self.speed_mps,
        )

        # Simulate raw IMU-derived position having small drift/noise
        self.raw_pos = Vec3(
            self.true_pos.x + random.gauss(0.0, NOISE_POS_STD),
            self.true_pos.y + random.gauss(0.0, NOISE_POS_STD),
            self.true_pos.z + random.gauss(0.0, NOISE_POS_STD),
        )

        # For now clamped = raw, but this is where you'd inject your fusion logic
        self.clamped_pos = Vec3(
            self.raw_pos.x + self.corr_offset.x,
            self.raw_pos.y + self.corr_offset.y,
            self.raw_pos.z + self.corr_offset.z,
        )
        self.clamped_vel = self.vel.copy()

        self.distance_in_segment += self.speed_mps * dt

    def current_yaw_noisy(self):
        return clamp_angle_deg(self.yaw_deg + random.gauss(0.0, NOISE_YAW_STD_DEG))


# ============================================================
# SERIALIZATION
# ============================================================

def vec_to_dict(v: Vec3):
    return {
        "x": round(v.x, 3),
        "y": round(v.y, 3),
        "z": round(v.z, 3),
    }

def build_packet(self_dev: SimDevice, peer_dev: SimDevice, t_s: float):
    dx = peer_dev.clamped_pos.x - self_dev.clamped_pos.x
    dy = peer_dev.clamped_pos.y - self_dev.clamped_pos.y
    dz = peer_dev.clamped_pos.z - self_dev.clamped_pos.z

    distance = vec_len(dx, dy, dz)
    self_yaw = self_dev.current_yaw_noisy()
    peer_yaw = peer_dev.current_yaw_noisy()

    packet = {
        "t_us": int(t_s * 1_000_000),
        "self_id": self_dev.device_id,
        "shared_frame": {
            "initial_yaw_locked": 1,
            "initial_yaw_deg": round(self_dev.initial_yaw_deg, 2),
            "shared_frame_locked": 1,
            "shared_yaw_offset_deg": round(peer_dev.initial_yaw_deg - self_dev.initial_yaw_deg, 2),
        },
        "position_velocity": {
            "raw_pos": vec_to_dict(self_dev.raw_pos),
            "raw_vel": vec_to_dict(self_dev.vel),
            "corr_offset": vec_to_dict(self_dev.corr_offset),
            "clamped_pos": vec_to_dict(self_dev.clamped_pos),
            "clamped_vel": vec_to_dict(self_dev.clamped_vel),
        },
        "peer_state": {
            "peer_id": peer_dev.device_id,
            "peer_t_us": int(t_s * 1_000_000),
            "peer_pos": vec_to_dict(peer_dev.clamped_pos),
            "peer_yaw_deg": round(peer_yaw, 2),
            "peer_init_yaw_deg": round(peer_dev.initial_yaw_deg, 2),
            "peer_speed_mps": round(peer_dev.speed_mps, 3),
        },
        "relative_to_peer": {
            "delta_xyz": {
                "x": round(dx, 3),
                "y": round(dy, 3),
                "z": round(dz, 3),
            },
            "distance_m": round(distance, 3),
            "bearing_world_deg": round(bearing_world_deg(dx, dy), 2),
            "bearing_local_deg": round(bearing_local_deg(self_yaw, dx, dy), 2),
        },
    }
    return packet


# ============================================================
# SIMULATION
# ============================================================

def run_sim():
    dev1 = SimDevice(
        device_id=1,
        start_pos=Vec3(0.0, 0.0, 0.0),
        yaw_deg=0.0,
    )

    dev2 = SimDevice(
        device_id=2,
        start_pos=Vec3(START_SEPARATION_M, 0.0, 0.0),
        yaw_deg=8.0,
    )

    dev1.initial_yaw_deg = dev1.yaw_deg
    dev2.initial_yaw_deg = dev2.yaw_deg

    packets_written = 0
    total_time = 0.0

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        for seg_idx in range(NUM_SEGMENTS):
            # Device 1 picks a fresh cave segment
            dev1.choose_new_segment()

            # Device 2 roughly follows, but with some deviation
            dev2.choose_new_segment(base_yaw_deg=dev1.yaw_deg + random.gauss(0.0, 15.0))

            steps_per_segment = int(SEGMENT_LENGTH_M / (SPEED_MPS * DT))

            for _ in range(steps_per_segment):
                dev1.step(DT)
                dev2.step(DT)

                pkt1 = build_packet(dev1, dev2, total_time)
                pkt2 = build_packet(dev2, dev1, total_time)

                f.write(json.dumps(pkt1) + "\n")
                f.write(json.dumps(pkt2) + "\n")

                packets_written += 2
                total_time += DT

    print("Done.")
    print(f"Wrote {packets_written} packets to {OUTPUT_FILE}")
    print(f"Simulated time: {total_time:.2f} s")
    print(f"Segments per device: {NUM_SEGMENTS}")
    print(f"Packet period: {DT * 1000:.1f} ms")


def post_to_api(base_url: str):
    """Generate packets and POST them to the backend API."""
    import urllib.request
    import urllib.error

    def api_post(path: str, payload: dict) -> dict:
        url = f"{base_url}{path}"
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as resp:
            return json.loads(resp.read().decode("utf-8"))

    # Start a run
    result = api_post("/api/comms/start", {"start": 1})
    run_id = result.get("run_id", "unknown")
    print(f"Started run: {run_id}")

    dev1 = SimDevice(device_id=1, start_pos=Vec3(0.0, 0.0, 0.0), yaw_deg=0.0)
    dev2 = SimDevice(device_id=2, start_pos=Vec3(START_SEPARATION_M, 0.0, 0.0), yaw_deg=8.0)
    dev1.initial_yaw_deg = dev1.yaw_deg
    dev2.initial_yaw_deg = dev2.yaw_deg

    packets_sent = 0
    total_time = 0.0

    for seg_idx in range(NUM_SEGMENTS):
        dev1.choose_new_segment()
        dev2.choose_new_segment(base_yaw_deg=dev1.yaw_deg + random.gauss(0.0, 15.0))
        steps_per_segment = int(SEGMENT_LENGTH_M / (SPEED_MPS * DT))

        for _ in range(steps_per_segment):
            dev1.step(DT)
            dev2.step(DT)

            pkt1 = build_packet(dev1, dev2, total_time)
            pkt2 = build_packet(dev2, dev1, total_time)

            # Strip fields not in CommsRequestSchema
            for pkt in (pkt1, pkt2):
                pkt.pop("t_us", None)
                pkt.pop("self_id", None)

            api_post("/api/comms/", pkt1)
            api_post("/api/comms/", pkt2)

            packets_sent += 2
            total_time += DT

        print(f"  Segment {seg_idx + 1}/{NUM_SEGMENTS} done ({packets_sent} packets sent)")

    # Stop the run
    api_post("/api/comms/stop", {"stop": 1})
    print(f"Run {run_id} stopped. {packets_sent} packets sent in {total_time:.2f}s simulated time.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate IMU cave navigation data")
    parser.add_argument("--post", action="store_true", help="POST packets to backend API instead of writing to file")
    parser.add_argument("--url", default="http://localhost:3000", help="Backend base URL (default: http://localhost:3000)")
    args = parser.parse_args()

    if args.post:
        post_to_api(args.url)
    else:
        run_sim()
