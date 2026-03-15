import type { Packet, Vector3 } from '@unihack/types';

const DT = 0.01;
const SEGMENT_LENGTH = 10;
const NUM_SEGMENTS = 15;
const EMIT_EVERY = 10; // downsample: emit 1 packet per 10 steps
const SPEED = 1.25;
const TURN_STD = 35;
const PITCH_STD = 10;
const PITCH_BIAS = -30; // downward bias — caves go deeper
const SEPARATION = 1.5;
const NOISE_POS = 0.02;
const NOISE_YAW = 0.8;

function rad(deg: number): number {
  return (deg * Math.PI) / 180;
}

function clampAngle(angle: number): number {
  while (angle > 180) angle -= 360;
  while (angle <= -180) angle += 360;
  return angle;
}

function vecLen(x: number, y: number, z: number): number {
  return Math.sqrt(x * x + y * y + z * z);
}

function gauss(mean: number, std: number): number {
  // Box-Muller transform
  const u1 = Math.random();
  const u2 = Math.random();
  return mean + std * Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
}

function direction(yawDeg: number, pitchDeg: number): [number, number, number] {
  const y = rad(yawDeg);
  const p = rad(pitchDeg);
  const cp = Math.cos(p);
  const len = vecLen(cp * Math.cos(y), cp * Math.sin(y), Math.sin(p));
  if (len < 1e-9) return [0, 0, 0];
  return [(cp * Math.cos(y)) / len, (cp * Math.sin(y)) / len, Math.sin(p) / len];
}

class SimDevice {
  id: number;
  pos: Vector3;
  vel: Vector3 = { x: 0, y: 0, z: 0 };
  yaw: number;
  pitch = 0;
  initYaw: number;
  speed = SPEED;
  private dir: [number, number, number];
  private distInSeg = 0;

  constructor(id: number, pos: Vector3, yaw: number) {
    this.id = id;
    this.pos = { ...pos };
    this.yaw = yaw;
    this.initYaw = yaw;
    this.dir = direction(yaw, 0);
  }

  newSegment(baseYaw?: number): void {
    this.yaw = baseYaw != null
      ? clampAngle(baseYaw + gauss(0, 12))
      : clampAngle(this.yaw + gauss(0, TURN_STD));
    this.pitch = Math.max(-30, Math.min(5, gauss(PITCH_BIAS, PITCH_STD)));
    this.dir = direction(this.yaw, this.pitch);
    this.distInSeg = 0;
  }

  step(dt: number): void {
    const [dx, dy, dz] = this.dir;
    this.pos.x += dx * this.speed * dt;
    this.pos.y += dy * this.speed * dt;
    this.pos.z += dz * this.speed * dt;
    this.vel = { x: dx * this.speed, y: dy * this.speed, z: dz * this.speed };
    this.distInSeg += this.speed * dt;
  }

  noisyPos(): Vector3 {
    return {
      x: this.pos.x + gauss(0, NOISE_POS),
      y: this.pos.y + gauss(0, NOISE_POS),
      z: this.pos.z + gauss(0, NOISE_POS),
    };
  }

  noisyYaw(): number {
    return clampAngle(this.yaw + gauss(0, NOISE_YAW));
  }
}

function round3(v: Vector3): Vector3 {
  return {
    x: Math.round(v.x * 1000) / 1000,
    y: Math.round(v.y * 1000) / 1000,
    z: Math.round(v.z * 1000) / 1000,
  };
}

function buildPacket(self: SimDevice, peer: SimDevice, tUs: number): Packet {
  const selfPos = round3(self.noisyPos());
  const peerPos = round3(peer.noisyPos());
  const dx = peerPos.x - selfPos.x;
  const dy = peerPos.y - selfPos.y;
  const dz = peerPos.z - selfPos.z;
  const dist = vecLen(dx, dy, dz);
  const selfYaw = self.noisyYaw();

  return {
    shared_frame: {
      initial_yaw_locked: 1,
      initial_yaw_deg: Math.round(self.initYaw * 100) / 100,
      shared_frame_locked: 1,
      shared_yaw_offset_deg: Math.round((peer.initYaw - self.initYaw) * 100) / 100,
    },
    position_velocity: {
      raw_pos: selfPos,
      raw_vel: round3(self.vel),
      corr_offset: { x: 0, y: 0, z: 0 },
      clamped_pos: selfPos,
      clamped_vel: round3(self.vel),
    },
    peer_state: {
      peer_id: peer.id,
      peer_t_us: tUs,
      peer_pos: peerPos,
      peer_yaw_deg: Math.round(peer.noisyYaw() * 100) / 100,
      peer_init_yaw_deg: Math.round(peer.initYaw * 100) / 100,
      peer_speed_mps: Math.round(peer.speed * 1000) / 1000,
    },
    relative_to_peer: {
      delta_xyz: round3({ x: dx, y: dy, z: dz }),
      distance_m: Math.round(dist * 1000) / 1000,
      bearing_world_deg: Math.round(Math.atan2(dy, dx) * (180 / Math.PI) * 100) / 100,
      bearing_local_deg: Math.round(
        clampAngle(Math.atan2(dy, dx) * (180 / Math.PI) - selfYaw) * 100,
      ) / 100,
    },
  };
}

export function generateDemoPackets(): Packet[] {
  const dev1 = new SimDevice(1, { x: 0, y: 0, z: 0 }, 0);
  const dev2 = new SimDevice(2, { x: SEPARATION, y: 0, z: 0 }, 8);

  const packets: Packet[] = [];
  let t = 0;
  const stepsPerSeg = Math.floor(SEGMENT_LENGTH / (SPEED * DT));

  for (let seg = 0; seg < NUM_SEGMENTS; seg++) {
    dev1.newSegment();
    dev2.newSegment(dev1.yaw + gauss(0, 15));

    for (let s = 0; s < stepsPerSeg; s++) {
      dev1.step(DT);
      dev2.step(DT);
      t += DT;

      if (s % EMIT_EVERY === 0) {
        const tUs = Math.round(t * 1_000_000);
        packets.push(buildPacket(dev1, dev2, tUs));
      }
    }
  }

  return packets;
}
