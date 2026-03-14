import type { CommsRequest, Vector3 } from '@unihack/types';

/**
 * Build a minimal CommsRequest with the fields that matter for processing.
 */
export function makePacket(opts: {
  devicePos: Vector3;
  deviceVel: Vector3;
  peerPos: Vector3;
  peerSpeed: number;
  timestampUs?: number;
}): CommsRequest {
  return {
    shared_frame: {
      initial_yaw_locked: 1,
      initial_yaw_deg: 0,
      shared_frame_locked: 1,
      shared_yaw_offset_deg: 0,
    },
    position_velocity: {
      raw_pos: opts.devicePos,
      raw_vel: opts.deviceVel,
      corr_offset: { x: 0, y: 0, z: 0 },
      clamped_pos: opts.devicePos,
      clamped_vel: opts.deviceVel,
    },
    peer_state: {
      peer_id: 2,
      peer_t_us: opts.timestampUs ?? 0,
      peer_pos: opts.peerPos,
      peer_yaw_deg: 0,
      peer_init_yaw_deg: 0,
      peer_speed_mps: opts.peerSpeed,
    },
    relative_to_peer: {
      delta_xyz: { x: 0, y: 0, z: 0 },
      distance_m: 0,
      bearing_world_deg: 0,
      bearing_local_deg: 0,
    },
  };
}

/**
 * Generate a sequence of packets:
 * - `stationaryCount` stationary samples at `startPos`
 * - `movingCount` samples moving linearly to `endPos` at constant speed
 * - `stationaryCount` stationary samples at `endPos`
 */
export function makeRunPackets(opts?: {
  stationaryCount?: number;
  movingCount?: number;
  startPos?: Vector3;
  endPos?: Vector3;
}): CommsRequest[] {
  const still = opts?.stationaryCount ?? 10;
  const moving = opts?.movingCount ?? 20;
  const start = opts?.startPos ?? { x: 0, y: 0, z: 0 };
  const end = opts?.endPos ?? { x: 10, y: 0, z: 0 };

  const packets: CommsRequest[] = [];
  let t = 0;

  // Stationary at start
  for (let i = 0; i < still; i++) {
    packets.push(
      makePacket({
        devicePos: { ...start },
        deviceVel: { x: 0, y: 0, z: 0 },
        peerPos: { ...start, x: start.x + 1 },
        peerSpeed: 0,
        timestampUs: t++ * 10000,
      }),
    );
  }

  // Moving from start to end
  for (let i = 0; i < moving; i++) {
    const frac = (i + 1) / moving;
    const pos = {
      x: start.x + (end.x - start.x) * frac,
      y: start.y + (end.y - start.y) * frac,
      z: start.z + (end.z - start.z) * frac,
    };
    const speed = 1.0;
    packets.push(
      makePacket({
        devicePos: pos,
        deviceVel: {
          x: (end.x - start.x) / moving * speed,
          y: (end.y - start.y) / moving * speed,
          z: (end.z - start.z) / moving * speed,
        },
        peerPos: { ...pos, x: pos.x + 1 },
        peerSpeed: speed,
        timestampUs: t++ * 10000,
      }),
    );
  }

  // Stationary at end
  for (let i = 0; i < still; i++) {
    packets.push(
      makePacket({
        devicePos: { ...end },
        deviceVel: { x: 0, y: 0, z: 0 },
        peerPos: { ...end, x: end.x + 1 },
        peerSpeed: 0,
        timestampUs: t++ * 10000,
      }),
    );
  }

  return packets;
}
