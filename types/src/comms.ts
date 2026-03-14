import { Vector3Schema } from './vector';
import { z } from 'zod';

export const CommsRequestSchema = z.object({
  shared_frame: z.object({
    initial_yaw_locked: z.literal(0).or(z.literal(1)),
    initial_yaw_deg: z.number(),
    shared_frame_locked: z.literal(0).or(z.literal(1)),
    shared_yaw_offset_deg: z.number(),
  }),
  position_velocity: z.object({
    raw_pos: Vector3Schema,
    raw_vel: Vector3Schema,
    corr_offset: Vector3Schema,
    clamped_pos: Vector3Schema,
    clamped_vel: Vector3Schema,
  }),
  peer_state: z.object({
    peer_id: z.number(),
    peer_t_us: z.number(),
    peer_pos: Vector3Schema,
    peer_yaw_deg: z.number(),
    peer_init_yaw_deg: z.number(),
    peer_speed_mps: z.number(),
  }),
  relative_to_peer: z.object({
    delta_xyz: Vector3Schema,
    distance_m: z.number(),
    bearing_world_deg: z.number(),
    bearing_local_deg: z.number(),
  }),
});

export const CommsResponseSchema = z.object({
  comms: z.array(z.string()),
});

export type CommsRequest = z.infer<typeof CommsRequestSchema>;
export type CommsResponse = z.infer<typeof CommsResponseSchema>;
