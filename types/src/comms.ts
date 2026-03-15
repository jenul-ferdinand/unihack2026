import { Vector3Schema } from './vector';
import { z } from 'zod';

export const PacketSchema = z.object({
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

export const CommsRequestSchema = z.object({
  sample_count: z.number(),
  samples: z.array(PacketSchema),
});

export const CommsResponseSchema = z.object({
  success: z.boolean(),
});

export const CommsStartResponseSchema = z.object({
  run_id: z.string(),
});

export const CommsStartSchema = z.object({
  start: z.literal(1),
});

export const CommsStopSchema = z.object({
  stop: z.literal(1),
});

export const PathPointSchema = z.object({
  device_pos: Vector3Schema,
  peer_pos: Vector3Schema,
  timestamp: z.string(),
});

export const RunSummarySchema = z.object({
  run_id: z.string(),
  status: z.enum(['active', 'completed']),
  created_at: z.string(),
  stopped_at: z.string().nullable(),
  point_count: z.number(),
});

export const RunsListResponseSchema = z.object({
  runs: z.array(RunSummarySchema),
});

export const RunDetailResponseSchema = z.object({
  run_id: z.string(),
  status: z.enum(['active', 'completed']),
  created_at: z.string(),
  stopped_at: z.string().nullable(),
  path: z.array(PathPointSchema),
});

export type Packet = z.infer<typeof PacketSchema>;
export type CommsRequest = z.infer<typeof CommsRequestSchema>;
export type CommsResponse = z.infer<typeof CommsResponseSchema>;
export type CommsStartResponse = z.infer<typeof CommsStartResponseSchema>;
export type CommsStart = z.infer<typeof CommsStartSchema>;
export type CommsStop = z.infer<typeof CommsStopSchema>;
export type PathPoint = z.infer<typeof PathPointSchema>;
export type RunSummary = z.infer<typeof RunSummarySchema>;
export type RunsListResponse = z.infer<typeof RunsListResponseSchema>;
export type RunDetailResponse = z.infer<typeof RunDetailResponseSchema>;
