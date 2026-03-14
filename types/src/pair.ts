import { z } from 'zod';

export const PairRequestSchema = z.object({
  ip: z.string(),
});

export const PairResponseSchema = z.object({
  status: z.enum(['waiting', 'paired']),
  ip: z.string(),
  role: z.enum(['device', 'peer']).optional(),
  partner_ip: z.string().optional(),
});

export const PairStatusResponseSchema = z.object({
  pending_count: z.number(),
  paired: z
    .object({
      device_ip: z.string(),
      peer_ip: z.string(),
    })
    .nullable(),
});

