import { z } from "zod";

export const CommsResponseSchema = z.object({
  comms: z.array(z.string()),
});

export type CommsResponse = z.infer<typeof CommsResponseSchema>;
