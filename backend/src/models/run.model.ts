import mongoose, { Schema, Document } from 'mongoose';
import type { Vector3 } from '@unihack/types';

export interface IPathPoint {
  device_pos: Vector3;
  peer_pos: Vector3;
  timestamp: string;
}

export interface IRun extends Document {
  status: 'active' | 'completed';
  created_at: Date;
  stopped_at: Date | null;
  raw_points: any[];
  path: IPathPoint[];
}

const Vec3Sub = { x: Number, y: Number, z: Number };

const PathPointSub = new Schema(
  {
    device_pos: Vec3Sub,
    peer_pos: Vec3Sub,
    timestamp: String,
  },
  { _id: false },
);

const RunSchema = new Schema<IRun>({
  status: { type: String, enum: ['active', 'completed'], default: 'active' },
  created_at: { type: Date, default: Date.now, index: { expires: '24h' } },
  stopped_at: { type: Date, default: null },
  raw_points: { type: Schema.Types.Mixed, default: [] },
  path: { type: [PathPointSub], default: [] },
});

export const Run = mongoose.model<IRun>('Run', RunSchema);
