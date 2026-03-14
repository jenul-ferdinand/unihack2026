import mongoose from 'mongoose';
import { MongoMemoryServer } from 'mongodb-memory-server';
import { OpenAPIHono } from '@hono/zod-openapi';
import { makeRunPackets } from '../../__tests__/helpers';
import { Run } from '../../models/run.model';

let mongoServer: MongoMemoryServer;
let app: OpenAPIHono;

beforeAll(async () => {
  mongoServer = await MongoMemoryServer.create();
  await mongoose.connect(mongoServer.getUri());

  // Import comms router after mongoose is connected so the RunService
  // and repository use the in-memory DB.
  const { default: comms } = await import('../../routes/comms');
  app = new OpenAPIHono();
  app.route('/api/comms', comms);
});

afterAll(async () => {
  await Run.deleteMany({});
  await mongoose.disconnect();
  await mongoServer.stop();
});

function json(body: object) {
  return {
    method: 'POST' as const,
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  };
}

describe('Comms E2E', () => {
  describe('full run lifecycle', () => {
    const packets = makeRunPackets({ stationaryCount: 10, movingCount: 20 });
    let runId: string;

    it('POST /api/comms/start — creates a new run', async () => {
      const res = await app.request('/api/comms/start', json({ start: 1 }));

      expect(res.status).toBe(200);
      const body = await res.json() as any;
      expect(body).toHaveProperty('run_id');
      expect(typeof body.run_id).toBe('string');
      runId = body.run_id;
    });

    it('POST /api/comms — accepts data points', async () => {
      for (const pkt of packets) {
        const res = await app.request('/api/comms', json(pkt));
        expect(res.status).toBe(200);
        const body = await res.json() as any;
        expect(body.success).toBe(true);
      }
    });

    it('POST /api/comms/stop — stops the run and applies ZUPT', async () => {
      const res = await app.request('/api/comms/stop', json({ stop: 1 }));

      expect(res.status).toBe(200);
      const body = await res.json() as any;
      expect(body.success).toBe(true);
    });

    it('GET /api/comms/runs — lists the completed run', async () => {
      const res = await app.request('/api/comms/runs');

      expect(res.status).toBe(200);
      const body = await res.json() as any;
      expect(body.runs.length).toBeGreaterThanOrEqual(1);

      const run = body.runs.find((r: any) => r.run_id === runId);
      expect(run).toMatchObject({
        run_id: runId,
        status: 'completed',
        point_count: packets.length,
      });
      expect(run).toHaveProperty('created_at');
      expect(run).toHaveProperty('stopped_at');
    });

    it('GET /api/comms/runs/:id — returns corrected path', async () => {
      const res = await app.request(`/api/comms/runs/${runId}`);

      expect(res.status).toBe(200);
      const body = await res.json() as any;

      expect(body.run_id).toBe(runId);
      expect(body.status).toBe('completed');
      expect(body.path).toHaveLength(packets.length);

      for (const point of body.path) {
        expect(point.device_pos).toEqual(
          expect.objectContaining({ x: expect.any(Number), y: expect.any(Number), z: expect.any(Number) }),
        );
        expect(point.peer_pos).toEqual(
          expect.objectContaining({ x: expect.any(Number), y: expect.any(Number), z: expect.any(Number) }),
        );
        expect(typeof point.timestamp).toBe('string');
      }
    });

    it('GET /api/comms/runs/:id — returns 404 for unknown id', async () => {
      const fakeId = new mongoose.Types.ObjectId().toString();
      const res = await app.request(`/api/comms/runs/${fakeId}`);

      expect(res.status).toBe(404);
      const body = await res.json() as any;
      expect(body).toHaveProperty('error');
    });

    it('POST /api/comms — returns success:false when no active run', async () => {
      const pkt = packets[0];
      const res = await app.request('/api/comms', json(pkt));

      expect(res.status).toBe(200);
      const body = await res.json() as any;
      expect(body.success).toBe(false);
    });
  });

  describe('demo run', () => {
    it('POST /api/comms/demo — creates a demo run with corrected path', async () => {
      const res = await app.request('/api/comms/demo', json({}));

      expect(res.status).toBe(200);
      const body = await res.json() as any;
      expect(typeof body.run_id).toBe('string');

      // Verify the run exists and has path data
      const detailRes = await app.request(`/api/comms/runs/${body.run_id}`);
      expect(detailRes.status).toBe(200);
      const detail = await detailRes.json() as any;

      expect(detail.status).toBe('completed');
      expect(detail.path.length).toBeGreaterThan(0);
      expect(detail.path[0]).toHaveProperty('device_pos');
      expect(detail.path[0]).toHaveProperty('peer_pos');
      expect(detail.path[0]).toHaveProperty('timestamp');
    }, 30000);
  });
});
