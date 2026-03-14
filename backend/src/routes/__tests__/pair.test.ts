import { OpenAPIHono } from '@hono/zod-openapi';
import pair, { resetPairState } from '../pair';

const app = new OpenAPIHono();
app.route('/api/pair', pair);

function json(body: object) {
  return {
    method: 'POST' as const,
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  };
}

beforeEach(() => {
  resetPairState();
});

describe('Pair API', () => {
  describe('waiting state', () => {
    it('returns waiting when first device posts', async () => {
      const res = await app.request('/api/pair', json({ ip: '192.168.1.10' }));
      const body = (await res.json()) as any;

      expect(res.status).toBe(200);
      expect(body).toMatchObject({ status: 'waiting', ip: '192.168.1.10' });
      expect(body.role).toBeUndefined();
      expect(body.partner_ip).toBeUndefined();
    });

    it('returns waiting when same device polls again', async () => {
      await app.request('/api/pair', json({ ip: '192.168.1.10' }));
      const res = await app.request('/api/pair', json({ ip: '192.168.1.10' }));
      const body = (await res.json()) as any;

      expect(body.status).toBe('waiting');
    });
  });

  describe('pairing', () => {
    it('pairs two devices on the same subnet', async () => {
      await app.request('/api/pair', json({ ip: '192.168.1.10' }));

      const res = await app.request('/api/pair', json({ ip: '192.168.1.20' }));
      const body = (await res.json()) as any;

      expect(body.status).toBe('paired');
      expect(body.ip).toBe('192.168.1.20');
      expect(['device', 'peer']).toContain(body.role);
      expect(body.partner_ip).toBe('192.168.1.10');
    });

    it('returns paired result when first device polls after pairing', async () => {
      await app.request('/api/pair', json({ ip: '192.168.1.10' }));
      await app.request('/api/pair', json({ ip: '192.168.1.20' }));

      const res = await app.request('/api/pair', json({ ip: '192.168.1.10' }));
      const body = (await res.json()) as any;

      expect(body.status).toBe('paired');
      expect(body.ip).toBe('192.168.1.10');
      expect(body.partner_ip).toBe('192.168.1.20');
    });

    it('assigns complementary roles to both devices', async () => {
      await app.request('/api/pair', json({ ip: '192.168.1.10' }));
      await app.request('/api/pair', json({ ip: '192.168.1.20' }));

      const resA = await app.request('/api/pair', json({ ip: '192.168.1.10' }));
      const resB = await app.request('/api/pair', json({ ip: '192.168.1.20' }));
      const bodyA = (await resA.json()) as any;
      const bodyB = (await resB.json()) as any;

      const roles = [bodyA.role, bodyB.role].sort();
      expect(roles).toEqual(['device', 'peer']);
    });
  });

  describe('different subnets', () => {
    it('does not pair devices on different subnets', async () => {
      await app.request('/api/pair', json({ ip: '192.168.1.10' }));

      const res = await app.request('/api/pair', json({ ip: '10.0.0.5' }));
      const body = (await res.json()) as any;

      expect(body.status).toBe('waiting');
    });
  });

  describe('GET /status', () => {
    it('returns empty state when no devices have posted', async () => {
      const res = await app.request('/api/pair/status');
      const body = (await res.json()) as any;

      expect(res.status).toBe(200);
      expect(body).toEqual({ pending_count: 0, paired: null });
    });

    it('returns pending_count when one device is waiting', async () => {
      await app.request('/api/pair', json({ ip: '192.168.1.10' }));

      const res = await app.request('/api/pair/status');
      const body = (await res.json()) as any;

      expect(body.pending_count).toBe(1);
      expect(body.paired).toBeNull();
    });

    it('returns paired IPs after two devices pair', async () => {
      await app.request('/api/pair', json({ ip: '192.168.1.10' }));
      await app.request('/api/pair', json({ ip: '192.168.1.20' }));

      const res = await app.request('/api/pair/status');
      const body = (await res.json()) as any;

      expect(body.pending_count).toBe(0);
      expect(body.paired).not.toBeNull();
      expect([body.paired.device_ip, body.paired.peer_ip].sort()).toEqual(
        ['192.168.1.10', '192.168.1.20'],
      );
    });

    it('clears pending after pairing completes', async () => {
      await app.request('/api/pair', json({ ip: '192.168.1.10' }));
      await app.request('/api/pair', json({ ip: '192.168.1.20' }));

      const res = await app.request('/api/pair/status');
      const body = (await res.json()) as any;

      expect(body.pending_count).toBe(0);
    });
  });
});
