import { OpenAPIHono, createRoute } from '@hono/zod-openapi';
import { PairRequestSchema, PairResponseSchema, PairStatusResponseSchema } from '@unihack/types';

/** Extract /24 subnet from an IP address (e.g. "192.168.1.10" → "192.168.1") */
function getSubnet(ip: string): string {
  return ip.split('.').slice(0, 3).join('.');
}

// Pending devices waiting for a partner, keyed by subnet
const pendingBySubnet = new Map<string, { ip: string; timestamp: number }>();

// Resolved pairings, keyed by IP
const pairedResults = new Map<
  string,
  { role: 'device' | 'peer'; partner_ip: string; timestamp: number }
>();

// Clean up stale entries
const PENDING_TTL_MS = 30_000;
const PAIRED_TTL_MS = 10 * 60_000;

function cleanStale() {
  const now = Date.now();
  for (const [subnet, entry] of pendingBySubnet) {
    if (now - entry.timestamp > PENDING_TTL_MS) {
      pendingBySubnet.delete(subnet);
    }
  }
  for (const [ip, entry] of pairedResults) {
    if (now - entry.timestamp > PAIRED_TTL_MS) {
      pairedResults.delete(ip);
    }
  }
}

/** Clear all paired results — called when a run is stopped */
export function clearPairing() {
  pairedResults.clear();
}

const pair = new OpenAPIHono();

pair.openapi(
  createRoute({
    method: 'post',
    path: '/',
    tags: ['Pair'],
    request: {
      body: {
        content: {
          'application/json': {
            schema: PairRequestSchema,
          },
        },
        description: 'Device IP address for pairing',
      },
    },
    responses: {
      200: {
        content: {
          'application/json': {
            schema: PairResponseSchema,
          },
        },
        description: 'Pairing request received',
      },
    },
  }),
  async (c) => {
    const { ip } = c.req.valid('json');
    const subnet = getSubnet(ip);

    cleanStale();

    // If this IP already has a pairing result, return it
    const existing = pairedResults.get(ip);
    if (existing) {
      return c.json(
        {
          status: 'paired' as const,
          ip,
          role: existing.role,
          partner_ip: existing.partner_ip,
        },
        200,
      );
    }

    const pending = pendingBySubnet.get(subnet);

    if (pending && pending.ip !== ip) {
      // Match found: randomly assign roles
      const deviceFirst = Math.random() < 0.5;
      const [deviceIp, peerIp] = deviceFirst
        ? [pending.ip, ip]
        : [ip, pending.ip];

      // Store results for both devices
      const now = Date.now();
      pairedResults.set(deviceIp, { role: 'device', partner_ip: peerIp, timestamp: now });
      pairedResults.set(peerIp, { role: 'peer', partner_ip: deviceIp, timestamp: now });

      // Clear the pending entry
      pendingBySubnet.delete(subnet);

      const result = pairedResults.get(ip)!;
      return c.json(
        {
          status: 'paired' as const,
          ip,
          role: result.role,
          partner_ip: result.partner_ip,
        },
        200,
      );
    }

    // No match: store as pending
    pendingBySubnet.set(subnet, { ip, timestamp: Date.now() });

    return c.json({ status: 'waiting' as const, ip }, 200);
  },
);

pair.openapi(
  createRoute({
    method: 'get',
    path: '/status',
    tags: ['Pair'],
    responses: {
      200: {
        content: {
          'application/json': {
            schema: PairStatusResponseSchema,
          },
        },
        description: 'Current pairing status',
      },
    },
  }),
  async (c) => {
    cleanStale();

    // Find first paired pair (device + peer)
    let paired: { device_ip: string; peer_ip: string } | null = null;
    for (const [ip, entry] of pairedResults) {
      if (entry.role === 'device') {
        paired = { device_ip: ip, peer_ip: entry.partner_ip };
        break;
      }
    }

    return c.json(
      {
        pending_count: pendingBySubnet.size,
        paired,
      },
      200,
    );
  },
);

// Helper for tests to reset pairing states
export function resetPairState() {
  pendingBySubnet.clear();
  pairedResults.clear();
}

export default pair;
