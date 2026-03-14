import type { CommsRequest, PathPoint } from '@unihack/types';

const STATIONARY_VEL_THRESH = 0.05; // m/s
const MIN_STILL_SAMPLES = 8;

interface Vec3 {
  x: number;
  y: number;
  z: number;
}

interface Anchor {
  index: number;
  pos: Vec3;
}

function velMagnitude(v: Vec3): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

function findAnchors(positions: Vec3[], velocities: Vec3[]): Anchor[] {
  const anchors: Anchor[] = [];
  let stillCount = 0;
  let sumX = 0,
    sumY = 0,
    sumZ = 0;
  let stillStart = 0;

  for (let i = 0; i < positions.length; i++) {
    const speed = velMagnitude(velocities[i]);

    if (speed < STATIONARY_VEL_THRESH) {
      if (stillCount === 0) stillStart = i;
      stillCount++;
      sumX += positions[i].x;
      sumY += positions[i].y;
      sumZ += positions[i].z;

      if (stillCount === MIN_STILL_SAMPLES) {
        const midIdx = Math.floor(stillStart + stillCount / 2);
        anchors.push({
          index: midIdx,
          pos: {
            x: sumX / stillCount,
            y: sumY / stillCount,
            z: sumZ / stillCount,
          },
        });
      }
    } else {
      stillCount = 0;
      sumX = sumY = sumZ = 0;
    }
  }

  return anchors;
}

function correctPath(positions: Vec3[], velocities: Vec3[]): Vec3[] {
  const anchors = findAnchors(positions, velocities);

  if (anchors.length < 2) {
    return positions;
  }

  const corrected = positions.map((p) => ({ ...p }));

  for (let a = 0; a < anchors.length - 1; a++) {
    const start = anchors[a];
    const end = anchors[a + 1];
    const segLen = end.index - start.index;
    if (segLen <= 0) continue;

    // Drift between where the path ended up vs where the anchor says it should be
    const driftX = corrected[end.index].x - end.pos.x;
    const driftY = corrected[end.index].y - end.pos.y;
    const driftZ = corrected[end.index].z - end.pos.z;

    for (let i = start.index; i <= end.index; i++) {
      const t = (i - start.index) / segLen;
      corrected[i].x -= driftX * t;
      corrected[i].y -= driftY * t;
      corrected[i].z -= driftZ * t;
    }
  }

  return corrected;
}

export function applyZuptCorrection(rawPoints: CommsRequest[]): PathPoint[] {
  if (rawPoints.length === 0) return [];

  // Extract device positions and velocities
  const devicePositions = rawPoints.map((p) => p.position_velocity.clamped_pos);
  const deviceVelocities = rawPoints.map((p) => p.position_velocity.clamped_vel);
  const correctedDevice = correctPath(devicePositions, deviceVelocities);

  // Extract peer positions — use peer velocity approximation from speed
  const peerPositions = rawPoints.map((p) => p.peer_state.peer_pos);
  const peerVelocities = rawPoints.map((p) => ({
    x: p.peer_state.peer_speed_mps,
    y: 0,
    z: 0,
  }));
  const correctedPeer = correctPath(peerPositions, peerVelocities);

  // Build PathPoint array
  return rawPoints.map((raw, i) => ({
    device_pos: correctedDevice[i],
    peer_pos: correctedPeer[i],
    timestamp: new Date(raw.peer_state.peer_t_us / 1000).toISOString(),
  }));
}
