import { applyZuptCorrection } from '../zupt';
import { makePacket, makeRunPackets } from '../../__tests__/helpers';

describe('applyZuptCorrection', () => {
  it('returns empty array for empty input', () => {
    expect(applyZuptCorrection([])).toEqual([]);
  });

  it('returns unchanged positions when there are fewer than 2 anchors', () => {
    // All moving — no stationary segments
    const packets = Array.from({ length: 20 }, (_, i) =>
      makePacket({
        devicePos: { x: i * 0.5, y: 0, z: 0 },
        deviceVel: { x: 1.0, y: 0, z: 0 },
        peerPos: { x: i * 0.5 + 1, y: 0, z: 0 },
        peerSpeed: 1.0,
        timestampUs: i * 10000,
      }),
    );

    const result = applyZuptCorrection(packets);

    expect(result).toHaveLength(20);
    // Positions should match input since no correction applied
    result.forEach((point, i) => {
      expect(point.device_pos.x).toBeCloseTo(i * 0.5, 5);
      expect(point.device_pos.y).toBeCloseTo(0, 5);
    });
  });

  it('returns PathPoint[] with correct shape and length', () => {
    const packets = makeRunPackets();
    const result = applyZuptCorrection(packets);

    expect(result).toHaveLength(packets.length);

    for (const point of result) {
      expect(point).toHaveProperty('device_pos');
      expect(point).toHaveProperty('peer_pos');
      expect(point).toHaveProperty('timestamp');
      expect(point.device_pos).toHaveProperty('x');
      expect(point.device_pos).toHaveProperty('y');
      expect(point.device_pos).toHaveProperty('z');
      expect(point.peer_pos).toHaveProperty('x');
      expect(point.peer_pos).toHaveProperty('y');
      expect(point.peer_pos).toHaveProperty('z');
      expect(typeof point.timestamp).toBe('string');
    }
  });

  it('preserves stationary anchor positions', () => {
    const packets = makeRunPackets({
      stationaryCount: 10,
      movingCount: 20,
      startPos: { x: 0, y: 0, z: 0 },
      endPos: { x: 10, y: 0, z: 0 },
    });

    const result = applyZuptCorrection(packets);

    // First few points should be near origin
    expect(result[0].device_pos.x).toBeCloseTo(0, 1);
    expect(result[0].device_pos.y).toBeCloseTo(0, 1);

    // Last few points should be near (10, 0, 0)
    const last = result[result.length - 1];
    expect(last.device_pos.x).toBeCloseTo(10, 1);
    expect(last.device_pos.y).toBeCloseTo(0, 1);
  });

  it('generates valid ISO timestamp strings', () => {
    const packets = makeRunPackets({ stationaryCount: 10, movingCount: 5 });
    const result = applyZuptCorrection(packets);

    for (const point of result) {
      const date = new Date(point.timestamp);
      expect(date.getTime()).not.toBeNaN();
    }
  });
});
