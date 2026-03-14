import { RunService } from '../run.service';
import { RunRepository } from '../../repositories/run.repository';
import { makePacket, makeRunPackets } from '../../__tests__/helpers';

function createMockRepo() {
  return {
    create: jest.fn(),
    findById: jest.fn(),
    findAll: jest.fn(),
    pushRawPoint: jest.fn(),
    completeRun: jest.fn(),
    closeOrphanedRuns: jest.fn(),
  } as unknown as jest.Mocked<RunRepository>;
}

function makeRunDoc(overrides: Record<string, any> = {}) {
  return {
    _id: { toString: () => 'run-123' },
    status: 'active' as const,
    created_at: new Date('2026-01-01T00:00:00Z'),
    stopped_at: null,
    raw_points: [],
    path: [],
    ...overrides,
  };
}

describe('RunService', () => {
  let service: RunService;
  let repo: jest.Mocked<RunRepository>;

  beforeEach(() => {
    repo = createMockRepo();
    service = new RunService(repo);
  });

  describe('startRun', () => {
    it('creates a new run and returns its id', async () => {
      repo.create.mockResolvedValue(makeRunDoc() as any);
      repo.closeOrphanedRuns.mockResolvedValue(undefined);

      const id = await service.startRun();

      expect(id).toBe('run-123');
      expect(repo.closeOrphanedRuns).toHaveBeenCalled();
      expect(repo.create).toHaveBeenCalled();
    });

    it('auto-closes an active run before starting a new one', async () => {
      // Start first run
      repo.create.mockResolvedValueOnce(makeRunDoc() as any);
      repo.closeOrphanedRuns.mockResolvedValue(undefined);
      await service.startRun();

      // Set up stop for the active run
      repo.findById.mockResolvedValueOnce(makeRunDoc({ raw_points: [] }) as any);
      repo.completeRun.mockResolvedValue(undefined);

      // Start second run
      repo.create.mockResolvedValueOnce(
        makeRunDoc({ _id: { toString: () => 'run-456' } }) as any,
      );

      const id = await service.startRun();

      expect(id).toBe('run-456');
      expect(repo.completeRun).toHaveBeenCalledWith('run-123', expect.any(Array));
    });
  });

  describe('addDataPoint', () => {
    it('returns false when no active run', async () => {
      const result = await service.addDataPoint(makePacket({
        devicePos: { x: 0, y: 0, z: 0 },
        deviceVel: { x: 0, y: 0, z: 0 },
        peerPos: { x: 1, y: 0, z: 0 },
        peerSpeed: 0,
      }));

      expect(result).toBe(false);
      expect(repo.pushRawPoint).not.toHaveBeenCalled();
    });

    it('pushes data when a run is active', async () => {
      repo.create.mockResolvedValue(makeRunDoc() as any);
      repo.closeOrphanedRuns.mockResolvedValue(undefined);
      repo.pushRawPoint.mockResolvedValue(undefined);
      await service.startRun();

      const packet = makePacket({
        devicePos: { x: 1, y: 2, z: 3 },
        deviceVel: { x: 0.5, y: 0, z: 0 },
        peerPos: { x: 2, y: 2, z: 3 },
        peerSpeed: 0.5,
      });

      const result = await service.addDataPoint(packet);

      expect(result).toBe(true);
      expect(repo.pushRawPoint).toHaveBeenCalledWith('run-123', packet);
    });
  });

  describe('stopRun', () => {
    it('returns false when no active run', async () => {
      expect(await service.stopRun()).toBe(false);
    });

    it('fetches raw points, runs ZUPT, and saves corrected path', async () => {
      const packets = makeRunPackets({ stationaryCount: 10, movingCount: 10 });

      repo.create.mockResolvedValue(makeRunDoc() as any);
      repo.closeOrphanedRuns.mockResolvedValue(undefined);
      repo.pushRawPoint.mockResolvedValue(undefined);
      await service.startRun();

      repo.findById.mockResolvedValue(
        makeRunDoc({ raw_points: packets }) as any,
      );
      repo.completeRun.mockResolvedValue(undefined);

      const result = await service.stopRun();

      expect(result).toBe(true);
      expect(repo.completeRun).toHaveBeenCalledWith(
        'run-123',
        expect.arrayContaining([
          expect.objectContaining({
            device_pos: expect.objectContaining({ x: expect.any(Number) }),
            peer_pos: expect.objectContaining({ x: expect.any(Number) }),
            timestamp: expect.any(String),
          }),
        ]),
      );
    });
  });

  describe('listRuns', () => {
    it('maps documents to RunSummary shape', async () => {
      repo.findAll.mockResolvedValue([
        makeRunDoc({
          status: 'completed',
          stopped_at: new Date('2026-01-01T01:00:00Z'),
          path: [{ device_pos: { x: 0, y: 0, z: 0 }, peer_pos: { x: 1, y: 0, z: 0 }, timestamp: '' }],
        }),
      ] as any);

      const runs = await service.listRuns();

      expect(runs).toHaveLength(1);
      expect(runs[0]).toEqual({
        run_id: 'run-123',
        status: 'completed',
        created_at: '2026-01-01T00:00:00.000Z',
        stopped_at: '2026-01-01T01:00:00.000Z',
        point_count: 1,
      });
    });
  });

  describe('getRunDetail', () => {
    it('returns null for unknown id', async () => {
      repo.findById.mockResolvedValue(null);
      expect(await service.getRunDetail('nonexistent')).toBeNull();
    });

    it('returns detail with path data', async () => {
      repo.findById.mockResolvedValue(
        makeRunDoc({
          status: 'completed',
          stopped_at: new Date('2026-01-01T01:00:00Z'),
          path: [
            { device_pos: { x: 1, y: 2, z: 3 }, peer_pos: { x: 4, y: 5, z: 6 }, timestamp: '2026-01-01T00:00:00.000Z' },
          ],
        }) as any,
      );

      const detail = await service.getRunDetail('run-123');

      expect(detail).not.toBeNull();
      expect(detail!.run_id).toBe('run-123');
      expect(detail!.path).toHaveLength(1);
      expect(detail!.path[0].device_pos).toEqual({ x: 1, y: 2, z: 3 });
    });
  });
});
