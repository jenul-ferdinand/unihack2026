import type { CommsRequest, RunSummary, RunDetailResponse } from '@unihack/types';
import { RunRepository } from '../repositories/run.repository';
import { applyZuptCorrection } from '../processing/zupt';

const repo = new RunRepository();

let activeRunId: string | null = null;

export class RunService {
  async startRun(): Promise<string> {
    // Auto-close any active run
    if (activeRunId) {
      await this.stopRun();
    }
    await repo.closeOrphanedRuns();

    const run = await repo.create();
    activeRunId = run._id.toString();
    return activeRunId;
  }

  async addDataPoint(data: CommsRequest): Promise<boolean> {
    if (!activeRunId) return false;
    await repo.pushRawPoint(activeRunId, data);
    return true;
  }

  async stopRun(): Promise<boolean> {
    if (!activeRunId) return false;

    const run = await repo.findById(activeRunId);
    if (!run || run.status !== 'active') {
      activeRunId = null;
      return false;
    }

    const correctedPath = applyZuptCorrection(run.raw_points as CommsRequest[]);
    await repo.completeRun(activeRunId, correctedPath);

    activeRunId = null;
    return true;
  }

  async listRuns(): Promise<RunSummary[]> {
    const runs = await repo.findAll();
    return runs.map((r) => ({
      run_id: r._id.toString(),
      status: r.status,
      created_at: r.created_at.toISOString(),
      stopped_at: r.stopped_at ? r.stopped_at.toISOString() : null,
      point_count: r.path.length,
    }));
  }

  async getRunDetail(id: string): Promise<RunDetailResponse | null> {
    const run = await repo.findById(id);
    if (!run) return null;

    return {
      run_id: run._id.toString(),
      status: run.status,
      created_at: run.created_at.toISOString(),
      stopped_at: run.stopped_at ? run.stopped_at.toISOString() : null,
      path: run.path.map((p) => ({
        device_pos: { x: p.device_pos.x, y: p.device_pos.y, z: p.device_pos.z },
        peer_pos: { x: p.peer_pos.x, y: p.peer_pos.y, z: p.peer_pos.z },
        timestamp: p.timestamp,
      })),
    };
  }
}
