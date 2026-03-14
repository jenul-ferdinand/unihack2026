import type { CommsRequest, Packet, RunSummary, RunDetailResponse } from '@unihack/types';
import { RunRepository } from '../repositories/run.repository';
import { applyZuptCorrection } from '../processing/zupt';
import { generateDemoPackets } from '../processing/demo';

export class RunService {
  private activeRunId: string | null = null;
  private repo: RunRepository;

  constructor(repo?: RunRepository) {
    this.repo = repo ?? new RunRepository();
  }

  async startRun(): Promise<string> {
    if (this.activeRunId) {
      await this.stopRun();
    }
    await this.repo.closeOrphanedRuns();

    const run = await this.repo.create();
    this.activeRunId = run._id.toString();
    return this.activeRunId;
  }

  async addDataPoint(data: CommsRequest): Promise<boolean> {
    if (!this.activeRunId) return false;
    await this.repo.pushRawPoints(this.activeRunId, data.samples);
    return true;
  }

  async stopRun(): Promise<boolean> {
    if (!this.activeRunId) return false;

    const run = await this.repo.findById(this.activeRunId);
    if (!run || run.status !== 'active') {
      this.activeRunId = null;
      return false;
    }

    const correctedPath = applyZuptCorrection(run.raw_points as Packet[]);
    await this.repo.completeRun(this.activeRunId, correctedPath);

    this.activeRunId = null;
    return true;
  }

  async listRuns(): Promise<RunSummary[]> {
    const runs = await this.repo.findAll();
    return runs.map((r) => ({
      run_id: r._id.toString(),
      status: r.status,
      created_at: r.created_at.toISOString(),
      stopped_at: r.stopped_at ? r.stopped_at.toISOString() : null,
      point_count: r.path.length,
    }));
  }

  async runDemo(): Promise<string> {
    const runId = await this.startRun();
    const packets = generateDemoPackets();
    await this.repo.pushRawPoints(runId, packets);
    await this.stopRun();
    return runId;
  }

  async getRunDetail(id: string): Promise<RunDetailResponse | null> {
    const run = await this.repo.findById(id);
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

export const runService = new RunService();
