import { Run, IRun, IPathPoint } from '../models/run.model';

export class RunRepository {
  async create(): Promise<IRun> {
    return Run.create({ status: 'active' });
  }

  async findById(id: string): Promise<IRun | null> {
    return Run.findById(id);
  }

  async findAll(): Promise<IRun[]> {
    return Run.find().sort({ created_at: -1 });
  }

  async pushRawPoint(runId: string, point: any): Promise<void> {
    await Run.updateOne({ _id: runId }, { $push: { raw_points: point } });
  }

  async completeRun(runId: string, path: IPathPoint[]): Promise<void> {
    await Run.updateOne(
      { _id: runId },
      {
        $set: {
          status: 'completed',
          stopped_at: new Date(),
          path,
          raw_points: [],
        },
      },
    );
  }

  async closeOrphanedRuns(): Promise<void> {
    await Run.updateMany(
      { status: 'active' },
      { $set: { status: 'completed', stopped_at: new Date(), raw_points: [] } },
    );
  }
}
