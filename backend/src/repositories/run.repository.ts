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

  async pushRawPoints(runId: string, points: any[]): Promise<void> {
    await Run.updateOne({ _id: runId }, { $push: { raw_points: { $each: points } } });
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

  async deleteById(id: string): Promise<boolean> {
    const result = await Run.deleteOne({ _id: id });
    return result.deletedCount > 0;
  }

  async closeOrphanedRuns(): Promise<void> {
    await Run.updateMany(
      { status: 'active' },
      { $set: { status: 'completed', stopped_at: new Date(), raw_points: [] } },
    );
  }
}
