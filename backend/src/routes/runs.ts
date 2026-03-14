import { OpenAPIHono, createRoute } from '@hono/zod-openapi';
import {
  CommsStartResponseSchema,
  RunsListResponseSchema,
  RunDetailResponseSchema,
} from '@unihack/types';
import { z } from 'zod';
import { runService } from '../services/run.service';

const runs = new OpenAPIHono();

runs.openapi(
  createRoute({
    method: 'post',
    path: '/demo',
    tags: ['Runs'],
    responses: {
      200: {
        content: {
          'application/json': {
            schema: CommsStartResponseSchema,
          },
        },
        description: 'Demo run created',
      },
    },
  }),
  async (c) => {
    const runId = await runService.runDemo();
    console.log('Demo run created, run_id:', runId);
    return c.json({ run_id: runId }, 200);
  },
);

runs.openapi(
  createRoute({
    method: 'get',
    path: '/',
    tags: ['Runs'],
    responses: {
      200: {
        content: {
          'application/json': {
            schema: RunsListResponseSchema,
          },
        },
        description: 'List of all runs',
      },
    },
  }),
  async (c) => {
    const list = await runService.listRuns();
    return c.json({ runs: list }, 200);
  },
);

runs.openapi(
  createRoute({
    method: 'get',
    path: '/{id}',
    tags: ['Runs'],
    request: {
      params: z.object({
        id: z.string(),
      }),
    },
    responses: {
      200: {
        content: {
          'application/json': {
            schema: RunDetailResponseSchema,
          },
        },
        description: 'Run detail with corrected path',
      },
      404: {
        content: {
          'application/json': {
            schema: z.object({ error: z.string() }),
          },
        },
        description: 'Run not found',
      },
    },
  }),
  async (c) => {
    const { id } = c.req.valid('param');
    const detail = await runService.getRunDetail(id);
    if (!detail) {
      return c.json({ error: 'Run not found' }, 404);
    }
    return c.json(detail, 200);
  },
);

export default runs;
