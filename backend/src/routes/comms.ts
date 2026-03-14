import { OpenAPIHono, createRoute } from '@hono/zod-openapi';
import {
  CommsRequestSchema,
  CommsResponseSchema,
  CommsStartResponseSchema,
  CommsStartSchema,
  CommsStopSchema,
  RunsListResponseSchema,
  RunDetailResponseSchema,
} from '@unihack/types';
import { z } from 'zod';
import { RunService } from '../services/run.service';

const comms = new OpenAPIHono();
const runService = new RunService();

comms.openapi(
  createRoute({
    method: 'post',
    path: '/',
    tags: ['Comms'],
    request: {
      body: {
        content: {
          'application/json': {
            schema: CommsRequestSchema,
          },
        },
        description: 'Hardware telemetry data',
      },
    },
    responses: {
      200: {
        content: {
          'application/json': {
            schema: CommsResponseSchema,
          },
        },
        description: 'Data received successfully',
      },
    },
  }),
  async (c) => {
    const data = c.req.valid('json');
    const ok = await runService.addDataPoint(data);
    if (!ok) {
      return c.json({ success: false }, 200);
    }
    return c.json({ success: true }, 200);
  },
);

comms.openapi(
  createRoute({
    method: 'post',
    path: '/start',
    tags: ['Comms'],
    request: {
      body: {
        content: {
          'application/json': {
            schema: CommsStartSchema,
          },
        },
      },
    },
    responses: {
      200: {
        content: {
          'application/json': {
            schema: CommsStartResponseSchema,
          },
        },
        description: 'Session started',
      },
    },
  }),
  async (c) => {
    const runId = await runService.startRun();
    console.log('Session started, run_id:', runId);
    return c.json({ run_id: runId }, 200);
  },
);

comms.openapi(
  createRoute({
    method: 'post',
    path: '/stop',
    tags: ['Comms'],
    request: {
      body: {
        content: {
          'application/json': {
            schema: CommsStopSchema,
          },
        },
      },
    },
    responses: {
      200: {
        content: {
          'application/json': {
            schema: CommsResponseSchema,
          },
        },
        description: 'Session stopped',
      },
    },
  }),
  async (c) => {
    const ok = await runService.stopRun();
    console.log('Session stopped, success:', ok);
    return c.json({ success: ok }, 200);
  },
);

comms.openapi(
  createRoute({
    method: 'get',
    path: '/runs',
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
    const runs = await runService.listRuns();
    return c.json({ runs }, 200);
  },
);

comms.openapi(
  createRoute({
    method: 'get',
    path: '/runs/{id}',
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

export default comms;
