import { OpenAPIHono, createRoute } from '@hono/zod-openapi';
import {
  CommsRequestSchema,
  CommsResponseSchema,
  CommsStartResponseSchema,
  CommsStartSchema,
  CommsStopSchema,
} from '@unihack/types';
import { runService } from '../services/run.service';

const comms = new OpenAPIHono();

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

export default comms;
