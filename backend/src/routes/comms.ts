import { OpenAPIHono, createRoute } from '@hono/zod-openapi';
import {
  CommsRequestSchema,
  CommsResponseSchema,
  CommsStartSchema,
  CommsStopSchema,
} from '@unihack/types';

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
    // TODO: Store data
    console.log('Received hardware data:', data);
    return c.json({ success: true }, 200);
  },
)

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
            schema: CommsResponseSchema,
          },
        },
        description: 'Session started',
      },
    },
  }),
  async (c) => {
    const data = c.req.valid('json');
    // TODO: Start session
    console.log('Session started:', data);
    return c.json({ comms: [] }, 200);
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
    const data = c.req.valid('json');
    // TODO: Stop session
    console.log('Session stopped:', data);
    return c.json({ comms: [] }, 200);
  },
);

export default comms;
