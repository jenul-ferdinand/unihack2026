import { swaggerUI } from '@hono/swagger-ui';
import { OpenAPIHono, createRoute, z } from '@hono/zod-openapi';
import { cors } from 'hono/cors';
import comms from './routes/comms';
import runs from './routes/runs';

const app = new OpenAPIHono();

if (process.env.NODE_ENV !== 'production') {
  app.use('*', cors());
}

app.route('/api/comms', comms);
app.route('/api/runs', runs);

app.openapi(
  createRoute({
    method: 'get',
    path: '/',
    responses: {
      200: {
        content: {
          'application/json': {
            schema: z.object({ message: z.string() }),
          },
        },
        description: 'Health check',
      },
    },
  }),
  (c) => {
    return c.json({ message: 'API is running' }, 200);
  },
);

app.doc('/doc', {
  openapi: '3.1.0',
  info: { title: 'API', version: '1.0.0' },
});

app.get('/ui', swaggerUI({ url: '/doc' }));

export default app;
