import 'dotenv/config';
import { serve } from '@hono/node-server';
import { connectDB } from './db';
import app from './app';

const PORT = Number(process.env.PORT) || 3000;

connectDB().then(() => {
  serve({ fetch: app.fetch, port: PORT }, () => {
    console.log(`Server running on port ${PORT}`);
  });
});
