import { serve } from "@hono/node-server";
import { swaggerUI } from "@hono/swagger-ui";
import { OpenAPIHono, createRoute, z } from "@hono/zod-openapi";
import comms from "./routes/comms";

const app = new OpenAPIHono();

app.route("/api/comms", comms);

const healthRoute = createRoute({
  method: "get",
  path: "/",
  responses: {
    200: {
      content: {
        "application/json": {
          schema: z.object({ message: z.string() }),
        },
      },
      description: "Health check",
    },
  },
});

app.openapi(healthRoute, (c) => {
  return c.json({ message: "API is running" }, 200);
});

app.doc("/doc", {
  openapi: "3.1.0",
  info: { title: "API", version: "1.0.0" },
});

app.get("/ui", swaggerUI({ url: "/doc" }));

const PORT = Number(process.env.PORT) || 3000;

serve({ fetch: app.fetch, port: PORT }, () => {
  console.log(`Server running on port ${PORT}`);
});
