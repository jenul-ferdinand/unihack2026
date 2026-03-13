import { OpenAPIHono, createRoute, z } from "@hono/zod-openapi";

const comms = new OpenAPIHono();

comms.openapi(
  createRoute({
    method: "get",
    path: "/",
    tags: ["Comms"],
    responses: {
      200: {
        content: {
          "application/json": {
            schema: z.object({
              comms: z.array(z.string()),
            }),
          },
        },
        description: "Upload data from hardware",
      },
    },
  }),
  (c) => {
    // TODO: Return nothing for now
    return c.json({ comms: [] }, 200);
  },
);

export default comms;
