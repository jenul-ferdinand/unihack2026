import { OpenAPIHono, createRoute } from "@hono/zod-openapi";
import { CommsResponseSchema } from "@unihack/types";

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
            schema: CommsResponseSchema,
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
