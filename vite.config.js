import { defineConfig } from "vite";

const proxy = { "/api/panthera": "http://127.0.0.1:5174" };

export default defineConfig({
  build: { target: "esnext" },
  optimizeDeps: { exclude: ["@mujoco/mujoco"] },
  server: { proxy },
  preview: { proxy },
});
