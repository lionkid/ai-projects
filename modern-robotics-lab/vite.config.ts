/// <reference types="vitest/config" />
import { defineConfig } from 'vite';

// Relative base so the built site works from any sub-path (GitHub Pages, local file server, ...).
export default defineConfig({
  base: './',
  resolve: {
    alias: { '@': new URL('./src', import.meta.url).pathname },
  },
  build: {
    target: 'es2022',
    chunkSizeWarningLimit: 1500,
  },
  test: {
    environment: 'node',
    include: ['src/**/*.test.ts'],
  },
});
