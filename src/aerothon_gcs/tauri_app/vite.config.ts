import { defineConfig, type Plugin } from "vite";
import react from "@vitejs/plugin-react";
import { gzipSync } from 'node:zlib';
import { readFile, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';

const compressedAssets: Plugin = {
  name: 'pi-compressed-assets',
  async writeBundle(options, bundle) {
    // Vite replaces dynamic-import preload markers after generateBundle.
    // Compress the actual written files so both HTTP encodings are identical.
    await Promise.all(Object.keys(bundle).filter(name=>/\.(js|css)$/.test(name)).map(async name=>{
      const path=resolve(options.dir!,name);
      await writeFile(path+'.gz',gzipSync(await readFile(path),{level:9}));
    }));
  },
};

// Tauri expects a fixed port and no clearing of the screen.
export default defineConfig({
  plugins: [react(), compressedAssets],
  clearScreen: false,
  server: { port: 5173, strictPort: true },
});
