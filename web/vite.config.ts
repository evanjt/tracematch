import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vite';

// Cross-Origin Isolation headers required for SharedArrayBuffer, which
// wasm-bindgen-rayon needs to spawn threads. Applied in dev (Vite),
// preview, and to the worker context. Production deploys must replicate
// these (nginx config, or the `coi-serviceworker` shim for GitHub Pages).
const crossOriginIsolation = {
	name: 'cross-origin-isolation',
	configureServer(server: import('vite').ViteDevServer) {
		server.middlewares.use((_req, res, next) => {
			res.setHeader('Cross-Origin-Opener-Policy', 'same-origin');
			res.setHeader('Cross-Origin-Embedder-Policy', 'require-corp');
			next();
		});
	},
	configurePreviewServer(server: import('vite').PreviewServer) {
		server.middlewares.use((_req, res, next) => {
			res.setHeader('Cross-Origin-Opener-Policy', 'same-origin');
			res.setHeader('Cross-Origin-Embedder-Policy', 'require-corp');
			next();
		});
	}
};

export default defineConfig({
	plugins: [crossOriginIsolation, sveltekit()],
	worker: {
		format: 'es'
	}
});
