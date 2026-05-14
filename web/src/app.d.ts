declare global {
	namespace App {}
}

// The WASM module is only imported from the analysis worker
// (src/lib/wasm/sectionWorker.ts), which types it inline via `WasmModule`.

export {};
