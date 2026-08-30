declare global {
	namespace App {}
}

// The generated WASM package is imported only by the analysis worker
// (src/lib/wasm/sectionWorker.ts). Its types come from the generated
// tracematch_wasm.d.ts, so the package must be built before typechecking.

export {};
