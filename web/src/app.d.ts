declare global {
	namespace App {}
}

// The generated WASM package is imported from two places: the analysis
// worker (src/lib/wasm/sectionWorker.ts) and MethodIllustration, which
// loads it on the main thread. Both take their types from the generated
// tracematch_wasm.d.ts, so the package must be built before typechecking.

export {};
