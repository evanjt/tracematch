declare global {
	namespace App {}
}

declare module '/wasm/tracematch_wasm.js' {
	const init: (path: string) => Promise<void>;
	export default init;
	export function createSignature(activityId: string, pointsJson: string, configJson: string): unknown;
	export function compareRoutes(sig1Json: string, sig2Json: string, configJson: string): unknown;
	export function groupRoutes(signaturesJson: string, configJson: string): unknown;
	export function detectSections(tracksJson: string, sportTypesJson: string, groupsJson: string, configJson: string): unknown;
	export function findSectionsInRoute(routeJson: string, sectionsJson: string, configJson: string): unknown;
}

export {};
