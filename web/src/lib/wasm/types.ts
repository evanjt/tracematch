export interface GpsPoint {
  latitude: number;
  longitude: number;
  elevation?: number;
}

export interface Bounds {
  min_lat: number;
  max_lat: number;
  min_lng: number;
  max_lng: number;
}

export interface RouteSignature {
  activity_id: string;
  points: GpsPoint[];
  total_distance: number;
  start_point: GpsPoint;
  end_point: GpsPoint;
  bounds: Bounds;
  center: GpsPoint;
}

export interface MatchResult {
  activity_id_1: string;
  activity_id_2: string;
  match_percentage: number;
  direction: string;
  amd: number;
}

// RouteGroup uses #[serde(rename_all = "camelCase")]
export interface RouteGroup {
  groupId: string;
  representativeId: string;
  activityIds: string[];
  sportType: string;
  bounds?: Bounds;
  customName?: string;
}

export interface SectionPortion {
  activityId: string;
  startIndex: number;
  endIndex: number;
  distanceMeters: number;
  direction: string;
}

// What a line's shape and profile say it is (Rust: SectionClass).
export type SectionClass = 'climb' | 'descent' | 'rolling' | 'flat' | 'loop';

// Computed inside detection, so always present on a returned section.
export interface Enrichment {
  elevationGainM?: number;
  elevationLossM?: number;
  avgGradePercent?: number;
  maxGradePercent?: number;
  straightness?: number;
  klass?: SectionClass;
  isLift: boolean;
}

// FrequentSection uses #[serde(rename_all = "camelCase")].
//
// Optional Rust fields arrive as `undefined`, not `null`:
// serde-wasm-bindgen's Serializer has serialize_missing_as_null = false.
// `name` is the exception, it is written explicitly.
export interface FrequentSection {
  id: string;
  name: string | null;
  sportType: string;
  polyline: GpsPoint[];
  representativeActivityId: string;
  // The reference triple's range into the representative activity.
  representativeRange?: [number, number];
  activityIds: string[];
  activityPortions: SectionPortion[];
  routeIds: string[];
  visitCount: number;
  distanceMeters: number;
  // A Rust HashMap, so serde-wasm-bindgen emits a JS Map, not an object.
  // Detection leaves it empty; it is populated on demand elsewhere.
  activityTraces: Map<string, GpsPoint[]>;
  confidence: number;
  observationCount: number;
  averageSpread: number;
  pointDensity: number[];
  scale?: string;
  isUserDefined: boolean;
  stability: number;
  elevationGainM?: number;
  avgGradePercent?: number;
  enrichment: Enrichment;
  // Only set when a caller runs ranking, which the site does not.
  rank?: RankFeatures;
  version: number;
  updatedAt?: string;
  createdAt?: string;
}

// Populated only by an explicit rank pass, kept so the shape is known.
export interface RankFeatures {
  [key: string]: unknown;
}

// Why the detector cut where it did, or why a candidate backed off.
// Rust: #[serde(tag = "kind", rename_all = "snake_case")].
export type BoundaryReason =
  | { kind: 'usage_change'; shared: number; mismatched: number }
  | {
      kind: 'fork';
      through: number;
      needed: number;
      branchLeavers: number;
      branchActivityIds: string[];
    }
  | { kind: 'backoff'; represented: number; probed: number; scoreMetres: number }
  | { kind: 'trim'; keptMetres: number; droppedMetres: number }
  | { kind: 'no_single_pass'; bestPenalty: number; portions: number }
  | { kind: 'low_support'; floor: number; droppedCells: number }
  | { kind: 'traffic_cliff'; thin: number; thick: number }
  | { kind: 'pass_end'; requeuedCells: number };

// A geolocated reason for a boundary, in place of a log line.
export interface BoundaryRecord {
  latitude: number;
  longitude: number;
  reason: BoundaryReason;
}

// What detectSectionsUnified returns.
export interface UnifiedDetection {
  sections: FrequentSection[];
  boundaries: BoundaryRecord[];
}

// SectionMatch has NO rename_all — stays snake_case
export interface SectionMatch {
  section_id: string;
  start_index: number;
  end_index: number;
  match_quality: number;
  same_direction: boolean;
}
