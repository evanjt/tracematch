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

// FrequentSection uses #[serde(rename_all = "camelCase")]
export interface FrequentSection {
  id: string;
  name: string | null;
  sportType: string;
  polyline: GpsPoint[];
  representativeActivityId: string;
  activityIds: string[];
  activityPortions: SectionPortion[];
  routeIds: string[];
  visitCount: number;
  distanceMeters: number;
  confidence: number;
  observationCount: number;
  averageSpread: number;
}

// SectionMatch has NO rename_all — stays snake_case
export interface SectionMatch {
  section_id: string;
  start_index: number;
  end_index: number;
  match_quality: number;
  same_direction: boolean;
}
