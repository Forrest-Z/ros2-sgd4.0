type CoordinatePair = [number, number];
type RefPointType = "Point";

type IntersectionType = "v2x-map";

type PossibleManeuvers = "straight" | "unknown";

export type LaneType = "vehicle" | "pedestrian";

type Connection = {
  connectedLaneId: number;
  possibleManeuvers: PossibleManeuvers;
  signalGroupId: number;
};

type NodeType = "LineString";

export type LaneObjectType = {
  bearing_deg: number;
  connections: Array<Connection>;
  egressLane: boolean;
  ingressLane: boolean;
  laneId: number;
  laneType: LaneType;
  nodes: { coordinates: Array<CoordinatePair>; type: NodeType };
  possibleManeuvers: PossibleManeuvers;
};

export type Intersection = {
  intersectionId: number;
  laneWidth_m: number;
  lanes: Array<LaneObjectType>;
  refPoint: { coordinates: CoordinatePair; type: RefPointType };
  type: IntersectionType;
};

export type MapData = Array<Intersection>;
