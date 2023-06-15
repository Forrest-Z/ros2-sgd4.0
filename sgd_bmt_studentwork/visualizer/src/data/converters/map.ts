import { MapData } from "../../types/data";
import data from "../processed/map.json";

export const mapData: MapData = data as MapData;

export const getPedestrians = (someData: MapData) =>
  someData.map((data) => {
    const pedestrianLanes = data.lanes.filter(
      (lane) => lane.laneType === "pedestrian"
    );

    const updatedLanes = data.lanes
      .filter((lane) => lane.laneType === "pedestrian")
      .map((lane) => {
        return {
          ...lane,
          nodes: {
            coordinates: pedestrianLanes
              .filter((pedestrianLane) => pedestrianLane.laneId === lane.laneId)
              .map((pedestrianLane) => pedestrianLane.nodes.coordinates)
              .reduce((acc, cur) => acc.concat(cur), []),
            type: lane.nodes.type,
          },
        };
      });

    // Return the updated MapData object
    return {
      ...data,
      lanes: updatedLanes,
    };
  });

// export function findDuplicates(
//   someInput: ReturnType<typeof getPedestrians>
// ): LaneObjectType[] {
//   const seen = new Set<number>();
//   const duplicates: LaneObjectType[] = [];

//   const result: Array<LaneObjectType> = [];
//   someInput.map((input) => {
//     input.lanes.forEach((lane) => result.push(lane));
//   });

//   console.debug("pedestrians with duplicates: ", result);

//   for (const el of result) {
//     if (seen.has(el.laneId)) {
//       duplicates.push(el);
//     } else {
//       seen.add(el.laneId);
//     }
//   }

//   return duplicates;
// }
