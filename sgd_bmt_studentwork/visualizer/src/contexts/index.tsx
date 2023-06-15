import React, { createContext, useState } from "react";
import { getPedestrians, mapData } from "../data/converters/map";
import { Intersection, MapData, WithChildren } from "../types/data";
import { BaseInfo } from "../components/GoogleMapsMarkerInfo";

const API_KEY = process.env.REACT_APP_GOOGLE_API ?? "";

export type ContextType = {
  apiKey: string;
  mapData: MapData;
  simplifiedMapData?: MapData;
  pedestrians?: MapData;
  currBaseInfo?: BaseInfo;
  currIntersection?: Intersection;
  setCurrBaseInfo: (info?: BaseInfo) => void;
};

const useAppHook = (): ContextType => {
  // remove deeply duplicates (all lanes)
  const [currBaseInfo, setCurrBaseInfoTmp] = useState<BaseInfo>();
  const [currIntersection, setCurrIntersection] = useState<Intersection>();

  const simplifiedMapData: MapData = Array.from(
    new Set(mapData.map((item) => JSON.stringify(item)))
  ).map((item) => JSON.parse(item));

  // remove all lanes differents than pedestrians
  const pedestrians = getPedestrians(simplifiedMapData);

  // console.log(pedestrians);

  const setCurrBaseInfo = (info?: BaseInfo) => {
    // pedestrians?.forEach((item) => {
    //   if (item.intersectionId === intesectionId) {
    //     item.lanes.find((lane) => lane.laneId === laneId);
    //   }
    // });
    const selectedIntersection = pedestrians.find(
      (item) => item.intersectionId === info?.intesectionId
    );
    if (selectedIntersection && selectedIntersection.intersectionId) {
      setCurrIntersection(selectedIntersection);
    }

    setCurrBaseInfoTmp(info);
  };

  // console.log(pedestrians);
  // const uniquePedestrians = findDuplicates(pedestrians);
  // console.log(uniquePedestrians);

  return {
    apiKey: API_KEY,
    mapData,
    simplifiedMapData,
    pedestrians,
    currBaseInfo,
    setCurrBaseInfo,
    currIntersection,
  };
};

export const defaultContext: ContextType = {
  apiKey: API_KEY,
  mapData,
  simplifiedMapData: undefined,
  pedestrians: undefined,
  currBaseInfo: undefined,
  currIntersection: undefined,
  setCurrBaseInfo: () => null,
};

export const AppContext = createContext(defaultContext);

type Props = WithChildren;
export const AppContextProvider = ({ children }: Props) => {
  const { Provider } = AppContext;
  const hook = useAppHook();

  return <Provider value={hook}>{children}</Provider>;
};
