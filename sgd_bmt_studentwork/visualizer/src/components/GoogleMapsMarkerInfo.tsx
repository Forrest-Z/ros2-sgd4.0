import React, { useContext, useEffect, useState } from "react";
import ReactJson from "react-json-view";
import { LaneType } from "../types/data";
import { AppContext } from "../contexts";
import { CloseIcon } from "./CloseIcon";

export type BaseInfo = {
  latLong: google.maps.LatLng | null;
  intesectionId: number;
  laneId: number;
  type: LaneType;
};

type Props = {
  baseInfo?: BaseInfo;
};

const MarkerInfoHeader = (props: { type?: LaneType }) => {
  const { type } = props;
  return (
    <div
      style={{
        padding: "20px",
        backgroundColor: "#282c34",
        color: "white",
      }}
    >
      <h2 style={{ fontSize: "80px", padding: "0px", margin: "0px" }}>
        🚦
        {type === "pedestrian" ? "👩‍🦯" : type === "vehicle" ? "🚗" : "❔"}
      </h2>
      <h1 style={{ marginLeft: "20px" }}>Marker Information</h1>
    </div>
  );
};

export const GoogleMapsMarkerInfo = (props: Props) => {
  const { baseInfo } = props;
  const { laneId, type, intesectionId, latLong } = baseInfo ?? {};
  const { setCurrBaseInfo, currIntersection } = useContext(AppContext);

  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    setIsVisible(Boolean(baseInfo?.intesectionId));
  }, [baseInfo?.intesectionId]);

  if (!baseInfo) return null;

  return (
    <div
      className={`myVisibleDiv ${isVisible ? "visible" : ""}`}
      style={{
        textAlign: "left",
        position: "absolute",
        backgroundColor: "white",
        width: "500px",
        boxShadow: "15px 30px 10px #00000066",
        bottom: 0,
      }}
    >
      <MarkerInfoHeader type={type} />
      <div
        style={{ position: "relative", padding: "20px", paddingLeft: "40px" }}
      >
        <h1 style={{ margin: "0px" }}>Lane ID: {laneId}</h1>
        <h2 style={{ margin: "0px" }}>Intersection ID: {intesectionId}</h2>

        <h3>
          Latitude:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;{latLong?.lat()}
          <br></br>
          Longitude:&nbsp;&nbsp;{latLong?.lng()}
        </h3>
        <div style={{ maxHeight: "30vh", overflow: "scroll" }}>
          {currIntersection && (
            <ReactJson src={currIntersection} collapsed={true} />
          )}
        </div>
        <p style={{ fontSize: "10px", textAlign: "end" }}>
          {latLong?.lat()} , {latLong?.lng()}
        </p>
      </div>

      <CloseIcon onClick={() => setCurrBaseInfo(undefined)} />
    </div>
  );
};
