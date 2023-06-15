import React, { useContext } from "react";
import { DevelopedByLabel } from "./DevelopedByLabel";
import { BaseInfo, GoogleMapsMarkerInfo } from "./GoogleMapsMarkerInfo";
import { GoogleMap, useJsApiLoader, Marker } from "@react-google-maps/api";
import { AppContext } from "../contexts";
import icon from "../icons/ampel.png";

const containerStyle = {
  display: "grid",
  width: "100%",
  height: "88vh",
};

const center = {
  lat: 53.5553711864759,
  lng: 9.979631054679727,
};

type Intersections = Array<{
  id: number;
  trafficLights?: Array<{
    position: Array<google.maps.LatLng | google.maps.LatLngLiteral>;
    id: number;
  }>;
}>;

type Props = {
  intersections?: Intersections;
  googleMapsApiKey: string;
};

const GoogleMaps = (props: Props) => {
  const { currBaseInfo, setCurrBaseInfo } = useContext(AppContext);
  const mapRef = React.createRef<GoogleMap>();
  const { intersections, googleMapsApiKey } = props;

  const onMapLoad = (map: google.maps.Map) => {
    //     const bounds = new window.google.maps.LatLngBounds(center);
    //     map.fitBounds(bounds);
  };

  const { isLoaded } = useJsApiLoader({
    id: "google-map-script",
    googleMapsApiKey,
  });

  const onMarkerClick = (
    event: google.maps.MapMouseEvent,
    payload: BaseInfo
  ) => {
    setCurrBaseInfo(payload);

    /*
    let tmpIntersections: Intersections = [];

    const targetIntersection = intersections?.find((item) => item.id === 1382);
    if (targetIntersection) {
      tmpIntersections.push(targetIntersection);
    }

    tmpIntersections.map(({ trafficLights, id: intesectionId }) => {
      console.log("tmpIntersections");

      return trafficLights?.map(({ position, id: laneId }, index) => {
        console.log(`trafficLights ${index}`);

        return position?.map((position) => {
          console.log("position");
        });
      });
    });
    */
  };

  return isLoaded &&
    Array.isArray(intersections) &&
    intersections?.length > 0 ? (
    <GoogleMap
      ref={mapRef}
      mapContainerStyle={containerStyle}
      center={center}
      mapTypeId="satellite"
      zoom={19}
      onLoad={onMapLoad}
    >
      <DevelopedByLabel />
      {intersections.map(({ trafficLights, id: intesectionId }) =>
        trafficLights?.map(({ position, id: laneId }, index) => {
          return position?.map((position) => {
            return (
              <Marker
                key={`${position.lat}_${position.lng}`}
                position={position}
                icon={icon}
                onClick={(event) =>
                  onMarkerClick(event, {
                    laneId,
                    intesectionId,
                    type: "pedestrian",
                    latLong: event.latLng,
                  })
                }
                label={{
                  text: `${laneId}`,
                  fontWeight: "600",
                  fontSize: "30px",
                  color: "white",
                  className: "marker-class",
                }}
              />
            );
          });
        })
      )}
      <GoogleMapsMarkerInfo baseInfo={currBaseInfo} />
    </GoogleMap>
  ) : (
    <></>
  );
};

export default React.memo(GoogleMaps);
