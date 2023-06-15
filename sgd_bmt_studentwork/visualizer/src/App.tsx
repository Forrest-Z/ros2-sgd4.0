import React, { useContext, useState, useEffect } from "react";
// import { LaneObjectType, MapData } from "./types/data";
import { AppContext } from "./contexts";
import GoogleMaps from "./components/GoogleMaps";
import logo from "./logo.svg";
import hawLogo from "./icons/haw.svg";
import "./App.css";
import "react-toastify/dist/ReactToastify.css";
import { ToastContainer } from "react-toastify";
import { toast } from "react-toastify";

function App() {
  const {
    mapData,
    simplifiedMapData,
    pedestrians,
    apiKey: googleMapsApiKey,
  } = useContext(AppContext);

  const total = mapData.length;
  const simplifiedTotal = simplifiedMapData?.length ?? 0;

  const inputRef = React.createRef<HTMLInputElement>();
  const [someKey, setApiKey] = useState("");

  useEffect(() => {
    console.warn("setting new google api key -> ", googleMapsApiKey);
    setApiKey(googleMapsApiKey);
  }, []);

  useEffect(() => {
    if (!googleMapsApiKey && !someKey) {
      toast.error(
        <div>
          <input ref={inputRef} type="text" placeholder="Google API key" />
          <button
            onClick={() => {
              toast.dismiss();
              setApiKey(inputRef.current?.value ?? "");
            }}
          >
            Submit
          </button>
        </div>,
        { closeButton: false, autoClose: false }
      );
    }
  }, [googleMapsApiKey, someKey, setApiKey, inputRef]);

  return (
    <div className="App">
      <header className="App-header">
        <div
          style={{
            textAlign: "left",
            marginLeft: "25px",
            color: "white",
            fontSize: "1.2rem",
            position: "absolute",
            left: 0,
          }}
        >
          <p>
            Unique intersection entries: <b>{simplifiedTotal}</b>
            <br></br>
            Pedestrians intersection entries: <b>{pedestrians?.length ?? 0}</b>
            <br></br>
            Collected intersection entries: <b>{total}</b>
            <br></br>
          </p>
        </div>
        <h2>Shared Dog 🚦 Traffic Lights</h2>
        <img
          src={hawLogo}
          width="140px"
          className="haw-logo "
          alt="haw-logo"
        ></img>
      </header>
      {someKey !== "" ? (
        <GoogleMaps
          googleMapsApiKey={someKey}
          intersections={pedestrians?.map((item) => ({
            id: item.intersectionId,
            trafficLights: item.lanes.map((lane) => ({
              id: lane.laneId,
              position: lane.nodes.coordinates?.map((coordinate) => ({
                lng: coordinate[0],
                lat: coordinate[1],
              })),
            })),
          }))}
        />
      ) : (
        <h1 style={{ marginTop: "200px" }}>
          Please enter your Google Maps API key first
        </h1>
      )}
      <ToastContainer
        position="top-right"
        // autoClose={5000}
        hideProgressBar={false}
        newestOnTop={false}
        closeOnClick={false}
        rtl={false}
        // pauseOnFocusLoss
        draggable={false}
        pauseOnHover
        theme="light"
      />
    </div>
  );
}

export default App;
