import React from "react";

export const DevelopedByLabel = () => {
  return (
    <p
      style={{
        fontSize: "12px",
        textAlign: "center",
        position: "absolute",
        justifySelf: "center",
        display: "flex",
        bottom: 0,
        color: "#282c34",
        fontWeight: "800",
        padding: "10px",
        paddingRight: "40px",
        paddingLeft: "40px",
        backgroundColor: "white",
        borderRadius: "15px",
        pointerEvents: "none",
      }}
    >
      Developed with ❤️ in Hamburg
    </p>
  );
};
