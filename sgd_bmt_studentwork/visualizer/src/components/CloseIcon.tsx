import React from "react";

type Props = {
  onClick?: () => void;
};

export const CloseIcon = (props: Props) => {
  const { onClick } = props;
  return (
    <div
      style={{
        width: "20px",
        height: "20px",
        borderRadius: "20px",
        position: "absolute",
        background: "red",
        color: "white",
        display: "flex",
        flexDirection: "row",
        alignItems: "center",
        justifyContent: "center",
        textAlign: "center",
        right: "-10px",
        top: "-10px",
        fontSize: "25px",
        fontFamily: "sans-serif",
        padding: "10px",
        cursor: "pointer",
      }}
      onClick={onClick}
    >
      x
    </div>
  );
};
