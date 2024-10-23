import React from "react";
import AMController from "./components/AM_controller";
import ArmManuelControl from "./components/Arm_Manuel_Controls";

function App() {
  return (
    <div className="container-arm mx-auto">
      <AMController/>
      <ArmManuelControl/>
    </div>
  );
}

export default App;

