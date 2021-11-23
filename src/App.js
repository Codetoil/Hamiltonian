import React, { Component } from "react";
import Sketch from "react-p5";
import State from "./State.js";

export default class App extends Component {
  simulationState;

  setup = (p5, canvasParentRef) => {
    p5.createCanvas(window.innerWidth, window.innerHeight).parent(
      canvasParentRef
    );
    p5.frameRate(this.fr);
    // use parent to render canvas in this ref (without that p5 render this canvas outside your component)
    p5.strokeWeight(5);
    this.simulationState = new State(p5);
    this.simulationState.restart();
  };

  draw = () => {
    this.simulationState.draw();
  };

  render() {
    return <Sketch setup={this.setup} draw={this.draw} />;
  }
}
