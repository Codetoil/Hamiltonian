import RK4 from "./RK4.js";

export default class State {
  p5;

  simulator;
  val1;
  val2;
  t;
  xmin = -2;
  xmax = 2;
  ymin = -2;
  ymax = 2;
  der;

  constructor(p5) {
    this.p5 = p5;
    this.simulator = new RK4(this.der, p5, 1e-15, undefined, undefined);
  }

  restart = () => {
    this.p5.background(255, 255, 255);
    this.val1 = 1;
    this.val2 = 0;
    this.t = 0;
    this.simulator.dt = 0.001;
    this.simulator.doContinue = true;
  };

  der = (y, t) => {
    return [
      (val1, val2, t) => {
        return -val2;
      },
      (val1, val2, t) => {
        return val1;
      },
    ].map(function (e) {
      return e(y[0], y[1], t);
    });
  };

  draw = () => {
    if (this.p5.mouseIsPressed) {
      this.restart();
    }
    if (this.simulator.doContinue) {
      this.p5.strokeWeight(2);
      this.p5.background(255, 255, 255);
      this.p5.line(
        this.p5.map(this.xmin, this.xmin, this.xmax, 0, this.p5.width),
        this.p5.map(0, this.ymin, this.ymax, this.p5.height, 0),
        this.p5.map(this.xmax, this.xmin, this.xmax, 0, this.p5.width),
        this.p5.map(0, this.ymin, this.ymax, this.p5.height, 0)
      );
      this.p5.line(
        this.p5.map(0, this.xmin, this.xmax, 0, this.p5.width),
        this.p5.map(this.ymin, this.ymin, this.ymax, this.p5.height, 0),
        this.p5.map(0, this.xmin, this.xmax, 0, this.p5.width),
        this.p5.map(this.ymax, this.ymin, this.ymax, this.p5.height, 0)
      );
      this.p5.strokeWeight(1);
      //line(map(positionx1, -10, 10, 0, width), map(positiony1, -10, 10, height, 0), map(positionx2, -10, 10, 0, width), map(positiony2, -10, 10, height, 0));
      //line(map(positionx1, -10, 10, 0, width), map(positiony1, -10, 10, height, 0), map(0, -10, 10, 0, width), map(0, -10, 10, height, 0));
      //line(map(0, -10, 10, 0, width), map(0, -10, 10, height, 0), map(positionx2, -10, 10, 0, width), map(positiony2, -10, 10, height, 0));
      this.p5.text("t: " + this.t, 10, 10);
      this.p5.text("dt: " + this.simulator.dt, 10, 20);
      this.p5.text("err: " + this.simulator.errorDisplayed, 10, 30);
      this.p5.text("val1: " + this.val1, 10, 40);
      this.p5.text("val2: " + this.val2, 10, 50);
      this.p5.fill(0, 0, 0);
      let tempThing = this.simulator.step(this.t, [this.val1, this.val2]);
      this.t = tempThing[0];
      this.val1 = tempThing[1][0];
      this.val2 = tempThing[1][1];
    }
  };
}
