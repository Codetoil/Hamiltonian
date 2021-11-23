import RK4 from "./RK4.js";

export default class State {
  p5;

  simulator;
  momentumx1;
  momentumy1;
  positionx1;
  positiony1;
  momentumx2;
  momentumy2;
  positionx2;
  positiony2;
  s;
  m1 = 1;
  m2 = 1;
  mc = 1000;
  k = -13.317;
  c = 0;
  xmin = -2;
  xmax = 2;
  ymin = -2;
  ymax = 2;
  deltaMomentumx = 0.0001;
  deltaMomentumy = 0.0001;
  deltaPositionx = 0.0001;
  deltaPositiony = 0.0001;
  der;

  constructor(p5) {
    this.p5 = p5;
    this.simulator = new RK4(this.der, p5, 1e-6, undefined, 1e5);
  }

  restart = () => {
    this.p5.background(255, 255, 255);
    this.positionx1 = 1.5;
    this.positiony1 = 0;
    this.positionx2 = 1;
    this.positiony2 = 0;
    this.momentumx1 = 0;
    this.momentumy1 = 70;
    this.momentumx2 = 0;
    this.momentumy2 = 100;
    this.s = 0;
    this.simulator.dt = 1e-10;
    this.simulator.doContinue = true;
    console.clear();
  };

  hamiltonian = (x1, y1, x2, y2, px1, py1, px2, py2, t) => {
    return (
      (this.k * this.m1 * this.m2) /
        this.p5.sqrt(
          this.p5.pow(this.p5.abs(x1 - x2), 2) +
            this.p5.pow(this.p5.abs(y1 - y2), 2)
        ) +
      (this.k * this.m1 * this.mc) /
        this.p5.sqrt(
          this.p5.pow(this.p5.abs(x1), 2) + this.p5.pow(this.p5.abs(y1), 2)
        ) +
      (this.k * this.m2 * this.mc) /
        this.p5.sqrt(
          this.p5.pow(this.p5.abs(x2), 2) + this.p5.pow(this.p5.abs(y2), 2)
        ) +
      0.5 * (1 / this.m1) * (this.p5.pow(px1, 2) + this.p5.pow(py1, 2)) +
      0.5 * (1 / this.m2) * (this.p5.pow(px2, 2) + this.p5.pow(py2, 2))
    );
  };

  forcex1 = (x1, y1, x2, y2, px1, py1, px2, py2, t) => {
    return (
      -(
        this.hamiltonian(
          x1 + this.deltaPositionx,
          y1,
          x2,
          y2,
          px1,
          py1,
          px2,
          py2,
          t
        ) -
        this.hamiltonian(
          x1 - this.deltaPositionx,
          y1,
          x2,
          y2,
          px1,
          py1,
          px2,
          py2,
          t
        )
      ) /
        (2 * this.deltaPositionx) -
      this.c * px1
    );
  };

  forcey1 = (x1, y1, x2, y2, px1, py1, px2, py2, t) => {
    return (
      -(
        this.hamiltonian(
          x1,
          y1 + this.deltaPositiony,
          x2,
          y2,
          px1,
          py1,
          px2,
          py2,
          t
        ) -
        this.hamiltonian(
          x1,
          y1 - this.deltaPositiony,
          x2,
          y2,
          px1,
          py1,
          px2,
          py2,
          t
        )
      ) /
        (2 * this.deltaPositiony) -
      this.c * py1
    );
  };

  velocityx1 = (x1, y1, x2, y2, px1, py1, px2, py2, t) => {
    return (
      (this.hamiltonian(
        x1,
        y1,
        x2,
        y2,
        px1,
        py1 + this.deltaMomentumx,
        px2,
        py2,
        t
      ) -
        this.hamiltonian(
          x1,
          y1,
          x2,
          y2,
          px1 - this.deltaMomentumx,
          py1,
          px2,
          py2,
          t
        )) /
      (2 * this.deltaMomentumx)
    );
  };

  velocityy1 = (x1, y1, x2, y2, px1, py1, px2, py2, t) => {
    return (
      (this.hamiltonian(
        x1,
        y1,
        x2,
        y2,
        px1,
        py1 + this.deltaMomentumy,
        px2,
        py2,
        t
      ) -
        this.hamiltonian(
          x1,
          y1,
          x2,
          y2,
          px1,
          py1 - this.deltaMomentumy,
          px2,
          py2,
          t
        )) /
      (2 * this.deltaMomentumy)
    );
  };

  forcex2 = (x1, y1, x2, y2, px1, py1, px2, py2, t) => {
    return (
      -(
        this.hamiltonian(
          x1,
          y1,
          x2 + this.deltaPositionx,
          y2,
          px1,
          py1,
          px2,
          py2,
          t
        ) -
        this.hamiltonian(
          x1,
          y1,
          x2 - this.deltaPositionx,
          y2,
          px1,
          py1,
          px2,
          py2,
          t
        )
      ) /
        (2 * this.deltaPositionx) -
      this.c * px2
    );
  };

  forcey2 = (x1, y1, x2, y2, px1, py1, px2, py2, t) => {
    return (
      -(
        this.hamiltonian(
          x1,
          y1,
          x2,
          y2 + this.deltaPositiony,
          px1,
          py1,
          px2,
          py2,
          t
        ) -
        this.hamiltonian(
          x1,
          y1,
          x2,
          y2 - this.deltaPositiony,
          px1,
          py1,
          px2,
          py2,
          t
        )
      ) /
        (2 * this.deltaPositiony) -
      this.c * py2
    );
  };
  velocityx2 = (x1, y1, x2, y2, px1, py1, px2, py2, t) => {
    return (
      (this.hamiltonian(
        x1,
        y1,
        x2,
        y2,
        px1,
        py1,
        px2 + this.deltaMomentumx,
        py2,
        t
      ) -
        this.hamiltonian(
          x1,
          y1,
          x2,
          y2,
          px1,
          py1,
          px2 - this.deltaMomentumx,
          py2,
          t
        )) /
      (2 * this.deltaMomentumx)
    );
  };

  velocityy2 = (x1, y1, x2, y2, px1, py1, px2, py2, t) => {
    return (
      (this.hamiltonian(
        x1,
        y1,
        x2,
        y2,
        px1,
        py1,
        px2,
        py2 + this.deltaMomentumy,
        t
      ) -
        this.hamiltonian(
          x1,
          y1,
          x2,
          y2,
          px1,
          py1,
          px2,
          py2 - this.deltaMomentumy,
          t
        )) /
      (2 * this.deltaMomentumy)
    );
  };

  der = (y, t) => {
    return [
      this.velocityx1,
      this.velocityy1,
      this.velocityx2,
      this.velocityy2,
      this.forcex1,
      this.forcey1,
      this.forcex2,
      this.forcey2,
    ].map(function (e) {
      return e(y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7], t);
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
      this.p5.strokeWeight(10);
      this.p5.point(
        this.p5.map(0, this.xmin, this.xmax, 0, this.p5.width),
        this.p5.map(0, this.ymin, this.ymax, this.p5.height, 0)
      );
      this.p5.strokeWeight(5);
      this.p5.point(
        this.p5.map(this.positionx1, this.xmin, this.xmax, 0, this.p5.width),
        this.p5.map(this.positiony1, this.ymin, this.ymax, this.p5.height, 0)
      );
      this.p5.point(
        this.p5.map(this.positionx2, this.xmin, this.xmax, 0, this.p5.width),
        this.p5.map(this.positiony2, this.ymin, this.ymax, this.p5.height, 0)
      );
      this.p5.strokeWeight(1);
      //line(map(positionx1, -10, 10, 0, width), map(positiony1, -10, 10, height, 0), map(positionx2, -10, 10, 0, width), map(positiony2, -10, 10, height, 0));
      //line(map(positionx1, -10, 10, 0, width), map(positiony1, -10, 10, height, 0), map(0, -10, 10, 0, width), map(0, -10, 10, height, 0));
      //line(map(0, -10, 10, 0, width), map(0, -10, 10, height, 0), map(positionx2, -10, 10, 0, width), map(positiony2, -10, 10, height, 0));
      this.p5.text("t: " + this.s, 10, 10);
      this.p5.text("dt: " + this.simulator.dt, 10, 20);
      this.p5.text("n: " + this.simulator.numberSteps, 10, 30);
      this.p5.text("err: " + this.simulator.errorDisplayed, 10, 40);
      this.p5.text(
        "H: " +
          this.hamiltonian(
            this.positionx1,
            this.positiony1,
            this.positionx2,
            this.positiony2,
            this.momentumx1,
            this.momentumy1,
            this.momentumx2,
            this.momentumy2,
            this.s
          ),
        10,
        50
      );
      this.p5.fill(0, 0, 0);
      let tempThing = this.simulator.step(this.s, [
        this.positionx1,
        this.positiony1,
        this.positionx2,
        this.positiony2,
        this.momentumx1,
        this.momentumy1,
        this.momentumx2,
        this.momentumy2,
      ]);
      this.s = tempThing[0];
      this.positionx1 = tempThing[1][0];
      this.positiony1 = tempThing[1][1];
      this.positionx2 = tempThing[1][2];
      this.positiony2 = tempThing[1][3];
      this.momentumx1 = tempThing[1][4];
      this.momentumy1 = tempThing[1][5];
      this.momentumx2 = tempThing[1][6];
      this.momentumy2 = tempThing[1][7];
    }
  };
}
