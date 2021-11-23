export default class RK4 {
  RKNodes = [0, 2 / 9, 1 / 3, 3 / 4, 1, 5 / 6];
  RKMatrix = [
    [],
    [2 / 9],
    [1 / 12, 1 / 4],
    [69 / 128, -243 / 128, 135 / 64],
    [-17 / 12, 27 / 4, -27 / 5, 16 / 15],
    [65 / 432, -5 / 16, 13 / 16, 4 / 27, 5 / 144],
  ];
  RKWeights = [1 / 9, 0, 2 / 20, 16 / 45, 1 / 12, 0];
  RKWweightsWeightedAverage = [47 / 450, 0, 12 / 25, 32 / 225, 1 / 30, 6 / 25];
  RKWeightsTruncationError = [-1 / 150, 0, 3 / 100, -16 / 75, -1 / 20, 6 / 25];
  maximumIterations;
  tol;
  tol2;
  doContinue = true;
  F;
  p5;
  dt;

  constructor(F, p5, tol = 1e-8, tol2 = 1e-3, maximumIterations = 1000) {
    this.F = F;
    this.p5 = p5;
    this.tol = tol;
    this.tol2 = tol2;
    this.maximumIterations = maximumIterations;
  }

  absoluteMaximumReducer = (previous, current) => {
    return this.p5.abs(current) > this.p5.abs(previous)
      ? this.p5.abs(current)
      : this.p5.abs(previous);
  };
  /**
   *
   * @param {number} kIndex
   * @param {number[][]} kValues
   * @param {number[]} JList
   */
  weightedAverageIncrement = (kIndex, kValues, JList) => {
    return JList.map((element, JListIndex) => {
      return Array(kIndex)
        .fill(0)
        .map((_v, kIndex2) => {
          return (
            kValues[kIndex2][JListIndex] *
            this.RKMatrix[kIndex][kIndex2] *
            this.dt
          );
        })
        .reduce((p, q) => p + q, element);
    });
  };
  truncationError = (kIndex, kValues, element) => {
    return Array(kValues.length)
      .fill(0)
      .map(
        (_v, kIndex2) =>
          kValues[kIndex2][kIndex] *
          this.RKWeightsTruncationError[kIndex2] *
          this.dt
      )
      .reduce((p, q) => p + q, element[kIndex]);
  };
  weightedAverage = (kIndex, kValues, truncationErrorList, element) => {
    return Array(kValues.length)
      .fill(0)
      .map(
        (_v, kIndex2) =>
          kValues[kIndex2][kIndex] *
          this.RKWweightsWeightedAverage[kIndex2] *
          this.dt
      )
      .reduce((p, q) => p + q, element[kIndex] + truncationErrorList[kIndex]);
  };

  step = (time, JList) => {
    if (time === undefined && JList === undefined) {
      throw new Error("t and J are undefined");
    } else if (time === undefined) {
      throw new Error("t is undefined");
    } else if (JList === undefined) {
      throw new Error("J is undefined");
    }
    var truncationErrorList;
    var kValues;

    this.numberSteps = 0;
    do {
      kValues = [];
      truncationErrorList = new Array(JList.length).fill(0);
      for (var kIndex = 0; kIndex < this.RKNodes.length; kIndex++) {
        kValues[kIndex] = this.F(
          this.weightedAverageIncrement(kIndex, kValues, JList),
          time + this.dt * this.RKNodes[kIndex]
        );
        if (kIndex === 0) {
          kValues[0] = this.F(JList, time + this.dt * this.RKNodes[0]);
        }
      }

      for (var index3 = 0; index3 < JList.length; index3++) {
        truncationErrorList[index3] = this.truncationError(
          index3,
          kValues,
          truncationErrorList
        );
      }

      this.dt *= this.p5.pow(
        this.tol /
          this.p5.abs(
            truncationErrorList.reduce(this.absoluteMaximumReducer, 0)
          ),
        0.05
      );
      this.numberSteps++;
    } while (
      this.p5.abs(
        this.p5.log(this.tol) -
          this.p5.log(
            this.p5.abs(
              truncationErrorList.reduce(this.absoluteMaximumReducer, 0)
            )
          )
      ) > this.tol2 &&
      this.numberSteps < this.maximumIterations
    );

    this.errorDisplayed = truncationErrorList.reduce(
      this.absoluteMaximumReducer,
      0
    );

    if (this.numberSteps >= this.maximumIterations) {
      this.doContinue = false;
      console.error(
        "Can't reduce tolerance of error below tolerance in " +
          this.maximumIterations +
          " iterations: " +
          this.p5.abs(
            this.p5.log(this.tol) - this.p5.log(this.errorDisplayed)
          ) +
          " > " +
          this.tol2
      );
      return [time, JList];
    }

    var JList1 = JList;
    for (var index = 0; index < JList1.length; index++) {
      JList1[index] = this.weightedAverage(
        index,
        kValues,
        truncationErrorList,
        JList1
      );
    }

    return [time + this.dt, JList1];
  };
}
