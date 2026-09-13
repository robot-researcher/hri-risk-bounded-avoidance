# Reproduction notes

The portable package targets Python 3.10+. The robot's original runtime was Ubuntu 18.04 / Python 3.6 on a Jetson Nano with a vendor ROS1 installation. Do not assume the portable package can be installed unchanged into that legacy environment.

The test suite exercises geometry, uncertainty calculations, primitive generation, persistence, swept margins, stale-observation holds and primitive-start checks. It is not a substitute for system-level robot validation.

For measured uncertainty, the input contains 30 stationary samples of encoder ticks for IDs 2, 3 and 4, corresponding to the arm's shoulder/elbow/wrist-pitch chain. Data are demeaned before covariance estimation. One tick is represented as 0.004188790204786391 radians. A diagonal quantization term of one tick squared divided by 12 is reported separately as a modeling allowance. Stationary quantized variation cannot identify absolute bias, tracking error, Gaussian tails or moving-joint uncertainty.

Each uncertainty regime rebuilds its own certificate library. Comparing selectors against a library built for a different covariance would not be a valid certification experiment. The experiment evaluates both adaptive and fixed-threshold decisions on the same clearance/goal grid, including a stale-input and wrong-start replay through the simulated adapter. No physical robot movements are implied.
