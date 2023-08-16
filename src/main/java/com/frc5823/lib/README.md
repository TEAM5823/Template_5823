# Library README

## Packages
- [`com.frc5823.lib.auto`](auto)

  Auto toolkit. Used for placing various base classes for actions and mode, as well as mode executors with exception handling.

- [`com.frc5823.lib.controllers`](controllers)

  Controller algorithm package, containing all controller algorithms, such as HeadingController for holonomic robot chassis orientation control, and commonly used `PIDF` controller algorithms.

- [`com.frc5823.lib.drivers`](drivers)

  Driver package, used to place classes that rewrap some devices on the robot that are used frequently or for which the native interface is not convenient enough. Such as `LazyTalonFX`, `Limelight` etc.

- [`com.frc5823.lib.geometry`](geometry)

  Geometry library developed by `Team254`.

- [`com.frc5823.lib.io`](io)

  Used to store commonly used IO devices, such as `StatefulXboxController`.

- [`com.frc5823.lib.kinematics`](kinematics)

  All forward and inverse kinematics solution classes.

- [`com.frc5823.lib.loops`](loops)

  Looper package. In the Command Base structure, the main executor of Command is `Command Scheduler`. Instead, we use `Looper` as the main executor of the robot program, Looper will execute Loop at a custom frequency, which describes the behavior of a control loop in the `onStart`, `onLoop`, `onStop` three different stages. Generally, each Subsystem has its own Loop.

- [`com.frc5823.lib.math`](math)

  Used to store all mathematical algorithms.

- [`com.frc5823.lib.planners`](planners)

  Planner package. planner is used to plan and optimize various behaviors of the robot, such as `DriveMotionPlanner` outputs chassis motion vectors according to the trajectory input.

- [`com.frc5823.lib.spline`](spline)

  Used to store all types of curves and curve generators, such as `CubicHermiteSpline` and `QuinticHermiteSpline`, etc.

- [`com.frc5823.lib.trajectory`](trajectory)

  Trajectory generation algorithm developed by `Team254`, providing a convenient and powerful trajectory generation interface, simplifying our trajectory tracking work.

- [`com.frc5823.lib.utils`](utils)

  All utility algorithms.