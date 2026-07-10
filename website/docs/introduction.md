# PlanningService API ###

The robot uses a cd to solve for motion plans which satisfy constraints. This `PlanningService` consists of an `app` which:
- runs a loop that has a `comms` layer listening for messages and publishing to the `client` which provides an abstraction to many different communication paradigms (lcm, ROS, Viam)
- a `planner` that tranlates the `PlanningProblemDefinition` into a set of `drake::Constraints` and solves the planning problem
- a `data` layer which stores pre-computed information to reduce planning times, and periodically makes insertions into the database to improve performance
- a `client` interface which inherits from the same `comms` library used by the `comms` layer and allows other code to use the service.

It is also possible to directly instantiate a `planner` in c++ code and not make use of the services architecture.

The user transmits a `robot_description` (either URDF or SDF), a `PlanningProblemDefinition` which consists of a `start`, a `goal`, and a set of rules for creating a set of `drake::Constraints` via the `communication_layer` which then populates a c++ struct and calls the planner. The `PlanningResult` is serialized by the `communication_layer` and transmitted back to the caller.

```mermaid
  graph LR;
    A[client] -- Serialize --> B[comms];
    B -- c++ api --> C[planner];
    C --> B;
    B -- Serialize --> A;
```

## Supported Functions

We support two different methods for finding a motion plan. Either arbitrary motion is allowed along as it satisfies constraints, or we require the specific motion of a frame and solve via IK.
```c++
// these are really the same
FindMotionPlan();
CreateActionPart();
ConstrainedMotionPlan();
```

```c++
FindLinearMotionPlan();
IkMotionPlan();
InverseKinematicsMotionPlan();
LinearMotionPlan();
FrameConstrainedToFunctionMotionPlan();
```
## Developing
### Prerequisites
docker and the docker compose plugin for docker

### Overview
The planning service is built with bazel inside a docker image.
There are two ways to run it
1. `bazelw` is an included wrapper that will run all of the bazel commands inside the image with appropriate mount points and configuration for a development setup. This is easiest to get setup.
2. `dazel` we are experimenting with dazel. To run with dazel clone https://github.com/nadirizr/dazel to your local system and install the dazel script somewhere on your local path, e.g. on linux:
```
cd $HOME
git clone git@github.com:nadirizr/dazel.git
cd dazel
pip install dazel
sudo ln -s $HOME/dazel/scripts/dazel /usr/local/bin/dazel
```
Dazel may be configured with the .dazelrc file.

### Common Tasks
`./run_unit_tests.sh` to build and test your changes
`docker compose build` to rebuild the image in case you have made Dockerfile changes


### Documentation
`./scripts/generate_docs.sh` to build and view doxygen-style documentation.
