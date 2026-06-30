# Planning Service

The Planning Service is a microservice-based solution for generic, multi-robot motion planning.

## Please Note: This repo is being provided "as-is" and it not maintained.

## Code Documentation
Detailed code documentation can be found hosted [here](https://sonyresearch.github.io/planning_service/).

## Overview

### Common Terminology
(Note: Maybe this should go somewhere else? Unsure.)
| Term | Definition |
|---|---|
| Scene | Some configuration of dynamic robot(s) relative to a static environment (consider a UR5 robot mounted on a flat table). This includes some specification for the geometry of each element in the scene. |
| Constraint | A function whose inputs are some part of the state of the robot, and lower and upper bounds on the output of that function. E.g., the position of an end effector relative to a point on the work surface, which must be within some maximum distance. |
| Planning Context | Any combination of some scene (i.e., geometry) plus constraints. This pair, together, comprises the primary resource on which the underlying algorithms operate. |
| GCS | **G**raph of **C**onvex **S**ets. A state-space representation consisting of a graph in which each vertex constitutes a continuous convex subregion of the space, and the presence of an edge between two vertices indicates overlap between the two corresponding regions. *Also*: Any algorithm used to compute shortest paths over such a graph. |
| IRIS | **I**terative **R**egional **I**nflation by **S**emidefinite programming. An algorithm to compute large polytopic regions of collision-free space using convex optimization. |
| ... | ... |

### Architecture
The "Planning Serivce" actually constitutes a small suite of microservices related to motion planning. They are:
* **Motion Planner**. This service provides an API to transmit *motion planning problems* - specified by (1) a context, (2) a start state, (3) a goal state, and (4, optional) some set of constraints - to a suite of underlying solvers, which will produce collision-free, smooth trajectories which satisfy the provided set of constraints. This is the service with which you can expect your robotic system to communicate "at runtime."
* **Artifact Builder**. This service provides an API to generate *planning artifacts* for a given context. These include IRIS regions for the GCS solver, as well as the RRT-generated roadmap. This service is typically run ahead of on-robot operation, and its generated artifacts deployed alongside the motion planner at runtime.
* **Visualizer**. This service provides an API to display a given context in a live [Meshcat](https://github.com/meshcat-dev/meshcat) visualizer sesssion, as well as tools to interact with the session:
  * Visualization of planning artifacts (IRIS regions, PRMs)
  * Visualization of geometric constraints (bounding box on an end effector)
  * Trajectory playback
  * Joint sliders
  * Position streaming support for live robots
  * ...

## Usage
### Services
Each of the above services is containerized and configured via `docker compose`. We provide a script, `scripts/start_services.sh`, as a wrapper around those services, to conveniently run these services in the desired mode. For example, to start up an instance of the planner and visualizer service, respectively, using the production image, a user should invoke
```
./scripts/start_services.sh --planner --viz
```
See `scripts/start_services.sh --help` for full details.

### Development
Our chosen build backend, Bazel, is fully confined to the containers on which we develop. We have created a simple wrapper, `bazelw`, to invoke the `bazel` API inside of our containerized environment for development. If a given command in vanilla Bazel is `bazel test //...`, then in this repo, it is equivalently, `./bazelw test //...`.

By default, our dev container maps no ports out to the host system. To run software with `bazelw` that exposes a port (e.g., our plan solving program which includes browser visualization), simply run your command as `./bazelw [-p|--ports] ...` instead.


### Client
The client libraries (Protobuf schema, C++ and Python client implementations) live in the [`client/`](client/) directory of this repository.


### Contributing
You can follow the [contributing guidelines](.github/CONTRIBUTING.md) to contribute to the planning service
