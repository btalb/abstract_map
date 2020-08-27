# Abstract Map Python Package

For full details see the [repository website](https://btalb.github.io/abstract_map/). This repository implements the abstract with the following features:

- a novel dynamics-based malleable spatial model for imagining unseen spaces from symbols (which includes simulated springs, friction, repulsive forces, & collision models)
- a visualiser & text-based commentator for introspection of your navigation system (both shown in videos on the [repository website](https://btalb.github.io/abstract_map/))
- easy ROS bindings for getting up & running in simulation or on a real robot
- serialisation methods for passing an entire abstract map state between machines, or saving to file

## Getting up & running

Launch an instance of the abstract map with a symbolic goal:

```bash
roslaunch abstract_map abstract_map.launch _goal:="Lion"
```

Visualise the abstract map:

```bash
rosrun abstract_map visualiser
```
