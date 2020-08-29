# Abstract Map Python Package

For full details see the [repository website](https://btalb.github.io/abstract_map/). This repository implements the abstract with the following features:

- a novel dynamics-based malleable spatial model for imagining unseen spaces from symbols (which includes simulated springs, friction, repulsive forces, & collision models)
- a visualiser & text-based commentator for introspection of your navigation system (both shown in videos on the [repository website](https://btalb.github.io/abstract_map/))
- easy ROS bindings for getting up & running in simulation or on a real robot
- serialisation methods for passing an entire abstract map state between machines, or saving to file

## Getting up & running

Launch an instance of the abstract map with a symbolic goal:

```bash
roslaunch abstract_map abstract_map.launch goal:="Lion"
```

Visualise the abstract map:

```bash
rosrun abstract_map visualiser
```

Human Cues Tag Reader Package
=============================

The repository is for a ROS package wrapping the AprilTag reading part of the Human Cues journal experiments. The function of the package is to:

- Provide config for launching an AprilTag detector package with images from the Guiabot
- Provide an interface for taking an AprilTag detection, and outputting the corresponding symbolic spatial information

Experiment definitions will not go here, they will be defined in isolation so they can be used here and in the Android app.


Human Cues Tag Experiments
=============================

Package provides the experiment definitions for the Human Cues Tag Experiments. An experiment consists of a series of mappings from tag_id to symbolic spatial information (with the type of information also specified), a default goal location, and an optional experiment name.
