---
layout: default
---

By using the abstract map, a robot navigation system is able to use symbols to purposefully navigate in unseen spaces. Here we show some videos of the abstract map in action, and link to the open source code we make available with the publication of results. The code includes a Python implementation of the abstract map, a 2D simulator for reproducing the zoo experiments, and a mobile phone app for playing symbolic spatial navigation games with a phone. 

# Open source abstract map resources

The first open source contribution provided is Python code for a full implementation of the abstract map available [here](https://github.com/btalb/abstract_map). The implementation features:

- a novel dynamics-based malleable spatial model for imagining unseen spaces from symbols (which includes simulated springs, friction, repulsive forces, & collision models)
- a visualiser & text-based commentator for introspection of your navigation system (both shown in videos on the [repository website](https://btalb.github.io/abstract_map/))
- easy ROS bindings for getting up & running in simulation or on a real robot
- tag readers & interpreters for extracting symbolic spatial information from [AprilTags](http://wiki.ros.org/apriltag_ros) 
- configuration files for the zoo experiments performed on GP-S11 of QUT's Gardens Point campus (see [the paper](https://doi.org/10.1109/TCDS.2020.2993855) for further details)
- serialisation methods for passing an entire abstract map state between machines, or saving to file

We also provide a [configured 2D simulator](https://github.com/btalb/abstract_map_simulator) for reproducing the zoo experiments in a simulation of our environment. The simulator package includes:
- world & launch files for a stage simulation of the GP-S11 environment on QUT's Gardens Point campus
- a tool for creating simulated tags in an environment & saving them to file,
- launch & config files for using the move_base navigation stack with gmapping to explore unseen simulated environments

Lastly, we provide code for the mobile application used by human participants in the zoo experiments. The [phone application](https://github.com/btalb/abstract_map_app), created with Android Studio, includes the following:
- opening screen for users to select experiment name & goal location
- live display of the camera to help users correctly capture a tag
- instant visual feedback when a tag is detected, with colouring to denote whether symbolic spatial information is not the goal (red), navigation information (orange), or the goal (green)
- experiment definitions & tag mappings are creatable via the same XML style used in the [abstract_map](https://github.com/btalb/abstract_map) package
- integration with the [native C AprilTags](https://github.com/AprilRobotics/apriltag) using the Android NDK

We hope these tools can help people engage with our research, and more importantly they can aid future research into the problem of developing robot that utilise symbols in their navigation processes.

# Videos of the abstract map in action

Below are videos for five different symbolic navigation tasks completed as part of the zoo experiments described in our paper. In the experiments the robot was placed in an environment with no existing map, and given a symbolic goal like "find the lion". Could the abstract map, using only the symbolic spatial information available from the [AprilTags](https://april.eecs.umich.edu/software/apriltag.html) in the environment, successfully find the goal?

Human participants who had never visited the environment before were given the same task, with the results showing the abstract map enables symbolic navigation performance comparable to humans in unseen built environments.

### Find the lion

<iframe width="640" height="360" src="https://www.youtube.com/embed/NdhVYIidyUw" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

### Find the kingfisher

<iframe width="640" height="360" src="https://www.youtube.com/embed/O3sRISE1juc" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

### Find the polar bear

<iframe width="640" height="360" src="https://www.youtube.com/embed/oOVVyGs8TgI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

### Find the anaconda

<iframe width="640" height="360" src="https://www.youtube.com/embed/MIdKRc71V2A" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

### Find the toilets

<iframe width="640" height="360" src="https://www.youtube.com/embed/vmssbxaEsyo" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# Citing our work

_If you use the Abstract Map in your research, please cite:_

```bibtex
TODO
```

# Further information

Please see our paper for full details of the abstract map's role in robot navigation systems & the zoo experiments shown above. 

If you want more information about the abstract map see our [related publications](https://scholar.google.com/citations?user=oDCvYTEAAAAJ&hl=en), or you can contact the authors via email: [b.talbot@qut.edu.au](mailto:b.talbot@qut.edu.au)
