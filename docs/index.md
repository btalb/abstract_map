---
layout: default
---

By using the abstract map, a robot navigation system is able to use symbols for purposefully navigate in unseen spaces. Here we show some videos of the abstract map in action, and link to a Python implementation of an abstract map that uses spring-based dynamics to imagine malleable spatial models for unseen spaces.

# Our implementation of the abstract map

Code is available on Github [here](https://github.com/btalb/abstract_map). The implementation features:

- a novel dynamics-based malleable spatial model for imagining unseen spaces from symbols (which includes simulated springs, friction, repulsive forces, & collision models)
- a visualiser & text-based commentator for introspection of your navigation system (both seen in the videos below)
- easy ROS bindings for getting up & running in simulation or on a real robot
- serialisation methods for passing an entire abstract map state between machines, or saving to file

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
