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

{% include video_player.html video_name="lion_5_short" %}

### Find the kingfisher

{% include video_player.html video_name="kingfisher_1_short" %}

### Find the polar bear

{% include video_player.html video_name="polar_bear_2_short" %}

### Find the anaconda

{% include video_player.html video_name="anaconda_4_short" %}

### Find the toilets

{% include video_player.html video_name="toilets_2_short" %}

# Citing our work

_If you use the Abstract Map in your research, please cite:_

```bibtex
TODO
```

# Further information

Please see our paper for full details of the abstract map's role in robot navigation systems & the zoo experiments shown above. 

If you want more information about the abstract map see our [related publications](https://scholar.google.com/citations?user=oDCvYTEAAAAJ&hl=en), or you can contact the authors via email: [b.talbot@qut.edu.au](mailto:b.talbot@qut.edu.au)
