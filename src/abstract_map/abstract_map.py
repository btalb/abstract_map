from __future__ import absolute_import

import abstract_map.spatial_layout as sl


class AbstractMap(object):
    """The abstract map, used to apply abstract ideas about space"""

    def __init__(self, goal, start_x, start_y, start_th):
        """Constructs a new empty abstract map, with a given symbolic goal"""
        self._goal = goal
        self._start_x = start_x
        self._start_y = start_y
        self._start_th = start_th

        # Initialise a spatial layout with the information provided
        self._spatial_layout = sl.SpatialLayout()
        # TODO add start mass, and constraint to origin

    # TODO abstract map needs to take in some sort of symbolic spatial
    # information, convert it into constraints, and feed them into the spatial
    # layout
