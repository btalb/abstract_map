from __future__ import absolute_import
import re

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

    def addSymbolicSpatialInformation(self, ssi, pose, tag_id=-1):
        """Adds new symbolic spatial information to the abstract map"""
        pass

    def updateSymbolicSpatialInformation(self, ssi, pose, tag_id):
        """Updates existing symbolic spatial information in the abstract map"""
        assert tag_id >= 0, "can't update SSI without a valid tag_id"
        pass


class _ComponentRegex(object):
    """Helper class to facilitate single compile regex"""
    FIGURE = re.compile(r'(?:^|, )([^,]*?) is')
    RELATION = re.compile(r'is\s((?:\w+)(?:\sof)?)')
    REFERENCES = re.compile(r'is\s\w+(?:\sof)?\s(.*?)(?:, from .*)?$')
    CONTEXT = re.compile(r'[Ff]rom ([^,]*).*$')

    SPLIT = re.compile(r', | and ')
    STRIP = re.compile(r'\b(?:and|the|a)\b')

    @staticmethod
    def get(component, string):
        """Runs regex component against string, extracting match"""
        g = re.search(component, string)
        if g is None:
            return ''
        elif component == _ComponentRegex.REFERENCES:
            return _ComponentRegex.stringToList(g.group(1))
        else:
            return _ComponentRegex.stripNoun(g.group(1))

    @staticmethod
    def stringToList(string):
        """Converts a string to the list of items"""
        return [
            _ComponentRegex.stripNoun(x).strip()
            for x in re.split(_ComponentRegex.SPLIT, string)
        ]

    @staticmethod
    def stripNoun(string):
        """Strips common words and characters from a noun"""
        return re.sub(_ComponentRegex.STRIP, '', string).strip()


def ssiToConstraints(ssi):
    """Converts a SSI string to a list of constraints"""
    figure, relation, references, context = _ssiToComponents(ssi)
    return _componentsToConstraints(figure, relation, references, context)


def _ssiToComponents(ssi):
    """Converts a SSI string to its symbolic components"""
    return (_ComponentRegex.get(_ComponentRegex.FIGURE, ssi),
            _ComponentRegex.get(_ComponentRegex.RELATION, ssi),
            _ComponentRegex.get(_ComponentRegex.REFERENCES, ssi),
            _ComponentRegex.get(_ComponentRegex.CONTEXT, ssi))


def _componentsToConstraints(figure, relation, references, context=""):
    """Converts symbolic components into a list of constraints"""
    pass
