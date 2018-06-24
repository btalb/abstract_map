from __future__ import absolute_import
import numpy as np
import pudb
import math
import re

import abstract_map.spatial_layout as sl


class AbstractMap(object):
    """The abstract map, used to apply abstract ideas about space"""
    TAG_SYNONYMS = ['here']

    def __init__(self, goal, start_x, start_y, start_th):
        """Constructs a new empty abstract map, with a given symbolic goal"""
        self._goal = goal
        self._start_x = start_x
        self._start_y = start_y
        self._start_th = start_th

        # Initialise a spatial layout with the information provided
        self._spatial_layout = sl.SpatialLayout()
        # TODO add start mass, and constraint to origin

    def _constraintsFromSsiMsg(self, ssi, pose, tag_id):
        """General function to convert all data in an SSI msg to constraints"""
        # Get the initial list of constraints from the SSI
        cs = ssiToConstraints(ssi)

        # Process the list, making any required adjustments
        mass_fixed = sl.MassFixed(('?'
                                   if tag_id is None else '#%d' % (tag_id)),
                                  np.array(pose[:2]))
        for c in cs:
            # Apply the tag_id
            c._tag_id = tag_id

            # Handle cases where we use context from the tag id pose to assist
            # in interpreting the symbolic spatial information
            b_is_tag = (c._mass_b is None or
                        c._mass_b.name in AbstractMap.TAG_SYNONYMS)
            c_is_tag = len(c.masses()) == 3 and (
                c._mass_c is None or
                c._mass_c.name in AbstractMap.TAG_SYNONYMS)
            if b_is_tag and c_is_tag:
                raise ValueError(
                    "Constraint (%s) suggests both mass B and C are context" %
                    (c))
            elif b_is_tag:
                c._mass_b = mass_fixed
            elif c_is_tag:
                c._mass_c = mass_fixed

        # Return the final list of constraints
        return cs

    def addSymbolicSpatialInformation(self, ssi, pose, tag_id=None):
        """Adds new symbolic spatial information to the abstract map"""
        cs = self._constraintsFromSsiMsg(ssi, pose, tag_id)
        if cs:
            self._spatial_layout.callInStep(
                self._spatial_layout.addConstraints, cs)

    def updateSymbolicSpatialInformation(self, ssi, pose, tag_id):
        """Updates existing symbolic spatial information in the abstract map"""
        assert tag_id >= 0, "can't update SSI without a valid tag_id"
        cs = self._constraintsFromSsiMsg(ssi, pose, tag_id)
        if cs:
            self._spatial_layout.callInStep(
                self._spatial_layout.updateConstraints, cs)


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
    # TODO this function needs to handle multiple pieces of ssi in the 1 string
    figure, relation, references, context = _ssiToComponents(ssi)
    return _componentsToConstraints(figure, relation, references, context)


def _ssiToComponents(ssi):
    """Converts a SSI string to its symbolic components"""
    f = _ComponentRegex.get(_ComponentRegex.FIGURE, ssi)
    r = _ComponentRegex.get(_ComponentRegex.RELATION, ssi)
    rs = _ComponentRegex.get(_ComponentRegex.REFERENCES, ssi)
    c = _ComponentRegex.get(_ComponentRegex.CONTEXT, ssi)

    # Return what we extracted from regex, otherwise assume it is a label
    # TODO make this less hacky...
    if not f:
        return (ssi, '', '', '')
    else:
        return (f, r, rs, c)


def _componentsToConstraints(figure, relation, references, context=""):
    """Converts symbolic components into a list of constraints"""
    mass_fig = sl.Mass(figure)
    mass_refs = [sl.Mass(r) for r in references]
    mass_con = sl.Mass(context)

    cs = []
    if not relation and not references and not context:
        # Is a label
        cs.append(
            sl.ConstraintDistance(mass_fig, None, sl.SAFE_DISTANCE,
                                  sl.STIFF_XL))
    elif relation in ['after', 'beyond', 'past']:
        cs.extend([
            sl.ConstraintAngleLocal(mass_fig, r, mass_con, math.pi, sl.STIFF_L)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, sl.DIST_UNIT, sl.STIFF_S)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(r, mass_con, sl.DIST_UNIT, sl.STIFF_S)
            for r in mass_refs
        ])
    elif relation in ['before', 'towards', 'toward']:
        cs.extend([
            sl.ConstraintAngleLocal(mass_fig, r, mass_con, 0, sl.STIFF_L)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, sl.DIST_UNIT, sl.STIFF_S)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(r, mass_con, sl.DIST_UNIT, sl.STIFF_S)
            for r in mass_refs
        ])
    elif relation in ['beside', 'by', 'near', 'with']:
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, 0.5 * sl.DIST_UNIT, sl.STIFF_L)
            for r in mass_refs
        ])
    elif relation in ['between']:
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, sl.DIST_UNIT, sl.STIFF_L)
            for r in mass_refs
        ])
    elif relation in ['down']:
        cs.append(
            sl.ConstraintAngleGlobal(mass_fig, mass_con, sl.DIR_ZERO,
                                     sl.STIFF_L))
        cs.append(
            sl.ConstraintDistance(mass_fig, mass_con, sl.DIST_UNIT,
                                  sl.STIFF_S))
    elif relation in ['up']:
        cs.append(
            sl.ConstraintAngleGlobal(mass_fig, mass_con, sl.DIR_ZERO - math.pi,
                                     sl.STIFF_L))
        cs.append(
            sl.ConstraintDistance(mass_fig, mass_con, sl.DIST_UNIT,
                                  sl.STIFF_S))
    elif relation in ['left of']:
        cs.extend([
            sl.ConstraintAngleLocal(mass_fig, r, mass_con, -0.5 * math.pi,
                                    sl.STIFF_L) for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, sl.DIST_UNIT, sl.STIFF_S)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_con, r, sl.DIST_UNIT, sl.STIFF_S)
            for r in mass_refs
        ])
    elif relation in ['right of']:
        cs.extend([
            sl.ConstraintAngleLocal(mass_fig, r, mass_con, 0.5 * math.pi,
                                    sl.STIFF_L) for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, sl.DIST_UNIT, sl.STIFF_S)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_con, r, sl.DIST_UNIT, sl.STIFF_S)
            for r in mass_refs
        ])
    return cs
