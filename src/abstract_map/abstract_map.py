from __future__ import absolute_import
import numpy as np
import pudb
import math
import re

import abstract_map.spatial_layout as sl


class AbstractMap(object):
    """The abstract map, used to apply abstract ideas about space"""
    TAG_SYNONYMS = ['here']

    def __init__(self, goal, start_x, start_y, start_th, log=False):
        """Constructs a new empty abstract map, with a given symbolic goal"""
        self._goal = goal
        self._start_x = start_x
        self._start_y = start_y
        self._start_th = start_th

        # Initialise a spatial layout with the information provided
        self._spatial_layout = sl.SpatialLayout(log=log)
        # TODO add start mass, and constraint to origin

    def _constraintsFromSsiMsg(self, ssi, pose, ssi_id=None):
        """General function to convert all data in an SSI msg to constraints"""
        # Create a fixed mass in advanced if it could be necessary
        is_label = ssiIsLabel(ssi)
        if pose is None:
            mass_fixed = None
        elif ssi_id is None:
            raise ValueError("Illogical input: Received a pose, but no ssi_id")
        else:
            mass_fixed = sl.MassFixed(
                '#%d' % (ssi_id[0]), np.array(pose[:2]), is_label=is_label)

        # Get the initial list of constraints from the SSI
        cs = ssiToConstraints(ssi, tag_pose=pose, tag_mass=mass_fixed)

        # Process the list, making any required adjustments
        for c in cs:
            # Apply the ssi_id and label status
            c._ssi_id = ssi_id
            if mass_fixed is None:
                c._source = sl.Constraint.SOURCE_NONE
            elif is_label:
                c._source = sl.Constraint.SOURCE_LABEL
            else:
                c._source = sl.Constraint.SOURCE_SIGN

            # Handle special cases where mass must be manually assigned
            if mass_fixed not in c.masses():
                # All we should every catch here is the tag synonyms but we
                # still have all cases here just in case...
                a_is_tag = (c._mass_a is None or not c._mass_a.name or
                            c._mass_a.name in AbstractMap.TAG_SYNONYMS)
                b_is_tag = (c._mass_b is None or not c._mass_b.name or
                            c._mass_b.name in AbstractMap.TAG_SYNONYMS)
                c_is_tag = len(c.masses()) == 3 and (
                    c._mass_c is None or not c._mass_c.name or
                    c._mass_c.name in AbstractMap.TAG_SYNONYMS)

                # If we've found only one mass that needs to be a tag, apply it
                if sum([a_is_tag, b_is_tag, c_is_tag]) > 1:
                    raise ValueError(
                        "Constraint (%s) suggests more than 1 mass is a tag" %
                        (c))
                elif a_is_tag:
                    c._mass_a = mass_fixed
                elif b_is_tag:
                    c._mass_b = mass_fixed
                elif c_is_tag:
                    c._mass_c = mass_fixed

        # Return the final list of constraints
        return cs

    def _hierarchyHintsFromSsiMsg(self, ssi, ssi_id=None):
        """Gets any hierarchy hints as (child, parent) tuples"""
        figs, rel, refs, con = _ssiToComponents(ssi)
        if not figs and ssi_id is not None:
            # A label observation
            return [('#%d' % (ssi_id[0]), f) for f in figs]
        elif rel == 'in':
            return [(f, r) for f in figs for r in refs]
        else:
            return []

    def addSymbolicSpatialInformation(self,
                                      ssi,
                                      pose,
                                      ssi_id=None,
                                      immediate=False):
        """Adds new symbolic spatial information to the abstract map"""
        # Integrate any provided hierarchical information
        hs = self._hierarchyHintsFromSsiMsg(ssi, ssi_id)
        for h in hs:
            self._spatial_layout.addHierarchy(h)

        # Get all of the constraints from the SSI
        cs = self._constraintsFromSsiMsg(ssi, pose, ssi_id)

        # Make the assumption that if we had hierarchy hints, all constraints
        # are hierarchical
        # TODO there has to be a less black magic way to implement this...
        if hs:
            for c in cs:
                if c._source == sl.Constraint.SOURCE_NONE:
                    c._source = sl.Constraint.SOURCE_HIERARCHICAL

        # Add the constraints to the spatial layout
        if cs:
            if immediate:
                self._spatial_layout.addConstraints(cs)
            else:
                self._spatial_layout.callInStep(
                    self._spatial_layout.addConstraints, cs)

            for c in cs:
                print("\tAdded: %s" % (c))

    def getToponymLocation(self, toponym):
        m = self._spatial_layout.getMass(toponym)
        return (None if m is None else m.pos)

    def updateSymbolicSpatialInformation(self, ssi, pose, ssi_id):
        """Updates existing symbolic spatial information in the abstract map"""
        assert ssi_id is not None, "can't update SSI without a valid ssi_id"
        cs = self._constraintsFromSsiMsg(ssi, pose, ssi_id)
        if cs:
            self._spatial_layout.callInStep(
                self._spatial_layout.updateConstraints, cs)


class _ComponentRegex(object):
    """Helper class tofacilitate single compile regex"""
    ARROW = re.compile(r'^\s*\$(\w+)\$')
    ARROW_FIGS = re.compile(r'^\s*\$\w+\$\s*(.*)\s*$')

    FIGURES = re.compile(r'(?:^|, )([^,]*?)\s(?:is|are)')
    RELATION = re.compile(r'(?:is|are)\s((?:\w+)(?:\sof)?)')
    REFERENCES = re.compile(r'(?:is|are)\s\w+(?:\sof)?\s(.*?)(?:, from .*)?$')
    CONTEXT = re.compile(r'[Ff]rom ([^,]*).*$')

    SPLIT = re.compile(r', | and ')
    STRIP = re.compile(r'\b(?:and|[Tt]he|a)\b')

    @staticmethod
    def get(component, string):
        """Runs regex component against string, extracting match"""
        g = re.search(component, string)
        if g is None:
            return ''
        elif (component == _ComponentRegex.REFERENCES or
              component == _ComponentRegex.FIGURES or
              component == _ComponentRegex.ARROW_FIGS):
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


def ssiIsLabel(ssi):
    figures, relation, references, context = _ssiToComponents(ssi)
    return not relation and not references and not context


def ssiToConstraints(ssi, tag_pose=None, tag_mass=None):
    """Converts a SSI string to a list of constraints"""
    # We should always have either both a tag pose & mass, or neither
    if sum([tag_pose is None, tag_mass is None]) == 1:
        raise ValueError(
            "Illogical input:"
            " A pose & mass must be supplied for a tag, or neither")

    # Get the components, & use that to return a flat list of constraints
    figures, relation, references, context = _ssiToComponents(ssi)
    return [
        c for f in figures for c in _componentsToConstraints(
            f,
            relation,
            references,
            context,
            tag_pose=tag_pose,
            tag_mass=tag_mass)
    ]


def _ssiToComponents(ssi):
    """Converts a SSI string to its symbolic components"""
    arrow = _ComponentRegex.get(_ComponentRegex.ARROW, ssi)
    if arrow:
        fs = _ComponentRegex.get(_ComponentRegex.ARROW_FIGS, ssi)
        r = arrow.upper()
        rs = []
        c = ''
    else:
        fs = _ComponentRegex.get(_ComponentRegex.FIGURES, ssi)
        r = _ComponentRegex.get(_ComponentRegex.RELATION, ssi)
        rs = _ComponentRegex.get(_ComponentRegex.REFERENCES, ssi)
        c = _ComponentRegex.get(_ComponentRegex.CONTEXT, ssi)

    # Return what we extracted from regex, otherwise assume it is a label
    # TODO make this less hacky...
    if not fs:
        return (_ComponentRegex.stringToList(ssi), '', [], '')
    else:
        return (fs, r, rs, c)


def _componentsToConstraints(figure,
                             relation,
                             references,
                             context="",
                             tag_pose=None,
                             tag_mass=None):
    """Converts symbolic components into a list of constraints"""
    mass_fig = sl.Mass(figure)
    mass_refs = [sl.Mass(r) for r in references]
    if not context and tag_mass is not None:
        mass_con = tag_mass
    else:
        mass_con = sl.Mass(context)

    cs = []
    if not relation and not references and not context:
        # Is a label
        cs.append(sl.ConstraintDistance(mass_fig, tag_mass, 1, sl.STIFF_XL))
        cs.append(
            sl.ConstraintAngleGlobal(mass_fig, tag_mass,
                                     sl._angleWrap(tag_pose[2] + np.pi),
                                     sl.STIFF_XL))
    elif relation in ['LEFT']:
        # Left arrow on a sign
        cs.append(sl.ConstraintDistance(mass_fig, tag_mass, 1, sl.STIFF_S))
        cs.append(
            sl.ConstraintAngleGlobal(mass_fig, tag_mass,
                                     sl._angleWrap(tag_pose[2] - 0.5 * np.pi),
                                     sl.STIFF_M))
    elif relation in ['RIGHT']:
        # Right arrow on a sign
        cs.append(sl.ConstraintDistance(mass_fig, tag_mass, 1, sl.STIFF_S))
        cs.append(
            sl.ConstraintAngleGlobal(mass_fig, tag_mass,
                                     sl._angleWrap(tag_pose[2] + 0.5 * np.pi),
                                     sl.STIFF_M))
    elif relation in ['UP']:
        # Up arrow on a sign
        cs.append(sl.ConstraintDistance(mass_fig, tag_mass, 1, sl.STIFF_S))
        cs.append(
            sl.ConstraintAngleGlobal(mass_fig, tag_mass,
                                     sl._angleWrap(tag_pose[2] + np.pi),
                                     sl.STIFF_M))
    elif relation in ['DOWN']:
        # Down arrow on a sign
        cs.append(sl.ConstraintDistance(mass_fig, tag_mass, 1, sl.STIFF_S))
        cs.append(
            sl.ConstraintAngleGlobal(mass_fig, tag_mass,
                                     sl._angleWrap(tag_pose[2]), sl.STIFF_M))
    elif relation in ['after', 'beyond', 'past']:
        cs.extend([
            sl.ConstraintAngleLocal(mass_fig, r, mass_con, math.pi, sl.STIFF_L)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, 1, sl.STIFF_S)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(r, mass_con, 1, sl.STIFF_S)
            for r in mass_refs
        ])
    elif relation in ['before', 'towards', 'toward']:
        cs.extend([
            sl.ConstraintAngleLocal(r, mass_fig, mass_con, math.pi, sl.STIFF_L)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, 1, sl.STIFF_S)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(r, mass_con, 1, sl.STIFF_S)
            for r in mass_refs
        ])
    elif relation in ['beside', 'by', 'near', 'with']:
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, 0.5, sl.STIFF_L)
            for r in mass_refs
        ])
    elif relation in ['between']:
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, 1, sl.STIFF_L)
            for r in mass_refs
        ])
    elif relation in ['down']:
        cs.append(
            sl.ConstraintAngleGlobal(mass_fig, mass_con, sl.DIR_ZERO,
                                     sl.STIFF_L))
        cs.append(sl.ConstraintDistance(mass_fig, mass_con, 1, sl.STIFF_S))
    elif relation in ['up']:
        cs.append(
            sl.ConstraintAngleGlobal(mass_fig, mass_con, sl.DIR_ZERO - math.pi,
                                     sl.STIFF_L))
        cs.append(sl.ConstraintDistance(mass_fig, mass_con, 1, sl.STIFF_S))
    elif relation in ['left of']:
        cs.extend([
            sl.ConstraintAngleLocal(mass_fig, r, mass_con, -0.5 * math.pi,
                                    sl.STIFF_L) for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, 1, sl.STIFF_S)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_con, r, 1, sl.STIFF_S)
            for r in mass_refs
        ])
    elif relation in ['right of']:
        cs.extend([
            sl.ConstraintAngleLocal(mass_fig, r, mass_con, 0.5 * math.pi,
                                    sl.STIFF_L) for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, 1, sl.STIFF_S)
            for r in mass_refs
        ])
        cs.extend([
            sl.ConstraintDistance(mass_con, r, 1, sl.STIFF_S)
            for r in mass_refs
        ])
    elif relation in ['in']:
        cs.extend([
            sl.ConstraintDistance(mass_fig, r, 1, sl.STIFF_XS)
            for r in mass_refs
        ])
    return cs
