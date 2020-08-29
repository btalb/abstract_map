import graphviz as gv
import os
import rospkg
import xml.etree.ElementTree as et

PACKAGE_NAME = 'abstract_map'

DEFAULT_HIERARCHY = 'experiments/zoo_hierarchy.xml'
DEFAULT_HIERARCHY_TOPIC = '/hierarchy'
DEFAULT_HIERARCHY_TEMP = '$HOME/tmp/'


def defaultHierarchyFilename():
    """Returns the filename for the default hierarchy"""
    return os.path.join(packagePath(), DEFAULT_HIERARCHY)


def loadHierarchy(fn):
    """Attempts to load a hierarchy from the specified file"""
    # Get all places, with their children from the xml
    root = et.parse(fn).getroot()
    hs = [(c.get('name'), None, [x.get('name')
                                 for x in c])
          for c in root.findall('.//place')]

    # Derive the parent for each hierarchy entry (there does not seem to be an
    # easy working method for getting the parent in the xml)
    hs = [(h[0], next((x[0] for x in hs if h[0] in x[2]),
                      None), h[2]) for h in hs]

    return hs


def packagePath():
    """Returns the root directory of the tag reader package"""
    return rospkg.RosPack().get_path(PACKAGE_NAME)


def viewHierarchy(hierarchy, out_dir=DEFAULT_HIERARCHY_TEMP):
    """Views a hierarchy by using graphviz"""
    hierarchy = [h + (str(i),) for i, h in enumerate(hierarchy)]
    g = gv.Digraph(name='hierarchy', directory=os.path.expandvars(out_dir))
    g.attr('graph',
           layout='twopi',
           ratio='fill',
           size='11.69,8.27!',
           margin='0.1')
    for h in hierarchy:
        g.node(h[-1], h[0].replace(' ', '\n'))
    g.edges([(next(x
                   for x in hierarchy
                   if x[0] == h[1])[-1], h[-1])
             for h in hierarchy
             if h[1] is not None])
    g.view()
