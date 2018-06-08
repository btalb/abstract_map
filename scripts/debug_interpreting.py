import abstract_map.abstract_map as am

try:
    input = raw_input
except NameError:
    pass

# yapf: disable
SSI_TESTS = [
    ("A is towards B", 'A', 'towards', ['B'], ''),
    ("A is left of B, from C", 'A', 'left of', ['B'], 'C'),
    ("A is between B and C, from D", 'A', 'between', ['B', 'C'], 'D'),
    ("From A, B is right of C", 'B', 'right of', ['C'], 'A'),
    ("From A, B is between C, D, E, and F",
        'B', 'between', ['C', 'D', 'E', 'F'], 'A'),
    ("Peter's office is past Gordon's office, from the foyer",
        'Peter\'s office', 'past', ['Gordon\'s office'], 'foyer'),
    ("A is among C, D, E, and F, from G",
        'A', 'among', ['C', 'D', 'E', 'F'], 'G')
]
# yapf: enable

SSI_KEY = [
    'here is in Hall, from here',
    'here is near A, from here',
    'B is down Hall, from here',
    'D is down Hall, from here',
    'F is down Hall, from here',
    'C is right of B, from here',
    'E is right of D, from here',
    'D is before F, from here',
    'D is after B, from here',
    'F is beyond D, from here',
    'G is beyond F, from here',
    'G is towards H, from here',
    'G is left of F, from here',
    'K is right of F, from here',
    'J is right of F, from here',
    'J is between K,and I, from here',
    'H is beside I, from here',
    'I is after J, from here',
    'K is near F, from here',
]


def main():
    print('Testing SSI conversions:')
    passed = True
    for t in SSI_TESTS:
        f, r, rs, c = am._ssiToComponents(t[0])
        p = f == t[1] and r == t[2] and rs == t[3] and c == t[4]
        passed = passed and p
        print("%s\t\"%s\" = ('%s', '%s', [%s], '%s')" %
              ('PASS' if p else 'FAIL', t[0], f, r, ', '.join(map(str, rs)), c))

    print("\nFinal overall results:\t%s" % ('PASS' if passed else 'FAIL'))

    print('\n\nTesting Key World conversions:')
    amap = am.AbstractMap('I', 0, 0, 0)
    for ssi in SSI_KEY:
        amap.addSymbolicSpatialInformation(ssi, (0, 0, 0))

    for c in amap._spatial_layout._constraints:
        print("Constraint: %s" % (c))

    for m in amap._spatial_layout._masses:
        print("Mass: %s" % (m.name))


if __name__ == '__main__':
    main()
