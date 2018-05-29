from abstract_map import abstract_map as am

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


if __name__ == '__main__':
    main()
