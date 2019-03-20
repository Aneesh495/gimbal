#!/usr/bin/env python3
import os, sys

def verify_clean():
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    em_dash_code = 0x2014
    en_dash_code = 0x2013
    bad = []
    for r, ds, fs in os.walk(base):
        if any(x in r for x in ['.git', 'build', '__pycache__']):
            continue
        for fn in fs:
            p = os.path.join(r, fn)
            try:
                with open(p, 'r', encoding='utf-8', errors='ignore') as f:
                    for i, l in enumerate(f, 1):
                        for char in l:
                            code = ord(char)
                            if code == em_dash_code or code == en_dash_code:
                                bad.append((p, i, 'Forbidden dash code ' + hex(code)))
                                break
            except Exception:
                pass
    if bad:
        print('Violations: ' + str(len(bad)))
        for p, i, s in bad:
            print(p + ':' + str(i) + ' -> ' + s)
        sys.exit(1)
    print('Clean: No em dashes or en dashes detected.')

if __name__ == '__main__':
    verify_clean()
