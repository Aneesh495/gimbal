#!/usr/bin/env python3
import os, sys

def verify_clean():
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    banned = {'—', '–'}
    bad = []
    for r, ds, fs in os.walk(base):
        if any(x in r for x in ['.git', 'build', '__pycache__']):
            continue
        for fn in fs:
            p = os.path.join(r, fn)
            try:
                with open(p, 'r', encoding='utf-8', errors='ignore') as f:
                    for i, l in enumerate(f, 1):
                        if any(c in l for c in banned):
                            bad.append((p, i, l.strip()))
            except Exception:
                pass
    if bad:
        print(f'Violations: {len(bad)}')
        for p, i, s in bad:
            print(f'{p}:{i} -> {s}')
        sys.exit(1)
    print('Clean: No em dashes')

if __name__ == '__main__':
    verify_clean()
