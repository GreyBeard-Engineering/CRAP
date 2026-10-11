"""Simple-mode solver checks.

Run:  python test_solver.py [--full]

Every case gets the fast clearance check over a full input turn; --full also runs the slow
exact (OCC) check in the assembled pose on every solved case.
"""
import itertools
import sys
import time

from part_generation_fcns.gearbox import EXAMPLE_KWARGS
from part_generation_fcns.solver import SolveError, solve, spec_from_kwargs


def check_example():
    t = time.time()
    res = solve(spec_from_kwargs(EXAMPLE_KWARGS))
    assert res.collisions == [], res.collisions
    print(f"example: solved in {res.iterations} iteration(s), no collisions "
          f"(collision check {res.check_seconds:.1f} s, total {time.time() - t:.1f} s)")


def sweep(full: bool):
    base = spec_from_kwargs(EXAMPLE_KWARGS)
    ok = failed = 0
    for roller_R, e, ratio in itertools.product((30, 40, 60), (0.8, 1.2, 1.6), (11, 23, 35)):
        spec = spec_from_kwargs(dict(EXAMPLE_KWARGS, roller_R=roller_R, e=e, ratio=ratio))
        tag = f"R={roller_R:<3} e={e:<4} ratio={ratio:<3}"
        try:
            res = solve(spec, max_iter=12)
            ok += 1
            extra = f"{res.iterations} it" + (f", {len(res.log)} fix(es)" if res.log else "")
            extra += f", check {res.check_seconds:.2f} s"
            if full:
                exact = res.gearbox.collisions(mode="exact")
                assert exact == [], f"{tag}: the fast check passed but the exact check found {exact}"
            print(f"  {tag} solved ({extra})")
        except SolveError as ex:
            failed += 1
            print(f"  {tag} rejected: {ex}")
    print(f"sweep: {ok} solved, {failed} rejected with a reason, 0 crashes")
    return ok


if __name__ == "__main__":
    check_example()
    assert sweep("--full" in sys.argv) > 0
    print("all checks passed")
