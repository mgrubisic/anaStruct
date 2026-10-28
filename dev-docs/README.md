# anaStruct fork — notes for developers

This folder documents the changes of this fork
([mgrubisic/anaStruct](https://github.com/mgrubisic/anaStruct)) on top of upstream
[anaStruct](https://github.com/ritchie46/anaStruct). The diff against upstream is
`git diff f85d14a` (the last upstream commit before the fork's first commit).

Main additions:

1. **Imposed displacements/rotations of supports** (support settlement) —
   `SystemElements.imposed_displacement()`, also `LoadCase.imposed_displacement()`.
2. **Thermal loads on elements** (uniform temperature change + linear gradient over the
   section depth) — `SystemElements.thermal_load()`, also available in
   `LoadCase`/`LoadCombination`.

and corrections of existing behaviour (each described in `SPOZNAJE.md`):

3. Exact stiffness of element end hinges and rotational springs (static condensation, springs
   in series) for all loads, with the matching fixed-end moments of distributed loads.
4. Detection of internal hinges that does not depend on the order of the elements; `discretize()`
   that works for any model.
5. Buckling factor and second order analysis (eigenvalue problem, geometric stiffness matrix,
   inclined supports, hinges and springs, plastic hinges, `LoadCombination`).
6. Loads on nodes with an inclined roller support; changes of the model after a solve;
   `LoadCombination` as one calculation; explanatory errors for imposed displacements in a
   direction that no support restrains.

Both main features follow the existing architecture of the package (loads stored in
`loads_*` dicts on the system, processed to equivalent nodal loads in
`anastruct/fem/system_components/assembly.py`, post-processed through the
standard pipeline).

## Quick start

```python
from anastruct import SystemElements

ss = SystemElements(EA=1e6, EI=1e4)
ss.add_element([[0, 0], [10, 0]])
ss.add_support_fixed(1)
ss.add_support_roll(2)

# settlement of 20 mm at the roller (uy is positive upwards like Fy: settlement < 0)
ss.imposed_displacement(2, uy=-0.02)

# uniform temperature increase + gradient over a 0.5 m deep section
ss.thermal_load(1, delta_t=20, alpha=1.2e-5, delta_t_gradient=15, h=0.5)

ss.solve()
print(ss.reaction_forces[2].Fy)
```

## Files in this folder

| File | Language | Contents |
|---|---|---|
| `README.md` (this file) | EN | Overview and map of the change set |
| `SKILLS.md` | EN | Usage recipes: API patterns, expected analytical values, examples |
| `MEMORY.md` | HR | Session state: what was done, which decisions were made, what remains open |
| `SPOZNAJE.md` | HR | Technical insights: architecture map, sign conventions, pitfalls, derivations, reviews |

## Change set (since upstream `f85d14a`)

| File | Change |
|---|---|
| `anastruct/fem/system.py` | Imposed displacements and thermal loads; `discretize()`, `insert_node()`, `remove_loads()` rewritten/extended; `_invalidate_solution()`, `_reset_plastic_state()`; plastic + second order in `solve()`; internal-hinge bookkeeping |
| `anastruct/fem/elements.py` | `constitutive_matrix()` (condensation), `end_rotational_stiffness`, geometric stiffness (`geometric_stiffness_matrix`, releases, truss), `refresh_nodes()` |
| `anastruct/fem/system_components/assembly.py` | `apply_temperature_load()`, prescribed displacements, inclined-roller loads, hinge/truss moment checks, restrained-DOF mask |
| `anastruct/fem/system_components/solver.py` | `smallest_buckling_factor()`, `det_linear_buckling()`, `second_order_step()`, plastic iteration |
| `anastruct/fem/system_components/util.py` | `check_internal_hinges()`, `describe_unrestrained_dof()` |
| `anastruct/fem/postprocess.py` | Inclined-support conversion (proper rotation), node data kept across solves; `reaction_forces[i].ux/uy/phi_z` are the real displacements |
| `anastruct/fem/util/load.py` | `LoadCase.thermal_load()/imposed_displacement()`; `LoadCombination` as one calculation (`_add_loads`) |
| `anastruct/fem/plotter/mpl.py` | Hinge symbol at every released element end |
| `tests/test_imposed_thermal.py` | 34 tests: imposed displacements and thermal loads against analytical solutions, signs, combinations, model edits |
| `tests/test_hinges_springs.py` | 17 tests: hinge/spring stiffness against analytical results, incl. thermal and distributed loads |
| `tests/test_discretize_internal_hinges.py` | 29 tests: internal-hinge detection, `discretize()` against the original model |
| `tests/test_buckling.py` | 22 tests: buckling factor against Euler columns, frames, springs, hinges, repeated solves |
| `tests/test_limitations.py` | 72 tests: inclined rollers, bar buckling, plastic hinges in second order, combinations, model edits after a solve, review findings |
| `tests/test_upstream_examples.py` | 6 tests: upstream examples whose nested tests are not run by pytest-describe, with corrected expectations |
| `examples/ex_27..29` | Settlement + rotation; thermal loads; thermal loads in a load combination |

## Running tests

```bash
MPLBACKEND=Agg python -m pytest           # whole suite (391 tests)
python -m pytest tests/test_imposed_thermal.py -v   # one file
```

Note: when running a script from outside the repo root, put the repo on the
path (`PYTHONPATH=.`) — otherwise the *installed* `anastruct` (without these
features) is imported instead of this working copy. Several upstream examples
(`examples/ex_4.py`, ...) show plots when imported: set `MPLBACKEND=Agg`.

`python -m mypy anastruct` reports only one pre-existing error in
`anastruct/basic.py:41` (present in upstream, not caused by this change set).
