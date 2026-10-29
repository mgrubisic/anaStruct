# anaStruct fork — developer guide

This folder documents the changes of this fork
([mgrubisic/anaStruct](https://github.com/mgrubisic/anaStruct)) on top of upstream
[anaStruct](https://github.com/ritchie46/anaStruct), and everything needed to continue its
development. The diff against upstream is `git diff f85d14a` (the last upstream commit before
the fork's first commit).

## Start here

| If you want to... | Read |
|---|---|
| continue the work (state, decisions, open items) | [`MEMORY.md`](MEMORY.md) (HR) |
| use the new features / copy a working recipe | [`SKILLS.md`](SKILLS.md) (EN) |
| understand how the solver works, sign conventions, pitfalls, why something was changed | [`SPOZNAJE.md`](SPOZNAJE.md) (HR) |
| see which file contains which change | the change set below |
| find user documentation | `docs/source/*.rst` (Sphinx, Read the Docs) |

## What the fork adds

New features:

1. **Imposed displacements/rotations of supports** (support settlement) —
   `SystemElements.imposed_displacement()`, also `LoadCase.imposed_displacement()`.
2. **Thermal loads on elements** (uniform temperature change + linear gradient over the
   section depth) — `SystemElements.thermal_load()`, also available in
   `LoadCase`/`LoadCombination`.

Corrections of existing (upstream) behaviour, each described in `SPOZNAJE.md`:

3. Exact stiffness of element end hinges and rotational springs (static condensation, springs
   in series) for all loads, with the matching fixed-end moments of distributed loads.
4. Detection of internal hinges that does not depend on the order of the elements; `discretize()`
   that works for any model.
5. Buckling factor and second order analysis (eigenvalue problem, geometric stiffness matrix,
   inclined supports, hinges and springs, plastic hinges, `LoadCombination`).
6. Loads on nodes with an inclined roller support; changes of the model after a solve;
   `LoadCombination` as one calculation; explanatory errors for imposed displacements in a
   direction that no support restrains.
7. Inherited upstream defects: plastic hinges as a rotational spring in series (exact
   redistribution), node ids with gaps after `remove_element`, `LoadCase.dead_load`,
   `validate()` before the first `solve()`, `float64` coordinates, zero-length elements and
   infinite springs, consistent loads with `invert_y_loads=False`.

All of it follows the existing architecture of the package (loads stored in `loads_*` dicts on
the system, processed to equivalent nodal loads in
`anastruct/fem/system_components/assembly.py`, post-processed through the standard pipeline).

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
| `README.md` (this file) | EN | Entry point: map of the change set, development workflow, architecture, checklists |
| `SKILLS.md` | EN | Usage recipes: API patterns, expected analytical values, errors, examples |
| `MEMORY.md` | HR | Session state: what was done, which decisions were made, what remains open |
| `SPOZNAJE.md` | HR | Technical insights: architecture map, sign conventions, pitfalls, derivations, reviews |

## Change set (since upstream `f85d14a`)

| File | Change |
|---|---|
| `anastruct/fem/system.py` | Imposed displacements, thermal loads and `dead_load()`; `discretize()`, `insert_node()`, `remove_loads()` rewritten/extended; `validate()` before the first solve; `_invalidate_solution()`, `_reset_plastic_state()`; plastic + second order in `solve()`; internal-hinge bookkeeping; `invert_y_loads=False` load factors |
| `anastruct/fem/elements.py` | `constitutive_matrix()` (condensation), `end_rotational_stiffness`, geometric stiffness (`geometric_stiffness_matrix`, `_stiffness_with_releases`, truss), `refresh_nodes()`, plastic hinge as a series spring (`plastic_stiffness`, `update_stiffness`, `reset_plastic`, `PLASTIC_*` constants) |
| `anastruct/fem/system_components/assembly.py` | `apply_temperature_load()`, prescribed displacements, inclined-roller loads, hinge/truss moment checks, restrained-DOF mask, `dof_index` everywhere |
| `anastruct/fem/system_components/solver.py` | `smallest_buckling_factor()`, `det_linear_buckling()`, `second_order_step()`, plastic iteration (secant in the flexibility, mechanism detection) |
| `anastruct/fem/system_components/util.py` | `dof_index()`, `check_internal_hinges()`, `describe_unrestrained_dof()`, `det_node_ids()` (max id + 1), `remove_node_id()` (supports compared by id) |
| `anastruct/fem/postprocess.py` | Inclined-support conversion (proper rotation), node data kept across solves; `reaction_forces[i].ux/uy/phi_z` are the real displacements |
| `anastruct/fem/util/load.py` | `LoadCase.thermal_load()/imposed_displacement()`; `LoadCombination` as one calculation (`_add_loads`) |
| `anastruct/fem/plotter/mpl.py` | Hinge symbol at every released element end |
| `anastruct/vertex.py` | `float64` coordinates rounded to 12 significant digits (`_snap`) |
| `tests/test_imposed_thermal.py` | 34 tests: imposed displacements and thermal loads against analytical solutions, signs, combinations, model edits |
| `tests/test_hinges_springs.py` | 17 tests: hinge/spring stiffness against analytical results, incl. thermal and distributed loads |
| `tests/test_discretize_internal_hinges.py` | 29 tests: internal-hinge detection, `discretize()` against the original model |
| `tests/test_buckling.py` | 22 tests: buckling factor against Euler columns, frames, springs, hinges, repeated solves |
| `tests/test_limitations.py` | 72 tests: inclined rollers, bar buckling, plastic hinges in second order, combinations, model edits after a solve, review findings |
| `tests/test_inherited_fixes.py` | 39 tests: plastic hinge as a series spring, mechanism detection, node-id gaps, `LoadCase.dead_load`, `validate()` before `solve()`, zero-length element, infinite spring, float64 vertices, `invert_y_loads=False` |
| `tests/test_upstream_examples.py` | 6 tests: upstream examples whose nested tests are not run by pytest-describe, with corrected expectations |
| `examples/ex_27..29` | Settlement + rotation; thermal loads; thermal loads in a load combination |

Upstream test files (`test_analytical.py`, `test_beam.py`, `test_e2e.py`, `test_truss.py`, ...) are
unchanged except where an expectation was wrong (see `SPOZNAJE.md`). Whole suite: **430 tests**.

## Development workflow

Environment: Python >= 3.10 (CI: 3.10–3.13), Poetry (`pyproject.toml`). The package is imported
directly from the repository; no installation is needed.

```bash
MPLBACKEND=Agg python -m pytest                  # whole suite (430 tests), coverage on by default
python -m pytest tests/test_imposed_thermal.py -v   # one file
python -m pytest --no-cov -q                     # fast, no coverage report
black --check .                                  # formatting (CI)
python -m pylint anastruct                       # linting, must stay 10.00/10 (CI)
python -m mypy -p anastruct                      # types (CI)
```

- CI (`.github/workflows/test.yaml`) runs on **pull requests only**: pylint, black, mypy and pytest
  on Python 3.12, pytest alone on 3.10, 3.11 and 3.13. Pushing straight to `master` skips it, so
  run all four commands locally before pushing.
- CI pins `pylint`, `mypy`, `black`, `pytest` and `pytest-describe` to the versions in
  `pyproject.toml`; a local environment may have other versions (e.g. a different `black`), so a
  formatting difference that appears only in CI is a version difference.
- `python -m mypy anastruct` reports only one pre-existing error in `anastruct/basic.py:41`
  (present in upstream, not caused by this change set).
- Run scripts from the repo root, or with `PYTHONPATH=.` — otherwise the *installed* `anastruct`
  (without these features) is imported instead of this working copy. Several upstream examples
  (`examples/ex_4.py`, ...) show plots when imported: set `MPLBACKEND=Agg`.
- **Tests must be plain `test_*` functions.** With the installed `pytest-describe`, nested
  `describe_/context_/it_` blocks are not executed (their assertions never run). Upstream tests
  use that pattern, so their assertions are not executed either (see `SPOZNAJE.md`).
- `commit_push.bat` in the repo root is a personal helper for this fork (asks for confirmation
  before it commits and pushes); it is not for upstream.
- The `readthedocs.yml` workflow triggers a Read the Docs build on pushes that touch docs; it skips
  itself when the webhook secrets are not configured.

## Architecture in one page

```
SystemElements (anastruct/fem/system.py)         public API: add_element, add_support_*, q_load,
   │                                             point_load, dead_load, thermal_load,
   │                                             imposed_displacement, solve, validate, discretize, ...
   ├── system_components/assembly.py             loads -> equivalent nodal loads, K assembly, supports
   ├── system_components/solver.py               plastic iteration, buckling, second order
   ├── system_components/util.py                 node ids, dof_index, hinge bookkeeping, helpers
   ├── elements.py (Element)                     stiffness: kinematic matrix B, constitutive matrix D,
   │                                             K = BᵀDB; geometric stiffness; plastic state
   ├── postprocess.py                            element forces -> node results -> reactions
   ├── util/load.py                              LoadCase, LoadCombination
   └── plotter/                                  matplotlib output
```

`SPOZNAJE.md`, section 1 has the data-flow pipeline of `solve()` step by step.

State that is cached on the system and must stay consistent when the model changes (all reset
by `_invalidate_solution()` unless noted):

| State | Meaning | Reset |
|---|---|---|
| `system_displacement_vector` | solution of the last solve; `None` = supports not processed | `_invalidate_solution()` |
| `_restrained_dofs`, `_prescribed_displacements` | mask of restrained dofs, imposed values | `_invalidate_solution()` |
| `buckling_factor` | factor of the last second-order solve | `_invalidate_solution()` |
| plastic state of the elements (`nodes_plastic`, `plastic_stiffness`) | plastic hinges of the last iteration | `_reset_plastic_state()` |
| `_node_index` | node id -> position (dense dof index, ids may have gaps) | set to `None` whenever `node_map` changes |
| `_internal_hinge_index`, `_explicit_internal_hinges` | internal-hinge bookkeeping | `check_internal_hinges()` / `remove_node_id()` |

Rules that avoid the bugs found so far:

- A dof position is **always** `dof_index(system, node_id)`, never `(node_id - 1) * 3`.
- A new node id is `max(existing ids) + 1`, never `len(...) + 1`.
- After a `solve()` the objects in `node_map` are replaced by copies with results: compare nodes
  **by id**, never by identity.
- A change to the model (`add_element`, `remove_element`, `add_support_*`, `add_internal_hinge`)
  must call `_invalidate_solution()`; a change to node ids must reset `_node_index`.
- Every new load type: store it in a `loads_*` dict, process it in `assembly.py`, copy it in
  `discretize()` / `insert_node()`, clear it in `remove_loads()`, add it to `LoadCase`
  (`_add_loads` in `util/load.py`), and test it in a `LoadCombination` (a load counted twice is
  the typical bug).
- Sign conventions are in `SPOZNAJE.md` section 4 — check a new feature against an independent
  analytical value, not only against another anaStruct result.

## Checklist before a commit

1. `black --check .`, `python -m pylint anastruct` (10.00/10), `python -m mypy -p anastruct`
   (only the known `basic.py:41` error).
2. `MPLBACKEND=Agg python -m pytest` — all green; new behaviour has a plain `test_*` test with
   an analytical expectation.
3. Examples still run: `MPLBACKEND=Agg PYTHONPATH=. python examples/ex_27_imposed_displacement.py`
   (and `ex_28`, `ex_29`, `ex_4`).
4. Docs: Sphinx page in `docs/source/`, the files in this folder (counts of tests, state in
   `MEMORY.md`, new insights in `SPOZNAJE.md`), and the behaviour-change list in the root
   `README.md` if upstream behaviour changed.
5. `git fetch` first; merge (no force-push) if `origin/master` moved.
