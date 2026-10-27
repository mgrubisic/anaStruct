# anaStruct fork — imposed displacements & thermal loads

This folder documents the two features added on top of upstream
[anaStruct](https://github.com/mgrubisic/anaStruct) in this working copy:

1. **Imposed displacements/rotations of supports** (support settlement) —
   `SystemElements.imposed_displacement()`
2. **Thermal loads on elements** (uniform temperature change + linear gradient
   over the section depth) — `SystemElements.thermal_load()`, also available in
   `LoadCase`/`LoadCombination`

Both follow the existing architecture of the package (loads stored in
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

# settlement of 20 mm at the roller (positive uy = downwards by default)
ss.imposed_displacement(2, uy=0.02)

# uniform temperature increase + gradient over a 0.5 m deep section
ss.thermal_load(1, delta_t=20, alpha=1.2e-5, delta_t_gradient=15, h=0.5)

ss.solve()
print(ss.reaction_forces[2].Fy)
```

## Files in this folder

| File | Language | Contents |
|---|---|---|
| `README.md` (this file) | EN | Feature overview and map of the change set |
| `SKILLS.md` | EN | Usage recipes: API patterns, expected analytical values, examples |
| `MEMORY.md` | HR | Session state: what was done, which decisions were made, what remains open |
| `SPOZNAJE.md` | HR | Technical insights: architecture map, sign conventions, pitfalls, derivation |

## Change set (diff vs. upstream `anaStruct-master (1).zip`)

| File | Change |
|---|---|
| `anastruct/fem/system.py` | Storage (`loads_displacement`, `loads_temperature`, `_prescribed_displacements`), methods `imposed_displacement()` + `thermal_load()`, prescribed-DOF scatter in `solve()`, `validate()` accepts load-free settlement analyses |
| `anastruct/fem/system_components/assembly.py` | `apply_temperature_load()` (registered in `prep_matrix_forces`), prescribed-DOF handling in `process_supports()`, `K_fr @ u_r` correction in `process_conditions()` |
| `anastruct/fem/elements.py` | Element attribute `temperature_load`, summed in `Element.__add__` |
| `anastruct/fem/postprocess.py` | `reaction_forces()` keeps the (possibly imposed) support displacements instead of zeroing them |
| `anastruct/fem/util/load.py` | `LoadCase.thermal_load()`, `LoadCase.imposed_displacement()` |
| `tests/test_imposed_thermal.py` | 30 plain test functions, verified against analytical solutions (incl. load combinations, model edits, sign conventions) |
| `examples/ex_27_imposed_displacement.py` | Settlement + imposed rotation |
| `examples/ex_28_thermal_load.py` | Uniform dT + gradient, restrained and free |
| `examples/ex_29_thermal_loadcombination.py` | Thermal loads in `LoadCase`/`LoadCombination` |

## Running tests

```bash
python -m pytest                          # whole suite (241 tests)
python -m pytest tests/test_imposed_thermal.py -v   # new tests only
```

Note: when running a script from outside the repo root, put the repo on the
path (`PYTHONPATH=.`) — otherwise the *installed* `anastruct` (without these
features) is imported instead of this working copy.

`python -m mypy anastruct` reports only one pre-existing error in
`anastruct/basic.py:41` (present in upstream, not caused by this change set).
