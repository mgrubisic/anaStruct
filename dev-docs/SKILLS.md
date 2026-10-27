# SKILLS — how to use imposed displacements and thermal loads

Practical recipes for the two features this fork adds to anaStruct. All
sign conventions below assume default settings (`invert_y_loads=True`).

## 1. Imposed displacement / rotation of a support

```python
ss.imposed_displacement(node_id, ux=None, uy=None, phi_z=None)
```

- The component must be **restrained by a support** at that node, otherwise
  `solve()` raises `FEMException("StabilityError", ...)`.
- Sign conventions follow the corresponding loads:
  - `ux` — like a positive `Fx` point load (positive = +x),
  - `uy` — like a positive `Fy` point load (positive = **downwards** by default),
  - `phi_z` — like a positive `Tz` moment load (radians).
- Not multiplied by `load_factor` (it is a kinematic condition, not a load).
- Repeated calls for the same node merge: only the passed components are
  overridden; pass `0` to clear a previously imposed component.
- Works with `add_support_fixed`, `add_support_hinged`, `add_support_roll`,
  `add_support_rotational`. Spring DOFs are not directly prescribable.
- Impose **before the first `solve()`** (same rule as for supports).

### Reference values (verified in tests)

| Case | Formula |
|---|---|
| Propped cantilever, settlement Δ of the prop | R_prop = −3EIΔ/L³ (pulls beam down), M_fixed = −3EIΔ/L² |
| Fixed-fixed beam, imposed rotation φ at one end | M_near = 4EIφ/L, M_far = 2EIφ/L, R = 6EIφ/L² |
| Fixed-fixed bar, imposed axial displacement dx | N = EA·dx/L (tension) |

Superposition with other loads holds (linear system), e.g. propped cantilever
with q-load and settlement: R_prop = 3qL/8 − 3EIΔ/L³.

See `examples/ex_27_imposed_displacement.py`.

## 2. Thermal load on an element

```python
ss.thermal_load(element_id, delta_t, alpha, delta_t_gradient=None, h=None)
```

- `delta_t` — uniform temperature change (positive = heating) → free strain
  `alpha * delta_t` → restrained axial force `EA * alpha * delta_t`.
- `delta_t_gradient` — temperature difference between the two faces of the
  section; **requires** `h` (section depth). Free curvature
  `kappa = alpha * delta_t_gradient / h` → restrained moment `EI * kappa`.
- A **positive gradient = warmer upper face** (the face pointing to +y for a
  horizontal element): a cantilever fixed at its first node deflects downwards
  (positive `uy`).
- Sequences are accepted (like `q_load`), e.g. `element_id=[1, 2]` with scalar
  or per-element `delta_t`/`alpha`.
- Multiplied by `load_factor`, so `LoadCase`/`LoadCombination` factors apply.
- Works on `general` and `truss` elements (truss: axial part only). Element-end
  hinges/springs are accounted for automatically (the constitutive matrix of
  the element is used to build the fixed-end forces).

### Reference values (verified in tests)

| Case | Result |
|---|---|
| Fixed-fixed beam, uniform dT | N = −EA·α·dT (compression), M = 0 |
| Cantilever, uniform dT | tip ux = α·dT·L, zero reactions |
| Fixed-fixed beam, gradient | constant M = EI·α·ΔT_grad/h, V = 0 |
| Cantilever, gradient | tip uy = κL²/2, zero reactions |
| Restrained truss bar, uniform dT | N = −EA·α·dT |

See `examples/ex_28_thermal_load.py`.

## 3. Thermal loads in load cases and combinations

```python
from anastruct import LoadCase, LoadCombination

lc_t = LoadCase("thermal")
lc_t.thermal_load(element_id=[1, 2], delta_t=15, alpha=1.2e-5)

lc_g = LoadCase("gradient")
lc_g.thermal_load(element_id=[1, 2], delta_t=0, alpha=1.2e-5,
                  delta_t_gradient=10, h=0.5)

comb = LoadCombination("SLS")
comb.add_load_case(lc_t, factor=1.0)
comb.add_load_case(lc_g, factor=1.0)
results = comb.solve(ss)   # dict per load case + "combination"
```

Imposed displacements are deliberately **not** part of `LoadCase` — they are a
support condition of the system, not a load. They are carried into every load
case of a `LoadCombination` unchanged (never factored).

See `examples/ex_29_thermal_loadcombination.py`.

## 4. Known limitations (by design decision)

- `remove_loads()` does **not** clear thermal loads; `discretize()` and
  `insert_node()` do **not** transfer thermal loads or imposed displacements
  to the new mesh.
- No plotting support for either feature yet (plots of loads don't show them;
  result plots like `show_bending_moment()` are correct).
- Element result curves `deflection`/`extension` (relative to the element
  chord) exclude the free thermal curvature/strain. Nodal displacements,
  reactions and M/N/V diagrams are exact.
- No Sphinx docs entries yet (`docs/source/loads.rst`, `supports.rst`).
