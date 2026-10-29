# SKILLS — usage recipes of the fork

Practical recipes for the features this fork adds to anaStruct (sections 1–3: imposed
displacements, thermal loads, load cases) and for the corrected behaviour of existing features
(sections 4–8: hinges, plastic hinges, dead load, `validate()`, `invert_y_loads`, model edits). All
sign conventions below assume default settings (`invert_y_loads=True`). Every number quoted in
a table or a comment is verified in `tests/`. For the reasons behind the behaviour see
`SPOZNAJE.md`; the map of the code is in `README.md`.

Contents: 1 imposed displacement · 2 thermal load · 3 load cases and combinations ·
4 hinges and springs · 5 plastic hinges (`mp=`) · 6 dead load · 7 `validate()` and errors ·
8 `invert_y_loads=False` and model edits · 9 known limitations.

## 1. Imposed displacement / rotation of a support

```python
ss.imposed_displacement(node_id, ux=None, uy=None, phi_z=None)
```

- The component must be **restrained by a support** at that node, otherwise
  `solve()` raises `FEMException("StabilityError", ...)`.
- Sign conventions follow the corresponding loads:
  - `ux` — like a positive `Fx` point load (positive = +x),
  - `uy` — like a positive `Fy` point load (positive = **+y, upwards** with the default
    `invert_y_loads=True`, so a settlement is a negative `uy`),
  - `phi_z` — like a positive `Tz` moment load (radians); the getters report rotations
    positive clockwise, so an imposed `phi_z` is reported with the opposite sign.
- Multiplied by `load_factor`, like the loads, so it superposes in a `LoadCombination`
  (use `LoadCase.imposed_displacement`).
- Repeated calls for the same node merge: only the passed components are
  overridden; pass `0` to clear a previously imposed component.
- Works with `add_support_fixed`, `add_support_hinged`, `add_support_roll`,
  `add_support_rotational`. Spring DOFs are not directly prescribable.
- Impose **before the first `solve()`** (same rule as for supports).

### Reference values (verified in tests)

| Case | Formula |
|---|---|
| Propped cantilever, prop moved by Δ (`uy = +Δ`; a settlement is `uy = −Δ`) | R_prop = −3EIΔ/L³, M_fixed = −3EIΔ/L² (opposite signs for a settlement) |
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
  horizontal element): the upper fibres are longer, so a cantilever fixed at its first
  node deflects downwards (`uy < 0` in `get_node_displacements`, which is positive
  towards +y) and a simply supported beam cambers upwards.
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
| Fixed-fixed beam, gradient | constant sagging moment EI·α·ΔT_grad/h (negative in anaStruct), V = 0 |
| Cantilever, gradient | tip uy = −κL²/2 (down), zero reactions |
| Simply supported beam, gradient | mid-span uy = +κL²/8 (camber), zero reactions |
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

Imposed displacements are available in a `LoadCase` as well:
`lc.imposed_displacement(node_id, ux=None, uy=None, phi_z=None)`. They are
multiplied by the load factor of their load case, so a settlement is counted
once and superposes correctly. A settlement set directly on the base system is
copied into every load case of a `LoadCombination` (like any other base-system
load), so put it in a `LoadCase`.

See `examples/ex_29_thermal_loadcombination.py`.

## 4. Hinges and rotational springs

```python
ss.add_element([[0, 0], [4, 0]], spring={2: 0})        # hinge at the end of node 2
ss.add_element([[4, 0], [8, 0]], spring={1: 5e3})      # rotational spring k = 5000 at node 1
ss.add_internal_hinge(node_id)                         # hinge in a node, for all elements joined there
```

- A hinge (`0`) is condensed exactly: a beam with one hinge has the stiffness 3EI/l (not 4EI/l).
  A finite spring `k` acts in series with the element (flexibilities add: `1/k`). A hinge wins over
  a spring at the same end.
- The fixed-end moments of distributed loads use the same end stiffness
  (`Element.end_rotational_stiffness`), so `q_load`, thermal loads and imposed displacements are
  exact with hinges and springs.
- The `spring` and `mp` dictionaries given to `add_element` are copied, never modified.
- `discretize()` puts hinges, springs and `mp` only on the outer sub-elements and gives the new
  model other ids (use `find_node_id`).
- A moment load on an internal hinge or on a node that only has truss elements raises a
  `FEMException` (the moment would vanish silently).

## 5. Plastic hinges (`mp=`)

```python
ss = SystemElements(EA=1e6, EI=5e3)
ss.add_element([[0, 0], [4, 0]], mp={1: 4.0})   # plastic moment 4.0 at node 1 (node 2: mp={2: ...})
ss.add_support_fixed(1)
ss.add_support_roll(2)
ss.q_load(-3.0, 1)             # the elastic fixed-end moment would be q L**2 / 8 = 6
ss.solve(max_iter=200)         # |Tz| at node 1 -> 4.0 (mp); roller reaction q L / 2 - mp / L = 5
```

- A plastic hinge is a rotational spring **in series** with the element. Its stiffness starts at
  `PLASTIC_INITIAL_STIFFNESS` (100) times 4EI/l and is iterated (secant in the flexibility) until
  the end moment is `mp`; the redistribution is exact (the three-moment equation and the
  rotation of the propped cantilever are reproduced within 0.3 %).
- An end that is unloaded below `mp` becomes elastic again. Every `solve()` starts from the elastic
  stiffness (no plastic state leaks from an earlier calculation).
- `mp` must be positive (`mp=0` raises an `AssertionError`; a hinge is `spring={node: 0}`).
- Beyond the collapse load the structure is a mechanism and `solve()` raises
  `FEMException("ConvergenceError", ...)`. The same error (with a hint to increase `max_iter`)
  is raised when the iteration does not converge in `max_iter` iterations.
- Combines with `solve(geometrical_non_linear=True)`: every iteration is a second order
  analysis (moments at the plastic hinges stay at `mp`); `buckling_factor` is that of the elastic
  structure.
- Not supported in the Cython element (`anastruct/fem/cython`).

## 6. Dead load (self-weight)

```python
ss.add_element([[0, 0], [6, 0]], g=2.0)    # weight per meter given when the element is added
ss.dead_load(1, 2.0)                       # or later; replaces the weight of the element, load_factor applies
ss.remove_loads(dead_load=True)            # removes the weight too (q-loads keep working)

lc = LoadCase("weight")
lc.dead_load(1, 2.0)                       # in a load case / combination (factors apply)
```

- A propped cantilever (fixed at node 1, roller at node 2, L = 6) with `g = 2` has the roller
  reaction 3gL/8 = 4.5; in a `LoadCombination` with factor 1.5 it is 6.75.
- In a `LoadCombination` the weight of a load case is **added** to the weight of the base system
  and counted once, in any order of the load cases.

## 7. `validate()` and errors that are `FEMException`

- `ss.validate()` works before the first `solve()`: `True` for a supported (stable) structure,
  `False` for an unsupported one (`solve()` then raises `FEMException("StabilityError", ...)`).
  It processes supports on a shallow copy; it does not change the results of `solve()`.
- `FEMException` (instead of `ZeroDivisionError`, `LinAlgError`, silent wrong results):
  - an element with two equal locations (`"Invalid parameters"`),
  - `add_support_spring` with `k` not finite or negative,
  - an imposed displacement in a direction no support restrains (section 9),
  - a moment on an internal hinge or on a truss-only node,
  - displacements above 1e6 after `solve()` (`"StabilityError"`),
  - a plastic mechanism or a non-converging plastic iteration (`"ConvergenceError"`).
- `load_factor = 0` (all loads zero) still raises `AssertionError("There are no forces on the
  structure")`.
- Vertex coordinates are `float64` rounded to 12 significant digits, so a computed coordinate
  (intersection, `discretize`) and the same coordinate typed in are one node.

## 8. `invert_y_loads=False` and model edits after a solve

With the default `invert_y_loads=True` a positive `Fy`, `q` and imposed `uy` act in +y (up); with
`invert_y_loads=False` they act in −y (gravity), as follows:

| Load | `invert_y_loads=False` compared with the default |
|---|---|
| `Fx`, `q_load(direction="x")`, `q_load(direction="parallel")` | the same |
| `Fy`, `q_load(direction="y"/"element"/"perpendicular")` | reversed |
| `q_perp` of an `"x"` / `"parallel"` load | reversed (it is y-like) |
| `point_load(rotation=r)` | rotation of the force vector, y-part reversed |
| `q_load(rotation=r)` | mirror image of the default: `rotation=0` is `"x"`, `rotation=90` is `"y"` |

Model edits are allowed after a `solve()`; the next `solve()` processes the changed model:

```python
ss.solve()
ss.remove_element(1)               # node ids now have a gap (2, 3, 4, ...)
ss.add_support_fixed(2)
ss.add_element([[12, 0], [16, 0]]) # gets the id max + 1 (5), never an existing id
ss.solve()
```

- `add_element`, `remove_element`, `add_support_*`, `add_internal_hinge` reset the solution
  (`_invalidate_solution()`): the displacement vector, the buckling factor and the plastic state.
  The supports, spring supports and imposed displacements of a removed node are removed with it.
- Node ids can have gaps; the dof of a node is `dof_index(system, node_id)`.
- An imposed displacement can only be set while there is no solution (before the first `solve()`
  or after a model edit).

## 9. Known limitations (by design decision)

- `remove_loads()` removes thermal loads and imposed displacements;
  `discretize()` and `insert_node()` carry thermal loads (and, for
  `discretize()`, imposed displacements) over to the new mesh.
- Imposed displacements must be set before the first `solve()` (otherwise a
  `FEMException` is raised).
- A displacement can only be imposed in a direction that a support **restrains**: not in
  the free direction of a roller, not as a rotation at a hinged support or an internal
  hinge, not at a spring support and not at a node without a support. An inclined roller
  restrains only the translation normal to its free direction, while `ux`/`uy` are global,
  so they can not be imposed there. A `FEMException` is raised (at `solve()`, and at
  `imposed_displacement()` for an inclined roller) whose message explains which support
  restrains what (`describe_unrestrained_dof` in `system_components/util.py`).
- `thermal_load()` on an element replaces its earlier thermal load (like
  `q_load`): give the uniform change and the gradient in one call.
- Positive gradient = warmer face on the left when looking from node 1 to
  node 2 (the upper face for non-vertical elements; for vertical elements it
  depends on the entry order).
- End hinges and rotational springs (`spring={node: k}`, `add_internal_hinge`)
  are handled exactly for all loads: the element stiffness is condensed
  (hinge: 3EI/l instead of 4EI/l; spring: flexibilities in series, see
  `anastruct/fem/elements.py::constitutive_matrix` and the "End releases"
  section in `docs/source/elements.rst`; section 4 above).
- A bar of a truss does not buckle between its nodes (no bending stiffness): model it as a general
  element with `spring={1: 0, 2: 0}` and give `discretize_kwargs`. A member with hinges at both ends
  is a string for the buckling analysis until it is divided; with one element per member the
  buckling factor of hinges/springs is within ~0.2 % and converges with `discretize`.
- Open edge cases: `load_factor=0`, `validate()` on a shallow copy, `Vertex.__eq__` with a tuple
  is exact (see `MEMORY.md`).
- No plotting support for either feature yet (plots of loads don't show them;
  result plots like `show_bending_moment()` are correct).
- Element result curves `deflection`/`extension` (relative to the element
  chord) exclude the free thermal curvature/strain. Nodal displacements,
  reactions and M/N/V diagrams are exact.
- Sphinx entries: `docs/source/loads.rst` and `docs/source/supports.rst`.
