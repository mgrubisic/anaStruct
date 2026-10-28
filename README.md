# anaStruct 2D Frames and Trusses
[![Python tests](https://github.com/ritchie46/anaStruct/actions/workflows/test.yaml/badge.svg)](https://github.com/ritchie46/anaStruct/actions/workflows/test.yaml)
[![Documentation Status](https://readthedocs.org/projects/anastruct-docs/badge/?version=latest)](http://anastructd-docs.readthedocs.io/en/latest/?badge=latest)
![PyPI - Version](https://img.shields.io/pypi/v/anastruct)
![PyPI - Downloads](https://img.shields.io/pypi/dm/anastruct)
![Latest Release](https://img.shields.io/github/release-date/ritchie46/anaStruct)
![Commits since latest release](https://img.shields.io/github/commits-since/ritchie46/anaStruct/latest)


Analyse 2D Frames and trusses for slender structures. Determine the bending moments, shear forces, axial forces and displacements.

## Fork additions

This fork ([mgrubisic/anaStruct](https://github.com/mgrubisic/anaStruct)) adds two load
types on top of upstream anaStruct, verified against analytical solutions in
`tests/test_imposed_thermal.py`:

* **Imposed displacements/rotations of supports** (support settlement):
  `ss.imposed_displacement(node_id, ux=None, uy=None, phi_z=None)` — also available in
  `LoadCase`/`LoadCombination` as `lc.imposed_displacement(...)`. Like the loads, a
  positive `uy` acts in the +y direction (upwards, like a positive `Fy`), so a settlement
  is a negative `uy`
* **Thermal loads on elements** (uniform temperature change and linear temperature
  gradient over the section depth):
  `ss.thermal_load(element_id, delta_t, alpha, delta_t_gradient=None, h=None)` —
  also available in `LoadCase`/`LoadCombination` as `lc.thermal_load(...)`

```python
from anastruct import SystemElements

ss = SystemElements(EA=1e6, EI=1e4)
ss.add_element([[0, 0], [10, 0]])
ss.add_support_fixed(1)
ss.add_support_roll(2)
ss.imposed_displacement(2, uy=-0.02)                             # 20 mm settlement (down)
ss.thermal_load(1, delta_t=20, alpha=1.2e-5,                     # uniform heating
                delta_t_gradient=15, h=0.5)                      # + gradient
ss.solve()
```

The fork also corrects the stiffness of **hinges and rotational springs** at element
ends (`spring={node: k}`, internal hinges): a hinge is condensed exactly (an element with
one hinge has 3EI/l instead of 4EI/l) and a finite spring `k` acts in series with the
element, instead of giving wrong values or NaN. See `docs/source/elements.rst`.
`discretize()` and the detection of internal hinges (independent of the order in which
the elements are added) were reworked as well, and the buckling factor of
`solve(geometrical_non_linear=True)` now agrees with the Euler buckling loads.

See `dev-docs/` for full documentation (usage recipes in `dev-docs/SKILLS.md`) and
`examples/ex_27_imposed_displacement.py`, `examples/ex_28_thermal_load.py`,
`examples/ex_29_thermal_loadcombination.py` for runnable examples.

## Installation

For the actively developed version:
```
$ pip install git+https://github.com/ritchie46/anaStruct.git
```

Or for a release:
```
$ pip install anastruct
```

## Read the docs!

[Documentation](http://anastruct-docs.readthedocs.io)

## Questions

Got a question? Please ask on [gitter](https://gitter.im/anaStruct/lobby).

## Includes

* trusses :heavy_check_mark:
* beams :heavy_check_mark:
* moment lines :heavy_check_mark:
* axial force lines :heavy_check_mark:
* shear force lines :heavy_check_mark:
* displacement lines :heavy_check_mark:
* hinged supports :heavy_check_mark:
* fixed supports :heavy_check_mark:
* spring supports :heavy_check_mark:
* q-load in elements direction :heavy_check_mark:
* point loads in global x, y directions on nodes :heavy_check_mark:
* dead load :heavy_check_mark:
* q-loads in global y direction :heavy_check_mark:
* hinged elements :heavy_check_mark:
* rotational springs :heavy_check_mark:
* non-linear nodes :heavy_check_mark:
* geometrical non linearity :heavy_check_mark:
* load cases and load combinations :heavy_check_mark:
* **imposed displacements/rotations of supports (settlement)** :heavy_check_mark:
* **thermal loads (uniform temperature change and gradient)** :heavy_check_mark:
* generic type of section - rectangle and circle :heavy_check_mark:
* EU, US, UK steel section database :heavy_check_mark:

## Examples

```python
from anastruct import SystemElements
import numpy as np

ss = SystemElements()
element_type = 'truss'

# Create 2 towers
width = 6
span = 30
k = 5e3

# create triangles
y = np.arange(1, 10) * np.pi
x = np.cos(y) * width * 0.5
x -= x.min()

for length in [0, span]:
    x_left_column = np.ones(y[::2].shape) * x.min() + length
    x_right_column = np.ones(y[::2].shape[0] + 1) * x.max() + length

    # add triangles
    ss.add_element_grid(x + length, y, element_type=element_type)
    # add vertical elements
    ss.add_element_grid(x_left_column, y[::2], element_type=element_type)
    ss.add_element_grid(x_right_column, np.r_[y[0], y[1::2], y[-1]], element_type=element_type)

    ss.add_support_spring(
        node_id=ss.find_node_id(vertex=[x_left_column[0], y[0]]),
        translation=2,
        k=k)
    ss.add_support_spring(
        node_id=ss.find_node_id(vertex=[x_right_column[0], y[0]]),
        translation=2,
        k=k)

# add top girder
ss.add_element_grid([0, width, span, span + width], np.ones(4) * y.max(), EI=10e3)

# Add stability elements at the bottom.
ss.add_truss_element([[0, y.min()], [width, y.min()]])
ss.add_truss_element([[span, y.min()], [span + width, y.min()]])

for el in ss.element_map.values():
    # apply wind load on elements that are vertical
    if np.isclose(np.sin(el.ai), 1):
        ss.q_load(
            q=1,
            element_id=el.id,
            direction='x'
        )

ss.show_structure()
ss.solve()
ss.show_displacement(factor=2)
ss.show_bending_moment()

```

![](doc/source/img/examples/tower_bridge_struct.png)

![](doc/source/img/examples/tower_bridge_displa.png)

![](doc/source/img/examples/tower_bridge_moment.png)


```python
from anastruct import SystemElements

ss = SystemElements(EA=15000, EI=5000)

# Add beams to the system.
ss.add_element(location=[0, 5])
ss.add_element(location=[[0, 5], [5, 5]])
ss.add_element(location=[[5, 5], [5, 0]])

# Add a fixed support at node 1.
ss.add_support_fixed(node_id=1)

# Add a rotational spring support at node 4.
ss.add_support_spring(node_id=4, translation=3, k=4000)

# Add loads.
ss.point_load(Fx=30, node_id=2)
ss.q_load(q=-10, element_id=2)

# Solve
ss.solve()

# Get visual results.
ss.show_structure()
ss.show_reaction_force()
ss.show_axial_force()
ss.show_shear_force()
ss.show_bending_moment()
ss.show_displacement()
```
![](images/rand/structure.png)

### Real world use case.
[Non linear water accumulation analysis](https://ritchievink.com/blog/2017/08/23/a-nonlinear-water-accumulation-analysis-in-python/)
