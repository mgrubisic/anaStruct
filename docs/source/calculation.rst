Calculation
===========

Once all the elements, supports and loads are in place, solving the calculation is as easy as calling the `solve`
method.

.. automethod:: anastruct.fem.system.SystemElements.solve

Non linear
##########

The model will automatically do a non linear calculation if there are non linear nodes present in the
SystemElements state. You can however force the model to do a linear calculation with the `force_linear` parameter.

Geometrical non linear
######################

To start a geometrical non linear calculation you'll need to set the `geometrical_non_linear` to True. It is also wise
to pass a `discretize_kwargs` dictionary.

.. code-block:: python

    ss.solve(geometrical_non_linear=True, discretize_kwargs=dict(n=20))

With this dictionary you can set the amount of discretization elements
generated during the geometrical non linear calculation. This calculation is an approximation and gets more accurate
with more discretization elements.

Buckling factor
...............

The calculation also determines the buckling factor, :code:`ss.buckling_factor`: the factor by which all the loads can
be increased until the structure buckles. It is the smallest positive :math:`\lambda` of the generalized eigenvalue
problem :math:`(K_0 + \lambda K_g) x = 0`, with :math:`K_0` the stiffness matrix of the first order analysis and
:math:`K_g` the geometric stiffness matrix for the normal forces of that analysis. Only compression gives a buckling
mode; if no compressive normal force acts on the free degrees of freedom the buckling factor is infinite
(:code:`float("inf")`).

The geometric stiffness matrix of a cubic beam element is an approximation, so the factor is found from above and
converges with the number of elements: one element gives :math:`12 EI/L^2` for a column pinned at both ends, 20 elements
the Euler load :math:`\pi^2 EI/L^2` within 0.01 %. Hinges and rotational springs at the ends of an element (see
:ref:`End releases <end-releases>`) are accounted for.

.. code-block:: python

    ss = SystemElements(EI=5e3, EA=1e6)
    ss.add_element([[0, 0], [0, 10]])  # a column; its top node is at y = 10
    ss.add_support_hinged(1)
    ss.add_support_roll(2, "y")
    ss.point_load(2, Fy=-1)  # a negative Fy pushes the top towards the base: compression
    ss.solve(geometrical_non_linear=True, discretize_kwargs=dict(n=20))
    print(ss.buckling_factor)  # 493.48, the Euler load of 1 * pi**2 * EI / L**2