Supports
========

The following kinds of support conditions are possible.

* hinged (the node is able to rotate, but cannot translate)
* roll (the node is able to rotate and translation is allowed in one direction)
* fixed (the node cannot translate and not rotate)
* spring (translation and rotation are allowed but only with a linearly increasing resistance)

add_support_hinged
##################

.. automethod:: anastruct.fem.system.SystemElements.add_support_hinged

Example
.......

.. code-block:: python

    ss.add_element(location=[5, 1])
    ss.add_support_hinged(node_id=[1, 2])
    ss.show_structure()

.. image:: img/supports/hinged.png

add_support_roll
################

.. automethod:: anastruct.fem.system.SystemElements.add_support_roll


Example
.......

.. code-block:: python

    ss.add_element(location=[5, 5])
    ss.add_support_roll(node_id=2, direction=1)
    ss.add_support_roll(node_id=1, direction=2)
    ss.show_structure()

.. image:: img/supports/roll.png

add_support_fixed
#################

.. automethod:: anastruct.fem.system.SystemElements.add_support_fixed

Example
.......

.. code-block:: python

    ss.add_element(location=[0, 2.5])
    ss.add_support_fixed(node_id=1)
    ss.show_structure()

.. image:: img/supports/fixed.png

add_support_spring
##################

Example
.......

.. code-block:: python

    ss.add_element(location=[5, 5])
    ss.add_support_spring(node_id=1, translation=3, k=1000)
    ss.add_support_spring(node_id=-1, translation=2, k=1000)
    ss.show_structure()

.. image:: img/supports/spring.png

.. automethod:: anastruct.fem.system.SystemElements.add_support_spring


Imposed displacements
#####################

A support settlement or an imposed rotation is prescribed to a degree of freedom that is restrained by a support.
Impose the displacements before the first :meth:`~anastruct.fem.system.SystemElements.solve`. In a
:class:`.LoadCombination` use :meth:`.LoadCase.imposed_displacement`, so that the settlement belongs to one load case.

.. automethod:: anastruct.fem.system.SystemElements.imposed_displacement

Example
.......

.. code-block:: python

    ss = SystemElements(EA=1e6, EI=1e4)
    ss.add_element([[0, 0], [10, 0]])
    ss.add_support_fixed(1)
    ss.add_support_roll(2)
    ss.imposed_displacement(node_id=2, uy=0.02)  # 20 mm downwards
    ss.solve()
