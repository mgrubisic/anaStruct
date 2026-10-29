from __future__ import annotations

import copy
from functools import lru_cache
from math import cos, sin
from typing import TYPE_CHECKING, Dict, List, Literal, Optional, Tuple

import numpy as np

from anastruct.basic import FEMException

if TYPE_CHECKING:
    from anastruct._types import ElementType
    from anastruct.fem.node import Node
    from anastruct.fem.system import Spring
    from anastruct.vertex import Vertex

try:
    from anastruct.fem.cython.celements import (  # pylint: disable=unused-import
        det_axial,
        det_moment,
        det_shear,
    )
except ImportError:
    from anastruct.fem.cython.elements import det_axial, det_moment, det_shear


# The matrices underneath are for slender beams, where the most deformation occurs due to bending.
# Shear deformation is not taken into account.

CACHE_BOUND = 32000


class Element:
    def __init__(
        self,
        id_: int,
        EA: float,
        EI: float,
        l: float,
        angle: float,
        vertex_1: Vertex,
        vertex_2: Vertex,
        type_: ElementType,
        section_name: str,
        spring: Optional[Spring] = None,
    ):
        """Create an element object

        Args:
            id_ (int): Integer representing the elements ID
            EA (float): Axial stiffness
            EI (float): Bending stiffness
            l (float): Length
            angle (float): Angle between element and x-axis
            vertex_1 (Vertex): Starting Vertex
            vertex_2 (Vertex): Ending Vertex
            type_ (str):
            section_name (str): Section name (for element annotation)
            spring (Optional[Spring], optional): Set a spring at node 1 or node 2.
                spring={1: k, 2: k}. Defaults to None.
        """
        self.id = id_
        self.type = type_
        self.EA = EA
        self.EI = EI
        self.l = l
        self.springs = spring
        self.vertex_1 = vertex_1  # location
        self.vertex_2 = vertex_2  # location
        self.angle = self.a1 = self.a2 = angle
        self.kinematic_matrix = kinematic_matrix(angle, angle, l)
        self.constitutive_matrix: np.ndarray
        self.stiffness_matrix: np.ndarray
        self.node_id1: int
        self.node_id2: int
        self.node_map: Dict[int, Node]
        self.element_displacement_vector: np.ndarray = np.empty(6)
        self.element_primary_force_vector: np.ndarray = np.zeros(
            6
        )  # acting external forces
        self.element_force_vector: np.ndarray = np.array([])
        self.q_load: tuple = (0.0, 0.0)
        self.q_perp_load: tuple = (0.0, 0.0)
        self.q_direction: Optional[str] = None
        self.q_angle: Optional[float] = None
        self.dead_load: float = 0.0
        # (alpha * delta_t, alpha * delta_t_gradient / h): free thermal strain and
        # curvature of the element
        self.temperature_load: Tuple[float, float] = (0.0, 0.0)
        self.N_1: Optional[float] = None
        self.N_2: Optional[float] = None
        self.bending_moment: Optional[np.ndarray] = None
        self.shear_force: Optional[np.ndarray] = None
        self.axial_force: Optional[np.ndarray] = None
        self.deflection: Optional[np.ndarray] = None
        self.total_deflection: Optional[np.ndarray] = None
        self.extension: Optional[np.ndarray] = None
        self.max_deflection: Optional[float] = None
        self.max_total_deflection: Optional[float] = None
        self.max_extension: Optional[float] = None
        self.nodes_plastic: List[bool] = [False, False]
        self.compile_constitutive_matrix(initial=True)
        self.compile_stiffness_matrix()
        self.section_name = section_name  # needed for element annotation

    @property
    def all_qp_load(self) -> List[float]:
        """All parallel q (distributed) loads

        Returns:
            List[float]: All parallel q (distributed) loads
        """
        if self.q_angle is not None:
            q_factor = sin(self.q_angle - self.angle)
            q_perp_factor = cos(self.q_angle - self.angle)
        else:
            q_factor = 1
            q_perp_factor = 0
        q = [
            i * q_factor + j * q_perp_factor
            for i, j in zip(self.q_load, self.q_perp_load)
        ]

        return [i + self.dead_load * cos(self.angle) for i in q]

    @property
    def all_qn_load(self) -> List[float]:
        """All normal q (distributed) loads

        Returns:
            List[float]: All normal q (distributed) loads
        """
        if self.q_angle is not None:
            q_factor = -cos(self.q_angle - self.angle)
            q_perp_factor = sin(self.q_angle - self.angle)
        else:
            q_factor = 0
            q_perp_factor = 1
        q = [
            i * q_factor + j * q_perp_factor
            for i, j in zip(self.q_load, self.q_perp_load)
        ]

        return [i + self.dead_load * -sin(self.angle) for i in q]

    @property
    def node_1(self) -> Node:
        """Starting node

        Returns:
            Node: Starting node
        """
        return self.node_map[self.node_id1]

    @property
    def node_2(self) -> Node:
        """Ending node

        Returns:
            Node: Ending node
        """
        return self.node_map[self.node_id2]

    @property
    def hinges(self) -> List[int]:
        """Node IDs of hinges

        Returns:
            List[int]: Node IDs of hinges
        """
        out = []

        assert self.springs is not None
        for k, v in self.springs.items():
            if v == 0:
                if k == 1:
                    out.append(self.node_id1)
                if k == 2:
                    out.append(self.node_id2)
        return out

    def determine_force_vector(self) -> Optional[np.ndarray]:
        """Determine the force vector of the element

        Returns:
            Optional[np.ndarray]: Force vector of the element
        """
        self.element_force_vector = np.dot(
            self.stiffness_matrix, self.element_displacement_vector
        )
        return self.element_force_vector

    def compile_stiffness_matrix(self) -> None:
        """Compile the stiffness matrix of the element"""
        self.stiffness_matrix = stiffness_matrix(
            self.constitutive_matrix, self.kinematic_matrix
        )

    def compile_kinematic_matrix(self) -> None:
        """Compile the kinematic matrix of the element"""
        self.kinematic_matrix = kinematic_matrix(self.a1, self.a2, self.l)

    @property
    def end_rotational_stiffness(self) -> Tuple[float, float]:
        """Rotational stiffness of the connections at the two element ends

        These are the stiffnesses that determine the fixed-end forces due to a
        distributed load: 0 for a hinge, the spring stiffness `k` for a rotational
        spring, and (practically) infinite for a rigid connection. A plastic hinge
        follows the (reduced) stiffness of the constitutive matrix, so that the
        end moment converges to the plastic moment. A truss element carries no
        moment, so both ends are hinges.

        Returns:
            Tuple[float, float]: Rotational stiffness at node 1 and node 2
        """
        if self.type == "truss":
            return 0.0, 0.0

        stiffness = []
        for end in (1, 2):
            released = self.constitutive_matrix[end][end] == 0
            spring = self.springs.get(end) if self.springs else None
            if released:
                stiffness.append(0.0)
            elif spring and not self.nodes_plastic[end - 1]:
                stiffness.append(float(spring))
            else:
                stiffness.append(float(self.constitutive_matrix[end][end]) * 1e6)
        return stiffness[0], stiffness[1]

    def compile_constitutive_matrix(self, initial: bool = False) -> None:
        """Compile the constitutive matrix of the element"""
        if initial:  # if element is just being created
            self.constitutive_matrix = constitutive_matrix(
                self.EA, self.EI, self.l, self.springs, False, False
            )
        else:
            self.constitutive_matrix = constitutive_matrix(
                self.EA,
                self.EI,
                self.l,
                self.springs,
                self.node_1.hinge,
                self.node_2.hinge,
            )

    def update_stiffness(self, factor: float, node: Literal[1, 2]) -> None:
        """Update the stiffness matrix of the element

        Args:
            factor (float): Factor to multiply the stiffness matrix with
            node (Literal[1, 2]): Node ID of the node to update (1 or 2)
        """
        if node == 1:
            self.constitutive_matrix[1][1] *= factor
            self.constitutive_matrix[1][2] *= factor
            self.constitutive_matrix[2][1] *= factor
        elif node == 2:
            self.constitutive_matrix[1][2] *= factor
            self.constitutive_matrix[2][1] *= factor
            self.constitutive_matrix[2][2] *= factor
        self.compile_stiffness_matrix()

    def compile_geometric_non_linear_stiffness_matrix(self) -> None:
        """Compile the geometric non-linear stiffness matrix of the element"""
        self.compile_stiffness_matrix()
        assert self.N_1 is not None
        self.stiffness_matrix += geometric_stiffness_matrix(
            self.l, self.N_1, self.a1, self.a2
        )

    def reset(self) -> None:
        """Reset the element's solve state"""
        self.element_displacement_vector = np.zeros(6)
        self.element_primary_force_vector = np.zeros(6)

    def __add__(self, other: Element) -> Element:
        """Add two elements

        Args:
            other (Element): Element to add

        Raises:
            FEMException: If the elements have different IDs

        Returns:
            Element: Sum of the two elements
        """
        if self.id != other.id:
            raise FEMException(
                "Wrong element:", "only elements with the same id can be added."
            )
        el = copy.deepcopy(self)
        for unit in [
            "bending_moment",
            "shear_force",
            "deflection",
            "total_deflection",
            "extension",
            "N_1",
            "N_2",
            "axial_force",
        ]:
            if getattr(el, unit) is None:
                setattr(el, unit, getattr(other, unit))
            else:
                setattr(el, unit, getattr(el, unit) + getattr(other, unit))
        el.max_deflection = (
            None if el.deflection is None else np.max(np.abs(el.deflection))
        )
        el.q_load = (el.q_load[0] + other.q_load[0], el.q_load[1] + other.q_load[1])
        el.q_perp_load = (
            el.q_perp_load[0] + other.q_perp_load[0],
            el.q_perp_load[1] + other.q_perp_load[1],
        )
        el.temperature_load = (
            el.temperature_load[0] + other.temperature_load[0],
            el.temperature_load[1] + other.temperature_load[1],
        )

        el.node_map[self.node_id1] = el.node_1 + other.node_1
        el.node_map[self.node_id2] = el.node_2 + other.node_2
        el.node_map[self.node_id1].ux = el.node_1.ux + other.node_1.ux
        el.node_map[self.node_id1].uy = el.node_1.uy + other.node_1.uy
        el.node_map[self.node_id1].phi_z = el.node_1.phi_z + other.node_1.phi_z
        el.node_map[self.node_id2].ux = el.node_2.ux + other.node_2.ux
        el.node_map[self.node_id2].uy = el.node_2.uy + other.node_2.uy
        el.node_map[self.node_id2].phi_z = el.node_2.phi_z + other.node_2.phi_z
        return el


@lru_cache(CACHE_BOUND)
def kinematic_matrix(a1: float, a2: float, l: float) -> np.ndarray:
    """Generate the kinematic matrix of an element

    Args:
        a1 (float): Angle of the element at node 1
        a2 (float): Angle of the element at node 2
        l (float): Length of the element

    Returns:
        np.ndarray: Kinematic matrix of the element
    """
    c1 = cos(a1)
    s1 = sin(a1)
    c2 = cos(a2)
    s2 = sin(a2)
    return np.array(
        [
            [-c1, s1, 0, c2, -s2, 0],
            [s1 / l, c1 / l, -1, -s2 / l, -c2 / l, 0],
            [-s1 / l, -c1 / l, 0, s2 / l, c2 / l, 1],
        ]
    )


def constitutive_matrix(
    EA: float,
    EI: float,
    l: float,
    spring: Optional["Spring"],
    node_1_hinge: Optional[bool],
    node_2_hinge: Optional[bool],
) -> np.ndarray:
    """Generate the constitutive matrix of an element

    The matrix relates the element deformations (axial elongation and the two end
    rotations relative to the chord) to the element forces (axial force and the two end
    moments). The end releases are taken into account as follows:

    * A hinge (a hinge node, or a rotational spring with stiffness 0) carries no moment.
      Its rotation is eliminated by static condensation (Schur complement) of the
      bending stiffness [[4, -2], [-2, 4]] * EI / l. An element with one hinge therefore
      has a rotational stiffness of 3 EI / l at the other end, and an element with two
      hinges has no bending stiffness.
    * A rotational spring k at an end acts in series with the element, so the
      flexibilities add: C = C_element + diag(1 / k1, 1 / k2). The result is exact for
      any k, from k -> 0 (hinge) to k -> infinity (rigid connection).
    * A hinge prevails over a spring at the same end.

    Args:
        EA (float): Axial stiffness
        EI (float): Bending stiffness
        l (float): Length
        spring (Optional[Spring]): Rotational spring stiffnesses at node 1 and node 2
            (0 is a hinge)
        node_1_hinge (Optional[bool]): Whether node 1 is a hinge
        node_2_hinge (Optional[bool]): Whether node 2 is a hinge

    Returns:
        np.ndarray: Constitutive matrix of the element
    """
    matrix = np.zeros((3, 3))
    matrix[0][0] = EA / l

    # bending part, relating the end rotations (relative to the chord) to the end moments
    bending = EI / l * np.array([[4.0, -2.0], [-2.0, 4.0]])

    # an end is released (hinge) when it is a hinge node or has a rotational spring
    # with stiffness 0
    spring = spring or {}
    spring_stiffness = [spring.get(1), spring.get(2)]  # None: no spring
    released = [
        bool(node_1_hinge) or spring_stiffness[0] == 0,
        bool(node_2_hinge) or spring_stiffness[1] == 0,
    ]
    spring_flexibility = [1 / k if k else 0.0 for k in spring_stiffness]
    active = [i for i in (0, 1) if not released[i]]
    condensed = [i for i in (0, 1) if released[i]]

    if active:
        # static condensation of the released rotations: the moment at a hinge is zero,
        # so the stiffness of the remaining rotations is the Schur complement
        # (a fixed-hinge element has 3EI/l instead of 4EI/l)
        stiffness = bending[np.ix_(active, active)]
        if condensed:
            stiffness = stiffness - (
                bending[np.ix_(active, condensed)]
                @ np.linalg.inv(bending[np.ix_(condensed, condensed)])
                @ bending[np.ix_(condensed, active)]
            )

        # rotational springs at the ends act in series with the (condensed) element:
        # the flexibilities add, C = C_element + diag(1 / k1, 1 / k2)
        flexibility = np.linalg.inv(stiffness) + np.diag(
            [spring_flexibility[i] for i in active]
        )
        stiffness = np.linalg.inv(flexibility)

        rows = [i + 1 for i in active]
        matrix[np.ix_(rows, rows)] = stiffness

    return matrix


def stiffness_matrix(
    var_constitutive_matrix: np.ndarray, var_kinematic_matrix: np.ndarray
) -> np.ndarray:
    """Generate the stiffness matrix of an element

    Args:
        var_constitutive_matrix (np.ndarray): Constitutive matrix of the element
        var_kinematic_matrix (np.ndarray): Kinematic matrix of the element

    Returns:
        np.ndarray: Stiffness matrix of the element
    """
    kinematic_transposed_times_constitutive = (
        var_kinematic_matrix.transpose() @ var_constitutive_matrix
    )
    return kinematic_transposed_times_constitutive @ var_kinematic_matrix  # type: ignore


def geometric_stiffness_matrix(l: float, N: float, a1: float, a2: float) -> np.ndarray:
    """Generate the geometric stiffness matrix of an element

    Args:
        l (float): Length
        N (float): Axial force
        a1 (float): Angle of the element at node 1
        a2 (float): Angle of the element at node 2

    Returns:
        np.ndarray: Geometric stiffness matrix of the element
    """
    c1 = cos(a1)
    s1 = sin(a1)
    c2 = cos(a2)
    s2 = sin(a2)
    # http://people.duke.edu/~hpgavin/cee421/frame-finite-def.pdf
    return (  # type: ignore
        N
        / l
        * np.array(
            [
                [
                    6 / 5 * s1**2,
                    -6 / 5 * s1 * c1,
                    -l / 10 * s1,
                    -6 / 5 * s2**2,
                    6 / 5 * s2 * c2,
                    -l / 10 * s2,
                ],
                [
                    -6 / 5 * s1 * c1,
                    6 / 5 * c1**2,
                    l / 10 * c1,
                    6 / 5 * s2 * c2,
                    -6 / 5 * c2**2,
                    l / 10 * c2,
                ],
                [
                    -l / 10 * s1,
                    l / 10 * c1,
                    2 * l**2 / 15,
                    l / 10 * s2,
                    -l / 10 * c2,
                    -(l**2) / 30,
                ],
                [
                    -6 / 5 * s1**2,
                    6 / 5 * s1 * c1,
                    l / 10 * s1,
                    6 / 5 * s2**2,
                    -6 / 5 * s1 * c2,
                    l / 10 * s2,
                ],
                [
                    6 / 5 * s1 * c1,
                    -6 / 5 * c1**2,
                    -l / 10 * c1,
                    -6 / 5 * s2 * c2,
                    6 / 5 * c2**2,
                    -l / 10 * c2,
                ],
                [
                    -l / 10 * s1,
                    l / 10 * c1,
                    -(l**2) / 30,
                    l / 10 * s2,
                    -l / 10 * c2,
                    2 * l**2 / 15,
                ],
            ]
        )
        * np.array([1, -1, 1, 1, -1, 1])
    )  # conversion from coordinate system
