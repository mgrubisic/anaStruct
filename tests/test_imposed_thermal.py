from pytest import approx, raises

from anastruct import LoadCase, LoadCombination, SystemElements
from anastruct.basic import FEMException

"""
NOTE: These tests validate imposed displacements/rotations of supports (support
settlements) and thermal loads on elements against analytical results from
engineering theory. General definitions:
- L = member length
- EA = axial stiffness
- EI = bending stiffness
- alpha = coefficient of thermal expansion
- dT = uniform temperature change
- dT_grad = temperature difference over the section depth h
- delta = imposed support settlement
- phi = imposed support rotation
"""

EA = 1.0e6
EI = 1.0e4
L = 10.0
ALPHA = 1.2e-5
DT = 20.0
H = 0.5
DT_GRAD = 15.0


def describe_imposed_displacement_tests():
    def context_propped_cantilever_settlement():
        # Propped cantilever with a downward settlement delta of the prop:
        # R_prop = 3 * EI * delta / L**3 (pulling the beam down, i.e. negative),
        # M_fixed = 3 * EI * delta / L**2
        delta = 0.02

        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])
        system.add_support_fixed(1)
        system.add_support_roll(2)
        system.imposed_displacement(2, uy=delta)
        system.solve()

        def it_results_in_correct_reactions():
            assert system.reaction_forces[2].Fy == approx(-3 * EI * delta / L**3)
            assert system.reaction_forces[1].Tz == approx(-3 * EI * delta / L**2)
            # vertical equilibrium
            assert system.reaction_forces[1].Fy + system.reaction_forces[2].Fy == approx(
                0
            )

        def it_reports_the_imposed_displacement():
            assert system.get_node_displacements(2)["uy"] == approx(delta)
            assert system.reaction_forces[2].uy == approx(delta)

    def context_fixed_fixed_imposed_rotation():
        # Fixed-fixed beam with an imposed rotation phi at node 2:
        # M_near = 4 * EI * phi / L, M_far = 2 * EI * phi / L, R = 6 * EI * phi / L**2
        phi = 0.01

        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])
        system.add_support_fixed([1, 2])
        system.imposed_displacement(2, phi_z=phi)
        system.solve()

        def it_results_in_correct_reactions():
            assert system.reaction_forces[2].Tz == approx(4 * EI * phi / L)
            assert system.reaction_forces[1].Tz == approx(2 * EI * phi / L)
            assert system.reaction_forces[1].Fy == approx(-6 * EI * phi / L**2)
            assert system.reaction_forces[2].Fy == approx(6 * EI * phi / L**2)

        def it_reports_the_imposed_rotation():
            # reported rotations have the opposite sign of the system displacement
            # vector, consistent with the moment_load sign convention
            assert system.get_node_displacements(2)["phi_z"] == approx(-phi)

    def context_fixed_fixed_imposed_axial_displacement():
        # Fixed-fixed bar with an imposed axial displacement dx at node 2:
        # N = EA * dx / L (tension)
        dx = 0.01

        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])
        system.add_support_fixed([1, 2])
        system.imposed_displacement(2, ux=dx)
        system.solve()

        def it_results_in_correct_reactions():
            assert system.reaction_forces[1].Fx == approx(-EA * dx / L)
            assert system.reaction_forces[2].Fx == approx(EA * dx / L)

        def it_reports_the_imposed_displacement():
            assert system.get_node_displacements(2)["ux"] == approx(dx)

    def context_settlement_superposition_with_q_load():
        # Propped cantilever with a q-load and a settlement of the prop:
        # R_prop = 3 * q * L / 8 - 3 * EI * delta / L**3
        q = 10.0
        delta = 0.02

        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])
        system.add_support_fixed(1)
        system.add_support_roll(2)
        system.q_load(q, 1)
        system.imposed_displacement(2, uy=delta)
        system.solve()

        def it_results_in_correct_superposition():
            assert system.reaction_forces[2].Fy == approx(
                3 * q * L / 8 - 3 * EI * delta / L**3
            )

    def context_unrestrained_degree_of_freedom():
        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])
        system.add_support_fixed(1)
        system.imposed_displacement(2, uy=0.01)

        def it_raises_on_solve():
            with raises(FEMException):
                system.solve()

    def context_no_displacement_imposed():
        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])

        def it_raises_on_input():
            with raises(FEMException):
                system.imposed_displacement(1)


def describe_thermal_load_tests():
    def context_fixed_fixed_uniform_temperature():
        # Fixed-fixed beam with a uniform temperature increase:
        # N = -EA * alpha * dT (compression), no bending
        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])
        system.add_support_fixed([1, 2])
        system.thermal_load(1, delta_t=DT, alpha=ALPHA)
        system.solve()

        def it_results_in_correct_reactions():
            assert system.reaction_forces[1].Fx == approx(EA * ALPHA * DT)
            assert system.reaction_forces[2].Fx == approx(-EA * ALPHA * DT)
            assert system.reaction_forces[1].Tz == approx(0)
            assert system.reaction_forces[1].Fy == approx(0)

        def it_results_in_compression():
            el = system.get_element_results(1)
            assert el["Nmin"] == approx(-EA * ALPHA * DT)
            assert el["Nmax"] == approx(-EA * ALPHA * DT)
            assert el["Mmin"] == approx(0)
            assert el["Mmax"] == approx(0)

    def context_cantilever_uniform_temperature():
        # Cantilever with a uniform temperature increase expands freely:
        # tip displacement = alpha * dT * L, no internal forces
        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])
        system.add_support_fixed(1)
        system.thermal_load(1, delta_t=DT, alpha=ALPHA)
        system.solve()

        def it_results_in_free_expansion():
            assert system.get_node_displacements(2)["ux"] == approx(ALPHA * DT * L)

        def it_results_in_no_internal_forces():
            assert system.reaction_forces[1].Fx == approx(0)
            el = system.get_element_results(1)
            assert el["Nmin"] == approx(0)
            assert el["Nmax"] == approx(0)

    def context_fixed_fixed_temperature_gradient():
        # Fixed-fixed beam with a temperature gradient over the section depth:
        # constant bending moment M = EI * alpha * dT_grad / h, no shear
        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])
        system.add_support_fixed([1, 2])
        system.thermal_load(1, delta_t=0, alpha=ALPHA, delta_t_gradient=DT_GRAD, h=H)
        system.solve()

        def it_results_in_constant_moment():
            m = EI * ALPHA * DT_GRAD / H
            assert system.reaction_forces[1].Tz == approx(m)
            assert system.reaction_forces[2].Tz == approx(-m)
            el = system.get_element_results(1)
            assert el["Mmin"] == approx(m)
            assert el["Mmax"] == approx(m)

        def it_results_in_no_shear():
            assert system.reaction_forces[1].Fy == approx(0)
            assert system.reaction_forces[2].Fy == approx(0)
            el = system.get_element_results(1)
            assert el["Qmin"] == approx(0)
            assert el["Qmax"] == approx(0)

    def context_cantilever_temperature_gradient():
        # Cantilever with a temperature gradient curves freely with a constant
        # curvature kappa = alpha * dT_grad / h: tip deflection = kappa * L**2 / 2,
        # no internal forces
        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])
        system.add_support_fixed(1)
        system.thermal_load(1, delta_t=0, alpha=ALPHA, delta_t_gradient=DT_GRAD, h=H)
        system.solve()

        def it_results_in_free_curvature():
            kappa = ALPHA * DT_GRAD / H
            assert system.get_node_displacements(2)["uy"] == approx(
                kappa * L**2 / 2, rel=1e-3
            )

        def it_results_in_no_internal_forces():
            assert system.reaction_forces[1].Tz == approx(0)
            assert system.reaction_forces[1].Fy == approx(0)

    def context_truss_uniform_temperature():
        # Fully restrained truss bar with a uniform temperature increase:
        # N = -EA * alpha * dT (compression)
        system = SystemElements()
        system.add_truss_element([[0, 0], [L, 0]], EA=EA)
        system.add_support_hinged(1)
        system.add_support_hinged(2)
        system.thermal_load(1, delta_t=DT, alpha=ALPHA)
        system.solve()

        def it_results_in_compression():
            assert system.reaction_forces[1].Fx == approx(EA * ALPHA * DT)
            assert system.reaction_forces[2].Fx == approx(-EA * ALPHA * DT)
            el = system.get_element_results(1)
            assert el["Nmin"] == approx(-EA * ALPHA * DT)
            assert el["Nmax"] == approx(-EA * ALPHA * DT)

    def context_gradient_requires_section_depth():
        system = SystemElements(EA=EA, EI=EI)
        system.add_element([[0, 0], [L, 0]])

        def it_raises_without_h():
            with raises(FEMException):
                system.thermal_load(1, delta_t=0, alpha=ALPHA, delta_t_gradient=DT_GRAD)

    def context_thermal_load_in_load_case():
        lc = LoadCase("thermal")
        lc.thermal_load(1, delta_t=DT, alpha=ALPHA)

        def it_applies_the_load_case():
            system = SystemElements(EA=EA, EI=EI)
            system.add_element([[0, 0], [L, 0]])
            system.add_support_fixed([1, 2])
            system.apply_load_case(lc)
            system.solve()
            assert system.reaction_forces[1].Fx == approx(EA * ALPHA * DT)

        def it_applies_the_combination_factor():
            system = SystemElements(EA=EA, EI=EI)
            system.add_element([[0, 0], [L, 0]])
            system.add_support_fixed([1, 2])
            comb = LoadCombination("ULS")
            comb.add_load_case(lc, factor=1.5)
            results = comb.solve(system)
            assert results["combination"].reaction_forces[1].Fx == approx(
                1.5 * EA * ALPHA * DT
            )
