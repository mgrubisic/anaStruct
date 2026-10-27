import matplotlib.pyplot as plt

from anastruct import LoadCase, LoadCombination
from anastruct.fem.system import SystemElements

# Two-span continuous beam exposed to a uniform temperature change and a
# temperature gradient, combined with a q-load in a load combination.
ss = SystemElements(EA=1e6, EI=1e4)
ss.add_element(location=[(0, 0), (6, 0)])
ss.add_element(location=[(6, 0), (12, 0)])
ss.add_support_hinged(node_id=1)
ss.add_support_roll(node_id=2)
ss.add_support_roll(node_id=3)

lc_thermal = LoadCase("thermal")
lc_thermal.thermal_load(element_id=[1, 2], delta_t=15, alpha=1.2e-5)

lc_gradient = LoadCase("gradient")
lc_gradient.thermal_load(
    element_id=[1, 2], delta_t=0, alpha=1.2e-5, delta_t_gradient=10, h=0.5
)

lc_traffic = LoadCase("traffic")
lc_traffic.q_load(q=10, element_id=[1, 2])  # positive q acts downwards by default

combination = LoadCombination("SLS")
combination.add_load_case(lc_thermal, factor=1.0)
combination.add_load_case(lc_gradient, factor=1.0)
combination.add_load_case(lc_traffic, factor=1.0)

if __name__ == "__main__":
    results = combination.solve(ss)

    for name, system in results.items():
        print(name, "-> middle support reaction Fy:", system.reaction_forces[2].Fy)
        system.show_bending_moment(show=False)
        plt.title(name)
        plt.show()  # type: ignore
