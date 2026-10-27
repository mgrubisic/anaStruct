from anastruct.fem.system import SystemElements

# Fixed-fixed beam with a uniform temperature increase of 20 degrees.
# The restrained free expansion causes a constant compression force
# N = -EA * alpha * dT.
ss = SystemElements(EA=1e6, EI=1e4)
ss.add_element(location=[(0, 0), (10, 0)])
ss.add_support_fixed(node_id=[1, 2])
ss.thermal_load(element_id=1, delta_t=20, alpha=1.2e-5)

# Fixed-fixed beam with a linear temperature gradient of 15 degrees over a
# section depth of 0.5 m (upper face warmer). The restrained free curvature
# kappa = alpha * dT_gradient / h causes a constant bending moment
# M = EI * alpha * dT_gradient / h.
ss2 = SystemElements(EA=1e6, EI=1e4)
ss2.add_element(location=[(0, 0), (10, 0)])
ss2.add_support_fixed(node_id=[1, 2])
ss2.thermal_load(element_id=1, delta_t=0, alpha=1.2e-5, delta_t_gradient=15, h=0.5)

# Cantilever with the same gradient curves freely: the tip deflects by
# kappa * L**2 / 2 and no internal forces develop.
ss3 = SystemElements(EA=1e6, EI=1e4)
ss3.add_element(location=[(0, 0), (10, 0)])
ss3.add_support_fixed(node_id=1)
ss3.thermal_load(element_id=1, delta_t=0, alpha=1.2e-5, delta_t_gradient=15, h=0.5)

if __name__ == "__main__":
    ss.solve()
    print("Uniform dT on fixed-fixed beam:")
    print("Axial reactions:", ss.reaction_forces[1].Fx, ss.reaction_forces[2].Fx)
    ss.show_axial_force()

    ss2.solve()
    print("\nTemperature gradient on fixed-fixed beam:")
    print("End moments:", ss2.reaction_forces[1].Tz, ss2.reaction_forces[2].Tz)
    ss2.show_bending_moment()

    ss3.solve()
    print("\nTemperature gradient on cantilever:")
    print("Tip displacement:", ss3.get_node_displacements(2))
    ss3.show_displacement()
