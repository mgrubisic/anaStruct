from anastruct.fem.system import SystemElements

# Propped cantilever with a support settlement of 20 mm at the roller.
ss = SystemElements(EA=1e6, EI=1e4)
ss.add_element(location=[(0, 0), (10, 0)])
ss.add_support_fixed(node_id=1)
ss.add_support_roll(node_id=2)
ss.imposed_displacement(node_id=2, uy=0.02)

# Fixed-fixed beam with an imposed rotation of 0.01 rad at the right end.
ss2 = SystemElements(EA=1e6, EI=1e4)
ss2.add_element(location=[(0, 0), (10, 0)])
ss2.add_support_fixed(node_id=[1, 2])
ss2.imposed_displacement(node_id=2, phi_z=0.01)

if __name__ == "__main__":
    ss.solve()
    print("Settlement of 20 mm at node 2:")
    print("Roller reaction Fy:", ss.reaction_forces[2].Fy)  # -3 * EI * delta / L**3
    print("Fixed end moment Tz:", ss.reaction_forces[1].Tz)  # 3 * EI * delta / L**2
    ss.show_bending_moment()

    ss2.solve()
    print("\nImposed rotation of 0.01 rad at node 2:")
    print("End moments:", ss2.reaction_forces[1].Tz, ss2.reaction_forces[2].Tz)
    ss2.show_bending_moment()
