from anastruct import SystemElements

def SolveTruss(user_members,support_nodes,force_nodes,force_dir,forces):
    try: 
        # Scaling to SI units and negating y-axis for accurate coordinate system
        scaled_members = []
        for start, end in user_members:
            scaled_members.append(((start[0]/1000, -start[1]/1000),(end[0]/1000, -end[1]/1000)))

        scaled_support_nodes = []
        for node in support_nodes:
            scaled_support_nodes.append((node[0]/1000, -node[1]/1000))

        scaled_f_nodes = []
        for node in force_nodes:
            scaled_f_nodes.append((node[0]/1000, -node[1]/1000))

        scaled_f = []
        for force in forces:
            scaled_f.append(force*1000)
        
        
        # Use consistent units: N, m, Pa
        E = 69e9       # Young's modulus of aluminum 6061-T6: 68.9 GPa
        sigma_y = 241e6   # Yield Strength of aluminum 6061-T6: 241 MPa
        A = 0.001       # Cross-sectional area: 1000 mm²

        ss = SystemElements(EA=E * A)

        members = {}
        for i in range(0, len(scaled_members)):     # Looping nodes into a dictionary of members
            mem_name = f"mem_{i}"
            members[mem_name] = ss.add_truss_element(location=[scaled_members[i][0], scaled_members[i][1]])

        # Setting boundary conditions
        for i in range(len(scaled_support_nodes)):
            node_id = ss.find_node_id(scaled_support_nodes[i])
            ss.add_support_hinged(node_id=node_id)

        # Setting applied forces
        for i in range(len(scaled_f_nodes)):
            node_id = ss.find_node_id(scaled_f_nodes[i])
            if force_dir[i] == "x":
                ss.point_load(node_id=node_id,Fx=scaled_f[i])
            elif force_dir[i] == "y":
                ss.point_load(node_id=node_id,Fy=scaled_f[i])
            else:
                raise TypeError(f"No direction given for force at index {i}")

        ss.solve()

        stress = []
        stress_norm = []
        for name, element_id in members.items():
            result = ss.get_element_results(element_id=element_id)
            force = result["Nmin"]
            stress.append(force / A)
            stress_norm.append((force/A)/sigma_y)

        return stress, stress_norm
    except:
        
        return None, None
