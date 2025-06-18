from gate_list import GateList, GateType, NEG_SYMBOL, to_abs_name


def gates_to_dimacs_file(gates: GateList, filepath_dimacs):
    """
    Convert a GateList to a DIMACS file.
    """
    # First determine number of vars and number of clauses
    num_vars = gates.nb_vars()
    num_clauses = sum((len(gate.inputs)+1 for gate in gates)) + len(gates.true_names)

    gate_name_to_id = {}

    def _get_gid(name):
        abs_name = to_abs_name(name)
        lid = gate_name_to_id.get(abs_name)
        if lid is None:
            lid = len(gate_name_to_id) + 1
            gate_name_to_id[abs_name] = lid
        if name.startswith(NEG_SYMBOL):
            return -lid
        return lid

    with open(filepath_dimacs, "w") as file_obj:
        file_obj.write("c BC-S1.2\n")
        file_obj.write(f"p cnf {num_vars} {num_clauses}\n")
        for gate in gates.gates:
            output_id = _get_gid(gate.output)
            input_ids = tuple(_get_gid(input_) for input_ in gate.inputs)
            if gate.gtype == GateType.AND:
                # h <-> l1 & l2
                # h v -l1 v -l2
                file_obj.write(f"{output_id}")
                for input_id in input_ids:
                    file_obj.write(f" {-input_id}")
                file_obj.write(" 0\n")
                # -h v l1, -h v l2
                for input_id in input_ids:
                    file_obj.write(f"{-output_id} {input_id} 0\n")
            elif gate.gtype == GateType.OR:
                # h <-> l1 v l2
                # -h v l1 v l2
                file_obj.write(f"{-output_id}")
                for input_id in input_ids:
                    file_obj.write(f" {input_id}")
                file_obj.write(" 0\n")
                # h v -l1, h v -l2
                for input_id in input_ids:
                    file_obj.write(f"{output_id} {-input_id} 0\n")
            else:
                assert gate.gtype == GateType.IDENTITY
                assert len(input_ids) == 1
                # h <-> l1
                # h v -l1
                file_obj.write(f"{output_id} {-input_ids[0]} 0\n")
                # -h v l1
                file_obj.write(f"{-output_id} {input_ids[0]} 0\n")
        # add true_lits
        for true_name in gates.true_names:
            true_id = _get_gid(true_name)
            file_obj.write(f"{true_id} 0\n")
