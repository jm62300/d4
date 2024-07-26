from gate_list import GateList, GateType, NEG_SYMBOL, to_abs_name


def gates_to_bcs_file(gates: GateList, filepath_bcs):
    from pathlib import Path
    Path(filepath_bcs).parent.mkdir(parents=True, exist_ok=True)
    with open(filepath_bcs, "w") as file_obj:
        file_obj.write("c BC-S1.2\n")
        # print true
        for true_name in gates.true_names:
            file_obj.write(f"T {true_name}\n")
        # print input
        for input_name in gates.iter_input_names():
            file_obj.write(f"I {input_name}\n")
        # print gates
        for gate in gates:
            if gate.gtype == GateType.AND:
                # G g3 := A i1 i2
                file_obj.write(f"G {gate.output} := A {' '.join(gate.inputs)}\n")
            elif gate.gtype == GateType.OR:
                # G g3 := O i1 i2
                file_obj.write(f"G {gate.output} := O {' '.join(gate.inputs)}\n")
            else:
                assert gate.gtype == GateType.IDENTITY
                assert len(gate.inputs) == 1
                # G g3 := I i1
                file_obj.write(f"G {gate.output} := I {gate.inputs[0]}\n")