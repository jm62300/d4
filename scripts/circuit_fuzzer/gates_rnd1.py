import math
import random
from typing import Dict

from gate_list import GateType, GateList, Gate, NEG_SYMBOL, to_abs_name


def get_default_rnd_args1() -> Dict:
    return {
        "nb_inputs": 10,
        "nb_gates": 20,
        "polarity_prob": 0.5,  # probability of a variable being positive
        "gate_type_OR_prob": 0.3,  # probability of a gate being an OR gate
        "aggregate_roots": GateType.OR,  # how to aggregate root nodes.
        "gate_inputs": 2.5,  # increases the number of inputs per gate
    }


def generate_rnd_gates1(gen_args: Dict = None) -> GateList:
    """ use get_default_rnd_args1() to pass arguments """
    if gen_args is None:
        gen_args = get_default_rnd_args1()
    nb_gates = gen_args["nb_gates"]
    nb_inputs = gen_args["nb_inputs"]
    prob_polarity_p = gen_args["polarity_prob"]
    prob_or = gen_args["gate_type_OR_prob"]
    sigma_gate_inputs = gen_args["gate_inputs"]
    aggregate_root_type = gen_args["aggregate_roots"]

    assert nb_gates > 1
    assert nb_inputs > 1
    gates = GateList()
    names = list()
    for i in range(nb_inputs):
        name = f"i{i}"
        names.append(name)

    # construct gates by combining previous (gate) names
    for i in range(nb_gates):
        gname = f"g{i}"
        # randomize inputs
        ginputs_nb = 2 + \
            abs(math.floor(random.gauss(mu=0.0, sigma=sigma_gate_inputs)))
        ginputs_nb = min(ginputs_nb, len(names))
        ginputs = []
        for _input in random.sample(names, ginputs_nb):
            _input = _input if random.random() < prob_polarity_p else f"{
                NEG_SYMBOL}{_input}"
            ginputs.append(_input)

        # random gate type
        gtype = GateType.OR if random.random() < prob_or else GateType.AND

        names.append(gname)
        gates.add_gate(Gate(gname, ginputs, gtype))

    # root gate
    # detect what are the root nodes, then combine them into an OR
    names = set(names)
    for gate in gates:
        names.difference_update({to_abs_name(x) for x in gate.inputs})
    assert len(names) > 0
    if len(names) == 1:
        gates.add_true_gate(names.pop())
    else:
        gates.add_gate(Gate("root", tuple(names), aggregate_root_type))
        gates.add_true_gate("root")
    return gates


def generate_rnd_gates1_1(gen_args: Dict = None,
                          max_inputs=10,
                          max_gates=100) -> GateList:
    """
    Equivalent to `generate_rnd_gates1`,
    but randomly varies the number of gates and inputs.
    """
    assert max_inputs > 2
    if gen_args is None:
        gen_args = get_default_rnd_args1()
    gen_args = gen_args.copy()
    # nb inputs
    gen_args["nb_inputs"] = 3 + random.randint(0, max_inputs - 3)
    nb_inputs = gen_args["nb_inputs"]
    # nb gates
    if nb_inputs == 3:
        max_gates = min(max_gates, 3)
    elif nb_inputs < 5:
        max_gates = min(max_gates, 7)
    elif nb_inputs < 10:
        max_gates = min(max_gates, nb_inputs*3)
    elif nb_inputs < 20:
        max_gates = min(max_gates, nb_inputs*5)
    else:
        max_gates = min(max_gates, nb_inputs*10)
    gen_args["nb_gates"] = 2 + random.randint(0, max_gates)

    return generate_rnd_gates1(gen_args)


def func_generate_rnd_gates1_1(max_inputs=20, max_gates=100):
    """
    Returns a reference to the method `generate_rnd_gates1_1`,
    with the arguments `max_inputs` and `max_gates` set.
    In this way, the returned reference can be used to
    generate a GateList by calling `result(gen_args)`.
    """
    return lambda gen_args: generate_rnd_gates1_1(gen_args, max_inputs=max_inputs, max_gates=max_gates)
