from enum import Enum
from collections import namedtuple
from typing import List

# Define a gate as a named tuple with an output (str), a tuple of inputs (str), and a gatetype
Gate = namedtuple('Gate', ['output', 'inputs', 'gtype'])


class GateType(Enum):
    AND = 1
    OR = 2
    IDENTITY = 3  # Simply passes the input to the output. Can be used as inefficient negation.


NEG_SYMBOL = "-"


def to_abs_name(name: str) -> str:
    """ Return the absolute name of a variable. """
    return name[1:] if name.startswith(NEG_SYMBOL) else name


def negate_name(name: str) -> str:
    """ Negate the given name. --x is converted into x. """
    if name.startswith(NEG_SYMBOL):
        return name[1:]
    else:
        return NEG_SYMBOL + name


class GateList:

    def __init__(self):
        self.gates: List[Gate] = []
        self.true_names = []
        self._gate_names = set()
        self._input_names = set()

    def add_gate(self, gate: Gate):
        self.gates.append(gate)
        # new gate name?
        abs_output = to_abs_name(gate.output)
        self._input_names.discard(abs_output)
        self._gate_names.add(abs_output)
        # new input names?
        for _input in gate.inputs:
            abs_input = to_abs_name(_input)
            if abs_input not in self._gate_names:
                self._input_names.add(abs_input)
        if gate.gtype == GateType.IDENTITY:
            assert len(gate.inputs) == 1
        else:
            assert len(gate.inputs) > 1

    def add_true_gate(self, name: str):
        assert type(name) is str
        self.true_names.append(name)
        # new name?
        abs_name = to_abs_name(name)
        if abs_name not in self._gate_names and abs_name not in self._input_names:
            self._input_names.add(abs_name)

    def iter_gate_names(self):
        for gname in self._gate_names:
            yield gname

    def iter_input_names(self):
        for iname in self._input_names:
            yield iname

    def nb_gates(self):
        return len(self.gates)

    def nb_inputs(self):
        return len(self._input_names)

    def nb_vars(self):
        return self.nb_gates() + self.nb_inputs()

    def is_name_used(self, name: str) -> bool:
        abs_name = to_abs_name(name)
        return abs_name in self._gate_names or abs_name in self._input_names

    def __iter__(self):
        for gate in self.gates:
            yield gate

    def __str__(self):
        r = "\n".join((str(gate) for gate in self))
        r += f"True names: {self.true_names}"
        return r
