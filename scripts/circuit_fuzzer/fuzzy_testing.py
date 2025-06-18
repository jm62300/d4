"""
Run this file to do fuzzy testing of circuit.

# Usage

Usage example,
```
fuzzy_testing.py -i ./counter -t 120 -n 10 --runconfigs cnf circuit_cnf circuit_cnf_equiv circuit_pcnf circuit_pcnf_dynbl circuit_pcnf_preproc0 circuit_pcnf_preproc1
```

Use the `-h` argument to print usage information.

```
usage: fuzzy_testing.py [-h] [-t T] [-n N] -i I
                        [--runconfigs {cnf,circuit,circuit_cnf,circuit_cnf_equiv,circuit_pcnf,circuit_pcnf_dynbl,circuit_pcnf_preproc0,circuit_pcnf_preproc1} [{cnf,circuit,circuit_cnf,circuit_cnf_equiv,circuit_pcnf,circuit_pcnf_preproc0,circuit_pcnf_preproc1} ...]]

options:
  -h, --help            show this help message and exit
  -t T                  The timeout for each instance (seconds).
  -n N                  Number of instances to try.
  -i I                  Filepath to a D4 counter binary.
  --runconfigs {cnf,circuit,circuit_cnf,circuit_cnf_equiv,circuit_pcnf,circuit_pcnf_dynbl,circuit_pcnf_preproc0,circuit_pcnf_preproc1} [{cnf,circuit,circuit_cnf,circuit_cnf_equiv,circuit_pcnf,circuit_pcnf_preproc0,circuit_pcnf_preproc1} ...]
                        Which configurations to test
```

In the main() method near the bottom you can find additional parameters you can tune,
that control the random circuit generation process.
As of now, the circuits are between 2-20 inputs nodes and 2-100 gates.

# Run configurations
// circuit: Run a BC file
COUNTER --input-type circuit -i BENCH

// cnf: Run a CNF file
COUNTER --input-type cnf -i BENCH

// circuit_cnf: Translate into CNF
COUNTER --input-type circuit -i BENCH --translate cnf

// circuit_pcnf: Translate into projected CNF
COUNTER --input-type circuit -i BENCH --translate pcnf

// circuit_pcnf_dynbl: Translate into projected CNF
COUNTER --input-type circuit -i BENCH --translate pcnf --occurrence-manager dynamicBlockedSimp

// circuit_cnf_equiv: Translate into CNF and use the preprocessor to keep the formula equivalent
COUNTER --input-type circuit -i BENCH --translate cnf  -p equiv

// circuit_pcnf_preproc0: Translate into projected CNF and use the preprocessor to forget existentially quantified variables without impacting the formula's size
COUNTER --input-type circuit -i BENCH --translate pcnf --occurrence-manager dynamicBlockedSimp -p compile-equiv --preproc-strong-elim 0

// circuit_pcnf_preproc1: Translate into projected CNF and use the preprocessor to forget existentially quantified variables
COUNTER --input-type circuit -i BENCH --translate pcnf --occurrence-manager dynamicBlockedSimp -p compile-equiv --preproc-strong-elim 1
"""
import pathlib
import random
import subprocess
import sys
import time
import re
from typing import List

from to_bcs import gates_to_bcs_file
from to_dimacs import gates_to_dimacs_file
from gate_list import GateType
from gates_rnd1 import generate_rnd_gates1, get_default_rnd_args1, func_generate_rnd_gates1_1

TIMEOUT_STR = "TIMEOUT"
VAL_ERR_STR = "VAL_ERR"


class ResultsCounter:

    def __init__(self, timeout, seed, filepath_cnf, filepath_circuit):
        self.results = dict()
        self.timeout = timeout
        self.seed = seed
        self.filepath_cnf = filepath_cnf
        self.filepath_circuit = filepath_circuit
        self.keep_instance_files = False
        self.finished = False

    def register_result(self, config_name, config_result):
        assert not self.finished
        self.results[config_name] = config_result
        if config_result == VAL_ERR_STR:
            self.keep_instance_files = True

    def mark_finished(self):
        self.finished = True

    def report_results(self):
        print(f"\tUsed seed {self.seed}")
        for key, value in self.results.items():
            if value == TIMEOUT_STR:
                print(f"\tTimeout for run_config: {key}")
            elif value == VAL_ERR_STR:
                print(f"\tError parsing the model count for seed {self.seed}, "
                      f"with run_config: {key}", file=sys.stderr)

        modelcounts = {key: value for key, value in self.results.items() if
                       value != TIMEOUT_STR and value != VAL_ERR_STR}
        if len(modelcounts) < 2:
            print("Could not verify model counts. Fewer than 2 configurations succeeded.")
            return
        unique_model_counts = set(mc for (key, mc) in modelcounts.items())
        if len(unique_model_counts) == 1:
            print(f"\tModel counts are the same: {unique_model_counts}")
        else:
            self.keep_instance_files = True
            print(f"\tModel counts are different for {
                  self.seed}", file=sys.stderr)
            for key, mc in modelcounts.items():
                print(f"\tModel count for {key}: {mc}", file=sys.stderr)

    def __del__(self):
        if self.finished and not self.keep_instance_files:
            pathlib.Path(self.filepath_cnf).unlink()
            pathlib.Path(self.filepath_circuit).unlink()


def extract_mc(output_str) -> int:
    """ Extract regex pattern \ns \d*\n """
    pattern = r"\ns (\d*)\n"
    match = re.search(pattern, output_str)
    if match is None:
        raise ValueError("Model count not found.")
    return int(match.group(1))


def run_d4(cmd: List[str], timeout: int):
    result = subprocess.run(cmd, capture_output=True,
                            text=True,
                            timeout=timeout)
    return result


def mc_cnf(filepath_counter, filepath_cnf, filepath_circuit, timeout):
    """
    Compute MC of the given cnf.
    May throw subprocess.TimeoutExpired.
    """
    cmd = [filepath_counter, "--input-type", "cnf", "-i", filepath_cnf]
    output = run_d4(cmd, timeout)
    assert output.returncode == 0, f"Return code {
        output.returncode} not 0 for {filepath_cnf}.\n{output.stdout}"
    mc = extract_mc(output.stdout)
    return mc


def mc_circuit(filepath_counter, filepath_cnf, filepath_circuit, timeout):
    """
    Compute MC of the given circuit.
    May throw subprocess.TimeoutExpired.
    """
    # raise NotImplementedError("Not implemented yet.")
    cmd = [filepath_counter, "--input-type", "circuit",
           "-i", filepath_circuit, "--remove-gates", "1"]
    output = run_d4(cmd, timeout)
    assert output.returncode == 0
    mc = extract_mc(output.stdout)
    return mc


def mc_circuit_via_cnf(filepath_counter, filepath_cnf, filepath_circuit, timeout):
    """
    Compute MC of the given circuit, via translation to cnf.
    May throw subprocess.TimeoutExpired.
    """
    cmd = [filepath_counter, "--input-type", "circuit",
           "-i", filepath_circuit, "--translate", "cnf"]
    output = run_d4(cmd, timeout)
    assert output.returncode == 0, f"Return code {
        output.returncode} not 0 for {filepath_circuit}.\n{output.stdout}"
    mc = extract_mc(output.stdout)
    return mc


def mc_circuit_via_cnf_equiv(filepath_counter, filepath_cnf, filepath_circuit, timeout):
    """
    Compute MC of the given circuit, via translation to cnf, and
    use the preprocessor to keep the formula equivalent.
    May throw subprocess.TimeoutExpired.
    """
    cmd = [filepath_counter, "--input-type", "circuit", "-i", filepath_circuit, "--translate", "cnf",
           "-p", "equiv"]
    output = run_d4(cmd, timeout)
    assert output.returncode == 0
    mc = extract_mc(output.stdout)
    return mc


def mc_circuit_via_pcnf(filepath_counter, filepath_cnf, filepath_circuit, timeout):
    """
    Compute MC of the given circuit, via translation to pcnf.
    May throw subprocess.TimeoutExpired.
    """
    cmd = [filepath_counter, "--input-type", "circuit",
           "-i", filepath_circuit, "--translate", "pcnf"]
    output = run_d4(cmd, timeout)
    assert output.returncode == 0
    mc = extract_mc(output.stdout)
    return mc


def mc_circuit_via_pcnf_dynbl(filepath_counter, filepath_cnf, filepath_circuit, timeout):
    """
    Compute MC of the given circuit, via translation to pcnf,
    with occurrence manager set to dynamic Blocked Simple.
    May throw subprocess.TimeoutExpired.
    """
    cmd = [filepath_counter, "--input-type", "circuit", "-i", filepath_circuit, "--translate", "pcnf",
           "--occurrence-manager", "dynamicBlockedSimp"]
    output = run_d4(cmd, timeout)
    assert output.returncode == 0
    mc = extract_mc(output.stdout)
    return mc


def mc_circuit_via_pcnf_preproc0(filepath_counter, filepath_cnf, filepath_circuit, timeout):
    """
    Compute MC of the given circuit, via translation to pcnf, and
    use preprocessor to forget existentially quantified variables
    without impacting the formula's size.
    May throw subprocess.TimeoutExpired.
    """
    cmd = [filepath_counter, "--input-type", "circuit", "-i", filepath_circuit, "--translate", "pcnf",
           "--occurrence-manager", "dynamicBlockedSimp", "-p", "compile-equiv",
           "--preproc-strong-elim", "0"]
    output = run_d4(cmd, timeout)
    assert output.returncode == 0
    mc = extract_mc(output.stdout)
    return mc


def mc_circuit_via_pcnf_preproc1(filepath_counter, filepath_cnf, filepath_circuit, timeout):
    """
    Compute MC of the given circuit, via translation to pcnf, and
    use preprocessor to forget existentially quantified variables.
    May throw subprocess.TimeoutExpired.
    """
    cmd = [filepath_counter, "--input-type", "circuit", "-i", filepath_circuit, "--translate", "pcnf",
           "--occurrence-manager", "dynamicBlockedSimp", "-p", "compile-equiv",
           "--preproc-strong-elim", "1"]
    output = run_d4(cmd, timeout)
    assert output.returncode == 0
    mc = extract_mc(output.stdout)
    return mc


def _fuzzy_test_instance(filepath_counter, gen_args, generate_function,
                         run_configs, timeout) -> ResultsCounter:
    """
    Execute each run configuration.
    """
    if generate_function is None:
        generate_function = generate_rnd_gates1
    seed = random.randrange(sys.maxsize)
    random.seed(a=seed)
    gates = generate_function(gen_args=gen_args)

    # Write to files
    filepath_cnf = f"./fuzzy_instance_{seed}.cnf"
    filepath_circuit = f"./fuzzy_instance_{seed}.bc"
    gates_to_dimacs_file(gates, filepath_cnf)
    gates_to_bcs_file(gates, filepath_circuit)

    results = ResultsCounter(timeout, seed, filepath_cnf, filepath_circuit)
    for run_config in run_configs:
        try:
            mc = run_config(filepath_counter, filepath_cnf,
                            filepath_circuit, timeout)
            results.register_result(run_config.__name__, str(mc))
        except subprocess.TimeoutExpired as err:
            results.register_result(run_config.__name__, TIMEOUT_STR)
        except ValueError as err:
            results.register_result(run_config.__name__, VAL_ERR_STR)
    results.mark_finished()
    return results


def fuzzy_test(filepath_counter, nb_instances, timeout, run_configs, gen_args, generate_function):
    assert run_configs is not None and len(run_configs) > 0
    if gen_args is None:
        gen_args = get_default_rnd_args1()

    print(f"Fuzzy testing {nb_instances} with timeout {timeout} per instance.")
    print(f"Trying {len(run_configs)} run configurations")
    for run_config in run_configs:
        print(f"\t{run_config.__name__}")
    print("==================================================================")
    for i in range(nb_instances):
        print(f"Instance {i + 1}/{nb_instances}")
        results = _fuzzy_test_instance(filepath_counter, gen_args,
                                       generate_function,
                                       run_configs, timeout)
        results.report_results()


# Each method takes as input
# (filepath_counter, filepath_cnf, filepath_circuit, timeout)
# and returns the model count.
# The filepath_cnf and filepath_circuit are the same instance.
# The filepath_counter is the path to the d4 counter binary.
# The timeout is the time limit for each instance.
# They may raise a TimeoutException, and a ValueError.
# The ValueError exception is raised when the mc could not be extracted.
run_config_map = {
    "cnf": mc_cnf,
    "circuit": mc_circuit,
    "circuit_cnf": mc_circuit_via_cnf,
    "circuit_cnf_equiv": mc_circuit_via_cnf_equiv,
    "circuit_pcnf": mc_circuit_via_pcnf,
    "circuit_pcnf_dynbl": mc_circuit_via_pcnf_dynbl,
    "circuit_pcnf_preproc0": mc_circuit_via_pcnf_preproc0,
    "circuit_pcnf_preproc1": mc_circuit_via_pcnf_preproc1,
}


def get_argparser():
    import argparse
    argparser = argparse.ArgumentParser()
    argparser.add_argument("-t", type=int, default=120,
                           help="The timeout for each instance (seconds).")
    argparser.add_argument("-n", type=int, default=100,
                           help="Number of instances to try.")
    argparser.add_argument("-i", type=str, required=True,
                           help="Filepath to a D4 counter binary.")
    argparser.add_argument("--runconfigs",
                           help="Which configurations to test",
                           choices=['cnf', 'circuit', 'circuit_cnf',
                                    'circuit_cnf_equiv', 'circuit_pcnf', 'circuit_pcnf_dynbl',
                                    'circuit_pcnf_preproc0', 'circuit_pcnf_preproc1'
                                    ], nargs="+")
    return argparser


def main(args):
    # parse args
    args = get_argparser().parse_args(args)
    _timeout = args.t
    _nb_inst = args.n
    _filepath_counter = args.i
    _run_configs = [run_config_map[run_config]
                    for run_config in args.runconfigs]

    gen_args = {
        "nb_inputs": 200,
        "nb_gates": 500,
        "polarity_prob": 0.7,  # probability of a variable being positive
        "gate_type_OR_prob": 0.3,  # probability of a gate being an OR gate
        "aggregate_roots": GateType.OR,  # how to aggregate root nodes.
        "gate_inputs": 2.5,  # increases the number of inputs per gate
    }
    fuzzy_test(filepath_counter=_filepath_counter,
               nb_instances=_nb_inst,
               timeout=_timeout,
               run_configs=_run_configs,
               gen_args=gen_args,
               # generate_function=generate_rnd_gates1),
               generate_function=func_generate_rnd_gates1_1(
                   max_inputs=100, max_gates=20),
               )


if __name__ == "__main__":
    _argv = sys.argv.copy()
    if len(_argv) == 1:
        _argv.append("-h")
    main(_argv[1:])
