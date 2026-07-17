# d4 Project

> **Note:** Some features of d4 are not yet available in the current version and require further development. To use the Max#SAT solver or the QBF counter, please use the tagged version v0.1.0.

**d4** is a tool designed to address a variety of computational problems beyond NP. It supports the following capabilities:

* Compiling a CNF formula into a decision-DNNF (with support for querying the compiled form)
* Counting (weighted) models of CNF or circuit formulas
* Projected (weighted) model counting over CNF formulas
* Model counting for QBF formulas
* Finding variable instantiations that maximize (possibly weighted or aggregated) model counts

---

## 🔧 Building and Using d4 as a Library

To build **d4**, ensure the following dependencies are installed:

* CMake (version ≥ 3.1)
* A C++ compiler with C++20 support
* `libgmp` and `libz`

Build the library with:

```bash
./build.sh
```

Optional flags:

* `-d`: Enable debug flags
* `-p`: Enable profiling flags
* `-s`: Enable static linking

After compilation, the static library `libd4.a` will be available in the `build/` directory.

---

## 🔍 How to Use d4

**d4** is a library and not a standalone executable. You need to link it with your own project. Check the `demo/` directory for usage examples.

> Note: You do *not* need to build the library separately for the demos, each demo handles its own build process.

---

### 🧮 Counter Mode

To use **d4** for model counting, navigate to:

```bash
cd demo/counter
./build.sh
```

Example usage with a CNF file:

```bash
./build/counter -i ../../instancesTest/cnfs/cnf5.cnf
```

To specify circuit input:

```bash
./build/counter -i ../../instancesTest/circuits/circ1.bc --input-type circuit
```

To enable the built-in BiPe preprocessor:

```bash
./build/counter -i ../../instancesTest/cnfs/cnf5.cnf -p sharp-equiv
```

For a full list of options:

```bash
./build/counter
```

> Input format follows the [MC 2024 competition guidelines](https://mccompetition.org/assets/files/mccomp_format_24.pdf)

---

### 🏗️ Compiler Mode

The compiler shares the same command-line interface as the counter. Go to:

```bash
cd demo/compiler
./build.sh
```

#### Querying a Decision-DNNF

You can query the compiled DNNF by passing a query file (or stdin) to `--query`.

* To **count models**: lines start with `m`, followed by literals, ending in `0`
* To **check satisfiability**: lines start with `d`, followed by literals, ending in `0`

Example:

```bash
$ ./build/compiler -i ../../instancesTest/cnfs/cnf5.cnf --query /dev/stdin
...
m 0
s 7106560
m 1 0
s 2842624
m -1 0
s 4263936
d 1 0
s SAT
```

#### Dumping the Decision-DNNF

To dump the compiled formula:

```bash
./build/compiler -i ../../instancesTest/cnfs/cnf5.cnf --dump-file /dev/stdout
```

> Format is compatible with [decDNNF reasoner](http://www.cril.univ-artois.fr/kc/d-DNNF-reasoner.html) and [decdnnf\_rs](https://crates.io/crates/decdnnf_rs)

For more options:

```bash
./build/compiler
```

---


## 📂 Repository Overview

```
d4/
├── build.sh                # Build script
├── build/                  # Output binaries and libraries
├── demo/
│   ├── counter/            # Model counting demo
│   ├── compiler/           # Compilation and querying demo
│   └── maxT/               # Max#SAT demo
├── instancesTest/          # Sample CNF/Circuit instances
```

# Input Formats

For the CNF format we use the standard DIMACS format (see https://mccompetition.org/assets/files/mccomp_format_24.pdf)

## circuit

We currently use our own defined format (BC-S1.2), where 
a formula is represented as a list of gates (input, output, gate_type), 
and a list of gates that must evaluate to true or false.

The BC-S1.2 format is the following:

Comment
* c *\n

WeightInfo
* c w literal weight\n

Var
* name

Literal
* Var
* -Var

Circuit
* Statement
* Statement\nCircuit

Statement
* G Var := FlatFormula\n 
* I var\n
* T literal\n

FlatFormula
* A LiteralList
* O LiteralList
* I Literal

LiteralList
* Literal
* Literal LiteralList

A statement defines a gate represented by a formula, or 
declares an input variable that is no gate, or 
declares a literal that must be true. Multiple of each statements may occur.

A formula is either an AND (A), OR (O), or identity (I). The latter
can also be used to denote negation `x := -y`.  
Negation is supported by minus sign in front of a name. 

A LiteralList in O, or A must contain at least 2 literals.


### Examples
A formula that is a single literal `-x`
```
c BC-S1.2
I x
T -x
```

A formula that is `(a & b) | -(-c & b)`
```
c BC-S1.2
I a
I b
I c
G g1 := A a b
G g2 := A -c b
G g3 := O g1 -g2
T g3
```

