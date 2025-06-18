# d4 project

Put some sentences to describe the context.

Description of the different methods supported by d4.


# How to Compile

First you need to initialize the submodule by using the following command line:
```console
git submodule update --init --recursive
git pull --recurse-submodules
git submodule update --remote --merge
```

In order to compile the project cmake (version>=3.1) has to
be installed. The following command lines then build and compile the
project.

```console
$ ./build.sh
```

The executable is called d4 and is in the build repository.

```console
$ ./build/d4 -h
```


# Architecture

How to add something to d4.

# Methods Implemented

The different methods available.

## Model Counting


## Projected Model Counting


## Knowledge Compilation


## Parallel Model Counting


# Input Formats

The different input formats available.

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

# cnf