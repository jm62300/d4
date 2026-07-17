The d-DNNF representation utilizes a precise structural format, serialized into specific node declarations and branch specifications.

### Node Outputs

Each node type dictates a specific output structure, defined by a character identifier followed by numerical parameters and a terminating zero.

* **True Node**: Outputs the character `t`, followed by its uniquely assigned index.
```text
t <id> 0

```


* **False Node**: Outputs the character `f`, followed by its uniquely assigned index.
```text
f <id> 0

```


* **Unary Node**: Outputs an OR node (`o`) along with its index, followed by a `0`.
```text
o <id> 0

```


* **Binary Deterministic Or Node**: Outputs an OR node (`o`) along with its index, followed by a `0`.
```text
o <id> 0

```


* **Decomposable And Node**: Outputs an AND node (`a`) along with its index, followed by a `0`.
```text
a <id> 0

```


For every child connected to this node, it outputs an edge relation formatted as the parent's index, the child's index, and a terminating `0`:
```text
<id_parent> <id_child> 0

```



### Branch Outputs

Whenever a branch is printed (which occurs from a Unary or Binary OR node), it outputs a single space-separated line. This line represents the structural edge along with any enforced unit literals, concluding with a `0`:

```text
<id_father> <id_son> <lit_1> ... <lit_nbUnits> 0

```

The parameters correspond to the following definitions:

* `<id_father>`: The index of the originating parent node.
* `<id_son>`: The index of the destination child node.
* `<lit_i>`: The specific unit literals applied along this branch.