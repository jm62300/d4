#!/usr/bin/env python3
"""
Circuit visualizer for BC-S1.2 files.

Usage:
  python visualize.py circuit.bc              # ASCII tree (default)
  python visualize.py circuit.bc --dot        # Graphviz DOT to stdout
  python visualize.py circuit.bc --render     # render with 'dot' → open SVG
  python visualize.py circuit.bc --render --out out.png
"""

from __future__ import annotations
import argparse
import os
import subprocess
import sys
import tempfile
from typing import Optional


# ---------------------------------------------------------------------------
# Parse BC-S1.2
# ---------------------------------------------------------------------------

def parse_bcs(path: str) -> tuple[
    list[str],                          # true_names
    list[str],                          # declared inputs (I lines)
    dict[str, tuple[str, list[str]]],   # name → (gtype, [input_tokens])
]:
    true_names: list[str] = []
    decl_inputs: list[str] = []
    gates: dict[str, tuple[str, list[str]]] = {}

    with open(path) as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("c"):
                continue
            if line.startswith("T "):
                true_names.append(line[2:].strip())
            elif line.startswith("I "):
                decl_inputs.append(line[2:].strip())
            elif line.startswith("G "):
                parts = line.split()
                name  = parts[1]
                gtype = parts[3]
                inps  = parts[4:]
                # For ATMOST the first token after type is the threshold integer.
                gates[name] = (gtype, inps)

    return true_names, decl_inputs, gates


def _abs(token: str) -> str:
    return token.lstrip("-")

def _neg(token: str) -> bool:
    return token.startswith("-")


# ---------------------------------------------------------------------------
# ASCII tree
# ---------------------------------------------------------------------------

_GATE_LABELS = {
    "A":  "AND",
    "O":  "OR",
    "X":  "XOR",
    "AM": "ATMOST",
    "I":  "ID",
}

# ANSI colours (auto-disabled when not a tty)
_USE_COLOR = sys.stdout.isatty()

def _c(text: str, code: str) -> str:
    return f"\033[{code}m{text}\033[0m" if _USE_COLOR else text

def _gate_color(gtype: str) -> str:
    return {"A": "34", "O": "32", "X": "35", "AM": "33", "I": "36"}.get(gtype, "0")


def _ascii_node(
    token: str,
    gates: dict,
    prefix: str,
    child_prefix: str,
    expanded: dict[str, int],   # name → depth first seen
    depth: int,
    out: list[str],
) -> None:
    neg  = _neg(token)
    name = _abs(token)
    sign = _c("¬", "31") + " " if neg else ""

    if name not in gates:
        # Primary input (leaf)
        out.append(prefix + sign + _c(name, "33"))
        return

    gtype, inps = gates[name]
    label = _GATE_LABELS.get(gtype, gtype)

    if name in expanded:
        # Already fully drawn — show a back-reference.
        out.append(prefix + sign + _c(name, "90") + f" [{label}] " + _c("↑ (already shown)", "90"))
        return

    expanded[name] = depth
    header = sign + _c(name, _gate_color(gtype)) + f" [{label}]"
    out.append(prefix + header)

    for i, inp in enumerate(inps):
        last = (i == len(inps) - 1)
        p  = child_prefix + ("└── " if last else "├── ")
        cp = child_prefix + ("    " if last else "│   ")
        _ascii_node(inp, gates, p, cp, expanded, depth + 1, out)


def ascii_tree(true_names: list[str], decl_inputs: list[str],
               gates: dict) -> str:
    lines: list[str] = []

    # Summary header
    all_inputs = sorted({
        _abs(inp)
        for _, inps in gates.values()
        for inp in inps
        if _abs(inp) not in gates
    } | set(decl_inputs))

    lines.append(_c("Inputs:", "1") + "  " + "  ".join(_c(v, "33") for v in all_inputs))
    lines.append(_c("Roots: ", "1") + "  " + "  ".join(_c(t, "32") for t in true_names))
    lines.append(_c(f"Gates:  {len(gates)}", "1"))
    lines.append("")

    expanded: dict[str, int] = {}
    for root in true_names:
        lines.append(_c("T", "1;32") + " " + _c(root, "32"))
        _ascii_node(root, gates, "    ", "    ", expanded, 0, lines)
        lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Graphviz DOT
# ---------------------------------------------------------------------------

_DOT_SHAPES = {
    "A":  ("box",     "#add8e6"),  # AND  — blue
    "O":  ("ellipse", "#90ee90"),  # OR   — green
    "X":  ("diamond", "#dda0dd"),  # XOR  — purple
    "AM": ("hexagon", "#ffa07a"),  # ATMOST — orange
    "I":  ("plain",   "#ffffff"),  # IDENTITY
}
_INPUT_SHAPE  = ("circle",    "#ffffcc")
_ROOT_SHAPE   = ("doublecircle", "#98fb98")


def to_dot(true_names: list[str], decl_inputs: list[str],
           gates: dict) -> str:
    lines = ["digraph circuit {",
             "  rankdir=BT;",
             "  node [fontname=monospace fontsize=11];",
             ""]

    all_inputs = sorted({
        _abs(inp)
        for _, inps in gates.values()
        for inp in inps
        if _abs(inp) not in gates
    } | set(decl_inputs))

    # Input nodes
    lines.append("  // primary inputs")
    lines.append("  { rank=same;")
    for v in all_inputs:
        sh, col = _INPUT_SHAPE
        lines.append(f'    "{v}" [shape={sh} style=filled fillcolor="{col}" label="{v}"];')
    lines.append("  }")
    lines.append("")

    # Gate nodes
    lines.append("  // gates")
    root_set = set(true_names)
    for name, (gtype, _) in gates.items():
        label = _GATE_LABELS.get(gtype, gtype)
        if name in root_set:
            sh, col = _ROOT_SHAPE
        else:
            sh, col = _DOT_SHAPES.get(gtype, ("box", "#ffffff"))
        lines.append(f'  "{name}" [shape={sh} style=filled fillcolor="{col}" label="{name}\\n{label}"];')
    lines.append("")

    # Edges (input token → gate name, label negation)
    lines.append("  // edges")
    for name, (_, inps) in gates.items():
        for tok in inps:
            src  = _abs(tok)
            neg  = _neg(tok)
            attr = ' [style=dashed label="¬"]' if neg else ""
            lines.append(f'  "{src}" -> "{name}"{attr};')
    lines.append("")

    # Force roots to the top rank
    if true_names:
        lines.append("  { rank=max; " + "; ".join(f'"{t}"' for t in true_names) + "; }")

    lines.append("}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Render with graphviz
# ---------------------------------------------------------------------------

def render(dot_src: str, out_path: Optional[str]) -> None:
    fmt = "png" if (out_path and out_path.endswith(".png")) else "svg"
    if out_path is None:
        fd, out_path = tempfile.mkstemp(suffix=f".{fmt}", prefix="circuit_")
        os.close(fd)
        auto_open = True
    else:
        auto_open = False

    try:
        proc = subprocess.run(
            ["dot", f"-T{fmt}", "-o", out_path],
            input=dot_src.encode(), check=True,
            capture_output=True,
        )
    except FileNotFoundError:
        print("ERROR: 'dot' not found — install Graphviz to use --render", file=sys.stderr)
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print("ERROR: graphviz failed:\n" + e.stderr.decode(), file=sys.stderr)
        sys.exit(1)

    print(f"Written: {out_path}", file=sys.stderr)
    if auto_open:
        opener = "xdg-open" if sys.platform.startswith("linux") else "open"
        subprocess.Popen([opener, out_path],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize a BC-S1.2 circuit file."
    )
    parser.add_argument("circuit", help="Path to the .bc file")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dot",    action="store_true", help="Emit Graphviz DOT source")
    mode.add_argument("--render", action="store_true", help="Render with graphviz and open")
    parser.add_argument("--out",  metavar="FILE",
                        help="Output file for --render (default: temp file)")
    args = parser.parse_args()

    true_names, decl_inputs, gates = parse_bcs(args.circuit)

    if args.dot or args.render:
        dot_src = to_dot(true_names, decl_inputs, gates)
        if args.dot:
            print(dot_src)
        else:
            render(dot_src, args.out)
    else:
        print(ascii_tree(true_names, decl_inputs, gates))


if __name__ == "__main__":
    main()
