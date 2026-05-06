import os
import re


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def camel_to_kebab(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1-\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1-\2', s1).lower()


def get_json_schema_info(cpp_type):
    cpp_type = cpp_type.strip()
    cpp_type = re.sub(r'^(?:public|private|protected)\s*:\s*', '', cpp_type).strip()

    if cpp_type in ['int', 'unsigned', 'size_t', 'long', 'short', 'unsigned int', 'long long']:
        return {"type": "integer"}
    if cpp_type in ['float', 'double']:
        return {"type": "number"}
    if cpp_type == 'bool':
        return {"type": "boolean"}
    if cpp_type in ['std::string', 'string']:
        return {"type": "string"}
    if cpp_type.startswith('std::vector') or cpp_type.startswith('vector'):
        return {"type": "array"}
    # Enums / managers → integer or string
    enum_keywords = ['Method', 'Type', 'Strategy', 'Name', 'ModeStore',
                     'ClauseRepresentation', 'Manager', 'Kind']
    if any(x in cpp_type for x in enum_keywords):
        return {"type": ["integer", "string"]}
    return {"type": "object", "title": cpp_type}


def dict_to_cpp_init_list(d, indent=12):
    """Recursively convert a Python dict to a C++ nlohmann initialiser list."""
    ind = " " * indent
    if isinstance(d, dict):
        if not d:
            return "nlohmann::json::object()"
        items = []
        for k, v in d.items():
            val_str = dict_to_cpp_init_list(v, indent + 4)
            # get_enum_doc<T>() must NOT be quoted
            if k == "description" and isinstance(v, str) and v.startswith("get_enum_doc<"):
                val_str = v
            items.append(f'{ind}{{"{k}", {val_str}}}')
        return "{\n" + ",\n".join(items) + "\n" + (" " * (indent - 4)) + "}"
    elif isinstance(d, list):
        if not d:
            return "nlohmann::json::array()"
        items = [f'{ind}{dict_to_cpp_init_list(v, indent + 4)}' for v in d]
        return "{\n" + ",\n".join(items) + "\n" + (" " * (indent - 4)) + "}"
    elif isinstance(d, str):
        if d.startswith("get_enum_doc<"):
            return d
        return '"' + d.replace('"', '\\"') + '"'
    elif isinstance(d, bool):
        return "true" if d else "false"
    elif isinstance(d, (int, float)):
        return str(d)
    return '""'


# ---------------------------------------------------------------------------
# Collect all headers reachable from Binding.hpp (direct #includes)
# ---------------------------------------------------------------------------

def collect_headers(binding_path, base_path):
    """Return the list of absolute paths of headers included in binding_path
    that live under base_path/src/."""
    with open(binding_path) as f:
        content = f.read()
    headers = []
    for inc in re.findall(r'#include\s*"(src/[^"]+)"', content):
        p = os.path.join(base_path, inc)
        if os.path.exists(p):
            headers.append(p)
    return headers


# ---------------------------------------------------------------------------
# Parse enums from C++ headers
# ---------------------------------------------------------------------------

def parse_enums_from_headers(headers):
    """Scan header files and return a dict:
        { enum_type_name : [list_of_constant_names] }
    Handles both 'enum Name { ... }' and 'enum class Name { ... }'.
    Ignores values that are numeric assignments (keeps constant names only).
    """
    enums = {}
    enum_re = re.compile(
        r'\benum\s+(?:class\s+)?(\w+)\s*(?::\s*\w+\s*)?\{([^}]*)\}',
        re.DOTALL
    )
    # matches a single enumerator: optional identifier, optional = value
    enumerator_re = re.compile(r'\b([A-Z_][A-Z0-9_]+)\b')

    for path in headers:
        try:
            with open(path) as f:
                content = f.read()
        except OSError:
            continue

        # Strip line comments and block comments to avoid false positives
        content_clean = re.sub(r'//[^\n]*', '', content)
        content_clean = re.sub(r'/\*.*?\*/', '', content_clean, flags=re.DOTALL)

        for m in enum_re.finditer(content_clean):
            enum_name = m.group(1)
            body = m.group(2)

            # Split on commas, grab the identifier before any '='
            constants = []
            for entry in body.split(','):
                entry = entry.strip()
                if not entry:
                    continue
                # Take the part before '=' if present
                ident_part = entry.split('=')[0].strip()
                # We only want UPPER_CASE identifiers (C-style enum constants)
                tokens = enumerator_re.findall(ident_part)
                if tokens:
                    constants.append(tokens[-1])

            if constants:
                enums[enum_name] = constants

    return enums


# ---------------------------------------------------------------------------
# Parse struct members from headers (types + optional Doxygen comments)
# ---------------------------------------------------------------------------

def parse_cpp_headers(headers, nlohmann_structs):
    struct_types = {}   # { struct_name : { var_name : cpp_type } }
    doc_comments = {}   # { (struct_name, var_name) : "description" }

    member_pattern = re.compile(
        r'(?:/\*\*?\s*(?:@|\\)brief\s*(.*?)\s*\*+/|///\s*(?:@|\\)brief\s*(.*?)\n)?'
        r'\s*(?:(?:public|private|protected)\s*:\s*)?'
        r'([a-zA-Z0-9_:<>]+(?:\s+[a-zA-Z0-9_:<>]+)*)\s+([a-zA-Z0-9_]+)\s*(?:=|;)',
        re.DOTALL | re.MULTILINE
    )

    for path in headers:
        try:
            with open(path) as f:
                header_content = f.read()
        except OSError:
            continue

        for class_match in re.finditer(
            r'(?:struct|class)\s+([a-zA-Z0-9_]+)\s*(?::\s*[^{]+)?\{(.*?)\};',
            header_content, re.DOTALL
        ):
            struct_name = class_match.group(1).strip()
            if struct_name not in nlohmann_structs:
                continue

            members_content = class_match.group(2)
            members = {}

            for mm in member_pattern.finditer(members_content):
                comment_group = mm.group(1) or mm.group(2)
                cpp_type = mm.group(3).strip()
                var_name = mm.group(4).strip()

                cpp_type = re.sub(r'^(?:public|private|protected)\s*:', '', cpp_type).strip()
                if cpp_type in ['public', 'private', 'protected', 'friend', 'static']:
                    continue
                if 'operator' in var_name or cpp_type.endswith(':'):
                    continue

                members[var_name] = cpp_type
                if comment_group:
                    clean = re.sub(r'^\s*\*+\s*', '', comment_group,
                                   flags=re.MULTILINE).strip()
                    doc_comments[(struct_name, var_name)] = clean

            if members:
                struct_types[struct_name] = members

    return struct_types, doc_comments


# ---------------------------------------------------------------------------
# Parse Binding.hpp for the list of bound structs and their field order
# ---------------------------------------------------------------------------

def parse_binding_hpp(binding_path):
    with open(binding_path) as f:
        content = f.read()
    bindings = {}
    for m in re.finditer(
        r'NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(?:_WITH_DEFAULT)?\(\s*([a-zA-Z0-9_]+)\s*,\s*(.*?)\)',
        content, re.DOTALL
    ):
        struct_name = m.group(1).strip()
        vars_list = [v.strip() for v in m.group(2).split(',') if v.strip()]
        bindings[struct_name] = vars_list
    return bindings


# ---------------------------------------------------------------------------
# Generate EnumRegistry.hpp  (fully dynamic, no hard-coded labels)
# ---------------------------------------------------------------------------

def generate_enum_registry(base_path, headers):
    """Scan headers for enums that are actually used as member types in the
    bound structs, then emit an EnumMetadata specialisation for each one."""

    all_enums = parse_enums_from_headers(headers)

    # Header of the generated file
    lines = [
        "#pragma once",
        "",
        "// THIS FILE IS GENERATED BY gen_providers.py. DO NOT EDIT MANUALLY.",
        "",
        "#include <nlohmann/json.hpp>",
        "#include <string>",
        "#include <map>",
        "#include <vector>",
        "",
    ]

    # Re-emit the same #includes that are in the binding header so that all
    # enum constants are visible.
    binding_path = os.path.join(base_path, 'src/binding/json/Binding.hpp')
    with open(binding_path) as f:
        for raw in f:
            m = re.match(r'#include\s*"(src/[^"]+)"', raw.strip())
            if m:
                lines.append(f'#include "{m.group(1)}"')

    lines += [
        "",
        "namespace d4 {",
        "",
        "    /**",
        "     * @brief Trait to provide metadata about enums.",
        "     */",
        "    template<typename EnumType>",
        "    struct EnumMetadata {",
        "        static std::map<int, std::string> mapping() { return {}; }",
        "        static std::string name() { return \"UnknownEnum\"; }",
        "    };",
        "",
        "    /**",
        "     * @brief Helper to generate a documentation string for an enum.",
        "     */",
        "    template<typename EnumType>",
        "    inline std::string get_enum_doc() {",
        "        auto m = EnumMetadata<EnumType>::mapping();",
        "        std::string doc = \"\";",
        "        for (auto const& [val, label] : m) {",
        "            if (!doc.empty()) doc += \", \";",
        "            doc += std::to_string(val) + \"=\" + label;",
        "        }",
        "        return doc;",
        "    }",
        "",
        "    // Specializations – auto-generated from C++ enum definitions",
        "",
    ]

    for enum_name, constants in sorted(all_enums.items()):
        entries = ", ".join(
            f'{{(int){c}, "{c}"}}'
            for c in constants
        )
        lines += [
            f"    template<> struct EnumMetadata<{enum_name}> {{",
            f'        static std::string name() {{ return "{enum_name}"; }}',
            f"        static std::map<int, std::string> mapping() {{",
            f"            return {{{entries}}};",
            f"        }}",
            f"    }};",
            "",
        ]

    lines += ["} // namespace d4", ""]

    out = "\n".join(lines)
    output_path = os.path.join(base_path, 'src/binding/json/EnumRegistry.hpp')
    with open(output_path, 'w') as f:
        f.write(out)
    print(f"Generated {output_path}  ({len(all_enums)} enums found)")
    return all_enums


# ---------------------------------------------------------------------------
# Generate SchemaProviders.hpp
# ---------------------------------------------------------------------------

def generate_providers():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_path = os.path.abspath(os.path.join(script_dir, '../../../'))
    if not os.path.exists(os.path.join(base_path, 'src')):
        base_path = os.getcwd()

    binding_path = os.path.join(base_path, 'src/binding/json/Binding.hpp')
    if not os.path.exists(binding_path):
        print(f"ERROR: Binding.hpp not found at {binding_path}")
        return

    headers = collect_headers(binding_path, base_path)

    # 1. Generate EnumRegistry.hpp (dynamic scan)
    all_enums = generate_enum_registry(base_path, headers)

    # Build a quick lookup: cpp_type_name → enum_type_name
    # (identity mapping since the type IS the enum name)
    enum_type_names = set(all_enums.keys())

    # 2. Parse bindings and struct member types
    bindings = parse_binding_hpp(binding_path)
    nlohmann_structs = set(bindings.keys())
    struct_member_types, header_docs = parse_cpp_headers(headers, nlohmann_structs)

    # 3. Build SchemaProviders.hpp
    out_cpp = """#pragma once

// THIS FILE IS GENERATED BY gen_providers.py. DO NOT EDIT MANUALLY.

#include <nlohmann/json.hpp>
#include "src/binding/json/Binding.hpp"
#include "src/binding/json/SchemaGenerator.hpp"
#include "src/binding/json/EnumRegistry.hpp"

namespace d4 {
"""

    memo_schemas = {}

    def get_struct_schema(s_name, visited=None):
        if visited is None:
            visited = set()
        if s_name in memo_schemas:
            return memo_schemas[s_name]
        if s_name in visited:
            return {"type": "object", "title": s_name}
        
        visited.add(s_name)
        v_list = bindings.get(s_name, [])
        m_types = struct_member_types.get(s_name, {})
        
        p = {}
        for var in v_list:
            c_type = m_types.get(var, "").strip()
            inj = {}

            # Description: prefer enum doc, else Doxygen comment
            if c_type in enum_type_names:
                inj["description"] = f"get_enum_doc<{c_type}>()"
            elif (s_name, var) in header_docs:
                inj["description"] = header_docs[(s_name, var)]

            # JSON type
            if c_type:
                info = get_json_schema_info(c_type)
                inj.update(info)
                # Recursively expand nested structs
                if info.get("type") == "object" and c_type in nlohmann_structs:
                    nested = get_struct_schema(c_type, visited.copy())
                    if "properties" in nested:
                        inj["properties"] = nested["properties"]
            else:
                # Heuristic from variable name
                if any(x in var for x in ['Method', 'Type', 'Strategy', 'Name', 'Manager']):
                    inj["type"] = ["integer", "string"]
                else:
                    inj["type"] = "object"
            
            p[var] = inj
        
        res = {"title": s_name, "properties": p}
        memo_schemas[s_name] = res
        return res

    for struct_name in bindings.keys():
        schema_dict = get_struct_schema(struct_name)
        if schema_dict.get("properties"):
            cpp_init_list = dict_to_cpp_init_list(schema_dict, 16)
            out_cpp += (
                f"\n    template<>\n"
                f"    struct SchemaProvider<{struct_name}> {{\n"
                f"        static nlohmann::json get() {{\n"
                f"            return {cpp_init_list};\n"
                f"        }}\n"
                f"    }};\n"
            )

    out_cpp += "\n} // namespace d4\n"

    output_path = os.path.join(base_path, 'src/binding/json/SchemaProviders.hpp')
    with open(output_path, 'w') as f:
        f.write(out_cpp)
    print(f"Generated {output_path}  ({len(bindings)} structs)")


if __name__ == "__main__":
    generate_providers()
