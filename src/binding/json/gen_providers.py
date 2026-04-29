import os
import re

def get_dsc_descriptions(base_path):
    dsc_dirs = [
        'demo/compiler/src',
        'demo/counter/src',
        'demo/erosion/src',
        'demo/maxT/src',
        'demo/maxsat/src',
        'demo/minT/src',
        'demo/minsat/src',
        'demo/projMc/src',
        'src/app'
    ]
    
    descriptions = {}
    
    for d in dsc_dirs:
        filepath = os.path.join(base_path, d, 'option.dsc')
        if not os.path.exists(filepath): continue
            
        with open(filepath, 'r') as f:
            for line in f:
                match = re.search(r'\(\"([^,\"]+)[^\"]*\",\s*(?:[^,]*,)?\s*\"([^\"]+)\"\)', line)
                if match:
                    key = match.group(1).split(',')[0].strip()
                    descriptions[key] = match.group(2)
                    
    return descriptions

def camel_to_kebab(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1-\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1-\2', s1).lower()

def get_json_schema_info(cpp_type):
    cpp_type = cpp_type.strip()
    cpp_type = re.sub(r'^(?:public|private|protected)\s*:\s*', '', cpp_type)
    cpp_type = cpp_type.strip()

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
    if any(x in cpp_type for x in ['Method', 'Type', 'Strategy', 'Name', 'Manager', 'Kind']):
        return {"type": ["integer", "string"]}
    
    return {"type": "object", "title": cpp_type}

def parse_cpp_headers(base_path, binding_path):
    with open(binding_path, 'r') as f:
        content = f.read()
    
    nlohmann_structs = re.findall(r'NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE\(\s*([a-zA-Z0-9_]+)', content)
    includes = re.findall(r'#include\s*\"(.*?)\"', content)
    
    struct_types = {}
    doc_comments = {} # { (struct_name, var_name): "description" }
    
    for include in includes:
        if not include.startswith('src/'): continue
        header_path = os.path.join(base_path, include)
        if not os.path.exists(header_path): continue
            
        with open(header_path, 'r') as f:
            header_content = f.read()
            
        for class_match in re.finditer(r'(?:struct|class)\s+([a-zA-Z0-9_]+)\s*(?::\s*[^{]+)?\{(.*?)\};', header_content, re.DOTALL):
            struct_name = class_match.group(1).strip()
            if struct_name not in nlohmann_structs: continue
            
            members_content = class_match.group(2)
            members = {}
            
            # Match members and optionally their prepended Doxygen comments
            # Support: /** @brief ... */ or /*! @brief ... */ or /// @brief ...
            # Regex captures the comment block optionally
            member_pattern = re.compile(
                r'(?:/\*\*?\s*(?:@|\\)brief\s*(.*?)\s*\*+/|///\s*(?:@|\\)brief\s*(.*?)\n)?' # Optional comment
                r'\s*(?:(?:public|private|protected)\s*:\s*)?' # Optional visibility
                r'([a-zA-Z0-9_:<>]+(?:\s+[a-zA-Z0-9_:<>]+)*)\s+([a-zA-Z0-9_]+)\s*(?:=|;)', 
                re.DOTALL | re.MULTILINE
            )
            
            for member_match in member_pattern.finditer(members_content):
                comment_group = member_match.group(1) or member_match.group(2)
                cpp_type = member_match.group(3).strip()
                var_name = member_match.group(4).strip()
                
                cpp_type = re.sub(r'^(?:public|private|protected)\s*:', '', cpp_type).strip()
                
                if cpp_type in ['public', 'private', 'protected', 'friend', 'static']: continue
                if 'operator' in var_name: continue
                if cpp_type.endswith(':'): continue 
                
                members[var_name] = cpp_type
                if comment_group:
                    # Clean up comment (remove leading *, newlines)
                    clean_comment = re.sub(r'^\s*\*+\s*', '', comment_group, flags=re.MULTILINE).strip()
                    doc_comments[(struct_name, var_name)] = clean_comment
            
            if members:
                struct_types[struct_name] = members
                
    return struct_types, doc_comments

def parse_binding_hpp(binding_path):
    with open(binding_path, 'r') as f:
        content = f.read()
    bindings = {}
    for match in re.finditer(r'NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE\(\s*([a-zA-Z0-9_]+)\s*,\s*(.*?)\)', content, re.DOTALL):
        struct_name = match.group(1).strip()
        vars_str = match.group(2)
        vars_list = [v.strip() for v in vars_str.split(',')]
        bindings[struct_name] = vars_list
    return bindings

def dict_to_cpp_init_list(d, indent=12):
    ind = " " * indent
    if isinstance(d, dict):
        if not d: return "nlohmann::json::object()"
        items = [f'{ind}{{\"{k}\", {dict_to_cpp_init_list(v, indent + 4)}}}' for k, v in d.items()]
        return "{\n" + ",\n".join(items) + "\n" + (" " * (indent - 4)) + "}"
    elif isinstance(d, list):
        if not d: return "nlohmann::json::array()"
        items = [f'{ind}{dict_to_cpp_init_list(v, indent + 4)}' for v in d]
        return "{\n" + ",\n".join(items) + "\n" + (" " * (indent - 4)) + "}"
    elif isinstance(d, str):
        return '"' + d.replace('"', '\\"') + '"'
    elif isinstance(d, bool):
        return "true" if d else "false"
    elif isinstance(d, (int, float)):
        return str(d)
    return '""'

def generate_providers():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if os.path.exists(os.path.join(script_dir, '../../../src')):
        base_path = os.path.abspath(os.path.join(script_dir, '../../../'))
    else:
        base_path = os.getcwd()
        
    binding_path = os.path.join(base_path, 'src/binding/json/Binding.hpp')
    if not os.path.exists(binding_path): return

    dsc_desc = get_dsc_descriptions(base_path)
    bindings = parse_binding_hpp(binding_path)
    struct_member_types, header_docs = parse_cpp_headers(base_path, binding_path)
    
    out_cpp = "#pragma once\n\n#include <nlohmann/json.hpp>\n#include \"src/binding/json/Binding.hpp\"\n#include \"src/binding/json/SchemaGenerator.hpp\"\n\nnamespace d4 {\n"
    
    for struct_name, vars_list in bindings.items():
        props = {}
        member_types = struct_member_types.get(struct_name, {})
        
        for var in vars_list:
            var_kebab = camel_to_kebab(var)
            
            # Priority 1: Header Comments (Future source of truth)
            # Priority 2: DSC files (Legacy source of truth)
            matched_desc = header_docs.get((struct_name, var)) or None
            if not matched_desc:
                for k, v in dsc_desc.items():
                    if k == var_kebab or k.endswith('-' + var_kebab):
                        matched_desc = v
                        break
                    
            injections = {}
            if matched_desc: injections["description"] = matched_desc
                
            cpp_type = member_types.get(var)
            if cpp_type:
                injections.update(get_json_schema_info(cpp_type))
            else:
                if 'Method' in var or 'Type' in var or 'Strategy' in var or 'Name' in var or 'Manager' in var:
                    injections["type"] = ["integer", "string"]
                else:
                    injections["type"] = "object"
                
            props[var] = injections
                
        if props:
            schema_dict = {"title": struct_name, "properties": props}
            cpp_init_list = dict_to_cpp_init_list(schema_dict, 16)
            out_cpp += f"\n    template<>\n    struct SchemaProvider<{struct_name}> {{\n        static nlohmann::json get() {{\n            return {cpp_init_list};\n        }}\n    }};\n"
            
    out_cpp += "\n} // namespace d4\n"
    output_path = os.path.join(base_path, 'src/binding/json/SchemaProviders.hpp')
    with open(output_path, 'w') as f:
        f.write(out_cpp)
    print(f"Generated {output_path}")

if __name__ == "__main__":
    generate_providers()
