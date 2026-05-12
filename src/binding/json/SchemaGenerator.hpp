#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <ostream>

namespace d4 {

    /**
     * @brief Specialize this trait for your configuration structures to provide
     * manual documentation, titles, or type overrides (such as Enums) per struct.
     * 
     * Example:
     * template<>
     * struct SchemaProvider<OptionsDpll> {
     *     static nlohmann::json get() {
     *         return {
     *             {"type", "object"},
     *             {"title", "OptionsDpll"},
     *             {"properties", {
     *                 {"max_conflicts", {
     *                     {"description", "Nombre max de conflits."}
     *                 }}
     *             }}
     *         };
     *     }
     * };
     */
    template<typename ConfigStruct>
    struct SchemaProvider {
        static nlohmann::json get() {
            return nlohmann::json::object();
        }
    };

    /**
     * @brief Helper for recursive schema generation allowing user schemas
     * and custom overrides to be merged with automatically inferred structs.
     */
    inline nlohmann::json _generate_schema_recursive(const nlohmann::json& default_instance, const nlohmann::json& manual_schema) {
        nlohmann::json schema;
        schema["type"] = "object";
        schema["additionalProperties"] = false;
        schema["properties"] = nlohmann::json::object();

        // Use manual schema title/etc. if provided:
        if (manual_schema.is_object()) {
            for (auto& [k, v] : manual_schema.items()) {
                if (k != "properties") {
                    schema[k] = v;
                }
            }
        }

        nlohmann::json manual_props = nlohmann::json::object();
        if (manual_schema.is_object() && manual_schema.contains("properties")) {
            manual_props = manual_schema["properties"];
        }

        for (auto& [key, val] : default_instance.items()) {
            nlohmann::json prop = nlohmann::json::object();
            nlohmann::json manual_prop = nlohmann::json::object();

            if (manual_props.contains(key)) {
                manual_prop = manual_props[key];
            }

            if (val.is_object()) {
                prop = _generate_schema_recursive(val, manual_prop);
            } else {
                if (val.is_number_integer()) prop["type"] = "integer";
                else if (val.is_number_float()) prop["type"] = "number";
                else if (val.is_boolean()) prop["type"] = "boolean";
                else if (val.is_string()) prop["type"] = "string";
                else if (val.is_array()) prop["type"] = "array";
                
                // Inject EXACT default value straight from C++ memory
                prop["default"] = val;

                // Merge in any custom json-schema properties (e.g., enum, description, default override)
                if (manual_prop.is_object()) {
                    for (auto& [inj_key, inj_val] : manual_prop.items()) {
                        prop[inj_key] = inj_val;
                    }
                }
            }
            
            schema["properties"][key] = prop;
        }
        
        // Also add any properties present in manual_props that were NOT in default_instance
        for (auto& [key, val] : manual_props.items()) {
            if (!schema["properties"].contains(key)) {
                schema["properties"][key] = val;
            }
        }
        
        return schema;
    }

    /**
     * @brief Generates a JSON Schema dynamically from a C++ struct definition.
     * Overrides and documentation are natively fetched if the developer specialized
     * the SchemaProvider trait for the ConfigStruct.
     * 
     * @tparam ConfigStruct A structure mapped with NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE
     * @return nlohmann::json The Draft-07 JSON Schema.
     */
    template<typename ConfigStruct>
    nlohmann::json generate_schema() {
        ConfigStruct default_obj;
        nlohmann::json base_j = default_obj;
        
        nlohmann::json manual_schema = SchemaProvider<ConfigStruct>::get();
        nlohmann::json schema = _generate_schema_recursive(base_j, manual_schema);
        schema["$schema"] = "http://json-schema.org/draft-07/schema#";
        return schema;
    }

    /**
     * @brief Formats a JSON Schema into a human-readable tree.
     */
    inline void to_pretty_tree(const nlohmann::json& schema, std::ostream& out, const std::string& prefix = "") {
        if (!schema.contains("properties")) return;

        auto& props = schema["properties"];
        size_t count = 0;
        for (auto it = props.begin(); it != props.end(); ++it, ++count) {
            bool is_last = (count == props.size() - 1);
            std::string name = it.key();
            auto& prop = it.value();

            out << prefix << (is_last ? "└── " : "├── ") << "\033[1;34m" << name << "\033[0m";

            if (prop.contains("type")) {
                auto& type = prop["type"];
                if (type.is_array()) {
                    out << " (";
                    for (size_t i = 0; i < type.size(); ++i) {
                        out << type[i].get<std::string>() << (i == type.size() - 1 ? "" : "|");
                    }
                    out << ")";
                } else if (type.is_string()) {
                    out << " (" << type.get<std::string>() << ")";
                }
            }

            if (prop.contains("default")) {
                out << " [default: \033[1;32m" << prop["default"].dump() << "\033[0m]";
            }

            if (prop.contains("description")) {
                out << " : " << prop["description"].get<std::string>();
            }

            out << "\n";

            if (prop.contains("properties")) {
                to_pretty_tree(prop, out, prefix + (is_last ? "    " : "│   "));
            }
        }
    }

} // namespace d4
