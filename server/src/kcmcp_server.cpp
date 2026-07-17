#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <chrono>
#include <cstdio>
#include <vector>
#include <map>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <nlohmann/json.hpp>

// D4 Solver headers
#include "src/problem/ProblemTypes.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/preproc/PreprocManager.hpp"
#include "ParserDimacs.hpp"
#include "api/solver/Solver.hpp"
#include <optree/Option.hpp>

using json = nlohmann::json;

// Helper to read exactly 'len' bytes
bool read_all(int fd, char* buf, size_t len) {
    size_t total = 0;
    while (total < len) {
        ssize_t n = read(fd, buf + total, len - total);
        if (n <= 0) {
            if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            return false;
        }
        total += n;
    }
    return true;
}

// Helper to write exactly 'len' bytes
bool write_all(int fd, const char* buf, size_t len) {
    size_t total = 0;
    while (total < len) {
        ssize_t n = write(fd, buf + total, len - total);
        if (n <= 0) {
            if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            return false;
        }
        total += n;
    }
    return true;
}

// Read a full KCMCP payload, combining chunks if the MORE flag is set
bool read_full_payload(int fd, uint8_t& out_type, uint32_t& out_request_id, std::string& out_payload) {
    out_payload.clear();
    bool more = true;
    uint8_t first_type = 0;
    uint32_t first_request_id = 0;
    bool is_first = true;

    while (more) {
        char header[10];
        if (!read_all(fd, header, 10)) return false;

        uint8_t type = header[0];
        uint8_t flags = header[1];
        uint32_t request_id = (static_cast<uint8_t>(header[2]) << 24) |
                              (static_cast<uint8_t>(header[3]) << 16) |
                              (static_cast<uint8_t>(header[4]) << 8) |
                              static_cast<uint8_t>(header[5]);
        uint32_t payload_len = (static_cast<uint8_t>(header[6]) << 24) |
                               (static_cast<uint8_t>(header[7]) << 16) |
                               (static_cast<uint8_t>(header[8]) << 8) |
                               static_cast<uint8_t>(header[9]);

        std::vector<char> chunk(payload_len);
        if (payload_len > 0) {
            if (!read_all(fd, chunk.data(), payload_len)) return false;
        }

        if (is_first) {
            first_type = type;
            first_request_id = request_id;
            is_first = false;
        } else {
            if (type != first_type || request_id != first_request_id) {
                return false;
            }
        }

        out_payload.append(chunk.data(), payload_len);
        more = (flags & 0x01) != 0;
    }

    out_type = first_type;
    out_request_id = first_request_id;
    return true;
}

// Send a full payload, chunking it if it exceeds max_payload (1 MiB)
void send_frame(int fd, uint8_t type, uint8_t flags, uint32_t request_id, const std::string& payload) {
    size_t total_len = payload.size();
    size_t chunk_limit = 1048576; // 1 MiB floor
    size_t offset = 0;

    do {
        size_t size = std::min(total_len - offset, chunk_limit);
        uint8_t chunk_flags = flags;
        if (offset + size < total_len) {
            chunk_flags |= 0x01; // set MORE flag
        }

        char header[10];
        header[0] = type;
        header[1] = chunk_flags;
        header[2] = (request_id >> 24) & 0xFF;
        header[3] = (request_id >> 16) & 0xFF;
        header[4] = (request_id >> 8) & 0xFF;
        header[5] = request_id & 0xFF;
        header[6] = (size >> 24) & 0xFF;
        header[7] = (size >> 16) & 0xFF;
        header[8] = (size >> 8) & 0xFF;
        header[9] = size & 0xFF;

        write_all(fd, header, 10);
        if (size > 0) {
            write_all(fd, payload.data() + offset, size);
        }

        offset += size;
    } while (offset < total_len);
}

// Send an error frame
void send_error(int fd, uint32_t request_id, uint16_t code, const std::string& message) {
    std::string payload;
    payload.push_back((code >> 8) & 0xFF);
    payload.push_back(code & 0xFF);
    payload.append(message);
    send_frame(fd, 0x03, 0x00, request_id, payload);
}

// Map custom options from JSON to OptionRegistry args
std::vector<std::string> map_d4_options(const json& d4_opts) {
    std::vector<std::string> args;
    for (auto& [key, val] : d4_opts.items()) {
        std::string arg_name = "";
        if (key == "solver") arg_name = "--dpll.solver.solverName";
        else if (key == "branching-heuristic") arg_name = "--dpll.branching.branchingHeuristicType";
        else if (key == "scoring-method") arg_name = "--dpll.branching.scoringMethodType";
        else if (key == "phase-heuristic") arg_name = "--dpll.branching.phaseHeuristicType";
        else if (key == "cache-method") arg_name = "--dpll.cache.cachingMethod";
        else if (key == "float") arg_name = "--dpll.isFloat";
        else if (key == "preproc") arg_name = "--preproc.preproc-method";

        if (!arg_name.empty()) {
            std::string arg_val = "";
            if (val.is_boolean()) {
                arg_val = val.get<bool>() ? "true" : "false";
            } else if (val.is_string()) {
                arg_val = val.get<std::string>();
            } else if (val.is_number()) {
                arg_val = std::to_string(val.get<int>());
            }
            args.push_back(arg_name + "=" + arg_val);
        }
    }
    return args;
}

void handle_client(int csocket) {
    std::cout << "[KCMCP Server] Client connected." << std::endl;

    // 1. Handshake HELLO
    uint8_t type = 0;
    uint32_t request_id = 0;
    std::string payload;

    if (!read_full_payload(csocket, type, request_id, payload)) {
        std::cerr << "Failed to read HELLO handshake." << std::endl;
        close(csocket);
        return;
    }

    if (type != 0x00) {
        send_error(csocket, 0, 8, "Expected HELLO handshake frame.");
        close(csocket);
        return;
    }

    try {
        json client_hello = json::parse(payload);
        if (!client_hello.contains("kcmcp") || !client_hello["kcmcp"].is_array() || client_hello["kcmcp"].size() < 2) {
            send_error(csocket, 0, 8, "Invalid HELLO JSON schema.");
            close(csocket);
            return;
        }
        int major = client_hello["kcmcp"][0].get<int>();
        if (major != 1) {
            send_error(csocket, 0, 8, "Unsupported protocol version (requires v1.x).");
            close(csocket);
            return;
        }
    } catch (const std::exception& e) {
        send_error(csocket, 0, 8, std::string("Malformed HELLO JSON: ") + e.what());
        close(csocket);
        return;
    }

    // Send Server HELLO
    json server_hello = {
        {"kcmcp", 1},
        {"engine", "d4/kcmcp-1.0"},
        {"max_payload", 1048576},
        {"operations", {"count", "wmc", "compile"}},
        {"input_formats", {"dimacs-cnf"}},
        {"output_formats", {
            {"count", {"decimal", "bigint"}},
            {"wmc", {"decimal"}},
            {"compile", {"ddnnf-nnf"}}
        }},
        {"features", {"persistent-cache"}} // cancel and progress are omitted for synchronous simplicity
    };
    send_frame(csocket, 0x00, 0x00, 0, server_hello.dump());

    // 2. Main Request loop
    while (true) {
        if (!read_full_payload(csocket, type, request_id, payload)) {
            std::cout << "[KCMCP Server] Client disconnected." << std::endl;
            break;
        }

        if (type == 0x08) { // BYE
            std::cout << "[KCMCP Server] Received BYE. Closing connection." << std::endl;
            send_frame(csocket, 0x08, 0x00, 0, "");
            break;
        }

        if (type == 0x06) { // PING
            send_frame(csocket, 0x07, 0x00, 0, ""); // PONG
            continue;
        }

        if (type != 0x01) { // Not a REQUEST
            send_error(csocket, request_id, 1, "Unsupported frame type.");
            continue;
        }

        // Parse REQUEST layout
        if (payload.size() < 6) {
            send_error(csocket, request_id, 3, "Request payload too short.");
            continue;
        }

        uint8_t operation = payload[0];
        uint8_t input_format = payload[1];
        uint8_t output_format = payload[2];
        uint16_t options_len = (static_cast<uint8_t>(payload[4]) << 8) | static_cast<uint8_t>(payload[5]);

        if (payload.size() < 6 + options_len) {
            send_error(csocket, request_id, 3, "Request options length mismatch.");
            continue;
        }

        if (input_format != 0) { // only dimacs-cnf is supported
            send_error(csocket, request_id, 2, "Unsupported input format.");
            continue;
        }

        std::string options_str = payload.substr(6, options_len);
        std::string problem_str = payload.substr(6 + options_len);

        json opts_json = json::object();
        if (options_len > 0) {
            try {
                opts_json = json::parse(options_str);
            } catch (const std::exception& e) {
                send_error(csocket, request_id, 3, std::string("Malformed options JSON: ") + e.what());
                continue;
            }
        }

        // Initialize Options & Registry
        d4::OptionDpllStyleMethod options;
        bipe::OptionPreproc optionPreproc;
        d4::OptionRegistry registry;
        options.registerTo(registry);
        optionPreproc.registerTo(registry);

        std::vector<std::string> solver_cli_args = {"d4_kcmcp_server"};
        if (opts_json.contains("d4") && opts_json["d4"].is_object()) {
            auto mapped = map_d4_options(opts_json["d4"]);
            solver_cli_args.insert(solver_cli_args.end(), mapped.begin(), mapped.end());
        }

        // Apply options via parseArgv
        if (solver_cli_args.size() > 1) {
            std::vector<char*> fake_argv;
            for (auto& arg : solver_cli_args) fake_argv.push_back(const_cast<char*>(arg.data()));
            fake_argv.push_back(nullptr);
            try {
                registry.parseArgv(fake_argv.size() - 1, fake_argv.data());
            } catch (const std::exception& e) {
                send_error(csocket, request_id, 3, std::string("Error configuring solver: ") + e.what());
                continue;
            }
        }

        // Parse Formula directly in-memory (no disk I/O)
        parser::Formula formula;
        parser::ParserDimacs parserDimacs;
        try {
            parserDimacs.parse_DIMACS_from_data(problem_str, formula);
        } catch (const std::exception& e) {
            send_error(csocket, request_id, 3, std::string("CNF Parse error: ") + e.what());
            continue;
        }

        // Apply "projset" options to quantifications
        if (opts_json.contains("projset") && opts_json["projset"].is_array()) {
            std::vector<int> proj;
            for (auto& item : opts_json["projset"]) {
                if (item.is_number()) proj.push_back(item.get<int>());
            }
            formula.quantifications.clear();
            formula.quantifications.push_back(proj);
            formula.type = "p cnf";
        }

        // Apply "weights" options to weightMap
        if (opts_json.contains("weights") && opts_json["weights"].is_object()) {
            for (auto& [key, val] : opts_json["weights"].items()) {
                int lit = std::stoi(key);
                std::string w_str = "";
                if (val.is_number()) w_str = std::to_string(val.get<double>());
                else if (val.is_string()) w_str = val.get<std::string>();
                if (!w_str.empty()) {
                    formula.weightMap[lit] = w_str;
                }
            }
            formula.weightType = parser::WeightType::FLOAT;
        }

        // Setup Solver
        std::stringstream logs;
        d4::api::Solver solver(formula.clauses, formula.nbVar, options);
        d4::api::WeightType wt = d4::api::WeightType::INT;
        if (formula.weightType == parser::WeightType::FLOAT) {
            wt = d4::api::WeightType::FLOAT;
        } else if (formula.weightType == parser::WeightType::COMPLEX) {
            wt = d4::api::WeightType::COMPLEX;
        }
        solver.setWeights(formula.weightMap, wt);
        auto start_time = std::chrono::high_resolution_clock::now();

        if (operation == 0 || operation == 1) { // count or wmc
            if (operation == 0 && output_format != 0 && output_format != 3) {
                send_error(csocket, request_id, 2, "Unsupported count output format (only decimal or bigint).");
                continue;
            }
            if (operation == 0 && output_format == 3 && formula.weightType != parser::WeightType::INT) {
                send_error(csocket, request_id, 2, "Bigint output format is only supported for unweighted formulas.");
                continue;
            }
            if (operation == 1 && output_format != 0) {
                send_error(csocket, request_id, 2, "Unsupported wmc output format (only decimal).");
                continue;
            }

            try {
                auto result = solver.count(logs);
                auto end_time = std::chrono::high_resolution_clock::now();
                int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                json meta = {
                    {"time_ms", elapsed_ms},
                    {"exact", true}
                };
                std::string meta_str = meta.dump();

                std::string result_data = "";
                if (output_format == 0) { // decimal (text string)
                    result_data = result->getResult();
                } else if (output_format == 3) { // bigint (raw big-endian bytes)
                    boost::multiprecision::mpz_int int_res = result->getIntResult();
                    if (int_res == 0) {
                        result_data.push_back(0x00);
                    } else {
                        size_t count = 0;
                        size_t num_bits = mpz_sizeinbase(int_res.backend().data(), 2);
                        size_t max_bytes = (num_bits + 7) / 8;
                        std::vector<char> buffer(max_bytes);
                        mpz_export(buffer.data(), &count, 1, 1, 1, 0, int_res.backend().data());
                        if (count > 0) {
                            result_data.assign(buffer.data(), count);
                        } else {
                            result_data.push_back(0x00);
                        }
                    }
                }

                // Construct RESULT payload
                std::string response_payload;
                response_payload.push_back(output_format);
                response_payload.push_back(0x00); // reserved
                response_payload.push_back((meta_str.size() >> 8) & 0xFF);
                response_payload.push_back(meta_str.size() & 0xFF);
                response_payload.append(meta_str);
                response_payload.append(result_data);

                send_frame(csocket, 0x02, 0x00, request_id, response_payload);
            } catch (const std::exception& e) {
                send_error(csocket, request_id, 6, std::string("Solver counting error: ") + e.what());
            }
        } else if (operation == 2) { // compile
            if (output_format != 4) { // only ddnnf-nnf is supported
                send_error(csocket, request_id, 2, "Unsupported compilation output format (only ddnnf-nnf).");
                continue;
            }

            try {
                auto result = solver.compile(logs);
                auto end_time = std::chrono::high_resolution_clock::now();
                int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                json meta = {
                    {"time_ms", elapsed_ms},
                    {"nodes", result->getNbNodes()},
                    {"edges", result->getNbEdges()},
                    {"exact", true}
                };
                std::string meta_str = meta.dump();
                std::string result_data = result->getNNFString();

                // Construct RESULT payload
                std::string response_payload;
                response_payload.push_back(output_format);
                response_payload.push_back(0x00); // reserved
                response_payload.push_back((meta_str.size() >> 8) & 0xFF);
                response_payload.push_back(meta_str.size() & 0xFF);
                response_payload.append(meta_str);
                response_payload.append(result_data);

                send_frame(csocket, 0x02, 0x00, request_id, response_payload);
            } catch (const std::exception& e) {
                send_error(csocket, request_id, 6, std::string("Solver compilation error: ") + e.what());
            }
        } else {
            send_error(csocket, request_id, 1, "Unsupported operation.");
        }
    }

    close(csocket);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <endpoint>\n"
                  << "  Endpoint can be a Unix socket (unix:/path/to/socket.sock) or TCP port (e.g. 50055)\n";
        return 1;
    }

    std::string endpoint = argv[1];
    int server_fd = -1;
    bool is_unix = false;
    std::string unix_path = "";

    if (endpoint.rfind("unix:", 0) == 0) {
        is_unix = true;
        unix_path = endpoint.substr(5);
        server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (server_fd < 0) {
            perror("socket(AF_UNIX) failed");
            return 1;
        }

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, unix_path.c_str(), sizeof(addr.sun_path) - 1);
        unlink(unix_path.c_str());

        if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("bind(AF_UNIX) failed");
            close(server_fd);
            return 1;
        }
        std::cout << "[KCMCP Server] Bound to Unix socket: " << unix_path << std::endl;
    } else {
        // TCP socket
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            perror("socket(AF_INET) failed");
            return 1;
        }

        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;

        int port = 50055;
        size_t colon = endpoint.find(':');
        if (colon != std::string::npos) {
            std::string host = endpoint.substr(0, colon);
            port = std::stoi(endpoint.substr(colon + 1));
            inet_pton(AF_INET, host.c_str(), &addr.sin_addr);
        } else {
            port = std::stoi(endpoint);
        }
        addr.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            perror("bind(AF_INET) failed");
            close(server_fd);
            return 1;
        }
        std::cout << "[KCMCP Server] Bound to TCP port: " << port << std::endl;
    }

    if (listen(server_fd, 5) < 0) {
        perror("listen failed");
        close(server_fd);
        return 1;
    }

    while (true) {
        int csocket = accept(server_fd, nullptr, nullptr);
        if (csocket < 0) {
            perror("accept failed");
            continue;
        }
        handle_client(csocket);
    }

    close(server_fd);
    if (is_unix) {
        unlink(unix_path.c_str());
    }
    return 0;
}
