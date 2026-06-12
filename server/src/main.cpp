#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <chrono>
#include <cstdio>
#include <vector>
#include <iomanip>

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include "d4.grpc.pb.h"

// d4 Solver and Semiring headers
#include "src/problem/ProblemTypes.hpp"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "src/preproc/PreprocManager.hpp"
#include "ParserDimacs.hpp"
#include "api/solver/Solver.hpp"
#include <optree/Option.hpp>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using d4::D4Solver;
using d4::CountRequest;
using d4::CountReply;
using d4::HelpRequest;
using d4::HelpReply;

class D4SolverServiceImpl final : public D4Solver::Service {
  Status GetHelp(ServerContext* context, const HelpRequest* request,
                 HelpReply* reply) override {
    d4::OptionDpllStyleMethod options;
    bipe::OptionPreproc optionPreproc;
    d4::OptionRegistry registry;
    options.registerTo(registry);
    optionPreproc.registerTo(registry);

    // Apply any arguments sent by the client so current values are accurate
    if (request->arguments_size() > 0) {
      std::vector<std::string> args = {"d4_grpc_server"};
      for (const auto& arg : request->arguments()) args.push_back(arg);
      std::vector<char*> argv;
      for (auto& a : args) argv.push_back(const_cast<char*>(a.data()));
      argv.push_back(nullptr);
      try {
        registry.parseArgv(static_cast<int>(argv.size()) - 1, argv.data());
      } catch (const std::exception& e) {
        reply->set_help_text(std::string("Error parsing arguments: ") + e.what());
        return Status::OK;
      }
    }

    std::stringstream ss;
    ss << "D4 gRPC Server Solver Options:\n";
    registry.displayHelp(ss);
    reply->set_help_text(ss.str());
    return Status::OK;
  }

  Status CountModels(ServerContext* context, const CountRequest* request,
                     CountReply* reply) override {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::stringstream logs_stream;
    
    logs_stream << "c [gRPC] Starting CountModels request...\n";
    
    // 1. Write the CNF formula to a temporary file
    std::string temp_filename = "temp_formula_" + std::to_string(rand()) + ".cnf";
    std::ofstream temp_file(temp_filename);
    if (!temp_file.is_open()) {
      reply->set_status(CountReply::ERROR);
      reply->set_logs("Error: Failed to open temp file for writing CNF formula.");
      return Status::OK;
    }
    temp_file << request->cnf_formula();
    temp_file.close();
    
    // 2. Parse the DIMACS file
    parser::Formula formula;
    parser::ParserDimacs parserDimacs;
    try {
      parserDimacs.parse_DIMACS(temp_filename, formula);
    } catch (const std::exception& e) {
      std::remove(temp_filename.c_str());
      reply->set_status(CountReply::ERROR);
      reply->set_logs(std::string("Error: Parsing failed: ") + e.what());
      return Status::OK;
    }
    
    // Clean up the temp file immediately
    std::remove(temp_filename.c_str());
    
    // 3. Initialize options and parse CLI arguments passed in gRPC request
    d4::OptionDpllStyleMethod options;
    bipe::OptionPreproc optionPreproc;

    d4::OptionRegistry registry;
    options.registerTo(registry);
    optionPreproc.registerTo(registry);
    
    // Map request arguments to argc/argv format
    std::vector<std::string> args = {"d4_grpc_server"};
    for (const auto& arg : request->arguments()) {
      args.push_back(arg);
    }
    std::vector<char*> argv;
    for (auto& arg : args) {
      argv.push_back(const_cast<char*>(arg.data()));
    }
    argv.push_back(nullptr);
    
    try {
      registry.parseArgv(argv.size() - 1, argv.data());
    } catch (const std::exception& e) {
      reply->set_status(CountReply::ERROR);
      reply->set_logs(std::string("Error: Parsing arguments failed: ") + e.what());
      return Status::OK;
    }
    
    // Force counting operation
    options.operationType = d4::OperationTypeManager::getOperatorType("counting");
    if (options.optionCacheManager.optionBucketManager.clauseRepresentation == d4::CACHE_INDEX) {
      options.optionSpecManager.needFastNotSatisfied = true;
    }
    
    // 4. Run the BIPE Preprocessor
    bipe::PreprocManager preprocManager;
    std::vector<int> projected;
    if (formula.quantifications.size() > 0 && formula.quantifications[0].size() > 0) {
      projected = formula.quantifications[0];
    } else {
      for (unsigned i = 1; i <= formula.nbVar; i++) {
        projected.push_back(i);
      }
    }
    
    logs_stream << "c [gRPC] Running preprocessor...\n";
    preprocManager.run(formula.nbVar, formula.clauses, projected,
                       std::vector<int>(), optionPreproc);
    
    // 5. Run solver using high-level API Solver
    std::string model_count_str = "";
    try {
      logs_stream << "c [gRPC] Starting D4 solver...\n";
      d4::api::Solver solver(formula, options);
      std::unique_ptr<d4::api::CountResult> result = solver.count(logs_stream);
      model_count_str = result->getResult();
      reply->set_status(CountReply::SATISFIABLE);
    } catch (const std::exception& e) {
      reply->set_status(CountReply::ERROR);
      logs_stream << "c [gRPC Error] Solver exception: " << e.what() << "\n";
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    
    reply->set_model_count(model_count_str);
    reply->set_logs(logs_stream.str());
    reply->set_elapsed_time_seconds(elapsed.count());
    
    return Status::OK;
  }
};

void RunServer(int port) {
  std::string server_address("0.0.0.0:" + std::to_string(port));
  D4SolverServiceImpl service;

  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "D4 gRPC Server listening on " << server_address << std::endl;
  server->Wait();
}

int main(int argc, char** argv) {
  bool showHelp = false;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      showHelp = true;
      break;
    }
  }

  d4::OptionDpllStyleMethod options;
  bipe::OptionPreproc optionPreproc;
  d4::OptionRegistry registry;
  options.registerTo(registry);
  optionPreproc.registerTo(registry);

  optree::Option<int> portOpt("port", "Specify port for the gRPC server", 50051);
  portOpt.registerTo(registry);

  if (showHelp) {
    std::cout << "Usage: " << argv[0] << " [options]\n"
              << "  -h, --help            Show this help screen\n\n"
              << "Solver & Server Options:\n";
    registry.displayHelp(std::cout);
    return 0;
  }

  try {
    registry.parseArgv(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Error parsing arguments: " << e.what() << "\n";
    return 1;
  }

  RunServer(portOpt.get());
  return 0;
}
