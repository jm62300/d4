#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <grpcpp/grpcpp.h>
#include "d4.grpc.pb.h"
#include "src/options/methods/OptionDpllStyleMethod.hpp"
#include "counter/src/OptionCounter.hpp"
#include "src/preproc/PreprocManager.hpp"
#include <optree/Option.hpp>
#include "OptionProjMc.hpp"


using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using d4::D4Solver;
using d4::CountRequest;
using d4::CountReply;
using d4::HelpRequest;
using d4::HelpReply;

class D4Client {
 public:
  D4Client(std::shared_ptr<Channel> channel)
      : stub_(D4Solver::NewStub(channel)) {}

  void GetHelp(const std::vector<std::string>& arguments = {}) {
    HelpRequest request;
    for (const auto& arg : arguments) request.add_arguments(arg);
    HelpReply reply;
    ClientContext context;

    std::cout << "[gRPC Client] Requesting available solver options..." << std::endl;
    Status status = stub_->GetHelp(&context, request, &reply);

    if (status.ok()) {
      std::cout << reply.help_text() << std::endl;
    } else {
      std::cout << "[gRPC Client Error] " << status.error_code() << ": " << status.error_message() << std::endl;
    }
  }

  void CountModels(const std::string& cnf_formula, const std::vector<std::string>& arguments) {
    CountRequest request;
    request.set_cnf_formula(cnf_formula);
    for (const auto& arg : arguments) {
      request.add_arguments(arg);
    }

    CountReply reply;
    ClientContext context;

    std::cout << "[gRPC Client] Sending request to D4 solver..." << std::endl;
    Status status = stub_->CountModels(&context, request, &reply);

    if (status.ok()) {
      std::cout << "[gRPC Client] Call successful!" << std::endl;
      std::cout << "--- Status: " << reply.status() << " ---" << std::endl;
      std::cout << "--- Model Count: " << reply.model_count() << " ---" << std::endl;
      std::cout << "--- Elapsed Time: " << reply.elapsed_time_seconds() << "s ---" << std::endl;
      std::cout << "--- Solver Logs ---" << std::endl;
      std::cout << reply.logs() << std::endl;
      std::cout << "-------------------" << std::endl;
    } else {
      std::cout << "[gRPC Client Error] " << status.error_code() << ": " << status.error_message() << std::endl;
    }
  }

 private:
  std::unique_ptr<D4Solver::Stub> stub_;
};

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
  d4::OptionCounter optionCounter;
  d4::OptionRegistry registry;
  options.registerTo(registry);
  optionPreproc.registerTo(registry);
  optionCounter.registerTo(registry);

  optree::Option<std::string> hostOpt("host", "Specify host for the gRPC server", "localhost");
  optree::Option<int> portOpt("port", "Specify port for the gRPC server", 50051);
  optree::Option<std::string> inputOpt("input", "Specify path to the input DIMACS or circuit file", "");
  d4::OptionProjMc projMcOpts;
  
  hostOpt.registerTo(registry);
  portOpt.registerTo(registry);
  inputOpt.registerTo(registry);
  projMcOpts.registerTo(registry);

  // Map -i to --input for registry parsing compatibility
  std::vector<std::string> parse_args;
  for (int i = 0; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-i") {
      parse_args.push_back("--input");
    } else {
      parse_args.push_back(arg);
    }
  }

  std::vector<char*> parse_argv;
  for (auto& a : parse_args) {
    parse_argv.push_back(const_cast<char*>(a.data()));
  }
  parse_argv.push_back(nullptr);


  try {
    registry.parseArgv(parse_argv.size() - 1, parse_argv.data());
  } catch (const std::exception& e) {
    if (!showHelp) {
      std::cerr << "Error parsing arguments: " << e.what() << "\n";
      return 1;
    }
  }

  // Filter out client-side options to forward only solver options to server
  std::vector<std::string> solver_args;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--host" || arg == "--port" || arg == "--input" || arg == "-i") {
      if (i + 1 < argc) ++i;
    } else if (arg.rfind("--host=", 0) == 0 || arg.rfind("--port=", 0) == 0 || arg.rfind("--input=", 0) == 0) {
      // skip
    } else if (arg == "-h" || arg == "--help" || arg == "--completion-script" || arg == "--dump-options") {
      // skip
    } else {
      solver_args.push_back(arg);
    }
  }

  std::string target_str = hostOpt.get() + ":" + std::to_string(portOpt.get());

  if (showHelp) {
    std::cout << "Usage: " << argv[0] << " [options]\n"
              << "  -h, --help            Show this help screen\n"
              << "  --host HOST           Specify host for the gRPC server (default: localhost)\n"
              << "  --port PORT           Specify port for the gRPC server (default: 50051)\n"
              << "  -i, --input FILE      Specify path to the input DIMACS or circuit file\n\n"
              << "[gRPC Client] Querying help from server at " << target_str << "...\n";
    D4Client client(grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));
    client.GetHelp(solver_args);
    return 0;
  }

  std::cout << "[gRPC Client] Connecting to " << target_str << "..." << std::endl;
  D4Client client(grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));

  std::string formula_data;
  std::string inputPath = inputOpt.get();
  if (!inputPath.empty()) {
    std::ifstream infile(inputPath);
    if (!infile.is_open()) {
      std::cerr << "Error: Could not open input file: " << inputPath << "\n";
      return 1;
    }
    std::stringstream buffer;
    buffer << infile.rdbuf();
    formula_data = buffer.str();
  } else {
    // Default fallback CNF
    formula_data =
        "p cnf 3 1\n"
        "1 2 3 0\n";
  }

  client.CountModels(formula_data, solver_args);

  return 0;
}

