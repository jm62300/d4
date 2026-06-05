#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <grpcpp/grpcpp.h>
#include "d4.grpc.pb.h"

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
  std::string host = "localhost";
  int port = 50051;
  bool showHelp = false;
  std::vector<std::string> solver_args;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      showHelp = true;
    } else if ((arg == "--host") && i + 1 < argc) {
      host = argv[++i];
    } else if ((arg == "-p" || arg == "--port") && i + 1 < argc) {
      port = std::stoi(argv[++i]);
    } else {
      // Remaining args are forwarded as solver options (e.g. --solver=glucose)
      solver_args.push_back(arg);
    }
  }

  std::string target_str = host + ":" + std::to_string(port);
  std::cout << "[gRPC Client] Connecting to " << target_str << "..." << std::endl;
  D4Client client(grpc::CreateChannel(
      target_str, grpc::InsecureChannelCredentials()));

  // --help : query the server for available options (with current values) and exit
  if (showHelp) {
    client.GetHelp(solver_args);
    return 0;
  }

  // Default: run a demo model count query
  // Formula: 3 variables, 1 clause (1 OR 2 OR 3) → 7 models
  std::string cnf_formula =
      "p cnf 3 1\n"
      "1 2 3 0\n";

  if (solver_args.empty())
    solver_args.push_back("--solver=glucose");

  client.CountModels(cnf_formula, solver_args);

  return 0;
}

