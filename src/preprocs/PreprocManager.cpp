#include "PreprocManager.hpp"
#include "PreprocBasic.hpp"

namespace d4
{

/**
   Create the preproc manager.

   @param[in] vm, the option for the preprocessing.
 */ 
PreprocManager *PreprocManager::makePreprocManager(po::variables_map &vm)
{
  std::string meth = vm["preproc"].as<std::string>();

  if(meth == "basic") return new PreprocBasic(vm);
  return NULL;
} // makePreprocManager

}
