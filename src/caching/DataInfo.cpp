#include "DataInfo.hpp"

namespace d4
{
/**
   Constructor.
 */
DataInfo::DataInfo()
{
  info1 = info2 = 0;
  stats = {0,0};
} // constructor

/**
   Constructor.

   We always at least have the following distribution:
   For info1 => |free (43 bytes)|nb var (21 bytes)|
   For info2 => |szData (26 bytes)|nb octets data (2 bytes)|nb octets var (2 bytes)| free (2 bytes)|

   @param[in] i1, the information place 1.
   @param[in] i2, the information place 2.
   @param[in] count, the counter value for initialization.
*/
DataInfo::DataInfo(uint64_t i1, uint32_t i2, unsigned count)
{
  info1 = i1;
  info2 = i2;
  stats = {count, 0};
} // constructor

}
