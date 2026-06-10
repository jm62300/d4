/*
 * Convenience forwarding header so that d4 sources can simply write
 * #include "3rdParty/htd/main.hpp" (the repository root is on the include
 * path). The htd headers include each other as <htd/...>, which resolves
 * through 3rdParty/htd/include and 3rdParty/htdGlue/include.
 */

#pragma once

#include <htd/main.hpp>
