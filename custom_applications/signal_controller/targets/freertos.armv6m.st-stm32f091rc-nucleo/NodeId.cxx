#include "openlcb/If.hxx"

#define OPENLCB_SVL_ID_PREFIX 0x050101014700ULL

#ifndef NODE_NUMBER
#error "FATAL: NODE_NUMBER was not set by build system!"
#endif

#if NODE_NUMBER > 250 || NODE_NUMBER < 0
#error "Node number won't fit into address space!"
#endif

extern const openlcb::NodeID NODE_ID;
const openlcb::NodeID NODE_ID = OPENLCB_SVL_ID_PREFIX | NODE_NUMBER;
// const openlcb::NodeID NODE_ID = 0x050101014702ULL;
