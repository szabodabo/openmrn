#include "openlcb/If.hxx"

// Per NMRA Standard S-9.7.0.3, every NMRA member gets
// 8 bits of address space: 03:00:AA:BB:CC:XX
// (NMRA member number AABBCC), node number XX.
#define NMRA_NUM_ID_PREFIX 0x030016620800ULL

#ifndef NODE_NUMBER
#error "FATAL: NODE_NUMBER was not set by build system!"
#endif

#if NODE_NUMBER > 250 || NODE_NUMBER < 0
#error "Node number won't fit into address space!"
#endif

extern const openlcb::NodeID NODE_ID;
const openlcb::NodeID NODE_ID = NMRA_NUM_ID_PREFIX | NODE_NUMBER;
