/** \copyright
 * Copyright (c) 2013, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file NMRAnetNode.hxx
 * This file defines NMRAnet nodes.
 *
 * @author Stuart W. Baker
 * @date 29 September 2013
 */


#ifndef _NMRAnetNode_hxx_
#define _NMRAnetNode_hxx_

#include "nmranet/NMRAnetIf.hxx"
#include "utils/RBTree.hxx"

namespace NMRAnet
{

class Node
{
public:
    /** Constructor.
     * @param node_id 48-bit unique Node ID
     * @param nmranet_if interface to bind the node to
     * @param model node decription
     */
    Node(NodeID node_id, If *nmranet_if, const char *model)
        : nodeID(node_id),
          model(model),
          userName(NULL),
          userDescription(NULL),
          state(UNINITIALIZED),
          nmranetIf(nmranet_if)
    {
        mutex.lock();
        HASSERT(idTree.find(nodeID) == NULL);
        idNode.key = nodeID;
        idNode.value = this;
        idTree.insert(&idNode);
        mutex.unlock();
    }

    /** Destructor.
     */
    ~Node()
    {
        mutex.lock();
        idTree.remove(nodeID);
        mutex.unlock();
    }

    /** Set the user name of the node for simple ident protocol.
     * @param user_name string to use for user name
     */
    void user_name(const char *user_name)
    {
        userName = user_name;
    }

    /** Set the user description of the node for simple ident protocol.
     * @param user_description string to use for user description
     */
    void user_description(const char *user_description)
    {
        userDescription = user_description;
    }

    /** Obtain a @ref Node instance from its Node ID.
     * @param node_id Node ID to lookup
     * @return @ref Node instance, else NULL if not found
     */
    static Node *find(NodeID node_id);
    
    /** Move node into the initialized state.
     */
    void initialized();
    
private:
    /** Send a verify node id number message.
     */
    void verify_id_number();

    /** Send an ident info reply message.
     * @param dst destination Node ID to respond to
     */
    void ident_info_reply(NodeHandle dst);
    
    /** Write a message from a node.
     * @param mti Message Type Indicator
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    int write(If::MTI mti, NodeHandle dst, Buffer *data);

    /** Operational states of the node */
    enum State
    {
        UNINITIALIZED = 0, /**< uninitialized node state */
        INITIALIZED /**< initialized node state */
    };

    /** Mutual exclusion for access to all nodes */
    static OSMutex mutex;
    
    /** 48-bit Node ID for this virtual node */
    NodeID nodeID;
    
    /** Model string */
    const char *model;
    
    /** User name string */
    const char *userName;
    
    /** User description string */
    const char *userDescription;
    
    /** Node's current operational state */
    State state;
    
    /** Interface that node will be bound to */
    If *nmranetIf;

    /** manufacturer string */
    static const char *manufacturer;
    
    /** hardware rev string */
    static const char *hardware_rev;
    
    /** software rev string */
    static const char *software_rev;

    /* Misc. constant expressions.
     */
    enum
    {
        SIMPLE_NODE_IDENT_VERSION_A = 0x01, /**< simple node identify prefix */
        SIMPLE_NODE_IDENT_VERSION_B = 0x01, /**< simple node identify prefix */
    };

    /** Tree of nodes sorted by node id */
    static RBTree <NodeID, Node*> idTree;
    
    /** Tree entry */
    RBTree <NodeID, Node*>::Node idNode;

    /** Default Constructor */
    Node();
    
    DISALLOW_COPY_AND_ASSIGN(Node);
    
    /** allow If class to access Node members */
    friend class If;
};

};

#endif /* _NMRAnetNode_hxx_ */

