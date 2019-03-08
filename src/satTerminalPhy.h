//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __ADDRESSING_TEST_SATTERMINALPHY_H_
#define __ADDRESSING_TEST_SATTERMINALPHY_H_

#include <omnetpp.h>
#include "satGatewayAccessFwd.h"
#include "inet/networklayer/ipv4/IPv4Datagram.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "satFrameRtn_m.h"
#include "CRDSA.h"
#include <algorithm>

//using namespace omnetpp;

/**
 * TODO - Generated class
 */
class SatTerminalPhy : public cSimpleModule
{
  protected:
    // FSM and its states
    cFSM fsm;
    enum {
      INIT = 0,
      IDLE = FSM_Steady(1),
      LOCAL_IN = FSM_Transient(1),
      REMOTE_IN = FSM_Transient(2),
    };

    // variables used
    int i;
    cPacket *incomingFrame;

    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

  private:

    void allocateFrame(int frameIndex);
    bool existsIn(int value, std::vector<std::vector<int> > set);

    int     remoteInGateId;
    cModule *satGatewayModule;
    double propagationDelay;
    double slotDuration;
    int slotsPerFrame;
    int numberOfReplicas;
    int currentFrameIndex;
    int maxNumberOfUniquePacketsPerFrame;
    CRDSA* mCRDSA;
    cPacketQueue* mPacketQueue;

    // Results
    int totalNumberOfPacketsSend;
};

#endif
