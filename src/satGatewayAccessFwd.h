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

#ifndef __ADDRESSING_TEST_SATGATEWAYACCESSFWD_H_
#define __ADDRESSING_TEST_SATGATEWAYACCESSFWD_H_

#include <omnetpp.h>
#include "satFrameFwd_m.h"
#include "inet/networklayer/ipv4/IPv4Datagram.h"
#include "inet/networklayer/ipv6/IPv6Datagram.h"
#include "inet/networklayer/common/L3Address.h"

//using namespace omnetpp;

/**
 * TODO - Generated class
 */
class SatGatewayAccessFwd : public cSimpleModule
{
private:
    int     nb_sat_terminals;

  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

  public:
    inet::L3Address *IPAddDTB;
    virtual void    registerIPAddress(inet::L3Address, int terminal_index);
};

#endif
