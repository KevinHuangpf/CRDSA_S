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

#include "satGatewayAccessFwd.h"

Define_Module(SatGatewayAccessFwd);

void SatGatewayAccessFwd::initialize()
{
    // Get network parameter
    nb_sat_terminals = getModuleByPath("Ipnet")->par("numSatTerminals");

    // Initialize IP addresses database
    IPAddDTB = new inet::L3Address[nb_sat_terminals];
}

void SatGatewayAccessFwd::handleMessage(cMessage *msg)
{
    // Get incoming packet and dest address
    cPacket *L3Datagram = (cPacket *)msg;
    inet::L3Address L3Datagram_dest_add;
    if (dynamic_cast<inet::IPv4Datagram *>(msg) != NULL)
    {
        inet::IPv4Datagram *incomingIPDatagram = (inet::IPv4Datagram *)L3Datagram;
        L3Datagram_dest_add = incomingIPDatagram->getDestAddress();
    }
    else
    {
        inet::IPv6Datagram *incomingIPDatagram = (inet::IPv6Datagram *)L3Datagram;
        L3Datagram_dest_add = incomingIPDatagram->getDestAddress();
    }
    //inet::NetworkDatagramBase *incomingIPDatagram = (inet::NetworkDatagramBase*) msg;

    EV << "------------------------------satGatewayAccessFwd : receive IPv4Datagram with dest add = " << L3Datagram_dest_add << endl;

    // Create L2 frame and encapsulate
    satFrameFwd *L2Frame = new satFrameFwd;
        // Determine L2 destination address
        int terminal_index=0;
        while (IPAddDTB[terminal_index] != L3Datagram_dest_add)
            terminal_index++;
        L2Frame->setDestL2Address(terminal_index);

        // Encapsulate IPv4Datagram
        L2Frame->encapsulate(L3Datagram);
        L2Frame->setHasPayload(1);

    // Send L2 frame
    send(L2Frame, "out");
}

void    SatGatewayAccessFwd::registerIPAddress(inet::L3Address newIPAddress, int terminal_index)
{
    IPAddDTB[terminal_index] = newIPAddress;
    EV << "------------------------------satGatewayAccessFwd : register " << terminal_index << " - L3 address : " << newIPAddress << endl;
}
