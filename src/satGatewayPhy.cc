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

#include "satGatewayPhy.h"

#include <fstream>

Define_Module(SatGatewayPhy);

void SatGatewayPhy::initialize()
{
    // Determine Id of remoteIn gate
    remoteInGateId = gate("remoteIn")->getId();

    // Get network parameter
    int nb_sat_terminals = getModuleByPath("Ipnet")->par("numSatTerminals");

    // Determine satellite terminal module
    satTerminalModule = new cModule*[nb_sat_terminals];
    for (int i=0; i<nb_sat_terminals; i++)
    {
        char module_name[25];
        sprintf(module_name, "Ipnet.satTerminal[%d]", i);
        satTerminalModule[i] = getModuleByPath(module_name);
    }

    totalNumberOfPacketsSend = 0;
    totalNumberOfPacketsReceived = 0;

    propagationDelay = par("propagationDelay");
    slotDuration = par("slotDuration");
    slotsPerFrame = par("slotsPerFrame");
    numberOfReplicas = par("numberOfReplicas");

    isFrameTriggered = false;

    mCRDSA = new CRDSA(numberOfReplicas, slotsPerFrame);
}

void SatGatewayPhy::handleMessage(cMessage *msg)
{
    // Get incoming packet and forward
    cPacket* incomingFrame = (cPacket*) msg;

    // frameTrigger
    if(msg->isSelfMessage())
    {
        // The previous frame is analysed and packets that do not interfere are send
        std::vector<SatFrameRtn*> retrievedPackets = retrievePackets();
        retrievePacketsTime = simTime();

        while(retrievedPackets.size() > 0)
        {
            delaySum = delaySum + retrievePacketsTime - retrievedPackets[0]->getSendingTime();

            send(retrievedPackets[0]->decapsulate(), "localOut");
            delete retrievedPackets[0];
            retrievedPackets.erase(retrievedPackets.begin());

            totalNumberOfPacketsReceived++;
        }
        isFrameTriggered = false;
        delete msg;
    }
    // Return link
    else if(incomingFrame->getArrivalGateId() == remoteInGateId)
    {
        int creationFrame = incomingFrame->getCreationTime().dbl()/(slotDuration*slotsPerFrame)+0.0001; // TODO: refactor
        if(!isFrameTriggered)
        {
            scheduleAt((creationFrame+1)*slotDuration*slotsPerFrame+propagationDelay-0.0001, new cMessage("frameTrigger"));
            isFrameTriggered = true;
        }
        mPacketBuffer.push_back((SatFrameRtn*)incomingFrame);
        totalNumberOfPacketsSend++;
    }
    else
    {
        // Get incoming L2 Frame and determine destination
        satFrameFwd *incomingL2Frame = (satFrameFwd *) incomingFrame;
        sendDirect(incomingL2Frame->decapsulate(), propagationDelay, 0, satTerminalModule[incomingL2Frame->getDestL2Address()], "satchannelforward");
        delete incomingL2Frame;
    }
}

void SatGatewayPhy::finish()
{
    for(unsigned int i = 0; i < mPacketBuffer.size(); i++)
    {
        delete mPacketBuffer[i];
        totalNumberOfPacketsReceived--;
    }
    double slot = simTime().dbl()/slotDuration;
    double G = (totalNumberOfPacketsSend / slot) / numberOfReplicas;
    double S = totalNumberOfPacketsReceived / slot;
    delayAvg = delaySum/totalNumberOfPacketsReceived ;
    std::ofstream outputFile;
    outputFile.open("SimulationResults", std::ios::out | std::ios::app);
    outputFile << numberOfReplicas << "\t";
    std::cout << "G = " << G << std::endl;
    outputFile << G << "\t";
    std::cout << "S = " << S << std::endl;
    outputFile << S << "\t";
    std::cout << "loss ratio = " << 1-S/G << std::endl;
    outputFile << 1-S/G <<  "\t";

    std::cout << "delayAvg = " << delayAvg << std::endl;
    outputFile << delayAvg <<  "\t";


    std::cout << "totalNumberOfPacketsSend = " << totalNumberOfPacketsSend << std::endl;
    outputFile << totalNumberOfPacketsSend << "\t";

    std::cout << "totalNumberOfPacketsReceived = " << totalNumberOfPacketsReceived << std::endl;
    outputFile << totalNumberOfPacketsReceived << std::endl;

    outputFile.close();
}

std::vector<SatFrameRtn*> SatGatewayPhy::retrievePackets()
{
    std::vector<SatFrameRtn*> retrievedPackets;
    while(mPacketBuffer.size() > 0)
    {
        bool foundSinglePacket = false;
        unsigned int i = 0;
        for(; i < mPacketBuffer.size(); i++)
        {
            int currentSlot = mPacketBuffer[i]->getSlotIndexWithinFrame();
            if(i == mPacketBuffer.size()-1)
            {
                foundSinglePacket = true;
                break;
            }
            else if(currentSlot == mPacketBuffer[i+1]->getSlotIndexWithinFrame())
            {
                do i++; while(i+1 < mPacketBuffer.size() && currentSlot == mPacketBuffer[i+1]->getSlotIndexWithinFrame());
            }
            else
            {
                foundSinglePacket = true;
                break;
            }
        }
        if(!foundSinglePacket)
        {
            for(unsigned int j = 0; j < mPacketBuffer.size(); j++)
                delete mPacketBuffer[j];
            mPacketBuffer.clear();
            return retrievedPackets;
        }
        int currentSrcAddress = mPacketBuffer[i]->getSrcAddress();
        int currentRandomSeed = mPacketBuffer[i]->getRandomSeed();
        retrievedPackets.push_back(mPacketBuffer[i]);
        mPacketBuffer.erase(mPacketBuffer.begin()+i);
        for(unsigned int j = 0; j < mPacketBuffer.size(); j++)
        {
            if(currentSrcAddress == mPacketBuffer[j]->getSrcAddress() && currentRandomSeed == mPacketBuffer[j]->getRandomSeed())
            {
                delete mPacketBuffer[j];
                mPacketBuffer.erase(mPacketBuffer.begin()+j);
                j--;
            }
        }
    }
    return retrievedPackets;
}
