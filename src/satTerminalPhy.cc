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

#include "satTerminalPhy.h"
#include <fstream>
Define_Module(SatTerminalPhy);

void SatTerminalPhy::initialize()
{
    // Determine Id of remoteIn gate
    remoteInGateId = gate("remoteIn")->getId();

    // Determine satellite terminal module
    satGatewayModule = getModuleByPath("Ipnet.satGateway");

    // Initialize fsm
    fsm.setName("fsm");
    i = 0;
    WATCH(i); // always put watches in initialize()
    scheduleAt(0.0, new cMessage);

    propagationDelay = par("propagationDelay");
    slotDuration = par("slotDuration");
    slotsPerFrame = par("slotsPerFrame");
    numberOfReplicas = par("numberOfReplicas");
    maxNumberOfUniquePacketsPerFrame = par("maxNumberOfUniquePacketsPerFrame");

    currentFrameIndex = 0;

    totalNumberOfPacketsSend = 0;

    mCRDSA = new CRDSA(numberOfReplicas, slotsPerFrame);

    mPacketQueue = new cPacketQueue;
}

void SatTerminalPhy::handleMessage(cMessage *msg)
{
    FSM_Switch(fsm)
    {
        case FSM_Exit(INIT):
        {
            delete msg;

            // Determine IP address of connected StandardHost
            inet::L3Address local_addr;
            char module_name[15];
            sprintf(module_name, "Ipnet.user[%d]", getParentModule()->getIndex());
            local_addr = inet::L3AddressResolver().resolve(module_name);
            EV << "------------------------------" << module_name << " L3 address : " << local_addr << endl;

            // Register IP address in gateway database (logon)
            cModule *satGatewayAccessFwdModule = getModuleByPath("Ipnet.satGateway.satGatewayAccessFwd");
            SatGatewayAccessFwd *callee = check_and_cast<SatGatewayAccessFwd *>(satGatewayAccessFwdModule);
            callee->registerIPAddress(local_addr, getParentModule()->getIndex());

            scheduleAt((currentFrameIndex+1)*slotDuration*slotsPerFrame, new cMessage("frameTrigger"));

            // transition to SLEEP state
            FSM_Goto(fsm,IDLE);
            break;
        }
        case FSM_Enter(IDLE):
            break;
        case FSM_Exit(IDLE):
        {
            incomingFrame = (cPacket*) msg;
            // If the incoming message is self scheduled
            if(msg->isSelfMessage())
            {
                if(!strcmp(msg->getName(), "frameTrigger"))
                {
                    currentFrameIndex++;
                    scheduleAt((currentFrameIndex+1)*slotDuration*slotsPerFrame, new cMessage("frameTrigger"));
                    allocateFrame(currentFrameIndex);
                    delete msg;
                }
                else
                {
                    sendDirect(incomingFrame, propagationDelay, 0, satGatewayModule, "satchannelreturn");
                    totalNumberOfPacketsSend++;

                    //std::cout << "getCreationTime = " << incomingFrame->getCreationTime() << std::endl;
                    //std::cout << incomingFrame->getSendingTime() << std::endl;
                }
            }
            else
            {
                if (incomingFrame->getArrivalGateId() == remoteInGateId)
                {
                    FSM_Goto(fsm,REMOTE_IN);
                }
                else
                {
                    FSM_Goto(fsm,LOCAL_IN);
                }
            }
            break;
        }
        case FSM_Enter(REMOTE_IN):
            send(incomingFrame, "localOut");
            break;
        case FSM_Exit(REMOTE_IN):
            FSM_Goto(fsm,IDLE);
            break;
        case FSM_Enter(LOCAL_IN):
        {
            mPacketQueue->insert((cPacket*)msg);
            break;
        }
        case FSM_Exit(LOCAL_IN):
            FSM_Goto(fsm,IDLE);
            break;
    }
}

void SatTerminalPhy::finish()
{
    delete mCRDSA;
    delete mPacketQueue;
    double Gt = totalNumberOfPacketsSend / (simTime().dbl()/slotDuration);
    std::cout << "Gt = " << Gt << std::endl;

/*    std::ofstream outputFile;
    outputFile.open("SimulationResults", std::ios::out | std::ios::app);

    outputFile << "totalNumberOfPacketsSend="<<totalNumberOfPacketsSend << std::endl;
    outputFile.close();*/
}

void SatTerminalPhy::allocateFrame(int frameIndex)
{
    int numberOfUniquePacketsToSend = std::min(mPacketQueue->getLength(), maxNumberOfUniquePacketsPerFrame);
    // The location arrays containing the slot index of each replica for each unique packet
    // The random seed is put at the end of each inner array
    std::vector<std::vector<int> > locationArrays;
    for(unsigned int i = 0; i < numberOfUniquePacketsToSend; i++)
    {
        std::vector<int> locationArray;
        bool arraysIntersect;
        do
        {
            arraysIntersect = false;
            int randomSeed = getRNG(0)->intRand()%((1<<12)-1);
            locationArray = mCRDSA->generateLocArray(randomSeed, getId());
            // Check if two elements are equal within the location array we just computed
            for(unsigned int j = 0; !arraysIntersect && j < locationArray.size() - 1; j++)
            {
                for(unsigned int k = j + 1; !arraysIntersect && k < locationArray.size(); k++)
                {
                    if(locationArray[j] == locationArray[k])
                        arraysIntersect = true;
                }
            }
            // Check if an element of the location array exists in the other location arrays
            for(unsigned int j = 0; !arraysIntersect && j < locationArray.size(); j++)
            {
                if(existsIn(locationArray[j], locationArrays))
                    arraysIntersect = true;
            }
            if(!arraysIntersect)
                locationArray.push_back(randomSeed);
        } while(arraysIntersect);
        locationArrays.push_back(locationArray);
    }
    std::sort(locationArrays.begin(),
              locationArrays.end(),
              [](const std::vector<int>& a, const std::vector<int>& b) { return a[0] < b[0]; }
              );
    for(int i = 0; i < locationArrays.size(); i++)
    {
        cPacket* packet = mPacketQueue->pop();
        for(int j = 0; j < locationArrays[i].size() - 1; j++)
        {
            SatFrameRtn* returnFrame = new SatFrameRtn;
            returnFrame->setRandomSeed(locationArrays[i][locationArrays[i].size() - 1]);
            returnFrame->setSrcAddress(getId());
            returnFrame->setSlotIndexWithinFrame(locationArrays[i][j]);
            returnFrame->encapsulate(packet->dup());
            scheduleAt(frameIndex*slotDuration*slotsPerFrame+locationArrays[i][j]*slotDuration, returnFrame);
        }
        delete packet;
    }
}

bool SatTerminalPhy::existsIn(int value, std::vector<std::vector<int> > set)
{
    for(unsigned int i = 0; i < set.size(); i++)
    {
        for(unsigned int j = 0; j < set[i].size(); j++)
        {
            if(value == set[i][j])
                return true;
        }
    }
    return false;
}
