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

package satnetnew;

import inet.common.misc.ThruputMeter;
import inet.linklayer.ppp.PPP;
import inet.networklayer.common.InterfaceTable;


module SatGateway
{
    parameters:
        @display("bgb=743,368");
        @networkNode;
        *.interfaceTableModule = default(absPath(".interfaceTable"));
 

    gates:
        inout pppg[] @labels(PPPFrame-conn);
        input satchannelreturn @directIn;

    submodules:
        localInterface: PPP {
            @display("p=81,218");
        }

        satGatewayPhy: SatGatewayPhy {
            @display("p=532,218");
        }

        interfaceTable: InterfaceTable {
            @display("p=65,46");
            displayAddresses = false;
        }

        thruputMeterFwd: ThruputMeterOffset {
            parameters:
            	@display("p=250,150;i=block/timer");
            	startTime = 10s;
        }
        
		thruputMeterRtn: ThruputMeterOffset {
		    parameters:
            	@display("p=250,218;i=block/timer");
            	startTime = 10s;
        }

        satGatewayAccessFwd: SatGatewayAccessFwd {
            @display("p=400,150");
        }

    connections:

        pppg++ <--> { @display("m=s"); } <--> localInterface.phys;

        // Return link
        satchannelreturn --> satGatewayPhy.remoteIn;
        //simpleFwdforward.localOut --> localInterface.netwIn;
        satGatewayPhy.localOut --> thruputMeterRtn.in;
        thruputMeterRtn.out --> localInterface.netwIn;

        // Forward link
        localInterface.netwOut --> thruputMeterFwd.in;
        thruputMeterFwd.out --> satGatewayAccessFwd.in;
        satGatewayAccessFwd.out --> satGatewayPhy.localIn;

}
