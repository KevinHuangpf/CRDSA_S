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


module SatTerminal
{
    parameters:
        @display("bgb=743,368");
        @networkNode;
        *.interfaceTableModule = default(absPath(".interfaceTable"));

    gates:
        inout pppg[] @labels(PPPFrame-conn);
        input satchannelforward @directIn;

    submodules:
        localInterface: PPP {
            @display("p=81,218");
        }
        
        satTerminalPhy: SatTerminalPhy {
            @display("p=532,218");
        }
        
        interfaceTable: InterfaceTable {
            @display("p=65,46");
            displayAddresses = false;
        }

        thruputMeter: ThruputMeter {
            @display("p=321,126");
        }
        
    connections:

        pppg++ <--> { @display("m=s"); } <--> localInterface.phys;
        
        // Forward link
        satchannelforward --> satTerminalPhy.remoteIn;
        satTerminalPhy.localOut --> localInterface.netwIn;
        
        // Return link
        localInterface.netwOut --> thruputMeter.in;
        thruputMeter.out --> satTerminalPhy.localIn;
}
