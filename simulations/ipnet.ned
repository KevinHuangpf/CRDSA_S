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

package satnetnew.simulations;


import satnetnew.SatTerminal;
import satnetnew.SatGateway;
import inet.linklayer.ethernet.EtherBus;
import inet.linklayer.ethernet.EtherHub;
import inet.networklayer.configurator.ipv4.IPv4NetworkConfigurator;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import ned.DatarateChannel;

network Ipnet
{
    parameters:
        int numSatTerminals @prompt("Number of satellite terminals") = default(1);

        @display("bgb=1025,272");
    types:
        channel pppline extends DatarateChannel
        {
            delay = 1us;
            datarate = 512Mbps;
        }
        channel ethline extends DatarateChannel
        {
            delay = 1us;
            datarate = 1Gbps;
        }
    submodules:
        user[numSatTerminals]: StandardHost {
            parameters:
                @display("p=100,100,c,30");
                //numUdpApps = 0; // Runs VoIP and Http
            	//numTcpApps = 1; // Runs Ftp
            	//tcpApp[10]: <tcpAppType> like TCPApp;
            gates:
                pppg[1];
        }
        applVoIPServer: StandardHost {
            @display("p=828,36");
        }
        applHttpServer: StandardHost {
            @display("p=878,100");
        }
        applFtpServer: StandardHost {
            @display("p=828,201");
        }
        configurator: IPv4NetworkConfigurator {
            config = xml("<config><interface hosts='*' address='192.168.x.x' netmask='255.255.x.x'/></config>");
            //config = xmldoc("ipconfig.xml");
            @display("p=250,36");
        }
        satTerminal[numSatTerminals]: SatTerminal {
            parameters:
                @display("i=device/receiverdish;p=300,100,c,30");
            	// i=device/receiverdish");
            gates:
                pppg[1];
        }
        satGateway: SatGateway {
            parameters:
                @display("p=500,100;i=device/receiverdish");
            gates:
                pppg[1];
        }
        router: Router {
            parameters:
                @display("p=700,100");
            gates:
                pppg[4];
                // ethg[1];
        }
    connections:
        for i=0..numSatTerminals-1 {
            user[i].pppg[0] <--> pppline <--> satTerminal[i].pppg[0];
        }
        satGateway.pppg[0] <--> pppline <--> router.pppg[0];
        router.pppg[1] <--> pppline <--> applVoIPServer.pppg++;
        router.pppg[2] <--> pppline <--> applHttpServer.pppg++;
        router.pppg[3] <--> pppline <--> applFtpServer.pppg++;
}
