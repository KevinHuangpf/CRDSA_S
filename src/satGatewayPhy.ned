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

//
// TODO auto-generated module
//
simple SatGatewayPhy
{
	parameters :
	    double propagationDelay = default(0.25);
        double slotDuration = default(0.001);
    	int slotsPerFrame = default(100);
    	int numberOfReplicas = default(2);
    	@signal[delay](type="float");
        @statistic[delay](title="delay"; source="delay"; record=mean, sum, stats, vector);
    	
	gates :
        input	localIn;
        input	remoteIn;
        output	localOut;
}
