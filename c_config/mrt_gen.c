#include <stdlib.h>
#include <stdio.h>

main()
{
int	numSatTerminals;
int	terminalIndex;
FILE	*mrt_file;
char	mrt_file_name[32];

numSatTerminals = 100;

for (terminalIndex=0; terminalIndex<numSatTerminals; terminalIndex++)
	{
	sprintf(mrt_file_name, "client%d.mrt", terminalIndex);
	mrt_file = fopen(mrt_file_name, "w");
	fprintf(mrt_file, "ifconfig:\n");
	fprintf(mrt_file, "\n");
	fprintf(mrt_file, "# Point to Point link 1 to Host 1\n");
	fprintf(mrt_file, "name: ppp0 inet_addr: 172.0.0.%d Mask: 255.255.255.0 MTU: 576 Metric: 1\n", terminalIndex+1);
	fprintf(mrt_file, "ifconfigend.\n");
	fprintf(mrt_file, "\n");
	fprintf(mrt_file, "route:\n");
	fprintf(mrt_file, "\n");
	fprintf(mrt_file, "#Destination Gateway Genmask Flags Metric Iface\n");
	fprintf(mrt_file, "default: 172.0.0.100 0.0.0.0 G 0 ppp0\n");
	fprintf(mrt_file, "\n");
	fprintf(mrt_file, "routeend.");
	}
}
