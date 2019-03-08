#include <stdlib.h>
#include <stdio.h>

main()
{
int	numSatTerminals;
int	terminalIndex;
FILE	*voip_file;

numSatTerminals = 100;
 voip_file = fopen("voip.ini", "w");

for (terminalIndex=0; terminalIndex<numSatTerminals; terminalIndex++)
	{
	  fprintf(voip_file, "[General]\n");
	  fprintf(voip_file, "## VoIP\n");
	  fprintf(voip_file, "**.user[*].udpApp[0].typename = \"SimpleVoIPSender\"\n");
	  fprintf(voip_file, "**.user[*].udpApp[0].destAddress = \"applVoIPServer\"\n");
	  fprintf(voip_file, "**.user[*].udpApp[0].talkspurtDuration = exponential(180s)\n");
	  fprintf(voip_file, "**.user[*].udpApp[0].silenceDuration = exponential(180s)\n");
	  fprintf(voip_file, "**.user[*].udpApp[0].startTime = exponential(60s)\n");
	  fprintf(voip_file, "**.user[*].udpApp[0].stopTime = -1s\n");
	  fprintf(voip_file, "\n");
	  for (terminalIndex=0; terminalIndex<numSatTerminals; terminalIndex++)
	    {
	  fprintf(voip_file, "**.user[%d].udpApp[0].destPort = %d\n", terminalIndex, 2000+terminalIndex);
	}
	  fprintf(voip_file, "\n");
	  fprintf(voip_file, "## UDP app for server\n");
	  fprintf(voip_file, "**.applVoIPServer.numUdpApps = %d\n", terminalIndex);
	  fprintf(voip_file, "**.applVoIPServer.udpApp[*].typename = \"SimpleVoIPReceiver\"\n");
	  	  for (terminalIndex=0; terminalIndex<numSatTerminals; terminalIndex++)
		    fprintf(voip_file, "**.applVoIPServer.udpApp[%d].localPort = %d\n", terminalIndex, 2000+terminalIndex);

	}
}
