# include <stdio.h>
#include <sys/socket.h>
#include <netdb.h>
#include <assert.h>
#include <string.h>

void receive()
{
	sockaddr_in si_me, si_other;
	int s;
	assert((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))!=-1);
	int port=7400;
	int broadcast=1;

	setsockopt(s, SOL_SOCKET, SO_BROADCAST,	&broadcast, sizeof broadcast);

	memset(&si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(port);
	si_me.sin_addr.s_addr = INADDR_ANY;
	
	assert(bind(s, (sockaddr *)&si_me, sizeof(sockaddr))!=-1);

	char buf[10000];
	unsigned slen=sizeof(sockaddr);
	recvfrom(s, buf, sizeof(buf)-1, 0, (sockaddr *)&si_other, &slen);

	printf("recv: %s\n", buf);
}

void end_buf()
{
	char txt [1000];
	txt[0] = 'i';
	txt[1] = 'n';
	txt[2] = 't';
	txt[3] = '\0';
	int len = strlen(txt);
	printf("%d\n", len);
	char * end = txt + len+1;
	*end = (char)9;
	printf("%d\n", (int)*end);
	len = strlen(txt);
	printf("%d\n", len);
}

int main(int argc, char** argv)
{		
	end_buf();
}
