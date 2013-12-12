# include <stdio.h>
#include <sys/socket.h>
#include <netdb.h>
#include <assert.h>
#include <string.h>

void receive(char * buf, int * nb_char)
{
    sockaddr_in si_me, si_other;
    int s;
    assert((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))!=-1);
    int port=7400;
    int broadcast=1;

    setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast);

    memset(&si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    si_me.sin_addr.s_addr = INADDR_ANY;
    
    assert(bind(s, (sockaddr *)&si_me, sizeof(sockaddr))!=-1);

    unsigned slen=sizeof(sockaddr);
    *nb_char = recvfrom(s, buf, 1000, MSG_WAITALL, (sockaddr *)&si_other, &slen);

    printf("recv: %s\n", buf);
    printf("nb_char: %d\n", *nb_char);
}

void end_buf(char * buf, int * nb_char)
{
    /*int len = strlen(buf);
    printf("%d\n", len);*/
    if (strcmp(buf, (char *)"int") == 0)
    {
		char * end = buf + *nb_char - 1;
        unsigned char result = (unsigned char)*end;
        unsigned int result2 = 0x00FF&result;
        printf("%u\n", result2);
    }
    else {
        printf("%s\n", buf);    
    }
}

int main(int argc, char** argv)
{   
    char buf[10000];    
    int nb_char;
    receive(buf, &nb_char);
    end_buf(buf, &nb_char);
}
