# include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netdb.h>
#include <assert.h>
#include <string>
#include <string.h>
#include <map>

typedef void (*OscFunction)(unsigned int);

struct comparer
{
    public:
    bool operator()(const std::string x, const std::string y)
    {
         return x.compare(y)<0;
    }
};

typedef std::map<std::string, OscFunction, comparer> DictOscFunction;

void f1(unsigned int a)
{
	printf("F1, a = %d\n", a);
}

void f2(unsigned int b)
{
	printf("F2, b = %d\n", b);
}

int init_socket(int port)
{
    sockaddr_in si_me;
    int s;
    assert((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))!=-1);
    int broadcast=1;

    setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast);

    memset(&si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    si_me.sin_addr.s_addr = INADDR_ANY;
    
    assert(bind(s, (sockaddr *)&si_me, sizeof(sockaddr))!=-1);

    return s;
}

void receive(int s, char * buf, int * nb_char)
{
    sockaddr_in si_other;
    
    unsigned slen=sizeof(sockaddr);
    *nb_char = recvfrom(s, buf, 1000, MSG_WAITALL, (sockaddr *)&si_other, &slen);

    printf("recv: %s\n", buf);
    printf("nb_char: %d\n", *nb_char);
}

void read_buf(char * buf, int * nb_char, DictOscFunction* dict)
{
    printf("Receive string : %s\n", buf);
    std::string functionName(buf);
   	char * end = buf + *nb_char - 1;
	unsigned char result = (unsigned char)*end;
	unsigned int result2 = 0x00FF&result;
	printf("Receive parameter : %u\n", result2);
	
	//OscFunction ptrF;
	DictOscFunction::const_iterator ptrF;
    //ptrF = (*dict)[functionName];
    ptrF = (*dict).find(functionName);
    if (ptrF != (*dict).end())
        (*ptrF->second)(result2);
}

int main(int argc, char** argv)
{   
    char buf[10000];    
    int nb_char;
    int port=7400;
    DictOscFunction dictOscFunctions;
    
    dictOscFunctions.insert(std::pair<std::string,OscFunction>("Particule",f1));
    dictOscFunctions.insert(std::pair<std::string,OscFunction>("Point",f2));
    
    int s = init_socket(port);
    
    for (int i = 0 ; i < 10 ; i++)
    {
        receive(s, buf, &nb_char);
        read_buf(buf, &nb_char, &dictOscFunctions);
    }
    
    close(s);
}
