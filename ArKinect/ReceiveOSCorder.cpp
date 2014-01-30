#include "ReceiveOSCorder.h"

ReceiveOSCorder::ReceiveOSCorder(const int port)
{
	sockaddr_in si_me;
    int broadcast=1;
    
	_port = port;
    
    if((_idSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) != -1)
    {		
		setsockopt(_idSocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast);

		memset(&si_me, 0, sizeof(si_me));
		si_me.sin_family = AF_INET;
		si_me.sin_port = htons(_port);
		si_me.sin_addr.s_addr = INADDR_ANY;
		
		if(bind(_idSocket, (sockaddr *)&si_me, sizeof(sockaddr)) == -1)
			_idSocket = -1;
	}
}

ReceiveOSCorder::~ReceiveOSCorder()
{
	if(_idSocket != -1)
		close(_idSocket);
}

void ReceiveOSCorder::addOscFunction(std::string keywordCommand, OscFunction callback)
{
	_dictOscFunctions.insert(std::pair<std::string,OscFunction>(keywordCommand,callback));
}

/**
 * Reception of commands from other devices with OSC requests.
 * Receives OSC messages, then calls the associated function with the received value.
 */
void ReceiveOSCorder::startCommunication()
{
	// Messages' buffer
	char buf[1000];    
    // Size of messages and socket's id
    int nb_char;
	while(_idSocket != -1)
	{
        // Receiving the message
		receive(buf, &nb_char);
        // If not empty, reading and executing it
		if(nb_char > 0)
			read_buf(buf, &nb_char);
	}
}

/**
 * Receive a message from the socket. The message is an OSC request.
 * @param s : socket that receives the message
 * @param buf : buffer in which the message is stored
 * @param nb_char : size of the received message
 */
void ReceiveOSCorder::receive(char * buf, int * nb_char)
{
	// Message's sender
    sockaddr_in si_other;
    
    // Reception
    unsigned slen=sizeof(sockaddr);
    *nb_char = recvfrom(_idSocket, buf, 1000, 0, (sockaddr *)&si_other, &slen);

    // We print the received message and its size
    printf("recv: %s\n", buf);
    printf("nb_char: %d\n", *nb_char);
}

/**
 * Read and executes a message received. Messages are composed of the name of a function, and a value.
 * @param buf : message
 * @param nb_char : size of the message
 */
void ReceiveOSCorder::read_buf(char * buf, int * nb_char)
{
	 // Acquiring the function's name
    std::string functionName(buf);
    // The value is at the end of the OSC message
   	char * end = buf + *nb_char - 1;
    // Getting the value, and converting it to an integer
	unsigned char result = (unsigned char)*end;
	result = 0x00FF&result;
	
    // We check if the dictionary contains the function
    DictOscFunction::const_iterator ptrF;
    ptrF = _dictOscFunctions.find(functionName);
    if (ptrF != _dictOscFunctions.end())
        // If so, we call the function with the given value as parameter
        (*ptrF->second)(result);
}
