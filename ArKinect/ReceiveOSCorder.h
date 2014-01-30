#ifndef __RECEIVE_OSC_ORDER__
#define __RECEIVE_OSC_ORDER__

#include <stdio.h>
#include <pthread.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netdb.h>
#include <assert.h>
#include <string>
#include <string.h>
#include <map>

/**
 * Functions called by a OSC message. Takes one integer as parameter.
 */
typedef void (*OscFunction)(unsigned int);

/**
 * Operator used to compare two functions' name
 */
struct comparer
{
    public:
    bool operator()(const std::string x, const std::string y)
    {
         return x.compare(y)<0;
    }
};

/**
 * Dictionary linking names to OscFunctions
 */
typedef std::map<std::string, OscFunction, comparer> DictOscFunction;

class ReceiveOSCorder{
	private:
		/** P_thread in charge of the reception of commands */
		pthread_t _taskCommand;
		
		/** Socket receiving messages */
		int _idSocket;
		
		/** Port to listen */
		int _port;
		
		/** Dictionary of functions */
		DictOscFunction _dictOscFunctions;
		
		/** Receive a message from the socket. The message is an OSC request */
		void receive(char * buf, int * nb_char);
		
		/** Read and executes a message received. Messages are composed of the name of a function, and a value */
		void read_buf(char * buf, int * nb_char);
		
	public:
		ReceiveOSCorder(const int port);
		~ReceiveOSCorder();
		
		void addOscFunction(std::string keywordCommand, OscFunction callback);
		void startCommunication();
};

#endif
