#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sstream>
#include <vector>
#include <string>

using namespace std;

#define SERVER_PORT htons(3333)

int main(int argc, char *argv[])
{
	string moves[20] = {"leftArmIn", "default", "leftArmIn", "leftArmShake", "hokeyPokey", "rightArmIn", "default", "rightArmIn", "rightArmShake", "hokeyPokey", "leftLegIn", "default", "leftLegIn", "leftLegShake", "hokeyPokey", "rightLegIn", "default", "rightLegIn", "rightLegShake", "hokeyPokey"};

	char receivedMsg[1000];
	
	for(int i=0; i<1000; i++)
	{
		receivedMsg[i] = '\0';
	}
	
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock < 0)
    {
        cout << "Cannot create socket" << endl;
        exit(1);
    }
	
    struct sockaddr_in remote;
    //string theHost = "10.40.163.118"; //IP address of the server
    //string theHost = "192.168.123.1";
	string theHost = argv[1];
    struct hostent *host = gethostbyname(theHost.c_str());
    remote.sin_port = SERVER_PORT;
	
    if(host == NULL)
    {
        cout << "gethostbyname failed" << endl;
        exit(1);
    }
	
    remote.sin_family = host->h_addrtype;
    remote.sin_addr.s_addr = ((struct in_addr *)(host->h_addr))->s_addr;
	
    if(connect(sock, (struct sockaddr *) &remote, sizeof(remote)) < 0)
    {
        cout << "connect failed" << endl;
        exit(1);
    }
	
    string message = "";
    bool flag = true;
	int index, status;
	int counter = 0;
	
    while(counter < 20)
    {
		message = moves[counter] + " 0 yes";
        send(sock, message.c_str(), message.length(), 0);

        cout << "Sent: " << message << endl;
			
		status = recv(sock, receivedMsg, 1000, 0);
			
		if(status > 0)
		{
			//first find the null "character"
			index = 0;
				
			while(receivedMsg[index] != '\0')
			{
				index++;
			}  
				
			string srvMessage(receivedMsg, receivedMsg + index); //copy the non-null subset of the char array to a string
				
			if(srvMessage.length() >= 1)
			{
				cout << "Client received: " << srvMessage << endl;
			}
				
			//clear everything for the next message, otherwise you have leftovers
			for(int i=0; i<1000; i++)
			{
				receivedMsg[i] = '\0';
			}

			counter++;
        }
    }
	
    //close(sock);
	
    return 0;
}