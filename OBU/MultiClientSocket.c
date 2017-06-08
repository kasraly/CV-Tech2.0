#include "MultiClientSocket.h"

int initSocket()
{
    int opt = 1;
    int i;

    //initialise all client_socket[] to 0 so not checked
	for (i = 0; i < MAX_CLIENTS; i++)
    {
		client_socket[i] = 0;
    }

    //create a master socket
	if( (master_socket = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
		perror("socket failed");
		exit(EXIT_FAILURE);
    }

    //set master socket to allow multiple connections , this is just a good habit, it will work without this
	if( setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) < 0 )
    {
		perror("setsockopt");
		exit(EXIT_FAILURE);
    }

    bzero(&address, sizeof(struct sockaddr));
	//type of socket created
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons( PORT );

	//bind the socket to localhost port 8888
	if (bind(master_socket, (struct sockaddr *)&address, sizeof(address))<0)
    {
		perror("bind failed");
		exit(EXIT_FAILURE);
    }
	printf("Listener on port %d \n", PORT);

	//try to specify maximum of 3 pending connections for the master socket
	if (listen(master_socket, MAX_CLIENTS) < 0)
    {
		perror("listen");
		exit(EXIT_FAILURE);
    }

    return 0;
}

int acceptConnection()
{

    int addrlen;
    int activity = 1;
    int sd;
    int max_sd;
    int valread;
    struct timeval socketCheckTimout = {0,1};
    char buffer[1024];

	//set of socket descriptors
	fd_set readfds;

	//accept the incoming connection
    addrlen = sizeof(address);
	//puts("Waiting for connections ...");
    while (activity > 0)
    {
    	//clear the socket set
    	FD_ZERO(&readfds);

    	//add master socket to set
    	FD_SET(master_socket, &readfds);
    	max_sd = master_socket;

    	int i;
    	//add child sockets to set
    	for ( i = 0 ; i < MAX_CLIENTS ; i++)
        {
            //socket descriptor
            sd = client_socket[i];

            //if valid socket descriptor then add to read list
            if(sd > 0)
                FD_SET( sd, &readfds);

            //highest file descriptor number, need it for the select function
            if(sd > max_sd)
                max_sd = sd;
        }

        //wait for an activity on one of the sockets , timeout is NULL , so wait indefinitely
        activity = select( max_sd + 1, &readfds, NULL, NULL, &socketCheckTimout);

        if ((activity < 0))
        {
            printf("select error");
        }
        int new_socket;
        //If something happened on the master socket , then its an incoming connection
        if (FD_ISSET(master_socket, &readfds))
        {
            if ((new_socket = accept(master_socket, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0)
            {
                perror("accept");
                exit(EXIT_FAILURE);
            }

            //inform user of socket number - used in send and receive commands
            printf("New connection , socket fd is %d , ip is : %s , port : %d \n", new_socket, inet_ntoa(address.sin_addr), ntohs(address.sin_port));
            //add new socket to array of sockets
            for (i = 0; i < MAX_CLIENTS; i++)
            {
                //if position is empty
                if( client_socket[i] == 0 )
                {
                    client_socket[i] = new_socket;
                    printf("Adding to list of sockets as %d\n", i);

                    break;
                }
            }
        }

        //else its some IO operation on some other socket :)
        for (i = 0; i < MAX_CLIENTS; i++)
        {
            sd = client_socket[i];

            if (FD_ISSET(sd, &readfds))
            {
                //Check if it was for closing , and also read the incoming message
                if ((valread = read( sd, buffer, 1024)) == 0)
                {
                    //Somebody disconnected , get his details and print
                    getpeername(sd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
                    printf("Host disconnected , ip %s , port %d \n", inet_ntoa(address.sin_addr), ntohs(address.sin_port));

                    //Close the socket and mark as 0 in list for reuse
                    close(sd);
                    client_socket[i] = 0;
                }
            }
        }
    }

    return 0;
}

int sendToClients(char *smartPhoneMsg)
{
    int k = 0;
    int i;
    for (i=0; i<MAX_CLIENTS ; i++)
    {
        if (client_socket[i] > 0)
        {
            if (-1==send(client_socket[i],smartPhoneMsg,strlen(smartPhoneMsg)+1,0))
                printf("send to client %d failed",i+1);
            k++;
        }
    }
    if (k == 0)
    {
        printf("No Andriod application running \n");
    }

    return k;
}

int closeSockets()
{
    int i;
    for(i=0; i<MAX_CLIENTS; i++)
    {
        if (client_socket[i]>0)
            close(client_socket[i]);
    }
    return 0;
}
