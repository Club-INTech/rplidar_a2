//
// Created by discord on 6/17/18.
//

#ifndef RPLIDARINTECHLIB_TCP_H
#define RPLIDARINTECHLIB_TCP_H

#define PORT (17685)

int socket_desc = 0;

ssize_t send_turn(int socket, LinkedValuesList* turn)
{

    int c = 0;
    char * buf = (char*)malloc(10000);
    char * cursor = buf;
    LinkedValuesList* current = turn;
    LinkedValuesList* toFree;

    printf("%f \n", turn->dist);

    while(current != NULL)
    {
        cursor += sprintf(cursor, "%.2f:%.2f;", current->dist, current->angle);

        toFree = current;
        current = (LinkedValuesList *) current->next;
        free(toFree);
        c++;
    }

    printf("%d values\n", c);

    (cursor-1)[0] = '\n';
    cursor[0] = 0;

    return write(socket, buf, strlen(buf));
}


//returns client's socket
int server_wait()
{
    int client_sock , c;
    struct sockaddr_in server , client;

    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1)
    {
        printf("Could not create socket");
        return -1;
    }

    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( PORT );

    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("bind failed. Error");
        return -11;
    }
    listen(socket_desc , 3);

    c = sizeof(struct sockaddr_in);

    client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
    if (client_sock < 0)
    {
        perror("accept failed");
        return -1;
    }
    return client_sock;
}

void close_server()
{
    close(socket_desc);
}


#endif //RPLIDARINTECHLIB_TCP_H
