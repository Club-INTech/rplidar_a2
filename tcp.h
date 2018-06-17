//
// Created by discord on 6/17/18.
//

#ifndef RPLIDARINTECHLIB_TCP_H
#define RPLIDARINTECHLIB_TCP_H

void send_turn(int socket, LinkedValuesList* turn)
{

    int c = 0;
    LinkedValuesList* current = turn;
    LinkedValuesList* toFree;

    printf("%f \n", turn->dist);

    while(current != NULL)
    {

        //TODO send values

        toFree = current;
        current = (LinkedValuesList *) current->next;
        free(toFree);
        c++;
    }

}

#endif //RPLIDARINTECHLIB_TCP_H
