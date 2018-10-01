//
// Created by discord on 5/21/18.
//

#ifndef RPLIDARINTECHLIB_LINKEDVALUESLIST_H
#define RPLIDARINTECHLIB_LINKEDVALUESLIST_H

struct linked_values_list
{
    volatile struct linked_values_list* next;
    float dist;
    float angle;
};

typedef struct linked_values_list LinkedValuesList;

#endif //RPLIDARINTECHLIB_LINKEDVALUESLIST_H