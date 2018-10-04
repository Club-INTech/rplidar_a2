//
// Created by discord on 5/21/18.
//

#ifndef RPLIDAR_A2_LINKEDVALUESLIST_H
#define RPLIDAR_A2_LINKEDVALUESLIST_H

struct linked_values_list
{
    volatile struct linked_values_list* next;
    float dist;
    float angle;
};

typedef struct linked_values_list LinkedValuesList;

#endif //RPLIDAR_A2_LINKEDVALUESLIST_H