

#ifndef MESSAGE_ENUMS_H
#define MESSAGE_ENUMS_H

enum SegmentStatus
{
    STATUS_FAILURE = -1,
    STATUS_PLANNING = 0,
    STATUS_SUCCESS = 1
} ;


enum TaskType
{
    kPathNormal = 0,
    kPathCyclic = 1
} ;

#endif /* MESSAGEENUMS_H */

