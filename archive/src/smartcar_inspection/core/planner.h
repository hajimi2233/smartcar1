#ifndef INSPECTION_PLANNER_H
#define INSPECTION_PLANNER_H

#include <stddef.h>

#define MAX_POINTS 10
#define MAX_EVENTS 128

typedef struct {
    int count;                 /* 2, 4, 6, 8 or 10, numbered consecutively. */
    char type[MAX_POINTS];     /* A: pass through; B: inspect and retreat. */
} Layout;

typedef enum {
    EVENT_POINT = 0,
    EVENT_OUTER_LOW,           /* Bypass at the 1/2 end. */
    EVENT_OUTER_HIGH,          /* Bypass at the largest-numbered end. */
    EVENT_END                 /* Follow bottom corridor to the exit at 1/2 end. */
} EventKind;

typedef struct {
    EventKind kind;
    int point;                 /* Public point IDs are one-based. */
    int inspect;               /* 1: inspect once; 0: transit without speech. */
    char type;
    int entry_side;            /* 0: enters toward bottom; 1: toward top. */
} PlanEvent;

typedef struct {
    PlanEvent events[MAX_EVENTS];
    int event_count;
    int order[MAX_POINTS];
    int order_count;
    unsigned visited;
    int final_side;            /* 0: top corridor; 1: bottom corridor. */
    int final_column;          /* Zero-based column along the public corridor. */
    int complete;             /* All tasks and a legal route to the exit planned. */
} Plan;

int parse_layout(const char *text, Layout *layout, char *error, size_t error_size);
int make_plan(const Layout *layout, Plan *plan);

#endif
