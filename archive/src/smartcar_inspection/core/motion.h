#ifndef INSPECTION_MOTION_H
#define INSPECTION_MOTION_H
#include "planner.h"
#define MAX_COMMANDS 512

typedef enum { MOTION_AUTO, MOTION_FORWARD_ONLY, MOTION_REVERSE_ONLY } MotionMode;
typedef enum {
    PHASE_ALIGN, PHASE_ENTER, PHASE_INSPECT, PHASE_PASS,
    PHASE_RETREAT, PHASE_CLEAR, PHASE_OUTER, PHASE_EXIT
} MotionPhase;
typedef struct { double x, y, yaw; } Pose2;

/* Front axle midpoint coordinates; yaw is derived, never imported. */
typedef struct {
    Pose2 entry[5][2], inspection[10], outer[2][2], start, end;
    char map_id[64], map_version[64], frame_id[64];
} MotionMap;

typedef struct {
    int task_id, source_event, region_id, inspect;
    char target_id[48];
    Pose2 target;
    int enforce_yaw;
    MotionMode mode;
    MotionPhase phase;
} MotionCommand;
typedef struct {
    MotionCommand commands[MAX_COMMANDS];
    int count;
    Pose2 start;
    char map_id[64], map_version[64], frame_id[64];
} MotionPlan;

typedef enum { SESSION_RUNNING, SESSION_FAILED, SESSION_DONE } SessionStatus;
typedef struct {
    const MotionPlan *plan;
    unsigned long long mission_id;
    int index;
    unsigned inspected;
    SessionStatus status;
} MotionSession;

void motion_map_example(MotionMap *map);
int motion_map_valid(const MotionMap *map);
int motion_map_load(const char *info_path, const char *points_path,
                    MotionMap *map, char *error, size_t size);
int make_motion_plan(const Layout *layout, const Plan *route, const MotionMap *map,
                     MotionPlan *out, char *error, size_t size);
const char *motion_mode_name(MotionMode mode);
const char *motion_phase_name(MotionPhase phase);
void print_motion_plan(const MotionPlan *plan);
int export_motion_plan(const char *path, const MotionPlan *plan);
/* Caller assigns a new mission_id for every run, including retries. */
void motion_session_begin(MotionSession *session, const MotionPlan *plan,
                          unsigned long long mission_id);
const MotionCommand *motion_session_current(const MotionSession *session);
/* Returns 0 for stale/duplicate feedback. Failure halts; it never skips a task. */
int motion_session_ack(MotionSession *session, unsigned long long mission_id,
                       int task_id, int success);
#endif
