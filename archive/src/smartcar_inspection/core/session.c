#include "motion.h"
#include <string.h>
void motion_session_begin(MotionSession *session, const MotionPlan *plan, unsigned long long mission_id)
{
    memset(session,0,sizeof(*session)); session->plan=plan; session->mission_id=mission_id;
    session->status=plan && plan->count>0 ? SESSION_RUNNING : SESSION_FAILED;
}
const MotionCommand *motion_session_current(const MotionSession *session)
{
    return session && session->plan && session->status==SESSION_RUNNING && session->index<session->plan->count
        ? &session->plan->commands[session->index] : NULL;
}
int motion_session_ack(MotionSession *session, unsigned long long mission_id, int task_id, int success)
{
    const MotionCommand *c=motion_session_current(session);
    if (!c || mission_id!=session->mission_id || task_id!=c->task_id || (success!=0 && success!=1)) return 0;
    if (!success) { session->status=SESSION_FAILED; return 1; }
    if (c->inspect) session->inspected|=1u<<(c->region_id-1);
    if (++session->index==session->plan->count) session->status=SESSION_DONE;
    return 1;
}

