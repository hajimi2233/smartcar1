#define _CRT_SECURE_NO_WARNINGS
#include "motion.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int bad(char *error, size_t size, const char *message)
{ snprintf(error,size,"%s",message); return 0; }
int motion_map_valid(const MotionMap *m)
{
    for (int c=0;c<5;++c) {
        Pose2 a=m->entry[c][0], b=m->entry[c][1];
        double dx=b.x-a.x, dy=b.y-a.y, length=hypot(dx,dy), last=0;
        if (!isfinite(length) || length<0.01) return 0;
        for (int r=0;r<2;++r) {
            Pose2 p=m->inspection[2*c+r];
            double t=((p.x-a.x)*dx+(p.y-a.y)*dy)/(length*length);
            double cross=fabs((p.x-a.x)*dy-(p.y-a.y)*dx)/length;
            if (!isfinite(t) || !isfinite(cross) || t<=last || t>=1 || cross>0.05) return 0;
            last=t;
        }
    }
    for (int s=0;s<2;++s) for (int t=0;t<2;++t)
        if (!isfinite(m->outer[s][t].x) || !isfinite(m->outer[s][t].y)) return 0;
    return isfinite(m->start.x) && isfinite(m->start.y) && isfinite(m->end.x) && isfinite(m->end.y);
}
static int add(MotionPlan *p, int event, int region, const char *name, Pose2 target,
               double yaw, int enforce, MotionMode mode, MotionPhase phase, int inspect)
{
    MotionCommand *c;
    if (p->count>=MAX_COMMANDS) return 0;
    c=&p->commands[p->count++]; memset(c,0,sizeof(*c));
    c->task_id=p->count; c->source_event=event; c->region_id=region;
    snprintf(c->target_id,sizeof(c->target_id),"%s",name);
    c->target=target; c->target.yaw=yaw; c->enforce_yaw=enforce;
    c->mode=mode; c->phase=phase; c->inspect=inspect;
    return 1;
}
int make_motion_plan(const Layout *layout, const Plan *route, const MotionMap *m,
                     MotionPlan *p, char *error, size_t size)
{
    int row=0, column=0; double yaw=0; unsigned seen=0;
    if (!layout || !route || !m || !p || layout->count<2 || layout->count>10 || layout->count%2 ||
        !route->complete || route->event_count<1 || route->event_count>MAX_EVENTS)
        return bad(error,size,"Invalid route.");
    if (!motion_map_valid(m)) return bad(error,size,"Invalid straight-channel geometry: entrance/order/centreline (5 cm tolerance).");
    memset(p,0,sizeof(*p)); p->start=m->start;
    strcpy(p->map_id,m->map_id); strcpy(p->map_version,m->map_version); strcpy(p->frame_id,m->frame_id);
    for (int e=0;e<route->event_count;++e) {
        const PlanEvent *v=&route->events[e]; char name[48];
        if (v->kind==EVENT_END) {
            if (row!=2 || e!=route->event_count-1 || seen!=(1u<<layout->count)-1)
                return bad(error,size,"Premature or invalid exit.");
            if (!add(p,e,0,"end",m->end,0,0,MOTION_AUTO,PHASE_EXIT,0)) goto overflow;
        } else if (v->kind==EVENT_OUTER_LOW || v->kind==EVENT_OUTER_HIGH) {
            int side=row/2, outer=v->kind==EVENT_OUTER_HIGH;
            if (row==1 || v->entry_side!=side) return bad(error,size,"Illegal bypass.");
            for (int k=0;k<2;++k) {
                int s=k ? 1-side : side;
                snprintf(name,sizeof(name),"outer_%s_%s",outer ? "right" : "left",s ? "bottom" : "top");
                if (!add(p,e,0,name,m->outer[outer][s],0,0,MOTION_AUTO,PHASE_OUTER,0)) goto overflow;
            }
            row=2-row;
        } else if (v->kind==EVENT_POINT) {
            int index=v->point-1, entry, exit_row, c; double travel;
            MotionMode mode;
            if (index<0 || index>=layout->count || (v->entry_side!=0 && v->entry_side!=1)) return bad(error,size,"Invalid point.");
            c=index/2; entry=index%2+v->entry_side; exit_row=index%2+1-v->entry_side;
            if (entry!=row || (row==1 && column!=c) || v->type!=layout->type[index] ||
                (v->type!='A' && v->type!='B')) return bad(error,size,"Inconsistent route topology.");
            travel=atan2(m->entry[c][1-v->entry_side].y-m->entry[c][v->entry_side].y,
                         m->entry[c][1-v->entry_side].x-m->entry[c][v->entry_side].x);
            if (row!=1) {
                yaw=travel;
                snprintf(name,sizeof(name),"col%d_%s_entry",c+1,row==0 ? "top" : "bottom");
                if (!add(p,e,v->point,name,m->entry[c][row/2],yaw,1,MOTION_AUTO,PHASE_ALIGN,0)) goto overflow;
            }
            column=c;
            mode=cos(yaw-travel)>0 ? MOTION_FORWARD_ONLY : MOTION_REVERSE_ONLY;
            if (v->inspect) {
                if ((seen & (1u<<index)) || mode!=MOTION_FORWARD_ONLY) return bad(error,size,"Repeated or non-forward inspection.");
                snprintf(name,sizeof(name),"inspect_%d",v->point);
                if (!add(p,e,v->point,name,m->inspection[index],yaw,1,mode,PHASE_INSPECT,1)) goto overflow;
                seen|=1u<<index;
            } else if (!(seen & (1u<<index))) return bad(error,size,"Uninspected transit.");
            if (v->type=='B') {
                if (!v->inspect) return bad(error,size,"Repeated B entry.");
                mode=MOTION_REVERSE_ONLY; row=entry;
            } else row=exit_row;
            /* No middle stop. A return across inspected A goes directly to outer entry. */
            if (row!=1) {
                snprintf(name,sizeof(name),"col%d_%s_entry",c+1,row==0 ? "top" : "bottom");
                if (!add(p,e,v->point,name,m->entry[c][row/2],yaw,1,mode,
                         mode==MOTION_REVERSE_ONLY ? PHASE_RETREAT : PHASE_PASS,0)) goto overflow;
            }
        } else return bad(error,size,"Unknown route event.");
    }
    if (!p->count || p->commands[p->count-1].phase!=PHASE_EXIT) return bad(error,size,"Missing exit.");
    return 1;
overflow:
    return bad(error,size,"Too many commands.");
}
const char *motion_mode_name(MotionMode mode)
{ return mode==MOTION_AUTO ? "AUTO" : mode==MOTION_FORWARD_ONLY ? "FORWARD_ONLY" : "REVERSE_ONLY"; }
const char *motion_phase_name(MotionPhase phase)
{
    static const char *names[]={"ALIGN","ENTER","INSPECT","PASS","RETREAT","CLEAR","OUTER","EXIT"};
    return phase>=PHASE_ALIGN && phase<=PHASE_EXIT ? names[phase] : "INVALID";
}
void print_motion_plan(const MotionPlan *p)
{
    printf("\n地图 %s / %s，坐标系 %s，参考点：前轴中点\n",p->map_id,p->map_version,p->frame_id);
    printf("起点 (%.3f, %.3f)，初始朝向由定位提供。\n",p->start.x,p->start.y);
    printf("任务  目标点                  阶段      模式           坐标(x,y)        巡检\n");
    for (int i=0;i<p->count;++i) {
        const MotionCommand *c=&p->commands[i];
        printf("%3d   %-23s %-8s %-13s (%7.3f,%7.3f) %s\n",c->task_id,c->target_id,
               motion_phase_name(c->phase),motion_mode_name(c->mode),c->target.x,c->target.y,c->inspect ? "是" : "否");
    }
    printf("入口朝向由通道推导；通道内保持朝向。绕行/终点不限定最终朝向。\n");
    printf("这是任务目标计划，尚未验证整车避障、转弯可行性，不会直接控制实车。\n");
}
int export_motion_plan(const char *path, const MotionPlan *p)
{
    FILE *f=fopen(path,"w"); int ok=1;
    if (!f) return 0;
    for (int i=0;i<p->count;++i) {
        const MotionCommand *c=&p->commands[i];
        if (fprintf(f,"{\"plan_only\":true,\"map_id\":\"%s\",\"map_version\":\"%s\",\"frame_id\":\"%s\","
            "\"target_reference\":\"front_axle_midpoint\",\"start_x\":%.9f,\"start_y\":%.9f,"
            "\"task_id\":%d,\"target_id\":\"%s\",\"source_event\":%d,\"region_id\":%d,\"x\":%.9f,\"y\":%.9f,"
            "\"yaw_rad\":%.9f,\"enforce_yaw\":%s,\"motion_mode\":\"%s\",\"phase\":\"%s\",\"inspect\":%s,"
            "\"stop_at_goal\":true,\"path_scope\":\"%s\",\"column_id\":%d}\n",
            p->map_id,p->map_version,p->frame_id,p->start.x,p->start.y,c->task_id,c->target_id,c->source_event,
            c->region_id,c->target.x,c->target.y,c->target.yaw,c->enforce_yaw ? "true" : "false",
            motion_mode_name(c->mode),motion_phase_name(c->phase),c->inspect ? "true" : "false",
            c->mode==MOTION_AUTO ? "PUBLIC_ONLY" : "CHANNEL_WITH_PORTALS",
            c->region_id ? (c->region_id+1)/2 : 0)<0) ok=0;
    }
    if (fclose(f)!=0) ok=0;
    return ok;
}
