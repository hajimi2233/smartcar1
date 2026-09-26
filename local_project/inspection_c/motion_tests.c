#define _CRT_SECURE_NO_WARNINGS
#include "motion.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while (0)
static int same(Pose2 a, Pose2 b) { return hypot(a.x-b.x,a.y-b.y)<1e-8; }
static void validate(const Layout *l, const Plan *r, const MotionMap *m, const MotionPlan *p)
{
    Pose2 prev=p->start; unsigned seen=0; MotionSession s; int inspected_count=0;
    CHECK(same(p->start,m->start)); CHECK(!strcmp(p->frame_id,m->frame_id));
    motion_session_begin(&s,p,42);
    for (int i=0;i<p->count;++i) {
        const MotionCommand *c=&p->commands[i];
        CHECK(c->task_id==i+1 && c->source_event>=0 && c->source_event<r->event_count);
        CHECK(c->phase!=PHASE_ENTER && c->phase!=PHASE_CLEAR);
        if (c->mode!=MOTION_AUTO) {
            double dx=c->target.x-prev.x, dy=c->target.y-prev.y;
            double along=dx*cos(c->target.yaw)+dy*sin(c->target.yaw);
            CHECK(c->enforce_yaw && fabs(c->target.yaw-prev.yaw)<1e-8);
            CHECK(fabs(dx*sin(c->target.yaw)-dy*cos(c->target.yaw))<1e-8);
            CHECK(c->mode==MOTION_FORWARD_ONLY ? along>0 : along<0);
            /* No segment may cross a B centre; reaching it and reversing is allowed. */
            for (int k=0;k<l->count;++k) if (l->type[k]=='B') {
                Pose2 b=m->inspection[k];
                double cross=fabs((b.x-prev.x)*dy-(b.y-prev.y)*dx);
                double dot=(b.x-prev.x)*dx+(b.y-prev.y)*dy;
                CHECK(!(cross<1e-8 && dot>1e-8 && dot<dx*dx+dy*dy-1e-8));
            }
        } else {
            CHECK(!c->inspect);
            CHECK(c->enforce_yaw==(c->phase==PHASE_ALIGN));
            CHECK(i==0 || p->commands[i-1].phase!=PHASE_INSPECT);
        }
        if (c->inspect) {
            int region=c->region_id-1;
            CHECK(c->phase==PHASE_INSPECT && c->mode==MOTION_FORWARD_ONLY);
            CHECK(region>=0 && region<l->count && !(seen & (1u<<region)));
            CHECK(same(c->target,m->inspection[region]));
            CHECK(c->region_id==r->order[inspected_count++]);
            seen|=1u<<region;
        }
        if (c->phase==PHASE_RETREAT) CHECK(c->mode==MOTION_REVERSE_ONLY && !c->inspect);
        CHECK(motion_session_current(&s)==c);
        CHECK(!motion_session_ack(&s,41,c->task_id,1));
        CHECK(!motion_session_ack(&s,42,c->task_id+1,1));
        CHECK(motion_session_ack(&s,42,c->task_id,1));
        CHECK(!motion_session_ack(&s,42,c->task_id,1));
        CHECK(s.inspected==seen); prev=c->target;
    }
    CHECK(seen==(1u<<l->count)-1 && s.status==SESSION_DONE);
    CHECK(p->commands[p->count-1].phase==PHASE_EXIT && same(prev,m->end));
    motion_session_begin(&s,p,43); CHECK(motion_session_ack(&s,43,1,0));
    CHECK(s.status==SESSION_FAILED && !motion_session_current(&s));
}
static Pose2 rotate(Pose2 p)
{ return (Pose2){10+p.x*cos(0.7)-p.y*sin(0.7),-4+p.x*sin(0.7)+p.y*cos(0.7),0}; }
static void write_info(int invalid)
{
    FILE *f=fopen("build/test_info.txt","wb"); CHECK(f);
    fprintf(f,"\xef\xbb\xbfmap_id=test\nmap_version=v2\nframe_id=map\nunits=%s\ntarget_reference=front_axle_midpoint\n",invalid ? "cm" : "m");
    CHECK(!fclose(f));
}
/* Exercise import independently of the supplied/demo coordinates. */
static void write_points(int fault)
{
    FILE *f=fopen("build/test_points.csv","wb"); CHECK(f);
    fputs("\xef\xbb\xbfpoint_id,x,y\r\n",f);
    for (int c=0;c<5;++c) {
        fprintf(f,"col%d_top_entry, %d, 5\r\ncol%d_bottom_entry,%d,-1\r\n",c+1,c,c+1,c);
    }
    for (int k=1;k<=10;++k) {
        if (fault==1 && k==10) continue;
        if (fault==3 && k==1) fputs("inspect_1,nan,3\n",f);
        else fprintf(f,"inspect_%d,%d,%d\n",k,(k-1)/2,fault==4 && k==1 ? -2 : (k%2 ? 3 : 1));
    }
    fputs("map_left_top,-1,5\nmap_left_bottom,-1,-1\nmap_right_top,5,5\nmap_right_bottom,5,-1\nstart,-0.3,5\nend,-1,-2\n",f);
    if (fault==2) fputs("inspect_1,0,3\n",f);
    if (fault==5) fputs("unknown,1,2\n",f);
    CHECK(!fclose(f));
}
int main(void)
{
    Layout l; Plan r; MotionMap m; MotionPlan p; char error[200]; int cases=0;
    CHECK(motion_map_load("map_info.txt","points.csv",&m,error,sizeof(error)));
    /* Use a controlled fixture so editing production CSV never invalidates tests. */
    motion_map_example(&m);
    for (int orientation=0;orientation<2;++orientation) {
        if (orientation) {
            for (int c=0;c<5;++c) for (int s=0;s<2;++s) m.entry[c][s]=rotate(m.entry[c][s]);
            for (int k=0;k<10;++k) m.inspection[k]=rotate(m.inspection[k]);
            for (int a=0;a<2;++a) for (int b=0;b<2;++b) m.outer[a][b]=rotate(m.outer[a][b]);
            m.start=rotate(m.start); m.end=rotate(m.end);
        }
        for (int n=2;n<=10;n+=2) for (unsigned mask=0;mask<(1u<<n);++mask) {
            l.count=n; for (int i=0;i<n;++i) l.type[i]=(mask & (1u<<i)) ? 'B' : 'A';
            CHECK(make_plan(&l,&r)); CHECK(make_motion_plan(&l,&r,&m,&p,error,sizeof(error)));
            validate(&l,&r,&m,&p); ++cases;
        }
    }
    motion_map_example(&m);
    CHECK(parse_layout("A1B2A3B4",&l,error,sizeof(error)) && make_plan(&l,&r));
    CHECK(make_motion_plan(&l,&r,&m,&p,error,sizeof(error)));
    CHECK(p.count==11 && p.commands[3].phase==PHASE_RETREAT);
    CHECK(!strcmp(p.commands[3].target_id,"col1_top_entry"));
    CHECK(same(p.commands[2].target,m.inspection[1]));
    CHECK(p.commands[2].target.yaw==p.commands[3].target.yaw);
    r.events[0].inspect=0; CHECK(!make_motion_plan(&l,&r,&m,&p,error,sizeof(error)));
    write_info(0); write_points(0);
    CHECK(motion_map_load("build/test_info.txt","build/test_points.csv",&m,error,sizeof(error)));
    CHECK(m.start.x==-0.3 && !strcmp(m.map_version,"v2"));
    for (int fault=1;fault<=5;++fault) {
        MotionMap old=m; write_points(fault);
        CHECK(!motion_map_load("build/test_info.txt","build/test_points.csv",&m,error,sizeof(error)));
        CHECK(!memcmp(&old,&m,sizeof(m)));
    }
    write_points(0); write_info(1);
    CHECK(!motion_map_load("build/test_info.txt","build/test_points.csv",&m,error,sizeof(error)));
    printf("PASS: %d motion cases (1364 layouts x 2 map rotations), direct retreats, shared centres,\n"
           "      imported start/end, feedback sequencing and CSV validation/atomic rejection.\n",cases);
    return 0;
}
