#define _CRT_SECURE_NO_WARNINGS
#include "motion.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

static int fail(char *error, size_t size, const char *message)
{ snprintf(error,size,"%s",message); return 0; }
static char *trim(char *s)
{
    char *end;
    while (isspace((unsigned char)*s)) ++s;
    end=s+strlen(s);
    while (end>s && isspace((unsigned char)end[-1])) --end;
    *end=0; return s;
}
static int line_read(FILE *f, char *line, size_t size, int first)
{
    if (!fgets(line,(int)size,f)) return ferror(f) ? -1 : 0;
    if (!strchr(line,'\n') && !feof(f)) return -1;
    if (first && strlen(line)>=3 && (unsigned char)line[0]==0xef &&
        (unsigned char)line[1]==0xbb && (unsigned char)line[2]==0xbf)
        memmove(line,line+3,strlen(line+3)+1);
    return 1;
}
static int safe_token(const char *s)
{
    if (!*s || strlen(s)>=64) return 0;
    for (;*s;++s) if (!((*s>='a' && *s<='z') || (*s>='A' && *s<='Z') ||
        (*s>='0' && *s<='9') || *s=='_' || *s=='-' || *s=='.' || *s=='/')) return 0;
    return 1;
}
static int number(char *s, double *value)
{
    char *end; s=trim(s);
    *value=strtod(s,&end);
    return end!=s && !*trim(end) && isfinite(*value);
}
static Pose2 *point(MotionMap *m, int index, char *name, size_t size)
{
    if (index<10) {
        snprintf(name,size,"col%d_%s_entry",index/2+1,index%2 ? "bottom" : "top");
        return &m->entry[index/2][index%2];
    }
    if (index<20) { snprintf(name,size,"inspect_%d",index-9); return &m->inspection[index-10]; }
    if (index<24) {
        snprintf(name,size,"outer_%s_%s",index<22 ? "left" : "right",index%2 ? "bottom" : "top");
        return &m->outer[(index-20)/2][index%2];
    }
    snprintf(name,size,"%s",index==24 ? "start" : "end");
    return index==24 ? &m->start : &m->end;
}
void motion_map_example(MotionMap *m)
{
    memset(m,0,sizeof(*m));
    strcpy(m->map_id,"demo_field"); strcpy(m->map_version,"demo_v1"); strcpy(m->frame_id,"map_demo");
    for (int c=0;c<5;++c) {
        m->entry[c][0]=(Pose2){1.2*c,4.8,0}; m->entry[c][1]=(Pose2){1.2*c,-0.8,0};
        m->inspection[2*c]=(Pose2){1.2*c,3,0}; m->inspection[2*c+1]=(Pose2){1.2*c,1,0};
    }
    m->outer[0][0]=(Pose2){-1,4.8,0}; m->outer[0][1]=(Pose2){-1,-0.8,0};
    m->outer[1][0]=(Pose2){5.8,4.8,0}; m->outer[1][1]=(Pose2){5.8,-0.8,0};
    m->start=(Pose2){-0.5,4.8,0}; m->end=(Pose2){-1,-1.8,0};
}
int motion_map_load(const char *info_path, const char *points_path, MotionMap *out, char *error, size_t size)
{
    MotionMap m={0};
    FILE *f=fopen(info_path,"r");
    char line[512]; unsigned seen=0; int status, first=1;
    const char *keys[]={"map_id","map_version","frame_id","units","target_reference"};
    if (!f) return fail(error,size,"Cannot open map_info.txt.");
    while ((status=line_read(f,line,sizeof(line),first))>0) {
        char *s=trim(line), *eq, *value; int k;
        first=0; if (!*s || *s=='#') continue;
        eq=strchr(s,'='); if (!eq) goto invalid_info;
        *eq=0; s=trim(s); value=trim(eq+1);
        for (k=0;k<5;++k) if (!strcmp(s,keys[k])) break;
        if (k==5 || (seen & (1u<<k)) || !safe_token(value)) goto invalid_info;
        if (k==0) strcpy(m.map_id,value);
        if (k==1) strcpy(m.map_version,value);
        if (k==2) strcpy(m.frame_id,value);
        if ((k==3 && strcmp(value,"m")) || (k==4 && strcmp(value,"front_axle_midpoint"))) goto invalid_info;
        seen|=1u<<k;
    }
    fclose(f);
    if (status<0 || seen!=31) return fail(error,size,"Invalid/incomplete map metadata (5 unique fields required).");
    f=fopen(points_path,"r");
    if (!f) return fail(error,size,"Cannot open points.csv.");
    status=line_read(f,line,sizeof(line),1);
    if (status!=1 || strcmp(trim(line),"point_id,x,y")) goto invalid_points;
    seen=0;
    while ((status=line_read(f,line,sizeof(line),0))>0) {
        char *s=trim(line), *x, *y, name[48]; Pose2 *p=NULL; int k;
        if (!*s) continue;
        x=strchr(s,','); if (!x) goto invalid_points; *x++=0;
        y=strchr(x,','); if (!y) goto invalid_points; *y++=0;
        if (strchr(y,',')) goto invalid_points;
        for (k=0;k<26;++k) { p=point(&m,k,name,sizeof(name)); if (!strcmp(trim(s),name)) break; }
        if (k==26 || (seen & (1u<<k)) || !number(x,&p->x) || !number(y,&p->y)) goto invalid_points;
        seen|=1u<<k;
    }
    fclose(f);
    if (status<0 || seen!=((1u<<26)-1)) return fail(error,size,"points.csv requires exactly 26 unique named points.");
    if (!motion_map_valid(&m)) return fail(error,size,"Invalid channel geometry: top/inspect order/bottom; centreline deviation exceeds 0.05 m.");
    *out=m; return 1;
invalid_info:
    fclose(f); return fail(error,size,"Invalid metadata: names, duplicates, units or target reference.");
invalid_points:
    fclose(f); return fail(error,size,"Invalid CSV: header, point name, duplicate, non-finite number or long line.");
}
