/* Coordinate adapter for the original A/B route planner. No entrance goals. */
#include "planner.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

typedef struct { double x, y; } Point;
static const char *names[16] = {
    "inspect_1", "inspect_2", "inspect_3", "inspect_4", "inspect_5",
    "inspect_6", "inspect_7", "inspect_8", "inspect_9", "inspect_10",
    "outer_left_top", "outer_left_bottom", "outer_right_top", "outer_right_bottom", "start", "end"
};
static int error(const char *text) { fprintf(stderr, "%s\n", text); return 1; }
static char *trim(char *s) {
    while (isspace((unsigned char)*s)) ++s;
    char *end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) --end;
    *end = 0; return s;
}
static int number(char *s, double *out) {
    char *end; s = trim(s); *out = strtod(s, &end);
    return end != s && !*trim(end) && isfinite(*out);
}
static int read_points(const char *path, Point points[16]) {
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    char line[512]; unsigned seen = 0;
    if (!fgets(line, sizeof(line), f) || strcmp(trim(line), "point_id,x,y")) goto bad;
    while (fgets(line, sizeof(line), f)) {
        if (!strchr(line, '\n') && !feof(f)) goto bad;
        char *s = trim(line), *x, *y;
        if (!*s) continue;
        x = strchr(s, ','); if (!x) goto bad; *x++ = 0;
        y = strchr(x, ','); if (!y) goto bad; *y++ = 0;
        if (strchr(y, ',')) goto bad;
        int i; for (i = 0; i < 16; ++i) if (!strcmp(trim(s), names[i])) break;
        if (i == 16 || (seen & (1u << i)) || !number(x, &points[i].x) ||
            !number(y, &points[i].y)) goto bad;
        seen |= 1u << i;
    }
    if (ferror(f) || seen != (1u << 16)-1) goto bad;
    fclose(f); return 1;
bad:
    fclose(f); return 0;
}
int main(int argc, char **argv) {
    Layout layout; Plan route; Point p[16]; char message[256];
    if (argc != 3) return error("Usage: inspection_plan14 A1B2...A10 points.csv (map frame, front axle midpoint)");
    if (!parse_layout(argv[1], &layout, message, sizeof(message))) return error(message);
    if (layout.count != 10) return error("The 16-point plan requires all 10 inspection types.");
    if (!read_points(argv[2], p)) return error("Expected exactly 16 unique named finite points; header: point_id,x,y");
    double yaw[5], down_x = 0, down_y = 0;
    for (int c = 0; c < 5; ++c) {
        double dx = p[2*c+1].x-p[2*c].x, dy = p[2*c+1].y-p[2*c].y;
        double length = hypot(dx, dy);
        if (!isfinite(length) || length < .01) return error("Paired inspection points must be distinct (at least 1 cm).");
        yaw[c] = atan2(dy, dx); down_x += dx/length; down_y += dy/length;
    }
    if (hypot(down_x, down_y) < 1e-6) return error("Inconsistent odd-to-even channel directions.");
    double down = atan2(down_y, down_x);
    for (int c = 0; c < 5; ++c)
        if (cos(yaw[c]-down) <= 0) return error("Odd/even channel points have reversed numbering.");
    /* Fixed numbering: 1/2 on the right, 9/10 on the left. */
    const int low_outer = 12;
    for (int b = 10; b <= 12; b += 2) {
        double dx=p[b+1].x-p[b].x, dy=p[b+1].y-p[b].y;
        if (hypot(dx,dy)<.01 || dx*cos(down)+dy*sin(down)<=0)
            return error("Outer top/bottom points must be distinct and ordered like channel odd/even points.");
    }
    if (!make_plan(&layout, &route) || !route.complete) return error("Could not generate a complete route.");
    Point previous = p[14]; double last_yaw = down; int task = 0;
    for (int e = 0; e < route.event_count; ++e) {
        PlanEvent v = route.events[e];
        int outer = v.kind == EVENT_OUTER_LOW || v.kind == EVENT_OUTER_HIGH;
        for (int step = 0; step < (outer ? 2 : 1); ++step) {
        int index;
        double heading;
        if (v.kind == EVENT_POINT) {
            /* Silent return/transit events belong to navigation, not extra goals. */
            if (!v.inspect) continue;
            index = v.point-1;
            heading = yaw[index/2] + (v.entry_side ? acos(-1.) : 0.);
        } else if (v.kind == EVENT_OUTER_LOW || v.kind == EVENT_OUTER_HIGH) {
            int base = v.kind == EVENT_OUTER_LOW ? low_outer : 10;
            int side = step ? 1-v.entry_side : v.entry_side;
            index = base+side;
            heading = atan2(p[base+1-v.entry_side].y-p[base+v.entry_side].y,
                            p[base+1-v.entry_side].x-p[base+v.entry_side].x);
        } else if (v.kind == EVENT_END) {
            index = 15;
            heading = hypot(p[index].x-previous.x, p[index].y-previous.y) < .001 ?
                last_yaw : atan2(p[index].y-previous.y, p[index].x-previous.x);
        } else return error("Unknown route event.");
        heading = atan2(sin(heading), cos(heading));
        printf("{\"task_id\":%d,\"target_id\":\"%s\",\"frame_id\":\"map\","
               "\"target_reference\":\"front_axle_midpoint\",\"x\":%.17g,\"y\":%.17g,"
               "\"yaw_rad\":%.17g,\"inspection_type\":\"%c\",\"inspect\":%s}\n",
               ++task, names[index], p[index].x, p[index].y, heading,
               index < 10 ? layout.type[index] : '-', v.inspect && v.kind == EVENT_POINT ? "true" : "false");
        previous = p[index]; last_yaw = heading;
        }
    }
    return ferror(stdout) ? 1 : 0;
}
