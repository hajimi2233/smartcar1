#include "planner.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

static int fail(char *error, size_t size, const char *message)
{
    if (error != NULL && size > 0) {
        snprintf(error, size, "%s", message);
    }
    return 0;
}

int parse_layout(const char *text, Layout *layout, char *error, size_t error_size)
{
    unsigned seen = 0;
    int highest = 0;
    const unsigned char *p = (const unsigned char *)text;
    if (text == NULL || layout == NULL) {
        return fail(error, error_size, "Null input.");
    }
    memset(layout, 0, sizeof(*layout));
    while (*p != '\0') {
        int id = 0;
        char type;
        while (isspace(*p) || *p == ',') ++p;
        if (*p == '\0') break;
        type = (char)toupper(*p++);
        if (type != 'A' && type != 'B') {
            return fail(error, error_size, "Expected A or B before each ID.");
        }
        while (isspace(*p)) ++p;
        if (!isdigit(*p)) {
            return fail(error, error_size, "Missing point ID after A/B.");
        }
        while (isdigit(*p)) {
            id = id * 10 + (*p++ - '0');
            if (id > MAX_POINTS) {
                return fail(error, error_size, "Point ID must be between 1 and 10.");
            }
        }
        if (id == 0) {
            return fail(error, error_size, "Point ID must start at 1.");
        }
        if (seen & (1u << (id - 1))) {
            return fail(error, error_size, "Duplicate point ID.");
        }
        seen |= 1u << (id - 1);
        layout->type[id - 1] = type;
        if (id > highest) highest = id;
    }
    if (highest == 0 || highest % 2 != 0) {
        return fail(error, error_size, "Provide complete columns: 2/4/6/8/10 points.");
    }
    if (seen != (1u << highest) - 1u) {
        return fail(error, error_size, "Point IDs must be consecutive from 1.");
    }
    layout->count = highest;
    return 1;
}

static int emit(const Layout *layout, Plan *plan, int index, int side)
{
    unsigned bit = 1u << index;
    PlanEvent *event;
    if (plan->event_count >= MAX_EVENTS) return 0;
    event = &plan->events[plan->event_count++];
    event->kind = EVENT_POINT;
    event->point = index + 1;
    event->type = layout->type[index];
    event->entry_side = side;
    event->inspect = (plan->visited & bit) == 0;
    if (event->inspect) {
        plan->visited |= bit;
        plan->order[plan->order_count++] = index + 1;
    }
    return 1;
}

static int use_outer(Plan *plan, int columns, int high, int *column, int *side)
{
    PlanEvent *event;
    if (plan->event_count >= MAX_EVENTS) return 0;
    event = &plan->events[plan->event_count++];
    memset(event, 0, sizeof(*event));
    event->kind = high ? EVENT_OUTER_HIGH : EVENT_OUTER_LOW;
    event->entry_side = *side;
    *column = high ? columns - 1 : 0;
    *side = 1 - *side;
    return 1;
}

static int bypass_outer(Plan *plan, int columns, int *column, int *side)
{
    int high = *column >= columns - 1 - *column;
    return use_outer(plan, columns, high, column, side);
}

/* Exit is beside the bottom corridor at the low-numbered end.
   Top -> low outer corridor -> bottom -> exit. Never enter a patrol zone. */
static int go_to_exit(Plan *plan, int columns, int *column, int *side)
{
    PlanEvent *event;
    if (*side == 0 && !use_outer(plan, columns, 0, column, side)) return 0;
    if (plan->event_count >= MAX_EVENTS) return 0;
    event = &plan->events[plan->event_count++];
    memset(event, 0, sizeof(*event));
    event->kind = EVENT_END;
    event->entry_side = 1;
    *column = 0;
    *side = 1;
    return 1;
}

/* Process one column. All exits return to a public corridor. */
static int visit_column(const Layout *layout, Plan *plan, int column,
                        int *side, unsigned pending[2])
{
    int near_point = 2 * column + *side;
    int far_point = 2 * column + 1 - *side;
    if (layout->type[near_point] == 'B') {
        if (!(plan->visited & (1u << near_point)) &&
            !emit(layout, plan, near_point, *side)) return 0;
        if (!(plan->visited & (1u << far_point))) {
            pending[1 - *side] |= 1u << column;
        }
        return 1; /* B: probe and retreat, never cross. */
    }
    if (!emit(layout, plan, near_point, *side)) return 0;
    if (layout->type[far_point] == 'B') {
        if (!(plan->visited & (1u << far_point)) &&
            !emit(layout, plan, far_point, *side)) return 0;
        /* Return through A silently. Do not re-enter a previously inspected B. */
        return emit(layout, plan, near_point, 1 - *side);
    }
    if (!emit(layout, plan, far_point, *side)) return 0;
    *side = 1 - *side;
    return 1;
}

int make_plan(const Layout *layout, Plan *plan)
{
    unsigned pending[2] = {0, 0};
    unsigned all;
    int side = 0, next_column = 0, current_column = 0;
    int columns, iterations = 0;
    if (layout == NULL || plan == NULL || layout->count < 2 ||
        layout->count > MAX_POINTS || layout->count % 2 != 0) return 0;
    for (int i = 0; i < layout->count; ++i) {
        if (layout->type[i] != 'A' && layout->type[i] != 'B') return 0;
    }
    memset(plan, 0, sizeof(*plan));
    columns = layout->count / 2;
    all = (1u << layout->count) - 1u;
    while (plan->visited != all) {
        int column = -1;
        int best_distance = columns + 1;
        if (++iterations > 4 * MAX_POINTS) return 0;
        /* Current-side backfill: nearest column first, not fixed ascending IDs.
           Column spacing is assumed uniform; this is not metric route optimization. */
        for (int c = 0; c < columns; ++c) {
            int distance = c > current_column ? c-current_column : current_column-c;
            if (plan->visited & (1u << (2*c + side))) {
                pending[side] &= ~(1u << c);
            }
            if ((pending[side] & (1u << c)) && distance < best_distance) {
                column = c;
                best_distance = distance;
            }
        }
        if (column >= 0) {
            pending[side] &= ~(1u << column);
        } else if (next_column < columns) {
            column = next_column++;
        } else {
            /* Both outer ends join top/bottom without entering a patrol zone. */
            if (!bypass_outer(plan, columns, &current_column, &side)) return 0;
            continue;
        }
        if (!visit_column(layout, plan, column, &side, pending)) return 0;
        current_column = column;
    }
    if (!go_to_exit(plan, columns, &current_column, &side)) return 0;
    plan->final_side = side;
    plan->final_column = current_column;
    plan->complete = plan->visited == all;
    return 1;
}
