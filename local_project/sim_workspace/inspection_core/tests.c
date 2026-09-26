#include "planner.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(test) do { if (!(test)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #test); exit(1); \
} } while (0)

/* Independent physical-graph flood fill: top/middle/bottom per column.
   Only A connects adjacent boundaries; B can be inspected from either end. */
static unsigned reachable_tasks(const Layout *layout)
{
    int reach[5][3] = {{0}};
    int changed = 1, columns = layout->count / 2;
    unsigned result = 0;
    reach[0][0] = 1;
    while (changed) {
        changed = 0;
        for (int c = 0; c < columns; ++c) {
            for (int r = 0; r < 3; ++r) {
                if (!reach[c][r]) continue;
                if ((c == 0 || c == columns-1) && r != 1 && !reach[c][2-r]) {
                    reach[c][2-r] = 1; changed = 1;
                }
                if (r != 1) {
                    if (c > 0 && !reach[c-1][r]) {reach[c-1][r]=1;changed=1;}
                    if (c+1 < columns && !reach[c+1][r]) {reach[c+1][r]=1;changed=1;}
                }
                if (r > 0 && layout->type[2*c+r-1] == 'A' && !reach[c][r-1]) {
                    reach[c][r-1]=1;changed=1;
                }
                if (r < 2 && layout->type[2*c+r] == 'A' && !reach[c][r+1]) {
                    reach[c][r+1]=1;changed=1;
                }
            }
        }
    }
    for (int i = 0; i < layout->count; ++i) {
        if (reach[i/2][i%2] || reach[i/2][i%2+1]) result |= 1u << i;
    }
    return result;
}

static void validate_trace(const Layout *layout, const Plan *plan)
{
    int column = 0, row = 0, order_index = 0, end_count = 0;
    unsigned seen = 0;
    for (int j = 0; j < plan->event_count; ++j) {
        const PlanEvent *event = &plan->events[j];
        int i = event->point - 1;
        int entry_row;
        if (event->kind == EVENT_END) {
            CHECK(j == plan->event_count-1);
            CHECK(seen == (1u << layout->count)-1u);
            CHECK(row == 2 && event->entry_side == 1);
            CHECK(!event->inspect && event->point == 0);
            column = 0; /* Bottom public corridor joins every column to the exit. */
            ++end_count;
            continue;
        }
        if (event->kind != EVENT_POINT) {
            CHECK(event->kind == EVENT_OUTER_LOW || event->kind == EVENT_OUTER_HIGH);
            CHECK(row == 0 || row == 2);
            CHECK(event->entry_side == row/2);
            CHECK(!event->inspect && event->point == 0);
            column = event->kind == EVENT_OUTER_LOW ? 0 : layout->count/2-1;
            row = 2-row;
            continue;
        }
        CHECK(i >= 0 && i < layout->count);
        CHECK(event->entry_side == 0 || event->entry_side == 1);
        entry_row = i % 2 + event->entry_side;
        if (column != i/2) {
            CHECK(row != 1); /* No horizontal travel through internal walls. */
            CHECK(row == entry_row);
            column = i/2;
        }
        CHECK(row == entry_row);
        CHECK(event->type == layout->type[i]);
        CHECK(event->inspect == ((seen & (1u << i)) == 0));
        if (event->inspect) {
            CHECK(order_index < plan->order_count);
            CHECK(plan->order[order_index++] == event->point);
            seen |= 1u << i;
        }
        if (event->type == 'A') row += event->entry_side == 0 ? 1 : -1;
        else CHECK(event->inspect); /* B returns to same side, no repeat probes. */
    }
    CHECK(row == 0 || row == 2);
    CHECK(end_count == 1 && row == 2 && column == 0);
    CHECK(plan->final_side == row/2);
    CHECK(plan->final_column == column);
    CHECK(plan->visited == seen);
    CHECK(plan->visited == reachable_tasks(layout));
    CHECK(order_index == plan->order_count);
    CHECK(plan->complete == (seen == (1u << layout->count)-1u));
}

static void expect_order(const char *input, const int *expected, int count)
{
    Layout layout;
    Plan plan;
    char error[160];
    CHECK(parse_layout(input, &layout, error, sizeof(error)));
    CHECK(make_plan(&layout, &plan));
    CHECK(plan.order_count == count);
    for (int i = 0; i < count; ++i) CHECK(plan.order[i] == expected[i]);
    validate_trace(&layout, &plan);
}

int main(void)
{
    const int sample1[] = {1,2,3,4};
    const int sample2[] = {1,3,4,2,6,5,7,8,10,9};
    /* Already inspected A5 is not re-inspected after the outer bypass. */
    const int backfill[] = {1,2,4,6,5,3};
    const int outer_regression[] = {1,2,4,6,8,10,9,7,5,3};
    const int all_blocked[] = {1,3,5,7,9,10,8,6,4,2};
    const int nearest_backfill[] = {1,3,5,6,4,2,8,7,9,10};
    const char *invalid[] = {"", "A1", "A1B1", "A1A4", "C1A2", "A0B2",
                             "A1B", "A1B2!", "A11B2", "A999999999999B2"};
    Layout layout;
    Plan plan;
    char error[160];
    int combinations = 0;
    expect_order("A1B2A3A4", sample1, 4);
    expect_order("B1B2A3A4A5A6A7A8A9A10", sample2, 10);
    expect_order("A1A2B3B4B5A6", backfill, 6);
    expect_order("A1A2B3B4A5B6A7B8A9B10", outer_regression, 10);
    expect_order("B1B2B3B4B5B6B7B8B9B10", all_blocked, 10);
    expect_order("B1B2B3B4A5A6A7A8A9A10", nearest_backfill, 10);
    CHECK(parse_layout("A1A2B3B4A5B6A7B8A9B10", &layout, error, sizeof(error)));
    CHECK(make_plan(&layout, &plan));
    CHECK(plan.events[6].kind == EVENT_OUTER_HIGH);
    CHECK(plan.events[7].point == 9 && plan.events[7].inspect);
    CHECK(parse_layout("A1A2", &layout, error, sizeof(error)));
    CHECK(make_plan(&layout, &plan));
    CHECK(plan.event_count == 3 && plan.events[2].kind == EVENT_END);
    CHECK(parse_layout("A1B2", &layout, error, sizeof(error)));
    CHECK(make_plan(&layout, &plan));
    CHECK(plan.events[plan.event_count-2].kind == EVENT_OUTER_LOW);
    CHECK(plan.events[plan.event_count-1].kind == EVENT_END);
    for (size_t i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i) {
        CHECK(!parse_layout(invalid[i], &layout, error, sizeof(error)));
    }
    CHECK(parse_layout(" b2, a1 ", &layout, error, sizeof(error)));
    CHECK(layout.count == 2 && layout.type[0] == 'A' && layout.type[1] == 'B');
    CHECK(!parse_layout(NULL, &layout, error, sizeof(error)));
    CHECK(!make_plan(NULL, &plan));
    for (int count = 2; count <= MAX_POINTS; count += 2) {
        for (unsigned mask = 0; mask < (1u << count); ++mask) {
            layout.count = count;
            for (int i = 0; i < count; ++i) layout.type[i] = (mask & (1u << i)) ? 'B' : 'A';
            CHECK(make_plan(&layout, &plan));
            CHECK(plan.complete); /* The outer ring makes both sides reachable. */
            validate_trace(&layout, &plan);
            ++combinations;
        }
    }
    printf("PASS: %d layouts; physical reachability, legal movement, unique inspections,\n"
           "      silent transit, exit after all tasks, examples and invalid inputs verified.\n", combinations);
    return 0;
}
