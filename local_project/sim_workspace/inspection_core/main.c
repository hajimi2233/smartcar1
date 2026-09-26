#include "planner.h"
#include "motion.h"

#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif

static void print_plan(const Layout *layout, const Plan *plan)
{
    printf("\n场地（同列上下连接，四周有公共通道）：\n");
    for (int side = 0; side < 2; ++side) {
        printf("  |");
        for (int i = side; i < layout->count; i += 2) {
            printf(" %c%-2d |", layout->type[i], i + 1);
        }
        printf("\n");
    }
    printf("\n巡检顺序：start");
    for (int i = 0; i < plan->order_count; ++i) printf(" -> %d", plan->order[i]);
    if (plan->complete) printf(" -> end");
    printf("\n完整行程：start");
    for (int i = 0; i < plan->event_count; ++i) {
        const PlanEvent *event = &plan->events[i];
        if (event->kind == EVENT_END) {
            printf(" -> end");
            continue;
        }
        if (event->kind != EVENT_POINT) {
            printf(" -> [%s外部绕行]", event->kind == EVENT_OUTER_LOW ? "左侧" : "右侧");
            continue;
        }
        if (event->inspect) printf(" -> %d", event->point);
        else printf(" -> (%d)", event->point);
    }
    printf("\n括号表示借道，不再巡检或播报；B 点表示进入后原侧退出。\n");
    printf("\n动作明细：\n");
    for (int i = 0; i < plan->event_count; ++i) {
        const PlanEvent *event = &plan->events[i];
        if (event->kind == EVENT_END) {
            printf("  %2d. 沿下方公共通道前往配置的 end，不巡检、不播报\n", i+1);
            continue;
        }
        if (event->kind != EVENT_POINT) {
            printf("  %2d. %s外部绕行：%s -> %s，不进入巡检区\n", i+1,
                   event->kind == EVENT_OUTER_LOW ? "左侧" : "右侧",
                   event->entry_side == 0 ? "上方" : "下方",
                   event->entry_side == 0 ? "下方" : "上方");
            continue;
        }
        printf("  %2d. %c%-2d 从%s方向接近：%s；%s\n", i+1, event->type,
               event->point, event->entry_side == 0 ? "上方" : "下方",
               event->inspect ? "首次巡检" : "仅借道、不播报",
               event->type == 'A' ? "通过" : "原侧退出");
    }
    printf("\n巡检完成：%d/%d；规划终点：%s。\n", plan->order_count,
           layout->count, plan->complete ? "end（坐标由文件提供）" : "未完成");
    if (!plan->complete) {
        printf("无法到达的巡检点：");
        for (int i = 0; i < layout->count; ++i) {
            if (!(plan->visited & (1u << i))) printf(" %d", i+1);
        }
        printf("\n规划未完成，请检查输入及通路模型。\n");
    }
    printf("本版为文本路线规划；实际出口坐标和到达反馈由导航执行端接入。\n\n");
}

static int show_commands(const Layout *layout, const Plan *route, const MotionMap *map)
{
    MotionPlan commands;
    char error[160];
    if (!make_motion_plan(layout,route,map,&commands,error,sizeof(error))) {
        fprintf(stderr,"姿态指令生成失败：%s\n",error);
        return 0;
    }
    print_plan(layout,route);
    print_motion_plan(&commands);
    if (!export_motion_plan("motion_commands.jsonl",&commands)) {
        fprintf(stderr,"无法写入 motion_commands.jsonl。\n");
        return 0;
    }
    printf("已导出 motion_commands.jsonl（本次成功结果覆盖上次导出）。\n");
    return 1;
}

int main(int argc, char **argv)
{
    char input[1024], error[160];
    Layout layout;
    Plan plan;
    MotionMap map;
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    if (argc != 1 && argc != 2 && argc != 4) {
        fprintf(stderr, "Usage: inspection_planner_v6.exe \"B1B2A3A4\" [map_info.txt points.csv]\n");
        return 1;
    }
    if (!motion_map_load(argc==4 ? argv[2] : "map_info.txt",argc==4 ? argv[3] : "points.csv",&map,error,sizeof(error))) {
        fprintf(stderr,"坐标配置错误：%s\n请从程序目录使用 run.cmd 启动。\n",error);
        return 1;
    }
    if (argc >= 2) {
        if (!parse_layout(argv[1], &layout, error, sizeof(error))) {
            fprintf(stderr, "输入错误 / Input error: %s\n", error);
            return 1;
        }
        if (!make_plan(&layout, &plan)) return 1;
        if (!show_commands(&layout,&plan,&map)) return 1;
        return plan.complete ? 0 : 2;
    }
    printf("巡检顺序规划器 V6（26 点 CSV，前轴中点，纯 C）\n输入 2~10 个点，必须为完整列，例如 B1B2A3A4。\n");
    printf("输入 q 退出，r 重新导入。每次规划前也会自动重读文件。\n");
    printf("起点需位于上方公共区，终点需可由下方公共区到达。\n");
    printf("已导入 %s / %s (%s)。自带文件为演示坐标，不会控制实车。\n",map.map_id,map.map_version,map.frame_id);
    for (;;) {
        printf("\n请输入 A/B 编号串 > ");
        fflush(stdout);
        if (fgets(input, sizeof(input), stdin) == NULL) break;
        if (!strchr(input, '\n') && !feof(stdin)) {
            int ch;
            while ((ch = getchar()) != '\n' && ch != EOF) {}
            printf("输入过长。\n");
            continue;
        }
        if (strcmp(input, "q\n") == 0 || strcmp(input, "Q\n") == 0 ||
            strcmp(input, "q") == 0 || strcmp(input, "Q") == 0) break;
        if (!motion_map_load("map_info.txt","points.csv",&map,error,sizeof(error))) {
            printf("导入失败：%s\n本次不生成任务，请修正文件后重试。\n",error);
            continue;
        }
        if (strcmp(input,"r\n")==0 || strcmp(input,"r")==0 || strcmp(input,"R\n")==0 || strcmp(input,"R")==0) {
            printf("已重新导入 26 点：%s / %s (%s)。\n",map.map_id,map.map_version,map.frame_id);
            continue;
        }
        if (!parse_layout(input, &layout, error, sizeof(error))) {
            printf("输入错误 / Input error: %s\n", error);
            continue;
        }
        if (!make_plan(&layout, &plan)) {
            fprintf(stderr, "内部规划错误。\n");
            return 1;
        }
        if (!show_commands(&layout,&plan,&map)) continue;
    }
    return 0;
}
