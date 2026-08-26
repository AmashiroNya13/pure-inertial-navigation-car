#include "test_middleware_task.h"
#include "task.h"
#include "tools_print.h"

void test_middleware_task_hello_callback(void)
{
    tools_print((const uint8*)"hello\r\n");
}

void test_middleware_task_run(void)
{
    task_trap(TASK1);
}
