#include "test_middleware_tools_host.h"
#include "tools_host.h"
#include "tools_print.h"

static void echo_callback(uint8 argc, uint8* argv[])
{
    (void)argc;

    for (uint8 i = 1u; i < argc; i++)
    {
        if (i > 1u)
        {
            tools_printf(" ");
        }

        tools_printf("%s", argv[i]);
    }

    tools_printf("\r\n");
}

void test_middleware_tools_host_echo(void)
{
    (void)tools_host_register((const uint8*)"echo", echo_callback);
    tools_println((const uint8*)"HOST echo command registered");
}
