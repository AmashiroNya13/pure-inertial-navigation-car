#include "test_middleware_tools_print.h"
#include "tools_print.h"

#define TEST_PRINT_DELAY 1000000u

static void test_delay(void)
{
    for (volatile uint32 i = 0; i < TEST_PRINT_DELAY; i++) { }
}

void test_middleware_tools_print_putc(void)
{
    test_delay();
    tools_print_putc('H');
    tools_print_putc('i');
    tools_print_putc('!');
    tools_print_putc('\r');
    tools_print_putc('\n');
}

void test_middleware_tools_print_print(void)
{
    test_delay();
    tools_print((const uint8*)"tools_print: hello\r\n");
}

void test_middleware_tools_print_println(void)
{
    test_delay();
    tools_println((const uint8*)"tools_println: hello");
}

void test_middleware_tools_print_printf(void)
{
    static uint32 test_case = 0u;

    test_delay();

    switch (test_case)
    {
    case 0u:
        tools_printf("%%u: u32=%u u8=%u u16=%u\r\n",
                     (unsigned int)1234567890u,
                     (unsigned int)250u,
                     (unsigned int)60000u);
        break;
    case 1u:
        tools_printf("%%d: pos=%d neg=%d zero=%d\r\n",
                     (int)12345,
                     (int)-12345,
                     (int)0);
        break;
    case 2u:
        tools_printf("%%x %%X: hex=%x HEX=%X zero=%X\r\n",
                     (unsigned int)0xDEADBEEFu,
                     (unsigned int)0xC0FFEEu,
                     (unsigned int)0u);
        break;
    case 3u:
        tools_printf("%%lu %%ld %%lx: lu=%lu ld=%ld lx=0x%lX\r\n",
                     (unsigned long)4000000000u,
                     (long)-2000000000,
                     (unsigned long)0xABCD1234u);
        break;
    case 4u:
        tools_printf("%%s %%c: str=%s ch=%c\r\n",
                     (const char*)"hello_world",
                     'Z');
        break;
    case 5u:
        tools_printf("mixed: %s %u %s %d 0x%X %c end\r\n",
                     (const char*)"cnt",
                     (unsigned int)42u,
                     (const char*)"val",
                     (int)-100,
                     (unsigned int)0xFF,
                     '!');
        break;
    default:
        test_case = 0u;
        return;
    }

    test_case++;
}

void test_middleware_tools_print_assert(void)
{
    static uint32 step = 0u;

    test_delay();

    if (step == 0u)
    {
        tools_println((const uint8*)"assert pass test:");
        TOOLS_PRINT_ASSERT(1 == 1);
        tools_println((const uint8*)"assert pass OK");
        step = 1u;
    }
    else
    {
        tools_println((const uint8*)"assert fail test:");
        TOOLS_PRINT_ASSERT(1 == 0);
    }
}

void test_middleware_tools_print_panic(void)
{
    test_delay();
    tools_printf("before panic\r\n");
    tools_panic((const uint8*)"test panic message");
}
