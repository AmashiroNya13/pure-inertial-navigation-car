#include "test_middleware_tools_log.h"
#include "tools_print.h"
#include "tools_log.h"

#define TEST_LOG_DELAY_COUNT 100000u

static void test_log_delay(void)
{
    for (volatile uint32 i = 0; i < TEST_LOG_DELAY_COUNT; i++) { }
}

void test_middleware_tools_log(void)
{
    static uint32 step = 0u;

    test_log_delay();

    if (step == 0u)
    {
        tools_println((const uint8*)"========== TOOLS_LOG_1 store ==========");

        tools_log(TOOLS_LOG_1, (const uint8*)"hello");
        tools_println((const uint8*)"  stored: hello");

        tools_log(TOOLS_LOG_1, (const uint8*)"12345");
        tools_println((const uint8*)"  stored: 12345");

        tools_log(TOOLS_LOG_1, (const uint8*)"abcdefghij");
        tools_println((const uint8*)"  stored: abcdefghij");

        tools_log(TOOLS_LOG_1, (const uint8*)"this is a long string that needs many pages");
        tools_println((const uint8*)"  stored: long string (47 chars)");

        step = 1u;
    }
    else
    {
        tools_println((const uint8*)"========== TOOLS_LOG_1 print ==========");
        tools_log_print_all(TOOLS_LOG_1);
        tools_println((const uint8*)"========== done ==========");

        step = 0u;
    }
}

void test_middleware_tools_log_multi(void)
{
    static uint32 step = 0u;

    test_log_delay();

    if (step == 0u)
    {
        tools_println((const uint8*)"========== single-instance store ==========");

        tools_log(TOOLS_LOG_1, (const uint8*)"LOG1_alpha");
        tools_println((const uint8*)"  log1: LOG1_alpha");

        tools_log(TOOLS_LOG_1, (const uint8*)"LOG1_beta");
        tools_println((const uint8*)"  log1: LOG1_beta");

        tools_log(TOOLS_LOG_1, (const uint8*)"LOG1_gamma_long_message_content");
        tools_println((const uint8*)"  log1: LOG1_gamma (multi-page)");

        step = 1u;
    }
    else
    {
        tools_println((const uint8*)"========== print LOG ==========");
        tools_log_print_all(TOOLS_LOG_1);
        tools_println((const uint8*)"========== done ==========");

        step = 0u;
    }
}

void test_middleware_tools_log_edge(void)
{
    static uint32 step = 0u;

    test_log_delay();

    if (step == 0u)
    {
        tools_println((const uint8*)"========== edge case store ==========");

        tools_log(TOOLS_LOG_1, NULL_PTR);
        tools_println((const uint8*)"  stored: NULL_PTR (expect skip)");

        tools_log(TOOLS_LOG_1, (const uint8*)"");
        tools_println((const uint8*)"  stored: empty (expect skip)");

        tools_log(TOOLS_LOG_1, (const uint8*)"x");
        tools_println((const uint8*)"  stored: x (1 char)");

        tools_log(TOOLS_LOG_1, (const uint8*)"ABCDE");
        tools_println((const uint8*)"  stored: ABCDE (exact page)");

        tools_log(TOOLS_LOG_1, (const uint8*)"ABCDEF");
        tools_println((const uint8*)"  stored: ABCDEF (1 byte overflow)");

        tools_log(TOOLS_LOG_1, (const uint8*)"0123456789");
        tools_println((const uint8*)"  stored: 0123456789 (2 pages)");

        step = 1u;
    }
    else
    {
        tools_println((const uint8*)"========== edge case print ==========");
        tools_log_print_all(TOOLS_LOG_1);
        tools_println((const uint8*)"========== done ==========");

        step = 0u;
    }
}

void test_middleware_tools_log_power_loss(void)
{
    tools_log_runtime_t* runtime = tools_log_runtime_table_get();
    uint64 sector_group = runtime[TOOLS_LOG_1].flash_runtime.sector_group;
    uint32 sector_index = 0u;

    while ((sector_group & 1u) == 0u)
    {
        sector_group >>= 1u;
        sector_index++;
    }

    driver_flash_sector_t* sector = driver_flash_sector[sector_index];
    uint8 first_byte = driver_flash_read8(sector->start_address);

    test_log_delay();

    if (first_byte == 0xFFu)
    {
        tools_println((const uint8*)"========== first boot: storing ==========");

        tools_log(TOOLS_LOG_1, (const uint8*)"pwr_loss_hello");
        tools_log(TOOLS_LOG_1, (const uint8*)"pwr_loss_this_is_a_long_test_string");

        tools_println((const uint8*)"logs stored, power off NOW then restart");
    }
    else
    {
        tools_println((const uint8*)"========== power cycled: verifying ==========");
        tools_printf("sector start first byte: 0x%02X\r\n", (unsigned int)first_byte);
        tools_log_print_all(TOOLS_LOG_1);
        tools_println((const uint8*)"========== done ==========");
    }
}


