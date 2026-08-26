#include "../../../inc/device/device_ext_flash/device_ext_flash.h"
#include "../../../config/device/device_ext_flash/device_ext_flash_register.h"

static driver_qspi_runtime_t* device_ext_flash_qspi_runtime_get(device_ext_flash_id_t ext_flash_id);
static void device_ext_flash_address_split(uint32 address, uint8* address_buffer);
static void device_ext_flash_write_command(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command);
static uint8 device_ext_flash_read_register(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command);
static void device_ext_flash_write_register(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command, uint8 value);
static void device_ext_flash_read_registers(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command, uint8* buffer, uint32 length);
static void device_ext_flash_read_address(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command, uint32 address, uint8* buffer, uint32 length);
static void device_ext_flash_fast_read_address(device_ext_flash_id_t ext_flash_id, uint32 address, uint8* buffer, uint32 length);
static void device_ext_flash_read_unique_id_register(device_ext_flash_id_t ext_flash_id, uint8* buffer);
static void device_ext_flash_write_address(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command, uint32 address, uint8* buffer, uint32 length);
static void device_ext_flash_erase_address(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command, uint32 address);
static void device_ext_flash_register_init(device_ext_flash_id_t ext_flash_id);

static driver_qspi_runtime_t* device_ext_flash_qspi_runtime_get(device_ext_flash_id_t ext_flash_id)
{
    device_ext_flash_runtime_t* ext_flash_runtime = device_ext_flash_runtime_table_get();
    return &ext_flash_runtime[ext_flash_id].qspi_runtime;
}

static void device_ext_flash_address_split(uint32 address, uint8* address_buffer)
{
    address_buffer[0] = (uint8)((address >> 16u) & 0xFFu);
    address_buffer[1] = (uint8)((address >> 8u) & 0xFFu);
    address_buffer[2] = (uint8)(address & 0xFFu);
}

#if defined (__TASKING__)
#pragma warning 536
#endif
static void device_ext_flash_write_command(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command)
{
    driver_qspi_write_8bit(device_ext_flash_qspi_runtime_get(ext_flash_id), (uint8)command);
}

static uint8 device_ext_flash_read_register(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command)
{
    return driver_qspi_read_8bit_register(device_ext_flash_qspi_runtime_get(ext_flash_id), (uint8)command);
}

static void device_ext_flash_write_register(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command, uint8 value)
{
    driver_qspi_write_8bit_register(device_ext_flash_qspi_runtime_get(ext_flash_id), (uint8)command, value);
}

static void device_ext_flash_read_registers(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command, uint8* buffer, uint32 length)
{
    driver_qspi_read_8bit_registers(device_ext_flash_qspi_runtime_get(ext_flash_id), (uint8)command, buffer, length);
}

static void device_ext_flash_read_address(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command, uint32 address, uint8* buffer, uint32 length)
{
    uint32 index;
    uint8 write_buffer[4u + DEVICE_EXT_FLASH_PAGE_SIZE];
    uint8 read_buffer[4u + DEVICE_EXT_FLASH_PAGE_SIZE];

    write_buffer[0] = (uint8)command;
    device_ext_flash_address_split(address, &write_buffer[1]);

    for (index = 4u; index < (length + 4u); index++)
    {
        write_buffer[index] = 0u;
    }

    driver_qspi_transfer_8bit(device_ext_flash_qspi_runtime_get(ext_flash_id), write_buffer, read_buffer, length + 4u);

    for (index = 0u; index < length; index++)
    {
        buffer[index] = read_buffer[index + 4u];
    }
}

static void device_ext_flash_fast_read_address(device_ext_flash_id_t ext_flash_id, uint32 address, uint8* buffer, uint32 length)
{
    uint32 index;
    uint8 write_buffer[5u + DEVICE_EXT_FLASH_PAGE_SIZE];
    uint8 read_buffer[5u + DEVICE_EXT_FLASH_PAGE_SIZE];

    write_buffer[0] = (uint8)DEVICE_EXT_FLASH_COMMAND_FAST_READ;
    device_ext_flash_address_split(address, &write_buffer[1]);
    write_buffer[4] = 0u;

    for (index = 5u; index < (length + 5u); index++)
    {
        write_buffer[index] = 0u;
    }

    driver_qspi_transfer_8bit(device_ext_flash_qspi_runtime_get(ext_flash_id), write_buffer, read_buffer, length + 5u);

    for (index = 0u; index < length; index++)
    {
        buffer[index] = read_buffer[index + 5u];
    }
}

static void device_ext_flash_read_unique_id_register(device_ext_flash_id_t ext_flash_id, uint8* buffer)
{
    uint32 index;
    uint8 write_buffer[5u + DEVICE_EXT_FLASH_UNIQUE_ID_LENGTH];
    uint8 read_buffer[5u + DEVICE_EXT_FLASH_UNIQUE_ID_LENGTH];

    write_buffer[0] = (uint8)DEVICE_EXT_FLASH_COMMAND_READ_UNIQUE_ID;
    write_buffer[1] = 0u;
    write_buffer[2] = 0u;
    write_buffer[3] = 0u;
    write_buffer[4] = 0u;

    for (index = 5u; index < (DEVICE_EXT_FLASH_UNIQUE_ID_LENGTH + 5u); index++)
    {
        write_buffer[index] = 0u;
    }

    driver_qspi_transfer_8bit(device_ext_flash_qspi_runtime_get(ext_flash_id),
                              write_buffer,
                              read_buffer,
                              DEVICE_EXT_FLASH_UNIQUE_ID_LENGTH + 5u);

    for (index = 0u; index < DEVICE_EXT_FLASH_UNIQUE_ID_LENGTH; index++)
    {
        buffer[index] = read_buffer[index + 5u];
    }
}

static void device_ext_flash_write_address(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command, uint32 address, uint8* buffer, uint32 length)
{
    uint32 index;
    uint8 write_buffer[4u + DEVICE_EXT_FLASH_PAGE_SIZE];

    if (length > DEVICE_EXT_FLASH_PAGE_SIZE)
    {
        length = DEVICE_EXT_FLASH_PAGE_SIZE;
    }

    write_buffer[0] = (uint8)command;
    device_ext_flash_address_split(address, &write_buffer[1]);

    for (index = 0u; index < length; index++)
    {
        write_buffer[index + 4u] = buffer[index];
    }

    driver_qspi_write_8bit_array(device_ext_flash_qspi_runtime_get(ext_flash_id), write_buffer, length + 4u);
}

static void device_ext_flash_erase_address(device_ext_flash_id_t ext_flash_id, device_ext_flash_command_t command, uint32 address)
{
    uint8 write_buffer[4u];

    write_buffer[0] = (uint8)command;
    device_ext_flash_address_split(address, &write_buffer[1]);

    driver_qspi_write_8bit_array(device_ext_flash_qspi_runtime_get(ext_flash_id), write_buffer, 4u);
}

static void device_ext_flash_register_init(device_ext_flash_id_t ext_flash_id)
{
    (void)ext_flash_id;
}
#if defined (__TASKING__)
#pragma warning restore
#endif

void device_ext_flash_init(device_ext_flash_id_t ext_flash_id)
{
    device_ext_flash_cfg_t* ext_flash_cfg = device_ext_flash_cfg_table_get();
    device_ext_flash_runtime_t* ext_flash_runtime = device_ext_flash_runtime_table_get();

    driver_dma_init(&ext_flash_cfg[ext_flash_id].tx_dma_cfg, &ext_flash_runtime[ext_flash_id].tx_dma_runtime);
    driver_qspi_init(&ext_flash_cfg[ext_flash_id].qspi_cfg, &ext_flash_runtime[ext_flash_id].qspi_runtime);
    driver_dma_init(&ext_flash_cfg[ext_flash_id].rx_dma_cfg, &ext_flash_runtime[ext_flash_id].rx_dma_runtime);
}

void device_ext_flash_init_all(void)
{
    device_ext_flash_init(DEVICE_EXT_FLASH_1);
}
