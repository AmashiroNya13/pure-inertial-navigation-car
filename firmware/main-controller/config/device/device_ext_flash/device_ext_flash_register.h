#ifndef MAD_CIRCUITS_DEVICE_EXT_FLASH_REGISTER_H
#define MAD_CIRCUITS_DEVICE_EXT_FLASH_REGISTER_H

#include "Ifx_Types.h"

#define DEVICE_EXT_FLASH_JEDEC_ID_LENGTH      ((uint32)3u)
#define DEVICE_EXT_FLASH_UNIQUE_ID_LENGTH     ((uint32)8u)
#define DEVICE_EXT_FLASH_PAGE_SIZE            ((uint32)256u)
#define DEVICE_EXT_FLASH_SECTOR_SIZE          ((uint32)4096u)
#define DEVICE_EXT_FLASH_BLOCK_SIZE_32K       ((uint32)32768u)
#define DEVICE_EXT_FLASH_BLOCK_SIZE_64K       ((uint32)65536u)
#define DEVICE_EXT_FLASH_ADDRESS_LENGTH       ((uint32)3u)
#define DEVICE_EXT_FLASH_FAST_READ_DUMMY_BYTE ((uint32)1u)

#define DEVICE_EXT_FLASH_MANUFACTURER_ID_WINBOND ((uint8)0xEFu)
#define DEVICE_EXT_FLASH_MEMORY_TYPE_W25Q128JV   ((uint8)0x40u)
#define DEVICE_EXT_FLASH_CAPACITY_W25Q128JV      ((uint8)0x18u)

typedef enum
{
    DEVICE_EXT_FLASH_COMMAND_WRITE_ENABLE              = 0x06u,
    DEVICE_EXT_FLASH_COMMAND_VOLATILE_SR_WRITE_ENABLE  = 0x50u,
    DEVICE_EXT_FLASH_COMMAND_WRITE_DISABLE             = 0x04u,
    DEVICE_EXT_FLASH_COMMAND_READ_STATUS_REG1          = 0x05u,
    DEVICE_EXT_FLASH_COMMAND_READ_STATUS_REG2          = 0x35u,
    DEVICE_EXT_FLASH_COMMAND_READ_STATUS_REG3          = 0x15u,
    DEVICE_EXT_FLASH_COMMAND_WRITE_STATUS_REG1         = 0x01u,
    DEVICE_EXT_FLASH_COMMAND_WRITE_STATUS_REG2         = 0x31u,
    DEVICE_EXT_FLASH_COMMAND_WRITE_STATUS_REG3         = 0x11u,
    DEVICE_EXT_FLASH_COMMAND_PAGE_PROGRAM              = 0x02u,
    DEVICE_EXT_FLASH_COMMAND_READ_DATA                 = 0x03u,
    DEVICE_EXT_FLASH_COMMAND_FAST_READ                 = 0x0Bu,
    DEVICE_EXT_FLASH_COMMAND_READ_JEDEC_ID            = 0x9Fu,
    DEVICE_EXT_FLASH_COMMAND_READ_UNIQUE_ID           = 0x4Bu,
    DEVICE_EXT_FLASH_COMMAND_READ_MANUFACTURER_DEVICE = 0x90u,
    DEVICE_EXT_FLASH_COMMAND_READ_SFDP                = 0x5Au,
    DEVICE_EXT_FLASH_COMMAND_SECTOR_ERASE_4K          = 0x20u,
    DEVICE_EXT_FLASH_COMMAND_BLOCK_ERASE_32K          = 0x52u,
    DEVICE_EXT_FLASH_COMMAND_BLOCK_ERASE_64K          = 0xD8u,
    DEVICE_EXT_FLASH_COMMAND_CHIP_ERASE               = 0xC7u,
    DEVICE_EXT_FLASH_COMMAND_ERASE_PROGRAM_SUSPEND    = 0x75u,
    DEVICE_EXT_FLASH_COMMAND_ERASE_PROGRAM_RESUME     = 0x7Au,
    DEVICE_EXT_FLASH_COMMAND_POWER_DOWN               = 0xB9u,
    DEVICE_EXT_FLASH_COMMAND_RELEASE_POWER_DOWN       = 0xABu,
    DEVICE_EXT_FLASH_COMMAND_ENABLE_RESET             = 0x66u,
    DEVICE_EXT_FLASH_COMMAND_RESET_DEVICE             = 0x99u,
} device_ext_flash_command_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int busy:1;
        unsigned int wel:1;
        unsigned int bp0:1;
        unsigned int bp1:1;
        unsigned int bp2:1;
        unsigned int tb:1;
        unsigned int sec:1;
        unsigned int srp0:1;
    } B;
} device_ext_flash_status_reg1_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int srp1:1;
        unsigned int qe:1;
        unsigned int not_used_2:1;
        unsigned int lb1:1;
        unsigned int lb2:1;
        unsigned int lb3:1;
        unsigned int cmp:1;
        unsigned int sus:1;
    } B;
} device_ext_flash_status_reg2_t;

typedef union
{
    uint8 U;
    struct
    {
        unsigned int not_used_0:1;
        unsigned int wps:1;
        unsigned int not_used_2:3;
        unsigned int drv:2;
        unsigned int hold_rst:1;
    } B;
} device_ext_flash_status_reg3_t;

typedef struct
{
    uint8 manufacturer_id;
    uint8 memory_type;
    uint8 capacity;
} device_ext_flash_jedec_id_t;

#endif
