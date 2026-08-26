#ifndef MAD_CIRCUITS_DRIVER_FLASH_H
#define MAD_CIRCUITS_DRIVER_FLASH_H

#include "IfxFlash.h"
#include "IfxScuWdt.h"

#define DRIVER_FLASH_SECTOR_MASK_D0_0_0  (1ULL << 0)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_1  (1ULL << 1)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_2  (1ULL << 2)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_3  (1ULL << 3)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_4  (1ULL << 4)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_5  (1ULL << 5)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_6  (1ULL << 6)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_7  (1ULL << 7)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_8  (1ULL << 8)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_9  (1ULL << 9)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_10 (1ULL << 10)
#define DRIVER_FLASH_SECTOR_MASK_D0_0_11 (1ULL << 11)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_0  (1ULL << 12)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_1  (1ULL << 13)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_2  (1ULL << 14)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_3  (1ULL << 15)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_4  (1ULL << 16)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_5  (1ULL << 17)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_6  (1ULL << 18)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_7  (1ULL << 19)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_8  (1ULL << 20)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_9  (1ULL << 21)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_10 (1ULL << 22)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_11 (1ULL << 23)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_12 (1ULL << 24)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_13 (1ULL << 25)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_14 (1ULL << 26)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_15 (1ULL << 27)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_16 (1ULL << 28)
#define DRIVER_FLASH_SECTOR_MASK_P0_0_17 (1ULL << 29)
#define DRIVER_FLASH_SECTOR_MASK_P0_1_18 (1ULL << 30)
#define DRIVER_FLASH_SECTOR_MASK_P0_1_19 (1ULL << 31)
#define DRIVER_FLASH_SECTOR_MASK_P0_1_20 (1ULL << 32)
#define DRIVER_FLASH_SECTOR_MASK_P0_1_21 (1ULL << 33)
#define DRIVER_FLASH_SECTOR_MASK_P0_1_22 (1ULL << 34)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_0  (1ULL << 35)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_1  (1ULL << 36)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_2  (1ULL << 37)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_3  (1ULL << 38)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_4  (1ULL << 39)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_5  (1ULL << 40)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_6  (1ULL << 41)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_7  (1ULL << 42)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_8  (1ULL << 43)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_9  (1ULL << 44)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_10 (1ULL << 45)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_11 (1ULL << 46)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_12 (1ULL << 47)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_13 (1ULL << 48)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_14 (1ULL << 49)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_15 (1ULL << 50)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_16 (1ULL << 51)
#define DRIVER_FLASH_SECTOR_MASK_P1_0_17 (1ULL << 52)
#define DRIVER_FLASH_SECTOR_MASK_P1_1_18 (1ULL << 53)
#define DRIVER_FLASH_SECTOR_MASK_P1_1_19 (1ULL << 54)
#define DRIVER_FLASH_SECTOR_MASK_P1_1_20 (1ULL << 55)
#define DRIVER_FLASH_SECTOR_MASK_P1_1_21 (1ULL << 56)
#define DRIVER_FLASH_SECTOR_MASK_P1_1_22 (1ULL << 57)
#define DRIVER_FLASH_SECTOR_MASK_P1_2_23 (1ULL << 58)
#define DRIVER_FLASH_SECTOR_MASK_P1_2_24 (1ULL << 59)

typedef struct
{
    uint32 flash_index;
    IfxFlash_FlashType flash_type;
    uint8 flash_physicalsector_index;
    uint8 flash_logicalsector_index;
    uint32 start_address;
    uint32 end_address;
    uint8 page_length;
} driver_flash_sector_t;

IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_0;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_1;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_2;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_3;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_4;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_5;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_6;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_7;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_8;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_9;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_10;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_D0_0_11;

IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_0;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_1;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_2;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_3;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_4;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_5;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_6;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_7;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_8;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_9;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_10;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_11;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_12;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_13;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_14;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_15;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_16;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_0_17;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_1_18;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_1_19;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_1_20;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_1_21;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P0_1_22;

IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_0;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_1;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_2;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_3;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_4;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_5;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_6;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_7;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_8;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_9;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_10;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_11;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_12;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_13;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_14;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_15;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_16;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_0_17;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_1_18;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_1_19;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_1_20;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_1_21;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_1_22;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_2_23;
IFX_EXTERN driver_flash_sector_t driver_flash_sector_P1_2_24;
IFX_EXTERN driver_flash_sector_t* driver_flash_sector[];

typedef struct
{
    uint64 sector_group;
} driver_flash_cfg_t;

typedef struct
{
    uint64 sector_group;
    uint64 sector_full_flag;
    uint32 group_capacity;
    uint32 storage_proportion;
    uint32 offset;
} driver_flash_runtime_t;

void driver_flash_init(driver_flash_cfg_t* flash_cfg, driver_flash_runtime_t* flash_runtime);
boolean driver_flash_autoDFlashGroupWrite(driver_flash_runtime_t* flash_runtime, uint32 word_l, uint32 word_u);
boolean driver_flash_autoPFlashGroupWrite(driver_flash_runtime_t* flash_runtime, uint32 (*word_l)[4], uint32 (*word_u)[4]);
boolean driver_flash_writeDFlashPageData(uint32 page_address, uint32 word_l, uint32 word_u);
boolean driver_flash_writePFlashPageData(uint32 page_address, uint32 (*word_l)[4], uint32 (*word_u)[4]);
boolean driver_flash_writeDFlashPages(uint32 page_address, uint32* word_l, uint32* word_u, uint32 page_count);
boolean driver_flash_writePFlashPages(uint32 page_address, uint32 (*word_l[])[4], uint32 (*word_u[])[4], uint32 page_count);
boolean driver_flash_writeDFlashPageDataSafety(uint32 page_address, uint32 word_l, uint32 word_u);
boolean driver_flash_writePFlashPageDataSafety(uint32 page_address, uint32 (*word_l)[4], uint32 (*word_u)[4]);
boolean driver_flash_writeDFlashPagesSafety(uint32 page_address, uint32* word_l, uint32* word_u, uint32 page_count);
boolean driver_flash_writePFlashPagesSafety(uint32 page_address, uint32 (*word_l[])[4], uint32 (*word_u[])[4], uint32 page_count);
uint32 driver_flash_start_address_get(driver_flash_runtime_t* flash_runtime);
uint8 driver_flash_read8(uint32 address);
uint32 driver_flash_read32(uint32 address);
void driver_flash_read(uint32 address, uint8* buffer, uint32 length);
void driver_flash_readPages(uint32 page_address, uint8* buffer, uint32 page_count);
void driver_flash_eraseSector(uint32 address);
void driver_flash_eraseGroup(driver_flash_runtime_t* flash_runtime);

#endif
