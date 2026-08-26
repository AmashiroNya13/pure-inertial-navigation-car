#include "../../../inc/device/device_int_flash/device_int_flash.h"

IFX_INLINE boolean device_int_flash_isAddressInGroup(device_int_flash_id_t int_flash_id, uint32 address);
IFX_INLINE boolean device_int_flash_isRangeInGroup(device_int_flash_id_t int_flash_id, uint32 address, uint32 length);

void device_int_flash_init(device_int_flash_id_t int_flash_id)
{
    device_int_flash_cfg_t* int_flash_cfg = device_int_flash_cfg_table_get();
    device_int_flash_runtime_t* int_flash_runtime = device_int_flash_runtime_table_get();

    driver_flash_init(&int_flash_cfg[int_flash_id].flash_cfg, &int_flash_runtime[int_flash_id].flash_runtime);
}

void device_int_flash_init_all(void)
{
    device_int_flash_init(DEVICE_INT_FLASH_1);
}

void device_int_flash_autoDFlashWrite(device_int_flash_id_t int_flash_id, uint32 word_l, uint32 word_u)
{
    device_int_flash_runtime_t* int_flash_runtime = device_int_flash_runtime_table_get();
    (void)driver_flash_autoDFlashGroupWrite(&int_flash_runtime[int_flash_id].flash_runtime, word_l, word_u);
}

void device_int_flash_autoPFlashWrite(device_int_flash_id_t int_flash_id, uint32 (*word_l)[4], uint32 (*word_u)[4])
{
    device_int_flash_runtime_t* int_flash_runtime = device_int_flash_runtime_table_get();
    (void)driver_flash_autoPFlashGroupWrite(&int_flash_runtime[int_flash_id].flash_runtime, word_l, word_u);
}

uint8 device_int_flash_read8(device_int_flash_id_t int_flash_id, uint32 address)
{
    if (device_int_flash_isRangeInGroup(int_flash_id, address, 1u) == FALSE)
    {
        return 0u;
    }

    return driver_flash_read8(address);
}

uint32 device_int_flash_read32(device_int_flash_id_t int_flash_id, uint32 address)
{
    if (device_int_flash_isRangeInGroup(int_flash_id, address, 4u) == FALSE)
    {
        return 0u;
    }

    return driver_flash_read32(address);
}

void device_int_flash_read(device_int_flash_id_t int_flash_id, uint32 address, uint8* buffer, uint32 length)
{
    if (device_int_flash_isRangeInGroup(int_flash_id, address, length) == FALSE)
    {
        return;
    }

    driver_flash_read(address, buffer, length);
}

uint32 device_int_flash_start_address_get(device_int_flash_id_t int_flash_id)
{
    device_int_flash_runtime_t* int_flash_runtime = device_int_flash_runtime_table_get();

    return driver_flash_start_address_get(&int_flash_runtime[int_flash_id].flash_runtime);
}

void device_int_flash_readPages(device_int_flash_id_t int_flash_id, uint32 page_address, uint8* buffer, uint32 page_count)
{
    device_int_flash_runtime_t* int_flash_runtime = device_int_flash_runtime_table_get();
    uint64 sector_group = int_flash_runtime[int_flash_id].flash_runtime.sector_group;
    uint32 sector_index = 0u;

    while ((sector_group & 1u) == 0u)
    {
        sector_group >>= 1u;
        sector_index++;
    }

    if (device_int_flash_isRangeInGroup(int_flash_id,
                                        page_address,
                                        driver_flash_sector[sector_index]->page_length * page_count) == FALSE)
    {
        return;
    }

    driver_flash_readPages(page_address, buffer, page_count);
}

boolean device_int_flash_writeDFlashPageData(device_int_flash_id_t int_flash_id, uint32 page_address, uint32 word_l, uint32 word_u)
{
    if (device_int_flash_isRangeInGroup(int_flash_id, page_address, IFXFLASH_DFLASH_PAGE_LENGTH) == FALSE)
    {
        return FALSE;
    }

    return driver_flash_writeDFlashPageData(page_address, word_l, word_u);
}

boolean device_int_flash_writePFlashPageData(device_int_flash_id_t int_flash_id, uint32 page_address, uint32 (*word_l)[4], uint32 (*word_u)[4])
{
    if (device_int_flash_isRangeInGroup(int_flash_id, page_address, IFXFLASH_PFLASH_PAGE_LENGTH) == FALSE)
    {
        return FALSE;
    }

    return driver_flash_writePFlashPageData(page_address, word_l, word_u);
}

boolean device_int_flash_writeDFlashPages(device_int_flash_id_t int_flash_id, uint32 page_address, uint32* word_l, uint32* word_u, uint32 page_count)
{
    if (device_int_flash_isRangeInGroup(int_flash_id,
                                        page_address,
                                        IFXFLASH_DFLASH_PAGE_LENGTH * page_count) == FALSE)
    {
        return FALSE;
    }

    return driver_flash_writeDFlashPages(page_address, word_l, word_u, page_count);
}

boolean device_int_flash_writePFlashPages(device_int_flash_id_t int_flash_id, uint32 page_address, uint32 (*word_l[])[4], uint32 (*word_u[])[4], uint32 page_count)
{
    if (device_int_flash_isRangeInGroup(int_flash_id,
                                        page_address,
                                        IFXFLASH_PFLASH_PAGE_LENGTH * page_count) == FALSE)
    {
        return FALSE;
    }

    return driver_flash_writePFlashPages(page_address, word_l, word_u, page_count);
}

void device_int_flash_eraseSector(device_int_flash_id_t int_flash_id)
{
    device_int_flash_runtime_t* int_flash_runtime = device_int_flash_runtime_table_get();
    uint64 sector_group = int_flash_runtime[int_flash_id].flash_runtime.sector_group;
    uint32 sector_index = 0u;

    while ((sector_group & 1u) == 0u)
    {
        sector_group >>= 1u;
        sector_index++;
    }

    driver_flash_eraseSector(driver_flash_sector[sector_index]->start_address);
}

void device_int_flash_eraseGroup(device_int_flash_id_t int_flash_id)
{
    device_int_flash_runtime_t* int_flash_runtime = device_int_flash_runtime_table_get();
    driver_flash_eraseGroup(&int_flash_runtime[int_flash_id].flash_runtime);
}

IFX_INLINE boolean device_int_flash_isAddressInGroup(device_int_flash_id_t int_flash_id, uint32 address)
{
    device_int_flash_runtime_t* int_flash_runtime = device_int_flash_runtime_table_get();
    uint64 sector_group = int_flash_runtime[int_flash_id].flash_runtime.sector_group;

    for (uint32 sector_index = 0u; sector_group != 0u; sector_index++)
    {
        if ((sector_group & 1u) != 0u)
        {
            driver_flash_sector_t* sector = driver_flash_sector[sector_index];

            if ((address >= sector->start_address) && (address <= sector->end_address))
            {
                return TRUE;
            }
        }

        sector_group >>= 1u;
    }

    return FALSE;
}

IFX_INLINE boolean device_int_flash_isRangeInGroup(device_int_flash_id_t int_flash_id, uint32 address, uint32 length)
{
    if (length == 0u)
    {
        return FALSE;
    }

    if ((address + length - 1u) < address)
    {
        return FALSE;
    }

    return (boolean)(device_int_flash_isAddressInGroup(int_flash_id, address)
                  && device_int_flash_isAddressInGroup(int_flash_id, address + length - 1u));
}

