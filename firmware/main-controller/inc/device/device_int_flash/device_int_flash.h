#ifndef MAD_CIRCUITS_DEVICE_INT_FLASH_H
#define MAD_CIRCUITS_DEVICE_INT_FLASH_H

#include "../../../config/device/device_int_flash/device_int_flash_cfg.h"

void device_int_flash_init(device_int_flash_id_t int_flash_id);
void device_int_flash_init_all(void);
void device_int_flash_autoDFlashWrite(device_int_flash_id_t int_flash_id, uint32 word_l, uint32 word_u);
void device_int_flash_autoPFlashWrite(device_int_flash_id_t int_flash_id, uint32 (*word_l)[4], uint32 (*word_u)[4]);

uint8 device_int_flash_read8(device_int_flash_id_t int_flash_id, uint32 address);
uint32 device_int_flash_read32(device_int_flash_id_t int_flash_id, uint32 address);
void device_int_flash_read(device_int_flash_id_t int_flash_id, uint32 address, uint8* buffer, uint32 length);
void device_int_flash_readPages(device_int_flash_id_t int_flash_id, uint32 page_address, uint8* buffer, uint32 page_count);
uint32 device_int_flash_start_address_get(device_int_flash_id_t int_flash_id);

void device_int_flash_eraseSector(device_int_flash_id_t int_flash_id);
void device_int_flash_eraseGroup(device_int_flash_id_t int_flash_id);



boolean device_int_flash_writeDFlashPageData(device_int_flash_id_t int_flash_id, uint32 page_address, uint32 word_l, uint32 word_u);
boolean device_int_flash_writePFlashPageData(device_int_flash_id_t int_flash_id, uint32 page_address, uint32 (*word_l)[4], uint32 (*word_u)[4]);
boolean device_int_flash_writeDFlashPages(device_int_flash_id_t int_flash_id, uint32 page_address, uint32* word_l, uint32* word_u, uint32 page_count);
boolean device_int_flash_writePFlashPages(device_int_flash_id_t int_flash_id, uint32 page_address, uint32 (*word_l[])[4], uint32 (*word_u[])[4], uint32 page_count);

#endif
