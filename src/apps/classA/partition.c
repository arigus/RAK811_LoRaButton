#include <stdint.h>
#include <string.h>
#include "board.h"

#define FLASH_BASE_ADDR  0x0801EC00

static void _ENABLE_FLASH(void)
{
    HAL_FLASH_Unlock(); 
}

static void _DISABLE_FLASH(void)
{
    HAL_FLASH_Lock(); 
}

static void read_data(uint32_t addr, void *buffer, uint16_t len)
{
    //(void)test_table;  
    memcpy(buffer, (void *)addr, len);
}

static void write_data(uint32_t wr_addr, void *buffer, uint16_t wr_len)
{
    int i;
    uint32_t PAGEError = 0;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t *wr_data = buffer;
    
    if (wr_addr % 4 != 0 || wr_len%4 != 0) {
        while(1);
    }  
   
    _ENABLE_FLASH();

    for (i = 0; i < wr_len/4; i++) {
        if (wr_addr % FLASH_PAGE_SIZE == 0) {
            
            EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
            EraseInitStruct.PageAddress = wr_addr;
            EraseInitStruct.NbPages     = 1;
            HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
        }
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, wr_addr, *wr_data);
        wr_data++;
        wr_addr += 4;
    }
    _DISABLE_FLASH();
}


uint32_t _get_boot_config(void)
{
    return (*(uint32_t *)FLASH_BASE_ADDR );
}

void _set_boot_config(uint32_t cfg)
{
   write_data(FLASH_BASE_ADDR, &cfg, 4);
}


