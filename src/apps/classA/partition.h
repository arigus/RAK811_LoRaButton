#ifndef _PARTITION_H_
#define _PARTITION_H_

#define CONFIG_MASK(BIT)       ((1UL << (BIT)))

#define CONFIG_DISABLE         CONFIG_MASK(0)
#define CONFIG_BOOT_E          CONFIG_MASK(1)

uint32_t _get_boot_config(void);
void _set_boot_config(uint32_t cfg);


#endif
