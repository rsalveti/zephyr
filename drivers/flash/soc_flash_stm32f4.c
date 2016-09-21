
#include <misc/sys_log.h>
#include <nanokernel.h>
#include <device.h>
#include <string.h>
#include <flash.h>
#include <errno.h>
#include <init.h>
#include <soc.h>

#include <flash_registers.h>

static const struct flash_map {
      uint32_t start;
      uint32_t len;
      uint16_t id;
} sector [] = {
#if (CONFIG_FLASH_BASE_ADDRESS == 0x08000000)
	/* st_stm32f4_init relocating the vector table to the start of flash */
	[0] = { .start = 0x08004000 , .len = KB(16),  .id = 0x0008 },
	[1] = { .start = 0x08008000 , .len = KB(16),  .id = 0x0010 },
	[2] = { .start = 0x0800c000 , .len = KB(16),  .id = 0x0018 },
	[3] = { .start = 0x08010000 , .len = KB(64),  .id = 0x0020 },
	[4] = { .start = 0x08020000 , .len = KB(128), .id = 0x0028 },
	[5] = { .start = 0x08040000 , .len = KB(128), .id = 0x0030 },
	[6] = { .start = 0x08060000 , .len = KB(128), .id = 0x0038 },
#else
	/* assume that the init code is not relocating the vector table */
	[0] = { .start = 0x08000000 , .len = KB(16),   .id = 0x0000 },
	[1] = { .start = 0x08004000 , .len = KB(16),   .id = 0x0008 },
	[2] = { .start = 0x08008000 , .len = KB(16),   .id = 0x0010 },
	[3] = { .start = 0x0800c000 , .len = KB(16),   .id = 0x0018 },
	[4] = { .start = 0x08010000 , .len = KB(64),   .id = 0x0020 },
	[5] = { .start = 0x08020000 , .len = KB(128),  .id = 0x0028 },
	[6] = { .start = 0x08040000 , .len = KB(128),  .id = 0x0030 },
	[7] = { .start = 0x08060000 , .len = KB(128),  .id = 0x0038 },
#endif
};

#define MAX_OFFSET  (sector[ARRAY_SIZE(sector) -1].len + \
	sector[ARRAY_SIZE(sector) - 1].start  - sector[0].start)

struct flash_priv {
	struct stm32f4x_flash *regs;
	struct nano_sem sem;
};

static int check_status(struct stm32f4x_flash *regs)
{
	uint32_t error = 0;

	if (regs->status & FLASH_FLAG_WRPERR) {
		error |= HAL_FLASH_ERROR_WRP;
	}

	if (regs->status & FLASH_FLAG_PGAERR) {
		error |= HAL_FLASH_ERROR_PGA;
	}

	if (regs->status & FLASH_FLAG_PGPERR) {
		error |= HAL_FLASH_ERROR_PGP;
	}

	if (regs->status & FLASH_FLAG_PGSERR) {
		error |= HAL_FLASH_ERROR_PGS;
	}

	if (regs->status & FLASH_FLAG_RDERR) {
		error |= HAL_FLASH_ERROR_RD;
	}

	if (regs->status & FLASH_FLAG_OPERR) {
		error |= HAL_FLASH_ERROR_OPERATION;
	}

	if (error) {
		SYS_LOG_ERR("error: 0x%x, status 0x%x\n", error, regs->status);
		return -1;
	}

	return 0;
}

static int get_sector(off_t offset)
{
	uint32_t addr = sector[0].start + offset;
	int i;

	for (i = 0; i < ARRAY_SIZE(sector) ; i++) {
		if (addr > sector[i].start) {
			continue;
		} else if (addr == sector[i].start) {
			return i;
		} else {
			return(i - 1);
		}
	}

	return i - 1;
}

static int wait_flash_idle(struct stm32f4x_flash *regs)
{
	int rc;

	rc = check_status(regs);
	if (rc < 0) {
		return -1;
	}

	while (regs->status & FLASH_FLAG_BSY) {
		continue;
	}

	return 0;
}

static void program_byte(uint8_t *p, uint8_t val, struct stm32f4x_flash *regs)
{
	regs->ctrl &= CR_PSIZE_MASK;
	regs->ctrl |= FLASH_PSIZE_BYTE;
	regs->ctrl |= FLASH_CR_PG;

	*p = val;

	while (regs->status & FLASH_FLAG_BSY) {
		continue;
	}

	regs->ctrl &= (~FLASH_CR_PG);
}

static int program_flash(uint32_t address, uint8_t data, struct flash_priv *p)
{
	int rc;

	rc = wait_flash_idle(p->regs);
	if (rc < 0) {
		return -1;
	}

	nano_sem_take(&p->sem, TICKS_UNLIMITED);
	program_byte((uint8_t *) address, data, p->regs);
	nano_sem_give(&p->sem);

	rc = wait_flash_idle(p->regs);

	return rc;
}

static int erase_sector(uint16_t sector, struct stm32f4x_flash *regs)
{
	int rc = 0;

	rc = wait_flash_idle(regs);
	if (rc < 0) {
		SYS_LOG_ERR("erasing sector\n");
		return -1;
	}

	regs->ctrl &= CR_PSIZE_MASK;

	/* assuming 3.3V just as stm32f4x_clock.c */
	regs->ctrl |= FLASH_PSIZE_WORD;
	regs->ctrl &= SECTOR_MASK;
	regs->ctrl |= FLASH_CR_SER | sector;
	regs->ctrl |= FLASH_CR_STRT;

 	while (regs->status & FLASH_FLAG_BSY) {
		continue;
	}

	rc = wait_flash_idle(regs);
	if (rc < 0) {
		SYS_LOG_ERR("erasing sector\n");
		return -1;
	}

	return 0;
}

static int flash_stm32f_erase(struct device *dev, off_t offset, size_t size)
{
	struct flash_priv *p = dev->driver_data;
	int start, end, i;
	int rc = 0;

	if (offset < 0 || offset > MAX_OFFSET) {
		return -1;
	}

	end = get_sector(offset + size - 1);
	start = get_sector(offset);

	for (i = start; i <= end ; i++ ) {
		rc = erase_sector(sector[i].id, p->regs);
		if (rc < 0) {
			return -1;
		}
	}

	return rc;
}

static int flash_stm32f_read(struct device *dev, off_t offset,
				void *data, size_t len)
{
	uint32_t addr = sector[0].start + offset;

	if (offset < 0 || offset > MAX_OFFSET) {
		return -1;
	}

	memcpy(data, (void *) addr, len);

	return 0;
}

static int flash_stm32f_write(struct device *dev, off_t offset,
				const void *data, size_t len)
{
	uint32_t addr = sector[0].start + offset;
	struct flash_priv *p = dev->driver_data;
	const uint8_t *sptr = data;
	int rc, i;

	if (offset < 0 || offset > MAX_OFFSET) {
		return -1;
	}

	p->regs->status = FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
		| FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR;

	for (i = 0; i < len / sizeof(*sptr); i++) {
		rc = program_flash(addr, sptr[i], p);
		if (rc < 0) {
			return -1;
		}

		addr += sizeof(*sptr);
	}

	return 0;
}

static int flash_stm32f_write_protection(struct device *dev, bool enable)
{
	return 0;
}

static struct flash_priv flash_data = {
	.regs = (struct stm32f4x_flash *) FLASH_R_BASE,
};

static struct flash_driver_api flash_stm32f4_api = {
	.write_protection = flash_stm32f_write_protection,
	.erase = flash_stm32f_erase,
	.write = flash_stm32f_write,
	.read = flash_stm32f_read,
};

static int stm32f4_flash_init(struct device *dev)
{
	struct flash_priv *p = dev->driver_data;

	nano_sem_init(&p->sem);
	nano_sem_give(&p->sem);

	if (p->regs->ctrl & FLASH_CR_LOCK) {
		p->regs->key = FLASH_KEY1;
		p->regs->key = FLASH_KEY2;
	}

	return 0;
}

DEVICE_AND_API_INIT(stm32f4x_flash, CONFIG_SOC_FLASH_STM32F4_DEV_NAME,
		    stm32f4_flash_init, &flash_data, NULL, SECONDARY,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &flash_stm32f4_api);

