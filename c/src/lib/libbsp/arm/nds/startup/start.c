/*
 * RTEMS for Nintendo DS platform initialization.
 *
 * Copyright (c) 2008 by Matthieu Bucchianeri <mbucchia@gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 *
 * http://www.rtems.com/license/LICENSE
 *
 * $Id$
 */

#include <bsp.h>
#include <nds.h>

/*
 * ld linker symbols.
 */

extern uint8_t __bss_start;
extern uint8_t __bss_end;
extern uint8_t _end;
extern uint8_t __ewram_end;

/*
 * address of start of free memory - should be updated after creating new
 * partitions or regions.
 */

static uint32_t heap_start;

/*
 * other bsp init functions.
 */

extern void bsp_libc_init (void *, uint32_t, int);

/*
 * This definition comes from ARM cpu code.
 */

extern unsigned int arm_cpu_mode;

/*
 * setup libc.
 */

void
bsp_pretasking_hook (void)
{
  uint32_t heap_size;

  printk ("[+] initializing heap\n");

  /* initialize heap with all remaining memory */
  heap_size = (uint32_t) & __ewram_end - heap_start;
  bsp_libc_init ((void *) heap_start, heap_size, 0);
}

/*
 * start the platform.
 */

void
bsp_start (void)
{
  Configuration.work_space_start = &_end;

  /* initialize irq management */
  BSP_rtems_irq_mngt_init ();

  /* setup console mode for lower screen */
  irqEnable (IRQ_VBLANK);
  videoSetMode (0);
  videoSetModeSub (MODE_0_2D | DISPLAY_BG0_ACTIVE);
  vramSetBankC (VRAM_C_SUB_BG);

  SUB_BG0_CR = BG_MAP_BASE (31);
  BG_PALETTE_SUB[255] = RGB15 (31, 31, 31);
  consoleInitDefault ((u16 *) SCREEN_BASE_BLOCK_SUB (31),
                      (u16 *) CHAR_BASE_BLOCK_SUB (0), 16);

  /* print status message */
  printk ("[+] kernel console started\n");

  /* set the cpu mode to system user */
  arm_cpu_mode = 0x1f;

  /* configure clock period */
  Configuration.microseconds_per_tick = 10000;  /* us */

  /* check memory space for rtems workspace */
  heap_start =
    Configuration.work_space_start +
    rtems_configuration_get_work_space_size ();
  if (heap_start > &__ewram_end) {
    printk ("[!] memory exhausted\n");
    bsp_cleanup ();
  }
}

/*
 * reset bss area.
 */

void
bss_reset (void)
{
  memset (&__bss_start, 0, (uint32_t) & __bss_end - (uint32_t) & __bss_start);
}

/*
 * reset the platform using bios call.
 */

void
bsp_reset (void)
{
  swiSoftReset ();
}

/*
 * clean up platform before reset.
 */

void
bsp_cleanup (void)
{
  printk ("[!] executive ended, rebooting\n");

  bsp_reset ();
}

/*
 * A few symbols needed by libnds but not used.
 */

#include "../include/sys/iosupport.h"
const devoptab_t *devoptab_list[STD_MAX];
void *punixTime;
