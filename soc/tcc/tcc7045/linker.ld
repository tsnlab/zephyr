/*
 *  Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */


OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)

/* Memory Definitions */
__VECTOR_START = 0x00000000;

DMA_NC_SIZE = 0x4000;
CAN_NC_SIZE = 0x2000;
SRAM_TOTAL_SIZE = 0x80000; /* 512KB */
ARM_STACK_SIZE = 0x680;
STACK_BASE_ADDR = __VECTOR_START + SRAM_TOTAL_SIZE - DMA_NC_SIZE - CAN_NC_SIZE;

MEMORY
{
	/*=========================================================
	  Region for Executing at SRAM with cacheable : 256KB
	  =========================================================*/
	  SRAM					(rwx)	: ORIGIN = 0x00000000, LENGTH = (SRAM_TOTAL_SIZE) 
}

/* Section Definitions */
SECTIONS
{
  /* first section is .text which is used for code */
  .text :
  {
	__TEXT_START__ = .;
	KEEP(*(__topst_reset))       /* Ensure __start is at the very beginning */
/*    *startup.S.o (.text)  */
/*    *reset.S.o (.text) */
    *(.text)
    __TEXT_END__ = .;
    . = ALIGN(4);
  } > SRAM

  /* .rodata section which is used for read-only data (constants) */
  .rodata :
  {
    __RODATA_START__ = .;


    *(.rodata)
    *(.rodata.*)
    *(.const)
    *(.const.*)
    . = ALIGN(0x20);
    __RODATA_END__ = .;
  } > SRAM

  /* .vector section which is used for vector table */
  .vector :
  {
  	__VECTOR_START_LOAD = LOADADDR (.vector);
  	__VECTOR_START__ = .;
    *vector.S.o (.vector .text .rodata .rodata*) 
    /* *vector_table.S.o (.vector .text .rodata .rodata*) */
    . = ALIGN(0x20);
    __VECTOR_END__ = .;
  /* } > SRAM AT > REMAP_FLASH POOKY 20240911 Origin */
  } > SRAM

  /* .data section which is used for initialized data */
  .data :
	AT ( LOADADDR (.vector) + SIZEOF (.vector) )
  {
  	__DATA_START_LOAD = LOADADDR (.data);
  	__DATA_START__ = .;
    *(.data)
    *(.data.*)
    . = ALIGN(0x20);
    __DATA_END__ = .;
  } > SRAM

  /* .bss section which is used for uninitialized data */
  .bss (NOLOAD):
  {
    . = ALIGN(0x20);
    __BSS_START__ = .;
    *(.bss)
    *(.bss.*)
    . = ALIGN(0x20);
    __BSS_END__ = .;
  } > SRAM

  _end = .;
  __end = _end;
  PROVIDE(end = .);
  /DISCARD/ : { *(.ARM.exidx* .ARM.extab* .glue* .vfp11_veneer .v4_bx .iplt .igot.* .rel.*) }
}

#include <zephyr/arch/arm/cortex_a_r/scripts/linker.ld>
