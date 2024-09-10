/*
 *  Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 #include <zephyr/arch/arm/cortex_a_r/scripts/linker.ld>
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
	  SRAM					(rwx)	: ORIGIN = 0x00000000, LENGTH = (SRAM_TOTAL_SIZE - ARM_STACK_SIZE - DMA_NC_SIZE - CAN_NC_SIZE) 

	/*=========================================================
	  Region for ARM STACK area : 0x680Byte
	  =========================================================*/
	  SRAM_ARM_STACK (rwx)	: ORIGIN = 0x00000000 + (SRAM_TOTAL_SIZE - ARM_STACK_SIZE - DMA_NC_SIZE - CAN_NC_SIZE), LENGTH = ARM_STACK_SIZE

	/*=========================================================
	  Region for Executing at SRAM with non-cacheable for CAN : 32KB
	  =========================================================*/
	  CAN_NC_SRAM			(rwx)	: ORIGIN = 0x00000000 + (SRAM_TOTAL_SIZE - DMA_NC_SIZE - CAN_NC_SIZE), LENGTH = CAN_NC_SIZE

	/*=========================================================
	  Region for Executing at SRAM with non-cacheable : 8KB, The dma address is valid only physical address(0x00000000), it can not use mirrored mem address(0x06000000)
	  =========================================================*/
	  NC_SRAM				(rwx)	: ORIGIN = 0x00000000 + (SRAM_TOTAL_SIZE - DMA_NC_SIZE), LENGTH = DMA_NC_SIZE

	/*=====================================================
	  Region for Executing at remap Flash area with cacheable
	  =====================================================*/
	  REMAP_FLASH			(rx)	: ORIGIN = 0x01043000, LENGTH = 0x00200000
}

/* Section Definitions */
SECTIONS
{
  /* first section is .text which is used for code */
  .text :
  {
  	__TEXT_START__ = .;
    *startup.o (.text)
    *(.text)
    __TEXT_END__ = .;
    . = ALIGN(4);
  } > REMAP_FLASH

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
  } > REMAP_FLASH

  /* .vector section which is used for vector table */
  .vector :
  {
  	__VECTOR_START_LOAD = LOADADDR (.vector);
  	__VECTOR_START__ = .;
    *vector.o (.vector .text .rodata .rodata*)
    . = ALIGN(0x20);
    __VECTOR_END__ = .;
  } > SRAM AT > REMAP_FLASH

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

/* arm_stack */
 .arm_stack (NOLOAD):
 {
 	  __stack_start = .;
 	  . += 0x680;
   	_stack_end = .;
 } > SRAM_ARM_STACK

  /* .nc_dma section which is used for non-cachable can message buffer */
 .nc_can (NOLOAD):
  AT ( ADDR (.bss) + SIZEOF (.bss) )
 {
 	  __nc_canstart = .;
 	  . += CAN_NC_SIZE;
   	_end_of_nc_can = .;
 } > CAN_NC_SRAM

 /* .nc_dma section which is used for non-cachable dma buffer */
 .nc_dma (NOLOAD):
  AT ( ADDR (.bss) + SIZEOF (.bss) )
 {
 	  __nc_dmastart = .;
 	  . += DMA_NC_SIZE;
   	_end_of_nc_dma = .;
 } > NC_SRAM

  _end = .;
  __end = _end;
  PROVIDE(end = .);
  /DISCARD/ : { *(.ARM.exidx* .ARM.extab* .glue* .vfp11_veneer .v4_bx .iplt .igot.* .rel.*) }
}
