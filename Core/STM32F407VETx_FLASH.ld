/*
 * STM32F407VET6 链接脚本（GCC / GNU ld）
 *
 * 存储空间：
 *   FLASH 512 KB @ 0x08000000
 *   SRAM  128 KB @ 0x20000000   (SRAM1 112K + SRAM2 16K，连续)
 *   CCM    64 KB @ 0x10000000   (只能被 CPU 访问，DMA 不可用)
 *
 * 注意：CCM 不能放 .data/.bss（启动代码不会初始化它），
 *       需要时用 __attribute__((section(".ccmram"))) 显式放进去。
 */

ENTRY(Reset_Handler)

/* 最高地址处放栈顶 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x800;   /* 2 KB heap（printf 用） */
_Min_Stack_Size = 0x800;   /* 2 KB stack             */

MEMORY
{
  FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (xrw)    : ORIGIN = 0x20000000, LENGTH = 128K
  CCMRAM (xrw)   : ORIGIN = 0x10000000, LENGTH = 64K
}

SECTIONS
{
  /* 中断向量表必须放在 Flash 起始处 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { . = ALIGN(4); *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM      : { . = ALIGN(4); __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH

  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* 已初始化变量：运行时在 RAM，初值存在 Flash */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* 未初始化变量 */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* CCM：需要手动放进去的变量 */
  .ccmram (NOLOAD) :
  {
    . = ALIGN(4);
    _sccmram = .;
    *(.ccmram)
    *(.ccmram*)
    . = ALIGN(4);
    _eccmram = .;
  } >CCMRAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
