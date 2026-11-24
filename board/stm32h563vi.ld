/* 本镜像从片内 Flash 独立启动，约定 TrustZone 已关闭。可用 Flash 为 2 MiB，
   运行数据仅使用第一块 SRAM（256 KiB）。ENTRY 指定 ELF 的程序入口。 */
ENTRY(Reset_Handler)
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
    SRAM1 (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

/* Cortex-M 栈从高地址向低地址增长。初始 MSP 在 SRAM1 末端的下一个地址，
   _sstack 比它低 8 KiB，ST 启动代码把这个下界写入 MSPLIM。 */
_estack = ORIGIN(SRAM1) + LENGTH(SRAM1);
_sstack = _estack - 8K;

SECTIONS
{
    /* 向量表占据 Flash 开头，前两个 32 位字分别装初始 MSP 和复位处理函数地址。
       KEEP 使链接器清除未引用段时仍保留向量表，硬件复位时直接读取它。 */
    .isr_vector : { KEEP(*(.isr_vector)) } >FLASH
    /* 收集各输入文件的可执行代码与只读常量。点号表示当前链接地址。 */
    .text :
    {
        *(.text .text.*)
        KEEP(*(.init))
        KEEP(*(.fini))
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } >FLASH
    /* Arm 的栈展开数据和索引，用于从当前函数还原其调用者。 */
    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } >FLASH
    /* 运行库通过起止符号遍历函数指针表。启动文件调用 __libc_init_array，
       它依次运行预初始化和初始化函数。fini_array 保存退出时的清理函数。 */
    .preinit_array (READONLY) :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array .preinit_array.*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH
    .init_array (READONLY) :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH
    .fini_array (READONLY) :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT_BY_INIT_PRIORITY(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH

    /* 带初值的可写全局和静态变量在 SRAM1 中运行，初值保存在 Flash。
       启动代码从 _sidata 开始复制到 [_sdata, _edata)，再进入应用。 */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*)
        . = ALIGN(4);
        _edata = .;
    } >SRAM1 AT>FLASH
    _sidata = LOADADDR(.data);
    /* .bss 为零初值的可写数据。NOLOAD 只预留内存，启动时清零，
       所以原始 BIN 不需要为这些变量存储逐字节的零。 */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss.* COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >SRAM1
    /* 将静态区末尾按 8 字节对齐，链接时检查它是否侵入上方栈空间。 */
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    ASSERT(. <= _sstack, "static data exceeds SRAM1 stack boundary")
    /DISCARD/ : { *(.note.GNU-stack) }
}
