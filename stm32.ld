/* Memory Spaces Definitions */
MEMORY {
		RAM      (RWX) : ORIGIN = 0x20000000     , LENGTH = 8K
        FLASH    (RX)  : ORIGIN = 0x08000000     , LENGTH = 128K
}

_estack         = ORIGIN(RAM)+LENGTH(RAM);      /* end of the stack */
_min_stack      = 0x100;                        /* minimum stack space to reserve for the user app */

/* Setting the entry point - the first instruction to execute in a program */
/* typical at the beginning of FLASH where the isrVector has to be */
ENTRY(isrVector)

/* check valid alignment for the vector table */
ASSERT(ORIGIN(FLASH) == ALIGN(ORIGIN(FLASH), 0x80), "Start of memory region flash not aligned for startup vector table");
 
SECTIONS
{
	/* the interupt service routine vector table, which goes to Flash */
	.isr_vector :
	{
	    . = ALIGN(4);
		*(.isr_vector)
	    . = ALIGN(4);
	} >FLASH

	/* the program code is stored in the .text section, which goes to Flash */
    .text :
    {
	    . = ALIGN(4);
        *(.text)				/* remaining code */
        *(.text.*)				/* remaining code */
        *(.rodata)				/* read-only data (constants) */
        *(.rodata*)
 
	    . = ALIGN(4);
		_etext = .;
		/* This is used by the startup in order to initialize the .data secion */
		_sidata = _etext;
	} >FLASH

	/* This is the initialized data section
	The program executes knowing that the data is in the RAM
	but the loader puts the initial values in the FLASH (inidata).
	The startup copies the initial values from FLASH to RAM. */
	.data  : AT ( _sidata )
	{
		. = ALIGN(4);
		/* This is used by the startup in order to initialize the .data secion */
		_sdata = . ;

		*(.data)
		*(.data.*)

		. = ALIGN(4);
		/* This is used by the startup in order to initialize the .data secion */
		_edata = . ;
	} >RAM

 	/* This is the uninitialized data section */
	.bss :
	{
		. = ALIGN(4);
		/* This is used by the startup in order to initialize the .bss secion */
		_sbss = .;

		*(.bss)
		*(COMMON)

		. = ALIGN(4);
		/* This is used by the startup in order to initialize the .bss secion */
		_ebss = . ;
	} >RAM
}
