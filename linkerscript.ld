
/* In the project, there is a const structure called `vector_table'.  This
structure contains all the vectors for the interrupts.  It is supposed to reside
in the `.vectors' section, which will be placed at top of the elf file.  The
extern operation forces the symbol to be included in the linker output.  If this
statement wasn't here, the vector table would be dropped because there are no
explicit references to it.  */
EXTERN(vector_table)
ENTRY(main)

MEMORY
{
	flash    (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
	ram      (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}

SECTIONS
{
    /* the output .text section will be put into flash ROM.  Since this is
    specified, there is no need to set the location counter. */
    .text : {      
          *(.vectors) /* Vector table */ . = ALIGN(16); *(.text*) /* Code */
          *(.text*)
    } >flash

    /* the output .data section will be put into RAM.  Since this region is
    specified, there is no need to to set the location counter.

    the Virtual Memory Address (runtime address) is what the code is linked to.
    This the phyical location of this initialized data is going to be in ROM,
    however.  Its location is set through the AT>rom expression.  In the reset
    handler, the .data section in rom will be copied to ram.*/
    .data : {
          _data_section_vma_start = .;
          *(.data*)
          _data_section_vma_end = .;
    } >ram AT>flash
    _data_section_lma_start = LOADADDR(.data);

    /* the .bss section is zero initialized data in RAM. */
    .bss : {
         _bss_section_vma_start = .;
         *(.bss*)
         _bss_section_vma_end = .;  
    } >ram
}

/* provides a definition for _stack if none exists in the program.  The vector
table uses `_stack' as the initial location of the stack pointer.  `_stack'
isn't defined in the program, so the linker will substitue the value derived
here for that symbol. */
PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
