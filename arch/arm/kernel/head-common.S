/*
 *  linux/arch/arm/kernel/head-common.S
 *
 *  Copyright (C) 1994-2002 Russell King
 *  Copyright (c) 2003 ARM Limited
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define ATAG_CORE 0x54410001
#define ATAG_CORE_SIZE ((2*4 + 3*4) >> 2)
#define ATAG_CORE_SIZE_EMPTY ((2*4) >> 2)

#ifdef CONFIG_CPU_BIG_ENDIAN
#define OF_DT_MAGIC 0xd00dfeed
#else
#define OF_DT_MAGIC 0xedfe0dd0 /* 0xd00dfeed in big-endian */
#endif

/*
 * Exception handling.  Something went wrong and we can't proceed.  We
 * ought to tell the user, but since we don't have any guarantee that
 * we're even running on the right architecture, we do virtually nothing.
 *
 * If CONFIG_DEBUG_LL is set we try to print out something about the error
 * and hope for the best (useful if bootloader fails to pass a proper
 * machine ID for example).
 */
	__HEAD

/* Determine validity of the r2 atags pointer.  The heuristic requires
 * that the pointer be aligned, in the first 16k of physical RAM and
 * that the ATAG_CORE marker is first and present.  If CONFIG_OF_FLATTREE
 * is selected, then it will also accept a dtb pointer.  Future revisions
 * of this function may be more lenient with the physical address.
 *
 * It is also checked whether the atags/dtb area is located before the
 * end of the kernel's bss section and would hence be overridden by zeros
 * later. In that case, the atags area is relocated to the '_end' symbol.
 *
 * r2 = atags or dtb
 * r8 = phys_offset
 *
 * Returns:
 *  r2 either valid atags pointer, valid dtb pointer, or zero
 *  r3, r5 - r7 corrupted
 */
__vet_atags:
	tst	r2, #0x3			@ aligned?
	bne	1f

	ldr	r5, [r2, #0]
#ifdef CONFIG_OF_FLATTREE
	ldr	r6, =OF_DT_MAGIC		@ is it a DTB?
	cmp	r5, r6
	bne	5f

	ldreq	r5, [r2, #4]			@ fdt total size is at offset 4 ...
#ifndef CONFIG_CPU_BIG_ENDIAN
	eor	r6, r5, r5, ror #16		@ ... and stored in be32 order
	mov	r6, r6, lsr #8
	bic	r6, r6, #0xff00
	eor	r5, r6, r5, ror #8
#endif /* !CONFIG_CPU_BIG_ENDIAN */

	add	r5, r5, #4			@ align the size to 32bit
	bic	r5, r5, #3
	b	4f
#endif /* CONFIG_OF_FLATTREE */

5:	cmp	r5, #ATAG_CORE_SIZE		@ is first tag ATAG_CORE?
	cmpne	r5, #ATAG_CORE_SIZE_EMPTY
	bne	1f
	ldr	r5, [r2, #4]
	ldr	r6, =ATAG_CORE
	cmp	r5, r6
	movne	r5, #4096			@ FIXME: we should walk the atags and
						@ determine the real size.
	bne	1f

4:	adr	r3, 6f
	ldmia	r3!, {r6, r7}

	@ The kernel end address is stored in virtual address space, but we're
	@ still in flat mapping. Hence, we have to do virt_to_phys() manually.
	subs	r6, r6, r7			@ r7 = PAGE_OFFSET
	add	r6, r6, r8			@ r8 = PHYS_OFFET

	cmp	r2, r6				@ is the atags pointer inside the
						@ kernel area?
	bgt	2f

	add	r3, r6, r5			@ relocate start = .end + length
	add	r6, r3, r5			@ relocate end = .end + length * 2
3:	cmp	r3, r6
	ldrne	fp, [r2], #4
	strne	fp, [r3], #4
	bne	3b

	subs	r2, r6, r5			@ rewind back to the new
						@ atags/dtb image start

2:	mov	pc, lr				@ atag/dtb pointer is ok

1:	mov	r2, #0
	mov	pc, lr
ENDPROC(__vet_atags)
	.align
6:	.long	_end				@ r6
	.long	PAGE_OFFSET			@ r7

/*
 * The following fragment of code is executed with the MMU on in MMU mode,
 * and uses absolute addresses; this is not position independent.
 *
 *  r0  = cp#15 control register
 *  r1  = machine ID
 *  r2  = atags/dtb pointer
 *  r9  = processor ID
 */
	__INIT
__mmap_switched:
	adr	r3, __mmap_switched_data

	ldmia	r3!, {r4, r5, r6, r7}
	cmp	r4, r5				@ Copy data segment if needed
1:	cmpne	r5, r6
	ldrne	fp, [r4], #4
	strne	fp, [r5], #4
	bne	1b

	mov	fp, #0				@ Clear BSS (and zero fp)
1:	cmp	r6, r7
	strcc	fp, [r6],#4
	bcc	1b

 ARM(	ldmia	r3, {r4, r5, r6, r7, sp})
 THUMB(	ldmia	r3, {r4, r5, r6, r7}	)
 THUMB(	ldr	sp, [r3, #16]		)
	str	r9, [r4]			@ Save processor ID
	str	r1, [r5]			@ Save machine type
	str	r2, [r6]			@ Save atags pointer
	cmp	r7, #0
	bicne	r4, r0, #CR_A			@ Clear 'A' bit
	stmneia	r7, {r0, r4}			@ Save control register values
	b	start_kernel
ENDPROC(__mmap_switched)

	.align	2
	.type	__mmap_switched_data, %object
__mmap_switched_data:
	.long	__data_loc			@ r4
	.long	_sdata				@ r5
	.long	__bss_start			@ r6
	.long	_end				@ r7
	.long	processor_id			@ r4
	.long	__machine_arch_type		@ r5
	.long	__atags_pointer			@ r6
#ifdef CONFIG_CPU_CP15
	.long	cr_alignment			@ r7
#else
	.long	0				@ r7
#endif
	.long	init_thread_union + THREAD_START_SP @ sp
	.size	__mmap_switched_data, . - __mmap_switched_data

/*
 * This provides a C-API version of __lookup_processor_type
 */
ENTRY(lookup_processor_type)
	stmfd	sp!, {r4 - r6, r9, lr}
	mov	r9, r0
	bl	__lookup_processor_type
	mov	r0, r5
	ldmfd	sp!, {r4 - r6, r9, pc}
ENDPROC(lookup_processor_type)

	__FINIT
	.text

/*
 * Read processor ID register (CP#15, CR0), and look up in the linker-built
 * supported processor list.  Note that we can't use the absolute addresses
 * for the __proc_info lists since we aren't running with the MMU on
 * (and therefore, we are not in the correct address space).  We have to
 * calculate the offset.
 *
 *	r9 = cpuid
 * Returns:
 *	r3, r4, r6 corrupted
 *	r5 = proc_info pointer in physical address space
 *	r9 = cpuid (preserved)
 */
__lookup_processor_type:
	adr	r3, __lookup_processor_type_data
	ldmia	r3, {r4 - r6}
	sub	r3, r3, r4			@ get offset between virt&phys
	add	r5, r5, r3			@ convert virt addresses to
	add	r6, r6, r3			@ physical address space
1:	ldmia	r5, {r3, r4}			@ value, mask
	and	r4, r4, r9			@ mask wanted bits
	teq	r3, r4
	beq	2f
	add	r5, r5, #PROC_INFO_SZ		@ sizeof(proc_info_list)
	cmp	r5, r6
	blo	1b
	mov	r5, #0				@ unknown processor
2:	mov	pc, lr
ENDPROC(__lookup_processor_type)

/*
 * Look in <asm/procinfo.h> for information about the __proc_info structure.
 */
	.align	2
	.type	__lookup_processor_type_data, %object
__lookup_processor_type_data:
	.long	.
	.long	__proc_info_begin
	.long	__proc_info_end
	.size	__lookup_processor_type_data, . - __lookup_processor_type_data

__error_p:
#ifdef CONFIG_DEBUG_LL
	adr	r0, str_p1
	bl	printascii
	mov	r0, r9
	bl	printhex8
	adr	r0, str_p2
	bl	printascii
	b	__error
str_p1:	.asciz	"\nError: unrecognized/unsupported processor variant (0x"
str_p2:	.asciz	").\n"
	.align
#endif
ENDPROC(__error_p)

__error:
#ifdef CONFIG_ARCH_RPC
/*
 * Turn the screen red on a error - RiscPC only.
 */
	mov	r0, #0x02000000
	mov	r3, #0x11
	orr	r3, r3, r3, lsl #8
	orr	r3, r3, r3, lsl #16
	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
	str	r3, [r0], #4
#endif
1:	mov	r0, r0
	b	1b
ENDPROC(__error)
