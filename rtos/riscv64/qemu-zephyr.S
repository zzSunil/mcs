
qemu-zephyr.elf:     file format elf64-littleriscv


Disassembly of section rom_start:

0000000083000000 <__start>:
SECTION_FUNC(vectors, __start)
#if defined(CONFIG_RISCV_GP)
	/* Initialize global pointer */
	.option push
	.option norelax
	la gp, __global_pointer$
    83000000:	00020197          	auipc	gp,0x20
    83000004:	47818193          	addi	gp,gp,1144 # 83020478 <__global_pointer$>
	 * CLINT direct mode
	 *
	 * Set mtvec (Machine Trap-Vector Base-Address Register)
	 * to _isr_wrapper.
	 */
	la t0, _isr_wrapper
    83000008:	00000297          	auipc	t0,0x0
    8300000c:	01828293          	addi	t0,t0,24 # 83000020 <_isr_wrapper>
	csrw xtvec, t0
    83000010:	10529073          	csrw	stvec,t0
#endif /* CONFIG_RISCV_HAS_CLIC&& !CONFIG_LEGACY_CLIC */

#endif /* CONFIG_RISCV_VECTORED_MODE */

	/* Jump to __reset */
	tail __reset
    83000014:	a011                	j	83000018 <__reset>
	...

Disassembly of section reset:

0000000083000018 <__reset>:
	/*
	 * jump to __initialize
	 * use call opcode in case __initialize is far away.
	 * This will be dependent on linker.ld configuration.
	 */
	call __initialize
    83000018:	1a0000ef          	jal	830001b8 <__initialize>
	...

Disassembly of section exceptions:

0000000083000020 <_isr_wrapper>:

#ifdef CONFIG_RISCV_SOC_HAS_ISR_STACKING
	SOC_ISR_SW_STACKING
#else
	/* Save caller-saved registers on current thread stack. */
	addi sp, sp, -__struct_arch_esf_SIZEOF
    83000020:	7135                	addi	sp,sp,-160
	DO_CALLER_SAVED(sr)		;
    83000022:	e416                	sd	t0,8(sp)
    83000024:	e81a                	sd	t1,16(sp)
    83000026:	ec1e                	sd	t2,24(sp)
    83000028:	f072                	sd	t3,32(sp)
    8300002a:	f476                	sd	t4,40(sp)
    8300002c:	f87a                	sd	t5,48(sp)
    8300002e:	fc7e                	sd	t6,56(sp)
    83000030:	e0aa                	sd	a0,64(sp)
    83000032:	e4ae                	sd	a1,72(sp)
    83000034:	e8b2                	sd	a2,80(sp)
    83000036:	ecb6                	sd	a3,88(sp)
    83000038:	f0ba                	sd	a4,96(sp)
    8300003a:	f4be                	sd	a5,104(sp)
    8300003c:	f8c2                	sd	a6,112(sp)
    8300003e:	fcc6                	sd	a7,120(sp)
    83000040:	e006                	sd	ra,0(sp)
#endif /* CONFIG_RISCV_SOC_HAS_ISR_STACKING */

	/* Save s0 in the esf and load it with &_current_cpu. */
	sr s0, __struct_arch_esf_s0_OFFSET(sp)
    83000042:	e922                	sd	s0,144(sp)
	get_current_cpu s0
    83000044:	0000c417          	auipc	s0,0xc
    83000048:	8ec40413          	addi	s0,s0,-1812 # 8300b930 <_kernel>

	/* Save MEPC register */
	csrr t0, xepc
    8300004c:	141022f3          	csrr	t0,sepc
	sr t0, __struct_arch_esf_xepc_OFFSET(sp)
    83000050:	e116                	sd	t0,128(sp)

	/* Save MSTATUS register */
	csrr t2, xstatus
    83000052:	100023f3          	csrr	t2,sstatus
	sr t2, __struct_arch_esf_xstatus_OFFSET(sp)
    83000056:	e51e                	sd	t2,136(sp)
	 */
#ifdef CONFIG_RISCV_SOC_EXCEPTION_FROM_IRQ
	jal ra, __soc_is_irq
	bnez a0, is_interrupt
#else
	csrr t0, xcause
    83000058:	142022f3          	csrr	t0,scause
	srli t0, t0, RISCV_MCAUSE_IRQ_POS
    8300005c:	03f2d293          	srli	t0,t0,0x3f
	bnez t0, is_interrupt
    83000060:	04029d63          	bnez	t0,830000ba <is_interrupt>
	/*
	 * If the exception is the result of an ECALL, check whether to
	 * perform a context-switch or an IRQ offload. Otherwise call _Fault
	 * to report the exception.
	 */
	csrr t0, xcause
    83000064:	142022f3          	csrr	t0,scause
	li t2, CONFIG_RISCV_MCAUSE_EXCEPTION_MASK
    83000068:	fff0039b          	addiw	t2,zero,-1
    8300006c:	13fe                	slli	t2,t2,0x3f
    8300006e:	13fd                	addi	t2,t2,-1
	and t0, t0, t2
    83000070:	0072f2b3          	and	t0,t0,t2

	/*
	 * If mcause == RISCV_EXC_ECALLM, handle system call from
	 * kernel thread.
	 */
	li t1, RISCV_EXC_ECALL
    83000074:	4325                	li	t1,9
	beq t0, t1, is_kernel_syscall
    83000076:	00628963          	beq	t0,t1,83000088 <is_kernel_syscall>
	 * Stack pointer is pointing to a struct_arch_esf structure, pass it
	 * to _Fault (via register a0).
	 * If _Fault shall return, set return address to
	 * no_reschedule to restore stack.
	 */
	mv a0, sp
    8300007a:	850a                	mv	a0,sp
	la ra, no_reschedule
    8300007c:	00000097          	auipc	ra,0x0
    83000080:	0b808093          	addi	ra,ra,184 # 83000134 <no_reschedule>
	tail _Fault
    83000084:	0e30006f          	j	83000966 <_Fault>

0000000083000088 <is_kernel_syscall>:
	 * again upon exiting the ISR.
	 *
	 * It's safe to always increment by 4, even with compressed
	 * instructions, because the ecall instruction is always 4 bytes.
	 */
	lr t0, __struct_arch_esf_xepc_OFFSET(sp)
    83000088:	628a                	ld	t0,128(sp)
	addi t0, t0, 4
    8300008a:	0291                	addi	t0,t0,4
	sr t0, __struct_arch_esf_xepc_OFFSET(sp)
    8300008c:	e116                	sd	t0,128(sp)
	li t1, MSTATUS_MPRV
	csrs mstatus, t1
#endif

	/* Determine what to do. Operation code is in t0. */
	lr t0, __struct_arch_esf_t0_OFFSET(sp)
    8300008e:	62a2                	ld	t0,8(sp)

	.if RV_ECALL_RUNTIME_EXCEPT != 0; .err; .endif
	beqz t0, do_fault
    83000090:	00028463          	beqz	t0,83000098 <do_fault>
	j reschedule
skip_schedule:
#endif

	/* default fault code is K_ERR_KERNEL_OOPS */
	li a0, 3
    83000094:	450d                	li	a0,3
	j 1f
    83000096:	a011                	j	8300009a <do_fault+0x2>

0000000083000098 <do_fault>:

do_fault:
	/* Handle RV_ECALL_RUNTIME_EXCEPT. Retrieve reason in a0, esf in A1. */
	lr a0, __struct_arch_esf_a0_OFFSET(sp)
    83000098:	6506                	ld	a0,64(sp)
1:	mv a1, sp
    8300009a:	858a                	mv	a1,sp

#ifdef CONFIG_EXCEPTION_DEBUG
	/* Allocate space for caller-saved registers on current thread stack */
	addi sp, sp, -__callee_saved_t_SIZEOF
    8300009c:	7159                	addi	sp,sp,-112

	/* Save callee-saved registers to be passed as 3rd arg */
	STORE_CALLEE_SAVED()		;
    8300009e:	e822                	sd	s0,16(sp)
    830000a0:	ec26                	sd	s1,24(sp)
    830000a2:	f04a                	sd	s2,32(sp)
    830000a4:	f44e                	sd	s3,40(sp)
    830000a6:	f852                	sd	s4,48(sp)
    830000a8:	fc56                	sd	s5,56(sp)
    830000aa:	e0da                	sd	s6,64(sp)
    830000ac:	e4de                	sd	s7,72(sp)
    830000ae:	e8e2                	sd	s8,80(sp)
    830000b0:	ece6                	sd	s9,88(sp)
    830000b2:	f0ea                	sd	s10,96(sp)
    830000b4:	f4ee                	sd	s11,104(sp)
	mv a2, sp
    830000b6:	860a                	mv	a2,sp

	tail z_riscv_fatal_error_csf
    830000b8:	adfd                	j	830007b6 <z_riscv_fatal_error_csf>

00000000830000ba <is_interrupt>:
	csrs mstatus, t1
2:
#endif

	/* Increment _current_cpu->nested */
	lw t1, ___cpu_t_nested_OFFSET(s0)
    830000ba:	00042303          	lw	t1,0(s0)
	addi t2, t1, 1
    830000be:	00130393          	addi	t2,t1,1
	sw t2, ___cpu_t_nested_OFFSET(s0)
    830000c2:	00742023          	sw	t2,0(s0)
	bnez t1, on_irq_stack
    830000c6:	00031763          	bnez	t1,830000d4 <on_irq_stack>

	/* Switch to interrupt stack */
	mv t0, sp
    830000ca:	828a                	mv	t0,sp
	lr sp, ___cpu_t_irq_stack_OFFSET(s0)
    830000cc:	00843103          	ld	sp,8(s0)

	/*
	 * Save thread stack pointer on interrupt stack
	 * In RISC-V, stack pointer needs to be 16-byte aligned
	 */
	addi sp, sp, -16
    830000d0:	1141                	addi	sp,sp,-16
	sr t0, 0(sp)
    830000d2:	e016                	sd	t0,0(sp)

00000000830000d4 <on_irq_stack>:
#ifdef CONFIG_TRACING_ISR
	call sys_trace_isr_enter
#endif

	/* Get IRQ causing interrupt */
	csrr a0, xcause
    830000d4:	14202573          	csrr	a0,scause
	li t0, CONFIG_RISCV_MCAUSE_EXCEPTION_MASK
    830000d8:	fff0029b          	addiw	t0,zero,-1
    830000dc:	12fe                	slli	t0,t0,0x3f
    830000de:	12fd                	addi	t0,t0,-1
	and a0, a0, t0
    830000e0:	00557533          	and	a0,a0,t0

	/*
	 * Clear pending IRQ generating the interrupt at SOC level
	 * Pass IRQ number to __soc_handle_irq via register a0
	 */
	jal ra, __soc_handle_irq
    830000e4:	088000ef          	jal	8300016c <__soc_handle_irq>

	/*
	 * Call corresponding registered function in _sw_isr_table.
	 * (table is 2-word wide, we should shift index accordingly)
	 */
	la t0, _sw_isr_table
    830000e8:	00004297          	auipc	t0,0x4
    830000ec:	e5828293          	addi	t0,t0,-424 # 83003f40 <_sw_isr_table>
	slli a0, a0, (RV_REGSHIFT + 1)
    830000f0:	0512                	slli	a0,a0,0x4
	add t0, t0, a0
    830000f2:	92aa                	add	t0,t0,a0

	/* Load argument in a0 register */
	lr a0, 0(t0)
    830000f4:	0002b503          	ld	a0,0(t0)

	/* Load ISR function address in register t1 */
	lr t1, RV_REGSIZE(t0)
    830000f8:	0082b303          	ld	t1,8(t0)

	/* Call ISR function */
	jalr ra, t1, 0
    830000fc:	000300e7          	jalr	t1

0000000083000100 <irq_done>:

#endif

irq_done:
	/* Decrement _current_cpu->nested */
	lw t2, ___cpu_t_nested_OFFSET(s0)
    83000100:	00042383          	lw	t2,0(s0)
	addi t2, t2, -1
    83000104:	13fd                	addi	t2,t2,-1
	sw t2, ___cpu_t_nested_OFFSET(s0)
    83000106:	00742023          	sw	t2,0(s0)
	bnez t2, no_reschedule
    8300010a:	02039563          	bnez	t2,83000134 <no_reschedule>

	/* nested count is back to 0: Return to thread stack */
	lr sp, 0(sp)
    8300010e:	6102                	ld	sp,0(sp)

#ifdef CONFIG_STACK_SENTINEL
	call z_check_stack_sentinel
    83000110:	251010ef          	jal	83001b60 <z_check_stack_sentinel>

0000000083000114 <check_reschedule>:
check_reschedule:

#ifdef CONFIG_MULTITHREADING

	/* Get pointer to current thread on this CPU */
	lr a1, ___cpu_t_current_OFFSET(s0)
    83000114:	680c                	ld	a1,16(s0)
	/*
	 * Get next thread to schedule with z_get_next_switch_handle().
	 * We pass it a NULL as we didn't save the whole thread context yet.
	 * If no scheduling is necessary then NULL will be returned.
	 */
	addi sp, sp, -16
    83000116:	1141                	addi	sp,sp,-16
	sr a1, 0(sp)
    83000118:	e02e                	sd	a1,0(sp)
	mv a0, zero
    8300011a:	00000513          	li	a0,0
	call z_get_next_switch_handle
    8300011e:	2f7020ef          	jal	83002c14 <z_get_next_switch_handle>
	lr a1, 0(sp)
    83000122:	6582                	ld	a1,0(sp)
	addi sp, sp, 16
    83000124:	0141                	addi	sp,sp,16
	beqz a0, no_reschedule
    83000126:	c519                	beqz	a0,83000134 <no_reschedule>

0000000083000128 <reschedule>:
	/*
	 * Perform context switch:
	 * a0 = new thread
	 * a1 = old thread
	 */
	call z_riscv_switch
    83000128:	0b8000ef          	jal	830001e0 <z_riscv_switch>

000000008300012c <z_riscv_thread_start>:

z_riscv_thread_start:
might_have_rescheduled:
	/* reload s0 with &_current_cpu as it might have changed or be unset */
	get_current_cpu s0
    8300012c:	0000c417          	auipc	s0,0xc
    83000130:	80440413          	addi	s0,s0,-2044 # 8300b930 <_kernel>

0000000083000134 <no_reschedule>:
	sb t1, _thread_offset_to_exception_depth(t0)
fp_trap_exit:
#endif

	/* Restore MEPC and MSTATUS registers */
	lr t0, __struct_arch_esf_xepc_OFFSET(sp)
    83000134:	628a                	ld	t0,128(sp)
	lr t2, __struct_arch_esf_xstatus_OFFSET(sp)
    83000136:	63aa                	ld	t2,136(sp)
	csrw xepc, t0
    83000138:	14129073          	csrw	sepc,t0
	csrw xstatus, t2
    8300013c:	10039073          	csrw	sstatus,t2
	sr t0, __struct_arch_esf_sp_OFFSET(sp)
2:
#endif

	/* Restore s0 (it is no longer ours) */
	lr s0, __struct_arch_esf_s0_OFFSET(sp)
    83000140:	644a                	ld	s0,144(sp)

#ifdef CONFIG_RISCV_SOC_HAS_ISR_STACKING
	SOC_ISR_SW_UNSTACKING
#else
	/* Restore caller-saved registers from thread stack */
	DO_CALLER_SAVED(lr)
    83000142:	62a2                	ld	t0,8(sp)
    83000144:	6342                	ld	t1,16(sp)
    83000146:	63e2                	ld	t2,24(sp)
    83000148:	7e02                	ld	t3,32(sp)
    8300014a:	7ea2                	ld	t4,40(sp)
    8300014c:	7f42                	ld	t5,48(sp)
    8300014e:	7fe2                	ld	t6,56(sp)
    83000150:	6506                	ld	a0,64(sp)
    83000152:	65a6                	ld	a1,72(sp)
    83000154:	6646                	ld	a2,80(sp)
    83000156:	66e6                	ld	a3,88(sp)
    83000158:	7706                	ld	a4,96(sp)
    8300015a:	77a6                	ld	a5,104(sp)
    8300015c:	7846                	ld	a6,112(sp)
    8300015e:	78e6                	ld	a7,120(sp)
    83000160:	6082                	ld	ra,0(sp)
#ifdef CONFIG_USERSPACE
	/* retrieve saved stack pointer */
	lr sp, __struct_arch_esf_sp_OFFSET(sp)
#else
	/* remove esf from the stack */
	addi sp, sp, __struct_arch_esf_SIZEOF
    83000162:	610d                	addi	sp,sp,160
#endif

#endif /* CONFIG_RISCV_SOC_HAS_ISR_STACKING */

	xret
    83000164:	10200073          	sret
    83000168:	0000                	unimp
	...

000000008300016c <__soc_handle_irq>:
 * SOC-specific function to handle pending IRQ number generating the interrupt.
 * Exception number is given as parameter via register a0.
 */
SECTION_FUNC(exception.other, __soc_handle_irq)
	/* Clear exception number from CSR mip register */
	li t1, 1
    8300016c:	4305                	li	t1,1
	sll t0, t1, a0
    8300016e:	00a312b3          	sll	t0,t1,a0
	csrrc t1, xip, t0
    83000172:	1442b373          	csrrc	t1,sip,t0

	/* Return */
	ret
    83000176:	8082                	ret
	...

Disassembly of section text:

000000008300017c <__clzdi2>:
    8300017c:	03855793          	srli	a5,a0,0x38
    83000180:	eb8d                	bnez	a5,830001b2 <__clzdi2+0x36>
    83000182:	03800793          	li	a5,56
    83000186:	17e1                	addi	a5,a5,-8
    83000188:	c791                	beqz	a5,83000194 <__clzdi2+0x18>
    8300018a:	00f55733          	srl	a4,a0,a5
    8300018e:	0ff77713          	zext.b	a4,a4
    83000192:	db75                	beqz	a4,83000186 <__clzdi2+0xa>
    83000194:	04000713          	li	a4,64
    83000198:	8f1d                	sub	a4,a4,a5
    8300019a:	00f55533          	srl	a0,a0,a5
    8300019e:	00008797          	auipc	a5,0x8
    830001a2:	e9a78793          	addi	a5,a5,-358 # 83008038 <__clz_tab>
    830001a6:	97aa                	add	a5,a5,a0
    830001a8:	0007c503          	lbu	a0,0(a5)
    830001ac:	40a7053b          	subw	a0,a4,a0
    830001b0:	8082                	ret
    830001b2:	03800793          	li	a5,56
    830001b6:	bff9                	j	83000194 <__clzdi2+0x18>

00000000830001b8 <__initialize>:
 */
SECTION_FUNC(TEXT, __initialize)
#ifndef CONFIG_RISCV_SBI_BOOT
	csrr a0, mhartid
#endif
	li t0, CONFIG_RV_BOOT_HART
    830001b8:	4289                	li	t0,2
	beq a0, t0, boot_first_core
    830001ba:	00550363          	beq	a0,t0,830001c0 <boot_first_core>
	j boot_secondary_core
    830001be:	a819                	j	830001d4 <boot_secondary_core>

00000000830001c0 <boot_first_core>:

	/*
	 * Initially, setup stack pointer to
	 * z_interrupt_stacks + __z_interrupt_stack_SIZEOF
	 */
	la sp, z_interrupt_stacks
    830001c0:	0001e117          	auipc	sp,0x1e
    830001c4:	e7010113          	addi	sp,sp,-400 # 8301e030 <z_interrupt_stacks>
	li t0, __z_interrupt_stack_SIZEOF
    830001c8:	6285                	lui	t0,0x1
    830001ca:	8002829b          	addiw	t0,t0,-2048 # 800 <CONFIG_ISR_STACK_SIZE>
	add sp, sp, t0
    830001ce:	9116                	add	sp,sp,t0

	/*
	 * Jump into C domain. z_prep_c zeroes BSS, copies rw data into RAM,
	 * and then enters kernel z_cstart
	 */
	call z_prep_c
    830001d0:	06d000ef          	jal	83000a3c <z_prep_c>

00000000830001d4 <boot_secondary_core>:
	la t0, riscv_cpu_boot_flag
	li t1, 1
	sr t1, 0(t0)
	j arch_secondary_cpu_init
#else
	j loop_unconfigured_cores
    830001d4:	a009                	j	830001d6 <loop_unconfigured_cores>

00000000830001d6 <loop_unconfigured_cores>:
#endif

loop_unconfigured_cores:
	wfi
    830001d6:	10500073          	wfi
	j loop_unconfigured_cores
    830001da:	bff5                	j	830001d6 <loop_unconfigured_cores>
    830001dc:	0000                	unimp
	...

00000000830001e0 <z_riscv_switch>:

/* void z_riscv_switch(k_thread_t *switch_to, k_thread_t *switch_from) */
SECTION_FUNC(TEXT, z_riscv_switch)

	/* Save the old thread's callee-saved registers */
	DO_CALLEE_SAVED(sr, a1)
    830001e0:	0415b823          	sd	ra,80(a1)
    830001e4:	eda0                	sd	s0,88(a1)
    830001e6:	f1a4                	sd	s1,96(a1)
    830001e8:	0725b423          	sd	s2,104(a1)
    830001ec:	0735b823          	sd	s3,112(a1)
    830001f0:	0745bc23          	sd	s4,120(a1)
    830001f4:	0955b023          	sd	s5,128(a1)
    830001f8:	0965b423          	sd	s6,136(a1)
    830001fc:	0975b823          	sd	s7,144(a1)
    83000200:	0985bc23          	sd	s8,152(a1)
    83000204:	0b95b023          	sd	s9,160(a1)
    83000208:	0ba5b423          	sd	s10,168(a1)
    8300020c:	0bb5b823          	sd	s11,176(a1)

	/* Save the old thread's stack pointer */
	sr sp, _thread_offset_to_sp(a1)
    83000210:	0425b423          	sd	sp,72(a1)

	/* Set thread->switch_handle = thread to mark completion */
	sr a1, ___thread_t_switch_handle_OFFSET(a1)
    83000214:	10b5b823          	sd	a1,272(a1)

	/* Get the new thread's stack pointer */
	lr sp, _thread_offset_to_sp(a0)
    83000218:	04853103          	ld	sp,72(a0)

#if defined(CONFIG_THREAD_LOCAL_STORAGE)
	/* Get the new thread's tls pointer */
	lr tp, _thread_offset_to_tls(a0)
    8300021c:	12053203          	ld	tp,288(a0)
	call z_thread_mark_switched_in
	mv a0, s0
#endif

	/* Restore the new thread's callee-saved registers */
	DO_CALLEE_SAVED(lr, a0)
    83000220:	05053083          	ld	ra,80(a0)
    83000224:	6d20                	ld	s0,88(a0)
    83000226:	7124                	ld	s1,96(a0)
    83000228:	06853903          	ld	s2,104(a0)
    8300022c:	07053983          	ld	s3,112(a0)
    83000230:	07853a03          	ld	s4,120(a0)
    83000234:	08053a83          	ld	s5,128(a0)
    83000238:	08853b03          	ld	s6,136(a0)
    8300023c:	09053b83          	ld	s7,144(a0)
    83000240:	09853c03          	ld	s8,152(a0)
    83000244:	0a053c83          	ld	s9,160(a0)
    83000248:	0a853d03          	ld	s10,168(a0)
    8300024c:	0b053d83          	ld	s11,176(a0)

	/* Return to arch_switch() or _irq_wrapper() */
	ret
    83000250:	8082                	ret

0000000083000252 <_OffsetAbsSyms>:
GEN_OFFSET_SYM(_cpu_arch_t, user_exc_sp);
GEN_OFFSET_SYM(_cpu_arch_t, user_exc_tmp0);
GEN_OFFSET_SYM(_cpu_arch_t, user_exc_tmp1);
#endif

GEN_ABS_SYM_END
    83000252:	8082                	ret

0000000083000254 <test_thread>:
static struct k_thread tthread[THREADS_NUM];
static char th_buffer[THREADS_NUM][DIGITS_NUM + 1];
static atomic_t th_counter = THREADS_NUM;

void test_thread(void *arg1, void *arg2, void *arg3)
{
    83000254:	81010113          	addi	sp,sp,-2032
    83000258:	7e813023          	sd	s0,2016(sp)
    8300025c:	7c913c23          	sd	s1,2008(sp)
    83000260:	7d313423          	sd	s3,1992(sp)
    83000264:	7e113423          	sd	ra,2024(sp)
    83000268:	7d213823          	sd	s2,2000(sp)
    8300026c:	7d413023          	sd	s4,1984(sp)
    83000270:	7b513c23          	sd	s5,1976(sp)
    83000274:	7b613823          	sd	s6,1968(sp)
	 * -------------------------------->8--------------------------------
	 */
	#define NEW_BASE	10000
	#define ARRAY_INIT	2000

	int array[LENGTH + 1] = {};
    83000278:	6605                	lui	a2,0x1
{
    8300027a:	a8010113          	addi	sp,sp,-1408
    8300027e:	89aa                	mv	s3,a0
    83000280:	84ae                	mv	s1,a1
	int array[LENGTH + 1] = {};
    83000282:	d2460613          	addi	a2,a2,-732 # d24 <CONFIG_ISR_STACK_SIZE+0x524>
    83000286:	4581                	li	a1,0
    83000288:	0028                	addi	a0,sp,8
    8300028a:	403030ef          	jal	83003e8c <memset>
	int carry = 0;
	int i, j;

	for (i = 0; i < LENGTH; i++) {
    8300028e:	6685                	lui	a3,0x1
	int array[LENGTH + 1] = {};
    83000290:	747d                	lui	s0,0xfffff
    83000292:	2d840793          	addi	a5,s0,728 # fffffffffffff2d8 <CONFIG_RISCV_MCAUSE_EXCEPTION_MASK+0x7ffffffffffff2d9>
    83000296:	d3068713          	addi	a4,a3,-720 # d30 <CONFIG_ISR_STACK_SIZE+0x530>
    8300029a:	973e                	add	a4,a4,a5
    8300029c:	002707b3          	add	a5,a4,sp
    830002a0:	d2868713          	addi	a4,a3,-728
    830002a4:	970a                	add	a4,a4,sp
		array[i] = ARRAY_INIT;
    830002a6:	7d000693          	li	a3,2000
    830002aa:	c394                	sw	a3,0(a5)
	for (i = 0; i < LENGTH; i++) {
    830002ac:	0791                	addi	a5,a5,4
    830002ae:	fee79ee3          	bne	a5,a4,830002aa <test_thread+0x56>

	for (i = LENGTH; i > 0; i -= 14) {
		int sum = 0, value;

		for (j = i; j > 0; --j) {
			sum = sum * j + NEW_BASE * array[j];
    830002b2:	6909                	lui	s2,0x2
    830002b4:	843e                	mv	s0,a5
	for (i = LENGTH; i > 0; i -= 14) {
    830002b6:	34800a13          	li	s4,840
	int carry = 0;
    830002ba:	4b01                	li	s6,0
			sum = sum * j + NEW_BASE * array[j];
    830002bc:	7109091b          	addiw	s2,s2,1808 # 2710 <CONFIG_MAIN_STACK_SIZE+0x1710>

		value = carry + sum / NEW_BASE;
		carry = sum % NEW_BASE;

		/* Convert 4-digit int to string */
		sprintf(buffer, "%.4d", value);
    830002c0:	00008a97          	auipc	s5,0x8
    830002c4:	e78a8a93          	addi	s5,s5,-392 # 83008138 <__clz_tab+0x100>
	int carry = 0;
    830002c8:	86a2                	mv	a3,s0
    830002ca:	8752                	mv	a4,s4
		int sum = 0, value;
    830002cc:	4601                	li	a2,0
			sum = sum * j + NEW_BASE * array[j];
    830002ce:	429c                	lw	a5,0(a3)
    830002d0:	02c7063b          	mulw	a2,a4,a2
		for (j = i; j > 0; --j) {
    830002d4:	16f1                	addi	a3,a3,-4
			sum = sum * j + NEW_BASE * array[j];
    830002d6:	02f907bb          	mulw	a5,s2,a5
    830002da:	9fb1                	addw	a5,a5,a2
			array[j] = sum % (j * 2 - 1);
    830002dc:	0017161b          	slliw	a2,a4,0x1
    830002e0:	367d                	addiw	a2,a2,-1
    830002e2:	02c7e5bb          	remw	a1,a5,a2
		for (j = i; j > 0; --j) {
    830002e6:	377d                	addiw	a4,a4,-1
			sum /= j * 2 - 1;
    830002e8:	02c7c63b          	divw	a2,a5,a2
			array[j] = sum % (j * 2 - 1);
    830002ec:	c2cc                	sw	a1,4(a3)
			sum /= j * 2 - 1;
    830002ee:	87b2                	mv	a5,a2
		for (j = i; j > 0; --j) {
    830002f0:	ff79                	bnez	a4,830002ce <test_thread+0x7a>
		value = carry + sum / NEW_BASE;
    830002f2:	0326463b          	divw	a2,a2,s2
		sprintf(buffer, "%.4d", value);
    830002f6:	8526                	mv	a0,s1
    830002f8:	85d6                	mv	a1,s5
	for (i = LENGTH; i > 0; i -= 14) {
    830002fa:	3a49                	addiw	s4,s4,-14
		buffer += 4;
    830002fc:	0491                	addi	s1,s1,4
	for (i = LENGTH; i > 0; i -= 14) {
    830002fe:	fc840413          	addi	s0,s0,-56
		value = carry + sum / NEW_BASE;
    83000302:	0166063b          	addw	a2,a2,s6
		carry = sum % NEW_BASE;
    83000306:	0327eb3b          	remw	s6,a5,s2
		sprintf(buffer, "%.4d", value);
    8300030a:	5d0030ef          	jal	830038da <sprintf>
	for (i = LENGTH; i > 0; i -= 14) {
    8300030e:	fa0a1de3          	bnez	s4,830002c8 <test_thread+0x74>
	return __atomic_fetch_add(target, value, __ATOMIC_SEQ_CST);
}

static inline atomic_val_t atomic_sub(atomic_t *target, atomic_val_t value)
{
	return __atomic_fetch_sub(target, value, __ATOMIC_SEQ_CST);
    83000312:	57fd                	li	a5,-1
    83000314:	0f50000f          	fence	iorw,ow
    83000318:	04f9b02f          	amoadd.d.aq	zero,a5,(s3)
	}

	atomic_dec(counter);
}
    8300031c:	58010113          	addi	sp,sp,1408
    83000320:	7e813083          	ld	ra,2024(sp)
    83000324:	7e013403          	ld	s0,2016(sp)
    83000328:	7d813483          	ld	s1,2008(sp)
    8300032c:	7d013903          	ld	s2,2000(sp)
    83000330:	7c813983          	ld	s3,1992(sp)
    83000334:	7c013a03          	ld	s4,1984(sp)
    83000338:	7b813a83          	ld	s5,1976(sp)
    8300033c:	7b013b03          	ld	s6,1968(sp)
    83000340:	7f010113          	addi	sp,sp,2032
    83000344:	8082                	ret

0000000083000346 <main>:

int main(void)
{
    83000346:	711d                	addi	sp,sp,-96
	uint32_t start_time, stop_time, cycles_spent, nanoseconds_spent;
	int i;

	printk("Calculate first %d digits of Pi independently by %d threads.\n",
    83000348:	4641                	li	a2,16
    8300034a:	0f000593          	li	a1,240
    8300034e:	00008517          	auipc	a0,0x8
    83000352:	df250513          	addi	a0,a0,-526 # 83008140 <__clz_tab+0x108>
{
    83000356:	ec86                	sd	ra,88(sp)
    83000358:	e8a2                	sd	s0,80(sp)
    8300035a:	e4a6                	sd	s1,72(sp)
    8300035c:	e0ca                	sd	s2,64(sp)
    8300035e:	fc4e                	sd	s3,56(sp)
    83000360:	f852                	sd	s4,48(sp)
    83000362:	f456                	sd	s5,40(sp)
    83000364:	f05a                	sd	s6,32(sp)
    83000366:	ec5e                	sd	s7,24(sp)
    83000368:	e862                	sd	s8,16(sp)
	printk("Calculate first %d digits of Pi independently by %d threads.\n",
    8300036a:	360000ef          	jal	830006ca <printk>

extern uint32_t sys_clock_cycle_get_32(void);

static inline uint32_t arch_k_cycle_get_32(void)
{
	return sys_clock_cycle_get_32();
    8300036e:	1d0010ef          	jal	8300153e <sys_clock_cycle_get_32>
    83000372:	00009917          	auipc	s2,0x9
    83000376:	1c690913          	addi	s2,s2,454 # 83009538 <th_buffer>
		};
		return (k_tid_t) arch_syscall_invoke6(parm0.x, parm1.x, parm2.x, parm3.x, parm4.x, (uintptr_t) &more, K_SYSCALL_K_THREAD_CREATE);
	}
#endif
	compiler_barrier();
	return z_impl_k_thread_create(new_thread, stack, stack_size, entry, p1, p2, p3, prio, options, delay);
    8300037a:	6485                	lui	s1,0x1
    8300037c:	0005099b          	sext.w	s3,a0
	       DIGITS_NUM, THREADS_NUM);

	/* Capture initial time stamp */
	start_time = k_cycle_get_32();

	for (i = 0; i < THREADS_NUM; i++) {
    83000380:	0000a417          	auipc	s0,0xa
    83000384:	0c840413          	addi	s0,s0,200 # 8300a448 <tthread>
    83000388:	0000ca97          	auipc	s5,0xc
    8300038c:	aa8a8a93          	addi	s5,s5,-1368 # 8300be30 <tstack>
    83000390:	0000bb17          	auipc	s6,0xb
    83000394:	338b0b13          	addi	s6,s6,824 # 8300b6c8 <z_malloc_heap>
    83000398:	8a4a                	mv	s4,s2
    8300039a:	00020c17          	auipc	s8,0x20
    8300039e:	8dec0c13          	addi	s8,s8,-1826 # 8301fc78 <th_counter>
    830003a2:	00000b97          	auipc	s7,0x0
    830003a6:	eb2b8b93          	addi	s7,s7,-334 # 83000254 <test_thread>
    830003aa:	22048493          	addi	s1,s1,544 # 1220 <CONFIG_MAIN_STACK_SIZE+0x220>
    830003ae:	58e9                	li	a7,-6
    830003b0:	87d2                	mv	a5,s4
    830003b2:	85d6                	mv	a1,s5
    830003b4:	8522                	mv	a0,s0
    830003b6:	e402                	sd	zero,8(sp)
    830003b8:	e002                	sd	zero,0(sp)
    830003ba:	4801                	li	a6,0
    830003bc:	8762                	mv	a4,s8
    830003be:	86de                	mv	a3,s7
    830003c0:	8626                	mv	a2,s1
    830003c2:	12840413          	addi	s0,s0,296
    830003c6:	123010ef          	jal	83001ce8 <z_impl_k_thread_create>
    830003ca:	9aa6                	add	s5,s5,s1
    830003cc:	0f1a0a13          	addi	s4,s4,241
    830003d0:	fd641fe3          	bne	s0,s6,830003ae <main+0x68>
			       (void *)&th_counter, (void *)th_buffer[i], NULL,
			       K_PRIO_COOP(10), 0, K_NO_WAIT);
	}

	/* Wait for all workers to finish their calculations */
	while (th_counter) {
    830003d4:	00020417          	auipc	s0,0x20
    830003d8:	8a440413          	addi	s0,s0,-1884 # 8301fc78 <th_counter>
    830003dc:	601c                	ld	a5,0(s0)
    830003de:	e7a5                	bnez	a5,83000446 <main+0x100>
    830003e0:	15e010ef          	jal	8300153e <sys_clock_cycle_get_32>
	}

	/* Capture final time stamp */
	stop_time = k_cycle_get_32();

	cycles_spent = stop_time - start_time;
    830003e4:	4135043b          	subw	s0,a0,s3
	nanoseconds_spent = (uint32_t)k_cyc_to_ns_floor64(cycles_spent);
    830003e8:	06400793          	li	a5,100
    830003ec:	02f4043b          	mulw	s0,s0,a5

	for (i = 0; i < THREADS_NUM; i++) {
    830003f0:	4481                	li	s1,0
		printk("Pi value calculated by thread #%d: %s\n", i, th_buffer[i]);
    830003f2:	00008a17          	auipc	s4,0x8
    830003f6:	d8ea0a13          	addi	s4,s4,-626 # 83008180 <__clz_tab+0x148>
	for (i = 0; i < THREADS_NUM; i++) {
    830003fa:	49c1                	li	s3,16
		printk("Pi value calculated by thread #%d: %s\n", i, th_buffer[i]);
    830003fc:	864a                	mv	a2,s2
    830003fe:	85a6                	mv	a1,s1
    83000400:	8552                	mv	a0,s4
	for (i = 0; i < THREADS_NUM; i++) {
    83000402:	2485                	addiw	s1,s1,1
		printk("Pi value calculated by thread #%d: %s\n", i, th_buffer[i]);
    83000404:	2c6000ef          	jal	830006ca <printk>
	for (i = 0; i < THREADS_NUM; i++) {
    83000408:	0f190913          	addi	s2,s2,241
    8300040c:	ff3498e3          	bne	s1,s3,830003fc <main+0xb6>
	}

	printk("All %d threads executed by %d cores in %d msec\n", THREADS_NUM,
    83000410:	000f46b7          	lui	a3,0xf4
    83000414:	2406869b          	addiw	a3,a3,576 # f4240 <_image_ram_size+0xd45b4>
    83000418:	02d456bb          	divuw	a3,s0,a3
    8300041c:	00008517          	auipc	a0,0x8
    83000420:	d8c50513          	addi	a0,a0,-628 # 830081a8 <__clz_tab+0x170>
    83000424:	4605                	li	a2,1
    83000426:	45c1                	li	a1,16
    83000428:	2a2000ef          	jal	830006ca <printk>
	       CORES_NUM, nanoseconds_spent / 1000 / 1000);
	return 0;
}
    8300042c:	60e6                	ld	ra,88(sp)
    8300042e:	6446                	ld	s0,80(sp)
    83000430:	64a6                	ld	s1,72(sp)
    83000432:	6906                	ld	s2,64(sp)
    83000434:	79e2                	ld	s3,56(sp)
    83000436:	7a42                	ld	s4,48(sp)
    83000438:	7aa2                	ld	s5,40(sp)
    8300043a:	7b02                	ld	s6,32(sp)
    8300043c:	6be2                	ld	s7,24(sp)
    8300043e:	6c42                	ld	s8,16(sp)
    83000440:	4501                	li	a0,0
    83000442:	6125                	addi	sp,sp,96
    83000444:	8082                	ret
		union { uintptr_t x; k_timeout_t val; } parm0 = { .val = timeout };
		return (int32_t) arch_syscall_invoke1(parm0.x, K_SYSCALL_K_SLEEP);
	}
#endif
	compiler_barrier();
	return z_impl_k_sleep(timeout);
    83000446:	4505                	li	a0,1
    83000448:	315020ef          	jal	83002f5c <z_impl_k_sleep>
    8300044c:	bf41                	j	830003dc <main+0x96>

000000008300044e <chunk_size>:
{
	chunk_unit_t *buf = chunk_buf(h);
	void *cmem = &buf[c];

	if (big_heap(h)) {
		return ((uint32_t *)cmem)[f];
    8300044e:	02059793          	slli	a5,a1,0x20
    83000452:	01d7d593          	srli	a1,a5,0x1d
    83000456:	952e                	add	a0,a0,a1
	return chunk_field(h, c, SIZE_AND_USED) & 1U;
}

static inline chunksz_t chunk_size(struct z_heap *h, chunkid_t c)
{
	return chunk_field(h, c, SIZE_AND_USED) >> 1;
    83000458:	4148                	lw	a0,4(a0)
}
    8300045a:	0015551b          	srliw	a0,a0,0x1
    8300045e:	8082                	ret

0000000083000460 <free_list_add>:
	h->free_bytes += chunksz_to_bytes(h, chunk_size(h, c));
#endif
}

static void free_list_add(struct z_heap *h, chunkid_t c)
{
    83000460:	1101                	addi	sp,sp,-32
    83000462:	e822                	sd	s0,16(sp)
    83000464:	e426                	sd	s1,8(sp)
    83000466:	e04a                	sd	s2,0(sp)
    83000468:	ec06                	sd	ra,24(sp)
    8300046a:	842a                	mv	s0,a0
    8300046c:	84ae                	mv	s1,a1
	chunk_set(h, c, LEFT_SIZE, size);
}

static inline bool solo_free_header(struct z_heap *h, chunkid_t c)
{
	return big_heap(h) && (chunk_size(h, c) == 1U);
    8300046e:	fe1ff0ef          	jal	8300044e <chunk_size>
    83000472:	2501                	sext.w	a0,a0
    83000474:	4905                	li	s2,1
    83000476:	05250263          	beq	a0,s2,830004ba <free_list_add+0x5a>
	return chunksz_in * CHUNK_UNIT - chunk_header_bytes(h);
}

static inline int bucket_idx(struct z_heap *h, chunksz_t sz)
{
	unsigned int usable_sz = sz - min_chunk_size(h) + 1;
    8300047a:	357d                	addiw	a0,a0,-1
	return 31 - __builtin_clz(usable_sz);
    8300047c:	1502                	slli	a0,a0,0x20
    8300047e:	9101                	srli	a0,a0,0x20
    83000480:	cfdff0ef          	jal	8300017c <__clzdi2>
    83000484:	03f00793          	li	a5,63
    83000488:	40a7853b          	subw	a0,a5,a0
	if (b->next == 0U) {
    8300048c:	00251713          	slli	a4,a0,0x2
    83000490:	9722                	add	a4,a4,s0
    83000492:	02049693          	slli	a3,s1,0x20
    83000496:	01d6d793          	srli	a5,a3,0x1d
    8300049a:	4b10                	lw	a2,16(a4)
    8300049c:	07a1                	addi	a5,a5,8
		((uint32_t *)cmem)[f] = val;
    8300049e:	00f405b3          	add	a1,s0,a5
    830004a2:	0791                	addi	a5,a5,4
    830004a4:	97a2                	add	a5,a5,s0
    830004a6:	e205                	bnez	a2,830004c6 <free_list_add+0x66>
		h->avail_buckets |= BIT(bidx);
    830004a8:	4454                	lw	a3,12(s0)
    830004aa:	00a91933          	sll	s2,s2,a0
    830004ae:	0126e6b3          	or	a3,a3,s2
    830004b2:	c454                	sw	a3,12(s0)
		b->next = c;
    830004b4:	cb04                	sw	s1,16(a4)
    830004b6:	c184                	sw	s1,0(a1)
    830004b8:	c384                	sw	s1,0(a5)
	if (!solo_free_header(h, c)) {
		int bidx = bucket_idx(h, chunk_size(h, c));
		free_list_add_bidx(h, c, bidx);
	}
}
    830004ba:	60e2                	ld	ra,24(sp)
    830004bc:	6442                	ld	s0,16(sp)
    830004be:	64a2                	ld	s1,8(sp)
    830004c0:	6902                	ld	s2,0(sp)
    830004c2:	6105                	addi	sp,sp,32
    830004c4:	8082                	ret
		return ((uint32_t *)cmem)[f];
    830004c6:	02061693          	slli	a3,a2,0x20
    830004ca:	01d6d713          	srli	a4,a3,0x1d
    830004ce:	0721                	addi	a4,a4,8
    830004d0:	9722                	add	a4,a4,s0
    830004d2:	4314                	lw	a3,0(a4)
		((uint32_t *)cmem)[f] = val;
    830004d4:	c194                	sw	a3,0(a1)
    830004d6:	c390                	sw	a2,0(a5)
    830004d8:	02069793          	slli	a5,a3,0x20
    830004dc:	01d7d693          	srli	a3,a5,0x1d
    830004e0:	9436                	add	s0,s0,a3
    830004e2:	c444                	sw	s1,12(s0)
    830004e4:	c304                	sw	s1,0(a4)
    830004e6:	bfd1                	j	830004ba <free_list_add+0x5a>

00000000830004e8 <sys_heap_init>:
	}
	return ptr2;
}

void sys_heap_init(struct sys_heap *heap, void *mem, size_t bytes)
{
    830004e8:	7179                	addi	sp,sp,-48
	if (IS_ENABLED(CONFIG_SYS_HEAP_SMALL_ONLY)) {
		/* Must fit in a 15 bit count of HUNK_UNIT */
		__ASSERT(bytes / CHUNK_UNIT <= 0x7fffU, "heap size is too big");
	} else {
		/* Must fit in a 31 bit count of HUNK_UNIT */
		__ASSERT(bytes / CHUNK_UNIT <= 0x7fffffffU, "heap size is too big");
    830004ea:	57fd                	li	a5,-1
{
    830004ec:	f406                	sd	ra,40(sp)
    830004ee:	f022                	sd	s0,32(sp)
    830004f0:	ec26                	sd	s1,24(sp)
    830004f2:	e84a                	sd	s2,16(sp)
    830004f4:	e44e                	sd	s3,8(sp)
    830004f6:	e052                	sd	s4,0(sp)
		__ASSERT(bytes / CHUNK_UNIT <= 0x7fffffffU, "heap size is too big");
    830004f8:	83f9                	srli	a5,a5,0x1e
    830004fa:	04c7f063          	bgeu	a5,a2,8300053a <sys_heap_init+0x52>
    830004fe:	00008597          	auipc	a1,0x8
    83000502:	dda58593          	addi	a1,a1,-550 # 830082d8 <__clz_tab+0x2a0>
    83000506:	1ee00693          	li	a3,494
    8300050a:	00008617          	auipc	a2,0x8
    8300050e:	cce60613          	addi	a2,a2,-818 # 830081d8 <__clz_tab+0x1a0>
    83000512:	00008517          	auipc	a0,0x8
    83000516:	d0650513          	addi	a0,a0,-762 # 83008218 <__clz_tab+0x1e0>
    8300051a:	204000ef          	jal	8300071e <assert_print>
    8300051e:	00008517          	auipc	a0,0x8
    83000522:	dda50513          	addi	a0,a0,-550 # 830082f8 <__clz_tab+0x2c0>
    83000526:	1f8000ef          	jal	8300071e <assert_print>
    8300052a:	1ee00593          	li	a1,494
	}

	/* Reserve the end marker chunk's header */
	__ASSERT(bytes > heap_footer_bytes(bytes), "heap size is too small");
    8300052e:	00008517          	auipc	a0,0x8
    83000532:	caa50513          	addi	a0,a0,-854 # 830081d8 <__clz_tab+0x1a0>
    83000536:	1de000ef          	jal	83000714 <assert_post_action>
    8300053a:	47a1                	li	a5,8
    8300053c:	02c7eb63          	bltu	a5,a2,83000572 <sys_heap_init+0x8a>
    83000540:	00008597          	auipc	a1,0x8
    83000544:	dd058593          	addi	a1,a1,-560 # 83008310 <__clz_tab+0x2d8>
    83000548:	1f200693          	li	a3,498
    8300054c:	00008617          	auipc	a2,0x8
    83000550:	c8c60613          	addi	a2,a2,-884 # 830081d8 <__clz_tab+0x1a0>
    83000554:	00008517          	auipc	a0,0x8
    83000558:	cc450513          	addi	a0,a0,-828 # 83008218 <__clz_tab+0x1e0>
    8300055c:	1c2000ef          	jal	8300071e <assert_print>
    83000560:	00008517          	auipc	a0,0x8
    83000564:	dd850513          	addi	a0,a0,-552 # 83008338 <__clz_tab+0x300>
    83000568:	1b6000ef          	jal	8300071e <assert_print>
    8300056c:	1f200593          	li	a1,498
    83000570:	bf7d                	j	8300052e <sys_heap_init+0x46>
	bytes -= heap_footer_bytes(bytes);
    83000572:	1661                	addi	a2,a2,-8

	/* Round the start up, the end down */
	uintptr_t addr = ROUND_UP(mem, CHUNK_UNIT);
    83000574:	00758913          	addi	s2,a1,7
	uintptr_t end = ROUND_DOWN((uint8_t *)mem + bytes, CHUNK_UNIT);
    83000578:	00c58433          	add	s0,a1,a2
	uintptr_t addr = ROUND_UP(mem, CHUNK_UNIT);
    8300057c:	ff897913          	andi	s2,s2,-8
	uintptr_t end = ROUND_DOWN((uint8_t *)mem + bytes, CHUNK_UNIT);
    83000580:	9861                	andi	s0,s0,-8
	chunksz_t heap_sz = (end - addr) / CHUNK_UNIT;
    83000582:	41240433          	sub	s0,s0,s2
    83000586:	800d                	srli	s0,s0,0x3
    83000588:	0004099b          	sext.w	s3,s0

	CHECK(end > addr);
	__ASSERT(heap_sz > chunksz(sizeof(struct z_heap)), "heap size is too small");
    8300058c:	4789                	li	a5,2
    8300058e:	0337eb63          	bltu	a5,s3,830005c4 <sys_heap_init+0xdc>
    83000592:	00008597          	auipc	a1,0x8
    83000596:	dc658593          	addi	a1,a1,-570 # 83008358 <__clz_tab+0x320>
    8300059a:	1fb00693          	li	a3,507
    8300059e:	00008617          	auipc	a2,0x8
    830005a2:	c3a60613          	addi	a2,a2,-966 # 830081d8 <__clz_tab+0x1a0>
    830005a6:	00008517          	auipc	a0,0x8
    830005aa:	c7250513          	addi	a0,a0,-910 # 83008218 <__clz_tab+0x1e0>
    830005ae:	170000ef          	jal	8300071e <assert_print>
    830005b2:	00008517          	auipc	a0,0x8
    830005b6:	d8650513          	addi	a0,a0,-634 # 83008338 <__clz_tab+0x300>
    830005ba:	164000ef          	jal	8300071e <assert_print>
    830005be:	1fb00593          	li	a1,507
    830005c2:	b7b5                	j	8300052e <sys_heap_init+0x46>

	struct z_heap *h = (struct z_heap *)addr;
	heap->heap = h;
    830005c4:	01253023          	sd	s2,0(a0)
	unsigned int usable_sz = sz - min_chunk_size(h) + 1;
    830005c8:	fff9851b          	addiw	a0,s3,-1
	return 31 - __builtin_clz(usable_sz);
    830005cc:	1502                	slli	a0,a0,0x20
	h->end_chunk = heap_sz;
    830005ce:	01392423          	sw	s3,8(s2)
	h->avail_buckets = 0;
    830005d2:	00092623          	sw	zero,12(s2)
    830005d6:	9101                	srli	a0,a0,0x20
    830005d8:	ba5ff0ef          	jal	8300017c <__clzdi2>
    830005dc:	3501                	addiw	a0,a0,-32
	h->allocated_bytes = 0;
	h->max_allocated_bytes = 0;
#endif

	int nb_buckets = bucket_idx(h, heap_sz) + 1;
	chunksz_t chunk0_size = chunksz(sizeof(struct z_heap) +
    830005de:	02400493          	li	s1,36
    830005e2:	9c89                	subw	s1,s1,a0
    830005e4:	048a                	slli	s1,s1,0x2
	return (bytes + CHUNK_UNIT - 1U) / CHUNK_UNIT;
    830005e6:	049d                	addi	s1,s1,7
    830005e8:	0034da13          	srli	s4,s1,0x3
    830005ec:	848d                	srai	s1,s1,0x3
				     nb_buckets * sizeof(struct z_heap_bucket));

	__ASSERT(chunk0_size + min_chunk_size(h) <= heap_sz, "heap size is too small");
    830005ee:	0024879b          	addiw	a5,s1,2
    830005f2:	02f9fb63          	bgeu	s3,a5,83000628 <sys_heap_init+0x140>
    830005f6:	00008597          	auipc	a1,0x8
    830005fa:	d9258593          	addi	a1,a1,-622 # 83008388 <__clz_tab+0x350>
    830005fe:	20c00693          	li	a3,524
    83000602:	00008617          	auipc	a2,0x8
    83000606:	bd660613          	addi	a2,a2,-1066 # 830081d8 <__clz_tab+0x1a0>
    8300060a:	00008517          	auipc	a0,0x8
    8300060e:	c0e50513          	addi	a0,a0,-1010 # 83008218 <__clz_tab+0x1e0>
    83000612:	10c000ef          	jal	8300071e <assert_print>
    83000616:	00008517          	auipc	a0,0x8
    8300061a:	d2250513          	addi	a0,a0,-734 # 83008338 <__clz_tab+0x300>
    8300061e:	100000ef          	jal	8300071e <assert_print>
    83000622:	20c00593          	li	a1,524
    83000626:	b721                	j	8300052e <sys_heap_init+0x46>

	for (int i = 0; i < nb_buckets; i++) {
		h->buckets[i].next = 0;
    83000628:	02000613          	li	a2,32
    8300062c:	9e09                	subw	a2,a2,a0
    8300062e:	02061793          	slli	a5,a2,0x20
    83000632:	01e7d613          	srli	a2,a5,0x1e
    83000636:	4581                	li	a1,0
    83000638:	01090513          	addi	a0,s2,16
    8300063c:	051030ef          	jal	83003e8c <memset>
	chunk_set(h, c, SIZE_AND_USED, size << 1);
    83000640:	0014979b          	slliw	a5,s1,0x1
			((uint32_t *)cmem)[SIZE_AND_USED] |= 1U;
    83000644:	0017e793          	ori	a5,a5,1
    83000648:	00f92223          	sw	a5,4(s2)
	set_chunk_size(h, 0, chunk0_size);
	set_left_chunk_size(h, 0, 0);
	set_chunk_used(h, 0, true);

	/* chunk containing the free heap */
	set_chunk_size(h, chunk0_size, heap_sz - chunk0_size);
    8300064c:	409989bb          	subw	s3,s3,s1
	void *cmem = &buf[c];
    83000650:	003a1793          	slli	a5,s4,0x3
		((uint32_t *)cmem)[f] = val;
    83000654:	00092023          	sw	zero,0(s2)
    83000658:	97ca                	add	a5,a5,s2
	chunk_set(h, c, SIZE_AND_USED, size << 1);
    8300065a:	0019971b          	slliw	a4,s3,0x1
		((uint32_t *)cmem)[f] = val;
    8300065e:	c3d8                	sw	a4,4(a5)
    83000660:	c384                	sw	s1,0(a5)
	void *cmem = &buf[c];
    83000662:	02041793          	slli	a5,s0,0x20
    83000666:	01d7d413          	srli	s0,a5,0x1d
		((uint32_t *)cmem)[f] = val;
    8300066a:	944a                	add	s0,s0,s2
			((uint32_t *)cmem)[SIZE_AND_USED] |= 1U;
    8300066c:	4785                	li	a5,1
		((uint32_t *)cmem)[f] = val;
    8300066e:	01342023          	sw	s3,0(s0)
			((uint32_t *)cmem)[SIZE_AND_USED] |= 1U;
    83000672:	c05c                	sw	a5,4(s0)
	set_chunk_size(h, heap_sz, 0);
	set_left_chunk_size(h, heap_sz, heap_sz - chunk0_size);
	set_chunk_used(h, heap_sz, true);

	free_list_add(h, chunk0_size);
}
    83000674:	7402                	ld	s0,32(sp)
    83000676:	70a2                	ld	ra,40(sp)
    83000678:	69a2                	ld	s3,8(sp)
    8300067a:	6a02                	ld	s4,0(sp)
	free_list_add(h, chunk0_size);
    8300067c:	85a6                	mv	a1,s1
    8300067e:	854a                	mv	a0,s2
}
    83000680:	64e2                	ld	s1,24(sp)
    83000682:	6942                	ld	s2,16(sp)
    83000684:	6145                	addi	sp,sp,48
	free_list_add(h, chunk0_size);
    83000686:	bbe9                	j	83000460 <free_list_add>

0000000083000688 <arch_printk_char_out>:
{
	ARG_UNUSED(c);

	/* do nothing */
	return 0;
}
    83000688:	4501                	li	a0,0
    8300068a:	8082                	ret

000000008300068c <char_out>:
}

static int char_out(int c, void *ctx_p)
{
	ARG_UNUSED(ctx_p);
	return _char_out(c);
    8300068c:	0001f797          	auipc	a5,0x1f
    83000690:	5f47b783          	ld	a5,1524(a5) # 8301fc80 <_char_out>
    83000694:	8782                	jr	a5

0000000083000696 <__printk_hook_install>:
	_char_out = fn;
    83000696:	0001f797          	auipc	a5,0x1f
    8300069a:	5ea7b523          	sd	a0,1514(a5) # 8301fc80 <_char_out>
}
    8300069e:	8082                	ret

00000000830006a0 <vprintk>:
}

void vprintk(const char *fmt, va_list ap)
{
    830006a0:	7179                	addi	sp,sp,-48
#ifdef CONFIG_PRINTK_SYNC
		k_spinlock_key_t key = k_spin_lock(&lock);
#endif

#ifdef CONFIG_PICOLIBC
		FILE console = FDEV_SETUP_STREAM((int(*)(char, FILE *))char_out,
    830006a2:	e002                	sd	zero,0(sp)
    830006a4:	4789                	li	a5,2
    830006a6:	00f10223          	sb	a5,4(sp)
{
    830006aa:	862e                	mv	a2,a1
		FILE console = FDEV_SETUP_STREAM((int(*)(char, FILE *))char_out,
    830006ac:	00000797          	auipc	a5,0x0
    830006b0:	fe078793          	addi	a5,a5,-32 # 8300068c <char_out>
						 NULL, NULL, _FDEV_SETUP_WRITE);
		(void) vfprintf(&console, fmt, ap);
    830006b4:	85aa                	mv	a1,a0
    830006b6:	850a                	mv	a0,sp
{
    830006b8:	f406                	sd	ra,40(sp)
		FILE console = FDEV_SETUP_STREAM((int(*)(char, FILE *))char_out,
    830006ba:	e802                	sd	zero,16(sp)
    830006bc:	ec02                	sd	zero,24(sp)
    830006be:	e43e                	sd	a5,8(sp)
		(void) vfprintf(&console, fmt, ap);
    830006c0:	2b6030ef          	jal	83003976 <__l_vfprintf>

#ifdef CONFIG_PRINTK_SYNC
		k_spin_unlock(&lock, key);
#endif
	}
}
    830006c4:	70a2                	ld	ra,40(sp)
    830006c6:	6145                	addi	sp,sp,48
    830006c8:	8082                	ret

00000000830006ca <printk>:
 *
 * @param fmt formatted string to output
 */

void printk(const char *fmt, ...)
{
    830006ca:	711d                	addi	sp,sp,-96
    830006cc:	f42e                	sd	a1,40(sp)
	va_list ap;

	va_start(ap, fmt);
    830006ce:	102c                	addi	a1,sp,40
{
    830006d0:	ec06                	sd	ra,24(sp)
    830006d2:	f832                	sd	a2,48(sp)
    830006d4:	fc36                	sd	a3,56(sp)
    830006d6:	e0ba                	sd	a4,64(sp)
    830006d8:	e4be                	sd	a5,72(sp)
    830006da:	e8c2                	sd	a6,80(sp)
    830006dc:	ecc6                	sd	a7,88(sp)
	va_start(ap, fmt);
    830006de:	e42e                	sd	a1,8(sp)

	vprintk(fmt, ap);
    830006e0:	fc1ff0ef          	jal	830006a0 <vprintk>

	va_end(ap);
}
    830006e4:	60e2                	ld	ra,24(sp)
    830006e6:	6125                	addi	sp,sp,96
    830006e8:	8082                	ret

00000000830006ea <z_thread_entry>:
 * This routine does not return, and is marked as such so the compiler won't
 * generate preamble code that is only used by functions that actually return.
 */
FUNC_NORETURN void z_thread_entry(k_thread_entry_t entry,
				 void *p1, void *p2, void *p3)
{
    830006ea:	7179                	addi	sp,sp,-48
    830006ec:	ec26                	sd	s1,24(sp)
    830006ee:	e84a                	sd	s2,16(sp)
    830006f0:	f406                	sd	ra,40(sp)
    830006f2:	f022                	sd	s0,32(sp)
    830006f4:	84aa                	mv	s1,a0
    830006f6:	892e                	mv	s2,a1
    830006f8:	e432                	sd	a2,8(sp)
    830006fa:	e036                	sd	a3,0(sp)
	if (z_syscall_trap()) {
		return (k_tid_t) arch_syscall_invoke0(K_SYSCALL_K_SCHED_CURRENT_THREAD_QUERY);
	}
#endif
	compiler_barrier();
	return z_impl_k_sched_current_thread_query();
    830006fc:	0cd020ef          	jal	83002fc8 <z_impl_k_sched_current_thread_query>

	sys_rand_get((uint8_t *)&stack_guard, sizeof(stack_guard));
	__stack_chk_guard = stack_guard;
	__stack_chk_guard <<= 8;
#endif	/* CONFIG_STACK_CANARIES */
	entry(p1, p2, p3);
    83000700:	6602                	ld	a2,0(sp)
    83000702:	65a2                	ld	a1,8(sp)
	z_tls_current = k_sched_current_thread_query();
    83000704:	00a23023          	sd	a0,0(tp) # 0 <vfscanf>
	entry(p1, p2, p3);
    83000708:	854a                	mv	a0,s2
    8300070a:	9482                	jalr	s1
#ifdef CONFIG_CURRENT_THREAD_USE_TLS

	/* Thread-local cache of current thread ID, set in z_thread_entry() */
	extern __thread k_tid_t z_tls_current;

	return z_tls_current;
    8300070c:	00023503          	ld	a0,0(tp) # 0 <vfscanf>
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_K_THREAD_ABORT);
		return;
	}
#endif
	compiler_barrier();
	z_impl_k_thread_abort(thread);
    83000710:	1fb020ef          	jal	8300310a <z_impl_k_thread_abort>

0000000083000714 <assert_post_action>:
	return a0;
}

static inline uintptr_t arch_syscall_invoke1(uintptr_t arg1, uintptr_t call_id)
{
	register unsigned long a0 __asm__ ("a0") = arg1;
    83000714:	4511                	li	a0,4
	register unsigned long t0 __asm__ ("t0") = call_id;
    83000716:	4281                	li	t0,0

	__asm__ volatile ("ecall"
    83000718:	00000073          	ecall
		k_oops();
	}
#endif

	k_panic();
}
    8300071c:	8082                	ret

000000008300071e <assert_print>:
EXPORT_SYMBOL(assert_post_action);

void assert_print(const char *fmt, ...)
{
    8300071e:	711d                	addi	sp,sp,-96
    83000720:	f42e                	sd	a1,40(sp)
	va_list ap;

	va_start(ap, fmt);
    83000722:	102c                	addi	a1,sp,40
{
    83000724:	ec06                	sd	ra,24(sp)
    83000726:	f832                	sd	a2,48(sp)
    83000728:	fc36                	sd	a3,56(sp)
    8300072a:	e0ba                	sd	a4,64(sp)
    8300072c:	e4be                	sd	a5,72(sp)
    8300072e:	e8c2                	sd	a6,80(sp)
    83000730:	ecc6                	sd	a7,88(sp)
	va_start(ap, fmt);
    83000732:	e42e                	sd	a1,8(sp)

	vprintk(fmt, ap);
    83000734:	f6dff0ef          	jal	830006a0 <vprintk>

	va_end(ap);
}
    83000738:	60e2                	ld	ra,24(sp)
    8300073a:	6125                	addi	sp,sp,96
    8300073c:	8082                	ret

000000008300073e <_ConfigAbsSyms>:
GEN_ABSOLUTE_SYM_KCONFIG(CONFIG_CHECK_INIT_PRIORITIES, 1);
GEN_ABSOLUTE_SYM_KCONFIG(CONFIG_WARN_DEPRECATED, 1);
GEN_ABSOLUTE_SYM_KCONFIG(CONFIG_ENFORCE_ZEPHYR_STDINT, 1);
GEN_ABSOLUTE_SYM_KCONFIG(CONFIG_LEGACY_GENERATED_INCLUDE_PATH, 1);

GEN_ABS_SYM_END
    8300073e:	8082                	ret

0000000083000740 <arch_irq_enable>:

	if (IS_ENABLED(CONFIG_3RD_LEVEL_INTERRUPTS) && (irq & mask3) != 0) {
		return 3;
	}

	if (IS_ENABLED(CONFIG_2ND_LEVEL_INTERRUPTS) && (irq & mask2) != 0) {
    83000740:	67c1                	lui	a5,0x10
    83000742:	f0078793          	addi	a5,a5,-256 # ff00 <__rom_region_size+0x6a18>
    83000746:	8fe9                	and	a5,a5,a0
    83000748:	e799                	bnez	a5,83000756 <arch_irq_enable+0x16>

	/*
	 * CSR mie register is updated using atomic instruction csrrs
	 * (atomic read and set bits in CSR register)
	 */
	xie = csr_read_set(xie, 1 << irq);
    8300074a:	4785                	li	a5,1
    8300074c:	00a7953b          	sllw	a0,a5,a0
    83000750:	10452573          	csrrs	a0,sie,a0
}
    83000754:	8082                	ret
		riscv_plic_irq_enable(irq);
    83000756:	73c0006f          	j	83000e92 <riscv_plic_irq_enable>

000000008300075a <z_riscv_irq_priority_set>:
    8300075a:	67c1                	lui	a5,0x10
    8300075c:	f0078793          	addi	a5,a5,-256 # ff00 <__rom_region_size+0x6a18>
    83000760:	8fe9                	and	a5,a5,a0
    83000762:	c399                	beqz	a5,83000768 <z_riscv_irq_priority_set+0xe>
void z_riscv_irq_priority_set(unsigned int irq, unsigned int prio, uint32_t flags)
{
	unsigned int level = irq_get_level(irq);

	if (level == 2) {
		riscv_plic_set_priority(irq, prio);
    83000764:	7320006f          	j	83000e96 <riscv_plic_set_priority>
	}
}
    83000768:	8082                	ret

000000008300076a <soc_interrupt_init>:
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    8300076a:	100177f3          	csrrci	a5,sstatus,2
__weak void soc_interrupt_init(void)
{
	/* ensure that all interrupts are disabled */
	(void)arch_irq_lock();

	csr_write(xie, 0);
    8300076e:	10405073          	csrwi	sie,0
	csr_write(xip, 0);
    83000772:	14405073          	csrwi	sip,0
}
    83000776:	8082                	ret

0000000083000778 <z_log_minimal_printk>:
#include <zephyr/llext/symbol.h>

#define HEXDUMP_BYTES_IN_LINE 8U

void z_log_minimal_printk(const char *fmt, ...)
{
    83000778:	711d                	addi	sp,sp,-96
    8300077a:	f42e                	sd	a1,40(sp)
	va_list ap;

	va_start(ap, fmt);
    8300077c:	102c                	addi	a1,sp,40
{
    8300077e:	ec06                	sd	ra,24(sp)
    83000780:	f832                	sd	a2,48(sp)
    83000782:	fc36                	sd	a3,56(sp)
    83000784:	e0ba                	sd	a4,64(sp)
    83000786:	e4be                	sd	a5,72(sp)
    83000788:	e8c2                	sd	a6,80(sp)
    8300078a:	ecc6                	sd	a7,88(sp)
	va_start(ap, fmt);
    8300078c:	e42e                	sd	a1,8(sp)
	vprintk(fmt, ap);
    8300078e:	f13ff0ef          	jal	830006a0 <vprintk>
	va_end(ap);
}
    83000792:	60e2                	ld	ra,24(sp)
    83000794:	6125                	addi	sp,sp,96
    83000796:	8082                	ret

0000000083000798 <sys_trace_idle>:

void sys_trace_isr_exit(void) {}

void sys_trace_isr_exit_to_scheduler(void) {}

void sys_trace_idle(void) {}
    83000798:	8082                	ret

000000008300079a <arch_cpu_idle>:
#include <zephyr/irq.h>
#include <zephyr/tracing/tracing.h>

#ifndef CONFIG_ARCH_HAS_CUSTOM_CPU_IDLE
void arch_cpu_idle(void)
{
    8300079a:	1141                	addi	sp,sp,-16
    8300079c:	e406                	sd	ra,8(sp)
	sys_trace_idle();
    8300079e:	ffbff0ef          	jal	83000798 <sys_trace_idle>
	__asm__ volatile("wfi");
    830007a2:	10500073          	wfi
	csr_set(xstatus, (key & XSTATUS_IEN));
    830007a6:	10016073          	csrsi	sstatus,2
	irq_unlock(XSTATUS_IEN);
}
    830007aa:	60a2                	ld	ra,8(sp)
    830007ac:	0141                	addi	sp,sp,16
    830007ae:	8082                	ret

00000000830007b0 <z_riscv_get_sp_before_exc>:
		sp = esf->sp;
	}
#endif

	return sp;
}
    830007b0:	0a050513          	addi	a0,a0,160
    830007b4:	8082                	ret

00000000830007b6 <z_riscv_fatal_error_csf>:
	z_riscv_fatal_error_csf(reason, esf, NULL);
}

FUNC_NORETURN void z_riscv_fatal_error_csf(unsigned int reason, const struct arch_esf *esf,
					   const _callee_saved_t *csf)
{
    830007b6:	1101                	addi	sp,sp,-32
    830007b8:	e822                	sd	s0,16(sp)
    830007ba:	e426                	sd	s1,8(sp)
    830007bc:	e04a                	sd	s2,0(sp)
    830007be:	ec06                	sd	ra,24(sp)
    830007c0:	892a                	mv	s2,a0
    830007c2:	842e                	mv	s0,a1
    830007c4:	84b2                	mv	s1,a2
#ifdef CONFIG_EXCEPTION_DEBUG
	if (esf != NULL) {
    830007c6:	0e058e63          	beqz	a1,830008c2 <z_riscv_fatal_error_csf+0x10c>
		LOG_ERR("     a0: " PR_REG "    t0: " PR_REG, esf->a0, esf->t0);
    830007ca:	6594                	ld	a3,8(a1)
    830007cc:	61b0                	ld	a2,64(a1)
    830007ce:	00008517          	auipc	a0,0x8
    830007d2:	c0250513          	addi	a0,a0,-1022 # 830083d0 <__clz_tab+0x398>
    830007d6:	04500593          	li	a1,69
    830007da:	f9fff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     a1: " PR_REG "    t1: " PR_REG, esf->a1, esf->t1);
    830007de:	6814                	ld	a3,16(s0)
    830007e0:	6430                	ld	a2,72(s0)
    830007e2:	04500593          	li	a1,69
    830007e6:	00008517          	auipc	a0,0x8
    830007ea:	c1250513          	addi	a0,a0,-1006 # 830083f8 <__clz_tab+0x3c0>
    830007ee:	f8bff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     a2: " PR_REG "    t2: " PR_REG, esf->a2, esf->t2);
    830007f2:	6c14                	ld	a3,24(s0)
    830007f4:	6830                	ld	a2,80(s0)
    830007f6:	04500593          	li	a1,69
    830007fa:	00008517          	auipc	a0,0x8
    830007fe:	c2650513          	addi	a0,a0,-986 # 83008420 <__clz_tab+0x3e8>
    83000802:	f77ff0ef          	jal	83000778 <z_log_minimal_printk>
#if defined(CONFIG_RISCV_ISA_RV32E)
		LOG_ERR("     a3: " PR_REG, esf->a3);
		LOG_ERR("     a4: " PR_REG, esf->a4);
		LOG_ERR("     a5: " PR_REG, esf->a5);
#else
		LOG_ERR("     a3: " PR_REG "    t3: " PR_REG, esf->a3, esf->t3);
    83000806:	7014                	ld	a3,32(s0)
    83000808:	6c30                	ld	a2,88(s0)
    8300080a:	04500593          	li	a1,69
    8300080e:	00008517          	auipc	a0,0x8
    83000812:	c3a50513          	addi	a0,a0,-966 # 83008448 <__clz_tab+0x410>
    83000816:	f63ff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     a4: " PR_REG "    t4: " PR_REG, esf->a4, esf->t4);
    8300081a:	7414                	ld	a3,40(s0)
    8300081c:	7030                	ld	a2,96(s0)
    8300081e:	04500593          	li	a1,69
    83000822:	00008517          	auipc	a0,0x8
    83000826:	c4e50513          	addi	a0,a0,-946 # 83008470 <__clz_tab+0x438>
    8300082a:	f4fff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     a5: " PR_REG "    t5: " PR_REG, esf->a5, esf->t5);
    8300082e:	7814                	ld	a3,48(s0)
    83000830:	7430                	ld	a2,104(s0)
    83000832:	04500593          	li	a1,69
    83000836:	00008517          	auipc	a0,0x8
    8300083a:	c6250513          	addi	a0,a0,-926 # 83008498 <__clz_tab+0x460>
    8300083e:	f3bff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     a6: " PR_REG "    t6: " PR_REG, esf->a6, esf->t6);
    83000842:	7c14                	ld	a3,56(s0)
    83000844:	7830                	ld	a2,112(s0)
    83000846:	04500593          	li	a1,69
    8300084a:	00008517          	auipc	a0,0x8
    8300084e:	c7650513          	addi	a0,a0,-906 # 830084c0 <__clz_tab+0x488>
    83000852:	f27ff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     a7: " PR_REG, esf->a7);
    83000856:	7c30                	ld	a2,120(s0)
    83000858:	04500593          	li	a1,69
    8300085c:	00008517          	auipc	a0,0x8
    83000860:	c8c50513          	addi	a0,a0,-884 # 830084e8 <__clz_tab+0x4b0>
    83000864:	f15ff0ef          	jal	83000778 <z_log_minimal_printk>
#endif /* CONFIG_RISCV_ISA_RV32E */
		LOG_ERR("     sp: " PR_REG, z_riscv_get_sp_before_exc(esf));
    83000868:	0a040613          	addi	a2,s0,160
    8300086c:	04500593          	li	a1,69
    83000870:	00008517          	auipc	a0,0x8
    83000874:	c9050513          	addi	a0,a0,-880 # 83008500 <__clz_tab+0x4c8>
    83000878:	f01ff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     ra: " PR_REG, esf->ra);
    8300087c:	6010                	ld	a2,0(s0)
    8300087e:	04500593          	li	a1,69
    83000882:	00008517          	auipc	a0,0x8
    83000886:	c9650513          	addi	a0,a0,-874 # 83008518 <__clz_tab+0x4e0>
    8300088a:	eefff0ef          	jal	83000778 <z_log_minimal_printk>
#if defined(CONFIG_RISCV_S_MODE)
		LOG_ERR("   sepc: " PR_REG, esf->xepc);
    8300088e:	6050                	ld	a2,128(s0)
    83000890:	04500593          	li	a1,69
    83000894:	00008517          	auipc	a0,0x8
    83000898:	c9c50513          	addi	a0,a0,-868 # 83008530 <__clz_tab+0x4f8>
    8300089c:	eddff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("sstatus: " PR_REG, esf->xstatus);
    830008a0:	6450                	ld	a2,136(s0)
    830008a2:	04500593          	li	a1,69
    830008a6:	00008517          	auipc	a0,0x8
    830008aa:	ca250513          	addi	a0,a0,-862 # 83008548 <__clz_tab+0x510>
    830008ae:	ecbff0ef          	jal	83000778 <z_log_minimal_printk>
#else
		LOG_ERR("   mepc: " PR_REG, esf->xepc);
		LOG_ERR("mstatus: " PR_REG, esf->xstatus);
#endif
		LOG_ERR("");
    830008b2:	04500593          	li	a1,69
    830008b6:	00008517          	auipc	a0,0x8
    830008ba:	caa50513          	addi	a0,a0,-854 # 83008560 <__clz_tab+0x528>
    830008be:	ebbff0ef          	jal	83000778 <z_log_minimal_printk>
	}

	if (csf != NULL) {
    830008c2:	c4c9                	beqz	s1,8300094c <z_riscv_fatal_error_csf+0x196>
#if defined(CONFIG_RISCV_ISA_RV32E)
		LOG_ERR("     s0: " PR_REG, csf->s0);
		LOG_ERR("     s1: " PR_REG, csf->s1);
#else
		LOG_ERR("     s0: " PR_REG "    s6: " PR_REG, csf->s0, csf->s6);
    830008c4:	60b4                	ld	a3,64(s1)
    830008c6:	6890                	ld	a2,16(s1)
    830008c8:	04500593          	li	a1,69
    830008cc:	00008517          	auipc	a0,0x8
    830008d0:	c9c50513          	addi	a0,a0,-868 # 83008568 <__clz_tab+0x530>
    830008d4:	ea5ff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     s1: " PR_REG "    s7: " PR_REG, csf->s1, csf->s7);
    830008d8:	64b4                	ld	a3,72(s1)
    830008da:	6c90                	ld	a2,24(s1)
    830008dc:	04500593          	li	a1,69
    830008e0:	00008517          	auipc	a0,0x8
    830008e4:	cb050513          	addi	a0,a0,-848 # 83008590 <__clz_tab+0x558>
    830008e8:	e91ff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     s2: " PR_REG "    s8: " PR_REG, csf->s2, csf->s8);
    830008ec:	68b4                	ld	a3,80(s1)
    830008ee:	7090                	ld	a2,32(s1)
    830008f0:	04500593          	li	a1,69
    830008f4:	00008517          	auipc	a0,0x8
    830008f8:	cc450513          	addi	a0,a0,-828 # 830085b8 <__clz_tab+0x580>
    830008fc:	e7dff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     s3: " PR_REG "    s9: " PR_REG, csf->s3, csf->s9);
    83000900:	6cb4                	ld	a3,88(s1)
    83000902:	7490                	ld	a2,40(s1)
    83000904:	04500593          	li	a1,69
    83000908:	00008517          	auipc	a0,0x8
    8300090c:	cd850513          	addi	a0,a0,-808 # 830085e0 <__clz_tab+0x5a8>
    83000910:	e69ff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     s4: " PR_REG "   s10: " PR_REG, csf->s4, csf->s10);
    83000914:	70b4                	ld	a3,96(s1)
    83000916:	7890                	ld	a2,48(s1)
    83000918:	04500593          	li	a1,69
    8300091c:	00008517          	auipc	a0,0x8
    83000920:	cec50513          	addi	a0,a0,-788 # 83008608 <__clz_tab+0x5d0>
    83000924:	e55ff0ef          	jal	83000778 <z_log_minimal_printk>
		LOG_ERR("     s5: " PR_REG "   s11: " PR_REG, csf->s5, csf->s11);
    83000928:	74b4                	ld	a3,104(s1)
    8300092a:	7c90                	ld	a2,56(s1)
    8300092c:	04500593          	li	a1,69
    83000930:	00008517          	auipc	a0,0x8
    83000934:	d0050513          	addi	a0,a0,-768 # 83008630 <__clz_tab+0x5f8>
    83000938:	e41ff0ef          	jal	83000778 <z_log_minimal_printk>
#endif /* CONFIG_RISCV_ISA_RV32E */
		LOG_ERR("");
    8300093c:	04500593          	li	a1,69
    83000940:	00008517          	auipc	a0,0x8
    83000944:	c2050513          	addi	a0,a0,-992 # 83008560 <__clz_tab+0x528>
    83000948:	e31ff0ef          	jal	83000778 <z_log_minimal_printk>
	}
#endif /* CONFIG_EXCEPTION_DEBUG */

#ifdef CONFIG_EXCEPTION_STACK_TRACE
	z_riscv_unwind_stack(esf, csf);
    8300094c:	85a6                	mv	a1,s1
    8300094e:	8522                	mv	a0,s0
    83000950:	39e000ef          	jal	83000cee <z_riscv_unwind_stack>
#endif /* CONFIG_EXCEPTION_STACK_TRACE */

	z_fatal_error(reason, esf);
    83000954:	85a2                	mv	a1,s0
    83000956:	854a                	mv	a0,s2
    83000958:	48f000ef          	jal	830015e6 <z_fatal_error>

000000008300095c <z_riscv_fatal_error>:
{
    8300095c:	1141                	addi	sp,sp,-16
	z_riscv_fatal_error_csf(reason, esf, NULL);
    8300095e:	4601                	li	a2,0
{
    83000960:	e406                	sd	ra,8(sp)
	z_riscv_fatal_error_csf(reason, esf, NULL);
    83000962:	e55ff0ef          	jal	830007b6 <z_riscv_fatal_error_csf>

0000000083000966 <_Fault>:

	return false;
}

void _Fault(struct arch_esf *esf)
{
    83000966:	7179                	addi	sp,sp,-48
    83000968:	ec26                	sd	s1,24(sp)
    8300096a:	f406                	sd	ra,40(sp)
    8300096c:	f022                	sd	s0,32(sp)
    8300096e:	e84a                	sd	s2,16(sp)
    83000970:	84aa                	mv	s1,a0
	}
#endif /* CONFIG_USERSPACE */

	unsigned long cause;

	cause = csr_read(xcause);
    83000972:	14202473          	csrr	s0,scause

#ifndef CONFIG_SOC_OPENISA_RV32M1
	unsigned long tval;
	tval = csr_read(xtval);
    83000976:	14302973          	csrr	s2,stval
#endif
	cause &= CONFIG_RISCV_MCAUSE_EXCEPTION_MASK;
    8300097a:	00141613          	slli	a2,s0,0x1
    8300097e:	8205                	srli	a2,a2,0x1
	LOG_ERR("");
    83000980:	04500593          	li	a1,69
    83000984:	00008517          	auipc	a0,0x8
    83000988:	bdc50513          	addi	a0,a0,-1060 # 83008560 <__clz_tab+0x528>
	cause &= CONFIG_RISCV_MCAUSE_EXCEPTION_MASK;
    8300098c:	e432                	sd	a2,8(sp)
	LOG_ERR("");
    8300098e:	debff0ef          	jal	83000778 <z_log_minimal_printk>
	switch (cause) {
    83000992:	5785                	li	a5,-31
    83000994:	8385                	srli	a5,a5,0x1
    83000996:	8c7d                	and	s0,s0,a5
    83000998:	6622                	ld	a2,8(sp)
    8300099a:	00008697          	auipc	a3,0x8
    8300099e:	cbe68693          	addi	a3,a3,-834 # 83008658 <__clz_tab+0x620>
    830009a2:	e809                	bnez	s0,830009b4 <_Fault+0x4e>
    830009a4:	00361713          	slli	a4,a2,0x3
    830009a8:	00008797          	auipc	a5,0x8
    830009ac:	e5878793          	addi	a5,a5,-424 # 83008800 <CSWTCH.1972>
    830009b0:	97ba                	add	a5,a5,a4
    830009b2:	6394                	ld	a3,0(a5)

#ifdef CONFIG_RISCV_S_MODE
	LOG_ERR(" scause: %ld, %s", cause, cause_str(cause));
    830009b4:	04500593          	li	a1,69
    830009b8:	00008517          	auipc	a0,0x8
    830009bc:	ca850513          	addi	a0,a0,-856 # 83008660 <__clz_tab+0x628>
    830009c0:	db9ff0ef          	jal	83000778 <z_log_minimal_printk>
	LOG_ERR(" mcause: %ld, %s", cause, cause_str(cause));
#endif

#ifndef CONFIG_SOC_OPENISA_RV32M1
#ifdef CONFIG_RISCV_S_MODE
	LOG_ERR("  stval: %lx", tval);
    830009c4:	04500593          	li	a1,69
    830009c8:	00008517          	auipc	a0,0x8
    830009cc:	cb050513          	addi	a0,a0,-848 # 83008678 <__clz_tab+0x640>
    830009d0:	864a                	mv	a2,s2
    830009d2:	da7ff0ef          	jal	83000778 <z_log_minimal_printk>
		z_riscv_pmp_stackguard_disable();
#endif /* CONFIG_PMP_STACK_GUARD */
		reason = K_ERR_STACK_CHK_FAIL;
	}

	z_riscv_fatal_error(reason, esf);
    830009d6:	85a6                	mv	a1,s1
    830009d8:	4501                	li	a0,0
    830009da:	f83ff0ef          	jal	8300095c <z_riscv_fatal_error>

00000000830009de <z_irq_spurious>:
#endif

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

FUNC_NORETURN void z_irq_spurious(const void *unused)
{
    830009de:	1101                	addi	sp,sp,-32
    830009e0:	ec06                	sd	ra,24(sp)
    830009e2:	e822                	sd	s0,16(sp)
#else
	unsigned long cause;

	ARG_UNUSED(unused);

	cause = csr_read(xcause);
    830009e4:	14202473          	csrr	s0,scause

	cause &= CONFIG_RISCV_MCAUSE_EXCEPTION_MASK;
    830009e8:	0406                	slli	s0,s0,0x1
    830009ea:	8005                	srli	s0,s0,0x1

	printk("Spurious interrupt detected! IRQ: %ld", cause);
    830009ec:	85a2                	mv	a1,s0
    830009ee:	00008517          	auipc	a0,0x8
    830009f2:	e9250513          	addi	a0,a0,-366 # 83008880 <CSWTCH.1972+0x80>
    830009f6:	cd5ff0ef          	jal	830006ca <printk>
	LOG_ERR("Spurious interrupt detected! IRQ: %ld", cause);
    830009fa:	8622                	mv	a2,s0
    830009fc:	04500593          	li	a1,69
    83000a00:	00008517          	auipc	a0,0x8
    83000a04:	ea850513          	addi	a0,a0,-344 # 830088a8 <CSWTCH.1972+0xa8>
    83000a08:	d71ff0ef          	jal	83000778 <z_log_minimal_printk>
#if defined(CONFIG_RISCV_HAS_PLIC)
	if (cause == RISCV_IRQ_EXT) {
    83000a0c:	47a5                	li	a5,9
    83000a0e:	02f41363          	bne	s0,a5,83000a34 <z_irq_spurious+0x56>
		unsigned int save_irq = riscv_plic_get_irq();
    83000a12:	4a6000ef          	jal	83000eb8 <riscv_plic_get_irq>
    83000a16:	0005061b          	sext.w	a2,a0
    83000a1a:	e432                	sd	a2,8(sp)
		const struct device *save_dev = riscv_plic_get_dev();
    83000a1c:	4a6000ef          	jal	83000ec2 <riscv_plic_get_dev>

		LOG_ERR("PLIC interrupt line causing the IRQ: %d (%p)", save_irq, save_dev);
    83000a20:	6622                	ld	a2,8(sp)
		const struct device *save_dev = riscv_plic_get_dev();
    83000a22:	86aa                	mv	a3,a0
		LOG_ERR("PLIC interrupt line causing the IRQ: %d (%p)", save_irq, save_dev);
    83000a24:	04500593          	li	a1,69
    83000a28:	00008517          	auipc	a0,0x8
    83000a2c:	eb050513          	addi	a0,a0,-336 # 830088d8 <CSWTCH.1972+0xd8>
    83000a30:	d49ff0ef          	jal	83000778 <z_log_minimal_printk>
	}
#endif
	z_riscv_fatal_error(K_ERR_SPURIOUS_IRQ, NULL);
    83000a34:	4581                	li	a1,0
    83000a36:	4505                	li	a0,1
    83000a38:	f25ff0ef          	jal	8300095c <z_riscv_fatal_error>

0000000083000a3c <z_prep_c>:
 *
 * This routine prepares for the execution of and runs C code.
 */

void z_prep_c(void)
{
    83000a3c:	1141                	addi	sp,sp,-16
    83000a3e:	e406                	sd	ra,8(sp)
	z_bss_zero();
    83000a40:	62d000ef          	jal	8300186c <z_bss_zero>
	z_data_copy();
#if defined(CONFIG_RISCV_SOC_INTERRUPT_INIT)
	soc_interrupt_init();
    83000a44:	d27ff0ef          	jal	8300076a <soc_interrupt_init>
#endif
	z_cstart();
    83000a48:	6cf000ef          	jal	83001916 <z_cstart>

0000000083000a4c <arch_new_thread>:
#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
	const struct soc_esf soc_esf_init = {SOC_ESF_INIT};
#endif

	/* Initial stack frame for thread */
	stack_init = (struct arch_esf *)Z_STACK_PTR_ALIGN(
    83000a4c:	f6060613          	addi	a2,a2,-160
 * @param ptr Proposed stack pointer address
 * @return Properly aligned stack pointer address
 */
static inline char *z_stack_ptr_align(char *ptr)
{
	return (char *)ROUND_DOWN(ptr, ARCH_STACK_PTR_ALIGN);
    83000a50:	9a41                	andi	a2,a2,-16
				);

	/* Setup the initial stack frame */
	stack_init->a0 = (unsigned long)entry;
	stack_init->a1 = (unsigned long)p1;
	stack_init->a2 = (unsigned long)p2;
    83000a52:	ea3c                	sd	a5,80(a2)
	 *    restored following the MSTATUS value set within the thread stack;
	 * 2) the core will jump to z_thread_entry, as the program
	 *    counter will be restored following the MEPC value set within the
	 *    thread stack.
	 */
	stack_init->xstatus = XSTATUS_DEF_RESTORE;
    83000a54:	12000793          	li	a5,288
    83000a58:	e65c                	sd	a5,136(a2)
		/* User thread */
		stack_init->xepc = (unsigned long)k_thread_user_mode_enter;

	} else {
		/* Supervisor thread */
		stack_init->xepc = (unsigned long)z_thread_entry;
    83000a5a:	00000797          	auipc	a5,0x0
    83000a5e:	c9078793          	addi	a5,a5,-880 # 830006ea <z_thread_entry>
    83000a62:	e25c                	sd	a5,128(a2)
	stack_init->a0 = (unsigned long)entry;
    83000a64:	e234                	sd	a3,64(a2)
	stack_init->a1 = (unsigned long)p1;
    83000a66:	e638                	sd	a4,72(a2)
	stack_init->a3 = (unsigned long)p3;
    83000a68:	05063c23          	sd	a6,88(a2)
#endif

	thread->callee_saved.sp = (unsigned long)stack_init;

	/* where to go when returning from z_riscv_switch() */
	thread->callee_saved.ra = (unsigned long)z_riscv_thread_start;
    83000a6c:	fffff797          	auipc	a5,0xfffff
    83000a70:	6c078793          	addi	a5,a5,1728 # 8300012c <z_riscv_thread_start>
	thread->callee_saved.sp = (unsigned long)stack_init;
    83000a74:	e530                	sd	a2,72(a0)
	thread->callee_saved.ra = (unsigned long)z_riscv_thread_start;
    83000a76:	e93c                	sd	a5,80(a0)

	/* our switch handle is the thread pointer itself */
	thread->switch_handle = thread;
    83000a78:	10a53823          	sd	a0,272(a0)
}
    83000a7c:	8082                	ret

0000000083000a7e <mica_ipi_handler>:
    *table_ptr = (void *)&resource_table;
    *length = sizeof(resource_table);
}

void mica_ipi_handler(const void *unused)
{
    83000a7e:	1141                	addi	sp,sp,-16
    83000a80:	e406                	sd	ra,8(sp)
    83000a82:	e022                	sd	s0,0(sp)
	csr_read_clear(xip, 1 << 1);
    83000a84:	144177f3          	csrrci	a5,sip,2
	uint32_t status;
	struct fw_resource_table *rsc_table;

	rsc_table_get(&rsc, &rsc_size);
	rsc_table = (struct fw_resource_table *)rsc;
	status = rsc_table->reserved[0];
    83000a88:	0000b417          	auipc	s0,0xb
    83000a8c:	04842403          	lw	s0,72(s0) # 8300bad0 <resource_table+0x8>

	printk("xcause 0x%lx status 0x%x\n",csr_read(xcause),status);
    83000a90:	142025f3          	csrr	a1,scause
    83000a94:	00008517          	auipc	a0,0x8
    83000a98:	e7c50513          	addi	a0,a0,-388 # 83008910 <CSWTCH.1972+0x110>
    83000a9c:	8622                	mv	a2,s0
    83000a9e:	c2dff0ef          	jal	830006ca <printk>
	printk("*************************\n");
    83000aa2:	00008517          	auipc	a0,0x8
    83000aa6:	e8e50513          	addi	a0,a0,-370 # 83008930 <CSWTCH.1972+0x130>
    83000aaa:	c21ff0ef          	jal	830006ca <printk>
	printk("* mica recive interrupt *\n");
    83000aae:	00008517          	auipc	a0,0x8
    83000ab2:	ea250513          	addi	a0,a0,-350 # 83008950 <CSWTCH.1972+0x150>
    83000ab6:	c15ff0ef          	jal	830006ca <printk>
	printk("*************************\n");
    83000aba:	00008517          	auipc	a0,0x8
    83000abe:	e7650513          	addi	a0,a0,-394 # 83008930 <CSWTCH.1972+0x130>
    83000ac2:	c09ff0ef          	jal	830006ca <printk>

	if(status == CPU_OFF_FUNCID){
    83000ac6:	840007b7          	lui	a5,0x84000
    83000aca:	0789                	addi	a5,a5,2 # ffffffff84000002 <CONFIG_RISCV_MCAUSE_EXCEPTION_MASK+0x7fffffff84000003>
    83000acc:	02f41163          	bne	s0,a5,83000aee <mica_ipi_handler+0x70>
		sbi_hsm_hart_stop();
	}
}
    83000ad0:	6402                	ld	s0,0(sp)
    83000ad2:	60a2                	ld	ra,8(sp)
			1UL, cpu, 0, 0, 0, 0);
}

static inline sbi_ret_t sbi_hsm_hart_stop(void)
{
	return sbi_ecall(SBI_EXT_HSM, SBI_EXT_HSM_HART_STOP, 0, 0, 0, 0, 0, 0);
    83000ad4:	00485537          	lui	a0,0x485
    83000ad8:	4881                	li	a7,0
    83000ada:	4801                	li	a6,0
    83000adc:	4781                	li	a5,0
    83000ade:	4701                	li	a4,0
    83000ae0:	4681                	li	a3,0
    83000ae2:	4601                	li	a2,0
    83000ae4:	4585                	li	a1,1
    83000ae6:	34d50513          	addi	a0,a0,845 # 48534d <_image_ram_size+0x4656c1>
    83000aea:	0141                	addi	sp,sp,16
    83000aec:	a051                	j	83000b70 <sbi_ecall>
    83000aee:	60a2                	ld	ra,8(sp)
    83000af0:	6402                	ld	s0,0(sp)
    83000af2:	0141                	addi	sp,sp,16
    83000af4:	8082                	ret

0000000083000af6 <mica_init>:

int mica_init(void)
{
    83000af6:	1141                	addi	sp,sp,-16
	IRQ_CONNECT(IPI_MCS, 0, mica_ipi_handler, NULL, 0);
    83000af8:	4601                	li	a2,0
    83000afa:	4581                	li	a1,0
    83000afc:	4505                	li	a0,1
{
    83000afe:	e406                	sd	ra,8(sp)
	IRQ_CONNECT(IPI_MCS, 0, mica_ipi_handler, NULL, 0);
    83000b00:	c5bff0ef          	jal	8300075a <z_riscv_irq_priority_set>
	irq_enable(IPI_MCS);
    83000b04:	4505                	li	a0,1
    83000b06:	c3bff0ef          	jal	83000740 <arch_irq_enable>

	printk("init mica irq\n");
    83000b0a:	00008517          	auipc	a0,0x8
    83000b0e:	e6650513          	addi	a0,a0,-410 # 83008970 <CSWTCH.1972+0x170>
    83000b12:	bb9ff0ef          	jal	830006ca <printk>

	return 0;
}
    83000b16:	60a2                	ld	ra,8(sp)
    83000b18:	4501                	li	a0,0
    83000b1a:	0141                	addi	sp,sp,16
    83000b1c:	8082                	ret

0000000083000b1e <arch_tls_stack_setup>:
#include <kernel_tls.h>
#include <zephyr/app_memory/app_memdomain.h>
#include <zephyr/sys/util.h>

size_t arch_tls_stack_setup(struct k_thread *new_thread, char *stack_ptr)
{
    83000b1e:	7179                	addi	sp,sp,-48
    83000b20:	e84a                	sd	s2,16(sp)
    83000b22:	e052                	sd	s4,0(sp)
 *
 * @return Total size of TLS data/bss areas
 */
static inline size_t z_tls_data_size(void)
{
	return (size_t)(uintptr_t)__tdata_size +
    83000b24:	00000913          	li	s2,0
	       (size_t)(uintptr_t)__tbss_size;
    83000b28:	00800a13          	li	s4,8
    83000b2c:	e44e                	sd	s3,8(sp)
	return (size_t)(uintptr_t)__tdata_size +
    83000b2e:	014909b3          	add	s3,s2,s4
    83000b32:	f022                	sd	s0,32(sp)

	/*
	 * Since we are populating things backwards,
	 * setup the TLS data/bss area first.
	 */
	stack_ptr -= z_tls_data_size();
    83000b34:	41358433          	sub	s0,a1,s3
{
    83000b38:	ec26                	sd	s1,24(sp)
 * @param dest Pointer to destination
 */
static inline void z_tls_copy(char *dest)
{
	/* Copy initialized data (tdata) */
	memcpy(dest, __tdata_start, (size_t)(uintptr_t)__tdata_size);
    83000b3a:	864a                	mv	a2,s2
    83000b3c:	84aa                	mv	s1,a0
    83000b3e:	00007597          	auipc	a1,0x7
    83000b42:	4fa58593          	addi	a1,a1,1274 # 83008038 <__clz_tab>
    83000b46:	8522                	mv	a0,s0
    83000b48:	f406                	sd	ra,40(sp)
    83000b4a:	32c030ef          	jal	83003e76 <memcpy>

	/* Clear BSS data (tbss) */
	dest += (size_t)(uintptr_t)__tdata_size;
	memset(dest, 0, (size_t)(uintptr_t)__tbss_size);
    83000b4e:	01240533          	add	a0,s0,s2
    83000b52:	8652                	mv	a2,s4
    83000b54:	4581                	li	a1,0
    83000b56:	336030ef          	jal	83003e8c <memset>

	/*
	 * Set thread TLS pointer which is used in
	 * context switch to point to TLS area.
	 */
	new_thread->tls = POINTER_TO_UINT(stack_ptr);
    83000b5a:	1284b023          	sd	s0,288(s1)

	return z_tls_data_size();
}
    83000b5e:	70a2                	ld	ra,40(sp)
    83000b60:	7402                	ld	s0,32(sp)
    83000b62:	64e2                	ld	s1,24(sp)
    83000b64:	6942                	ld	s2,16(sp)
    83000b66:	6a02                	ld	s4,0(sp)
    83000b68:	854e                	mv	a0,s3
    83000b6a:	69a2                	ld	s3,8(sp)
    83000b6c:	6145                	addi	sp,sp,48
    83000b6e:	8082                	ret

0000000083000b70 <sbi_ecall>:

sbi_ret_t sbi_ecall(int ext, int fid, unsigned long arg0,
			      unsigned long arg1, unsigned long arg2,
			      unsigned long arg3, unsigned long arg4,
			      unsigned long arg5)
{
    83000b70:	8e2a                	mv	t3,a0
    83000b72:	8eae                	mv	t4,a1
    83000b74:	8532                	mv	a0,a2
    83000b76:	85b6                	mv	a1,a3
    83000b78:	863a                	mv	a2,a4
    83000b7a:	86be                	mv	a3,a5
    83000b7c:	1141                	addi	sp,sp,-16
    83000b7e:	8742                	mv	a4,a6
    83000b80:	87c6                	mv	a5,a7
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83000b82:	10017373          	csrrci	t1,sstatus,2
    83000b86:	2301                	sext.w	t1,t1
	register unsigned long a1 __asm__ ("a1") = (unsigned long) (arg1);
	register unsigned long a2 __asm__ ("a2") = (unsigned long) (arg2);
	register unsigned long a3 __asm__ ("a3") = (unsigned long) (arg3);
	register unsigned long a4 __asm__ ("a4") = (unsigned long) (arg4);
	register unsigned long a5 __asm__ ("a5") = (unsigned long) (arg5);
	register unsigned long a6 __asm__ ("a6") = (unsigned long) (fid);
    83000b88:	8876                	mv	a6,t4
	register unsigned long a7 __asm__ ("a7") = (unsigned long) (ext);
    83000b8a:	88f2                	mv	a7,t3
	__asm__ volatile ("ecall"
    83000b8c:	00000073          	ecall
		: "+r" (a0), "+r" (a1)
		: "r" (a2), "r" (a3), "r" (a4), "r" (a5), "r" (a6), "r" (a7)
		: "memory");
	ret.error = (int32_t) a0;
    83000b90:	2501                	sext.w	a0,a0
    83000b92:	c02a                	sw	a0,0(sp)
	ret.value = a1;
    83000b94:	2581                	sext.w	a1,a1
    83000b96:	c22e                	sw	a1,4(sp)


	if (irq) {
    83000b98:	00030663          	beqz	t1,83000ba4 <sbi_ecall+0x34>
	csr_set(xstatus, (key & XSTATUS_IEN));
    83000b9c:	00237313          	andi	t1,t1,2
    83000ba0:	10032073          	csrs	sstatus,t1
		arch_irq_unlock(irq);
	}


	return ret;
    83000ba4:	6502                	ld	a0,0(sp)
}
    83000ba6:	0141                	addi	sp,sp,16
    83000ba8:	8082                	ret

0000000083000baa <in_stack_bound>:
static bool in_stack_bound(uintptr_t addr, const struct k_thread *const thread,
			   const struct arch_esf *esf)
{
	ARG_UNUSED(esf);

	if (!IS_ALIGNED(addr, sizeof(uintptr_t))) {
    83000baa:	00757793          	andi	a5,a0,7
{
    83000bae:	872a                	mv	a4,a0
	if (!IS_ALIGNED(addr, sizeof(uintptr_t))) {
    83000bb0:	eb99                	bnez	a5,83000bc6 <in_stack_bound+0x1c>
	start = thread->stack_info.start;
    83000bb2:	79fc                	ld	a5,240(a1)
	return (addr >= start) && (addr < end);
    83000bb4:	4501                	li	a0,0
    83000bb6:	00f76963          	bltu	a4,a5,83000bc8 <in_stack_bound+0x1e>
	end = Z_STACK_PTR_ALIGN(thread->stack_info.start + thread->stack_info.size);
    83000bba:	7df4                	ld	a3,248(a1)
    83000bbc:	97b6                	add	a5,a5,a3
    83000bbe:	9bc1                	andi	a5,a5,-16
	return (addr >= start) && (addr < end);
    83000bc0:	00f73533          	sltu	a0,a4,a5
	if ((thread->base.user_options & K_USER) != 0) {
		return in_user_thread_stack_bound(addr, thread);
	}
#endif /* CONFIG_USERSPACE */

	return in_kernel_thread_stack_bound(addr, thread);
    83000bc4:	8082                	ret
		return false;
    83000bc6:	4501                	li	a0,0
}
    83000bc8:	8082                	ret

0000000083000bca <walk_stackframe>:
#else  /* !CONFIG_FRAME_POINTER */
register uintptr_t current_stack_pointer __asm__("sp");
static void walk_stackframe(riscv_stacktrace_cb cb, void *cookie, const struct k_thread *thread,
			    const struct arch_esf *esf, stack_verify_fn vrfy,
			    const _callee_saved_t *csf)
{
    83000bca:	7159                	addi	sp,sp,-112
    83000bcc:	e8ca                	sd	s2,80(sp)
    83000bce:	e0d2                	sd	s4,64(sp)
    83000bd0:	fc56                	sd	s5,56(sp)
    83000bd2:	f85a                	sd	s6,48(sp)
    83000bd4:	f45e                	sd	s7,40(sp)
    83000bd6:	f486                	sd	ra,104(sp)
    83000bd8:	f0a2                	sd	s0,96(sp)
    83000bda:	eca6                	sd	s1,88(sp)
    83000bdc:	e4ce                	sd	s3,72(sp)
    83000bde:	f062                	sd	s8,32(sp)
    83000be0:	ec66                	sd	s9,24(sp)
    83000be2:	e86a                	sd	s10,16(sp)
    83000be4:	e46e                	sd	s11,8(sp)
    83000be6:	8a2a                	mv	s4,a0
    83000be8:	8aae                	mv	s5,a1
    83000bea:	8b32                	mv	s6,a2
    83000bec:	8936                	mv	s2,a3
    83000bee:	8bba                	mv	s7,a4
	uintptr_t sp;
	uintptr_t ra;
	uintptr_t *ksp, last_ksp = 0;

	if (esf != NULL) {
    83000bf0:	c2a9                	beqz	a3,83000c32 <walk_stackframe+0x68>
		/* Unwind the provided exception stack frame */
		sp = z_riscv_get_sp_before_exc(esf);
    83000bf2:	8536                	mv	a0,a3
    83000bf4:	bbdff0ef          	jal	830007b0 <z_riscv_get_sp_before_exc>
		ra = esf->xepc;
    83000bf8:	08093483          	ld	s1,128(s2)
		sp = z_riscv_get_sp_before_exc(esf);
    83000bfc:	842a                	mv	s0,a0
		sp = csf->sp;
		ra = csf->ra;
	}

	ksp = (uintptr_t *)sp;
	for (int i = 0; (i < MAX_STACK_FRAMES) && vrfy((uintptr_t)ksp, thread, esf) &&
    83000bfe:	4981                	li	s3,0
	uintptr_t *ksp, last_ksp = 0;
    83000c00:	4d81                	li	s11,0
	return (addr >= (uintptr_t)&__text_region_start) && (addr < (uintptr_t)&__text_region_end);
    83000c02:	fffffc17          	auipc	s8,0xfffff
    83000c06:	57ac0c13          	addi	s8,s8,1402 # 8300017c <__clzdi2>
    83000c0a:	00003c97          	auipc	s9,0x3
    83000c0e:	292c8c93          	addi	s9,s9,658 # 83003e9c <__rodata_region_start>
	for (int i = 0; (i < MAX_STACK_FRAMES) && vrfy((uintptr_t)ksp, thread, esf) &&
    83000c12:	4d21                	li	s10,8
    83000c14:	864a                	mv	a2,s2
    83000c16:	85da                	mv	a1,s6
    83000c18:	8522                	mv	a0,s0
    83000c1a:	9b82                	jalr	s7
    83000c1c:	c931                	beqz	a0,83000c70 <walk_stackframe+0xa6>
    83000c1e:	048df963          	bgeu	s11,s0,83000c70 <walk_stackframe+0xa6>
	return (addr >= (uintptr_t)&__text_region_start) && (addr < (uintptr_t)&__text_region_end);
    83000c22:	0384fa63          	bgeu	s1,s8,83000c56 <walk_stackframe+0x8c>
			 */
			i++;
		}
		last_ksp = (uintptr_t)ksp;
		/* Unwind to the previous frame */
		ra = ((struct arch_esf *)ksp++)->ra;
    83000c26:	6004                	ld	s1,0(s0)
    83000c28:	00840793          	addi	a5,s0,8
{
    83000c2c:	8da2                	mv	s11,s0
    83000c2e:	843e                	mv	s0,a5
    83000c30:	b7d5                	j	83000c14 <walk_stackframe+0x4a>
	} else if ((csf == NULL) || (csf == &_current->callee_saved)) {
    83000c32:	cb89                	beqz	a5,83000c44 <walk_stackframe+0x7a>
    83000c34:	0000b717          	auipc	a4,0xb
    83000c38:	d0c73703          	ld	a4,-756(a4) # 8300b940 <_kernel+0x10>
    83000c3c:	04870713          	addi	a4,a4,72
    83000c40:	00e79863          	bne	a5,a4,83000c50 <walk_stackframe+0x86>
		sp = current_stack_pointer;
    83000c44:	840a                	mv	s0,sp
		ra = (uintptr_t)walk_stackframe;
    83000c46:	00000497          	auipc	s1,0x0
    83000c4a:	f8448493          	addi	s1,s1,-124 # 83000bca <walk_stackframe>
    83000c4e:	bf45                	j	83000bfe <walk_stackframe+0x34>
		sp = csf->sp;
    83000c50:	6380                	ld	s0,0(a5)
		ra = csf->ra;
    83000c52:	6784                	ld	s1,8(a5)
    83000c54:	b76d                	j	83000bfe <walk_stackframe+0x34>
	return (addr >= (uintptr_t)&__text_region_start) && (addr < (uintptr_t)&__text_region_end);
    83000c56:	fd94f8e3          	bgeu	s1,s9,83000c26 <walk_stackframe+0x5c>
			if (!cb(cookie, ra, POINTER_TO_UINT(ksp))) {
    83000c5a:	8622                	mv	a2,s0
    83000c5c:	85a6                	mv	a1,s1
    83000c5e:	8556                	mv	a0,s5
    83000c60:	9a02                	jalr	s4
    83000c62:	c519                	beqz	a0,83000c70 <walk_stackframe+0xa6>
			i++;
    83000c64:	2985                	addiw	s3,s3,1
		ra = ((struct arch_esf *)ksp++)->ra;
    83000c66:	6004                	ld	s1,0(s0)
    83000c68:	00840793          	addi	a5,s0,8
	for (int i = 0; (i < MAX_STACK_FRAMES) && vrfy((uintptr_t)ksp, thread, esf) &&
    83000c6c:	fda990e3          	bne	s3,s10,83000c2c <walk_stackframe+0x62>
	}
}
    83000c70:	70a6                	ld	ra,104(sp)
    83000c72:	7406                	ld	s0,96(sp)
    83000c74:	64e6                	ld	s1,88(sp)
    83000c76:	6946                	ld	s2,80(sp)
    83000c78:	69a6                	ld	s3,72(sp)
    83000c7a:	6a06                	ld	s4,64(sp)
    83000c7c:	7ae2                	ld	s5,56(sp)
    83000c7e:	7b42                	ld	s6,48(sp)
    83000c80:	7ba2                	ld	s7,40(sp)
    83000c82:	7c02                	ld	s8,32(sp)
    83000c84:	6ce2                	ld	s9,24(sp)
    83000c86:	6d42                	ld	s10,16(sp)
    83000c88:	6da2                	ld	s11,8(sp)
    83000c8a:	6165                	addi	sp,sp,112
    83000c8c:	8082                	ret

0000000083000c8e <print_trace_address>:
#define LOG_STACK_TRACE(idx, sfp, ra, name, offset)                                                \
	LOG_ERR("     %2d: " SFP ": " PR_REG " ra: " PR_REG, idx, sfp, ra)
#endif /* CONFIG_SYMTAB */

static bool print_trace_address(void *arg, unsigned long ra, unsigned long sfp)
{
    83000c8e:	1141                	addi	sp,sp,-16
    83000c90:	e406                	sd	ra,8(sp)
#ifdef CONFIG_SYMTAB
	uint32_t offset = 0;
	const char *name = symtab_find_symbol_name(ra, &offset);
#endif /* CONFIG_SYMTAB */

	LOG_STACK_TRACE((*i)++, sfp, ra, name, offset);
    83000c92:	411c                	lw	a5,0(a0)
{
    83000c94:	86b2                	mv	a3,a2
    83000c96:	872e                	mv	a4,a1
	LOG_STACK_TRACE((*i)++, sfp, ra, name, offset);
    83000c98:	0017861b          	addiw	a2,a5,1
    83000c9c:	c110                	sw	a2,0(a0)
    83000c9e:	04500593          	li	a1,69
    83000ca2:	00008517          	auipc	a0,0x8
    83000ca6:	cde50513          	addi	a0,a0,-802 # 83008980 <CSWTCH.1972+0x180>
    83000caa:	863e                	mv	a2,a5
    83000cac:	acdff0ef          	jal	83000778 <z_log_minimal_printk>

	return true;
}
    83000cb0:	60a2                	ld	ra,8(sp)
    83000cb2:	4505                	li	a0,1
    83000cb4:	0141                	addi	sp,sp,16
    83000cb6:	8082                	ret

0000000083000cb8 <in_fatal_stack_bound>:
	if (!IS_ALIGNED(addr, sizeof(uintptr_t))) {
    83000cb8:	00757693          	andi	a3,a0,7
{
    83000cbc:	87aa                	mv	a5,a0
	if (!IS_ALIGNED(addr, sizeof(uintptr_t))) {
    83000cbe:	e695                	bnez	a3,83000cea <in_fatal_stack_bound+0x32>
	if ((thread == NULL) || arch_is_in_isr()) {
    83000cc0:	c591                	beqz	a1,83000ccc <in_fatal_stack_bound+0x14>
    83000cc2:	0000b717          	auipc	a4,0xb
    83000cc6:	c6e72703          	lw	a4,-914(a4) # 8300b930 <_kernel>
    83000cca:	cf19                	beqz	a4,83000ce8 <in_fatal_stack_bound+0x30>
	start = (uintptr_t)K_KERNEL_STACK_BUFFER(z_interrupt_stacks[cpu_id]);
    83000ccc:	0001d717          	auipc	a4,0x1d
    83000cd0:	36470713          	addi	a4,a4,868 # 8301e030 <z_interrupt_stacks>
	return (addr >= start) && (addr < end);
    83000cd4:	4501                	li	a0,0
    83000cd6:	00e7eb63          	bltu	a5,a4,83000cec <in_fatal_stack_bound+0x34>
    83000cda:	0001e517          	auipc	a0,0x1e
    83000cde:	b5650513          	addi	a0,a0,-1194 # 8301e830 <z_idle_stacks>
    83000ce2:	00a7b533          	sltu	a0,a5,a0
    83000ce6:	8082                	ret
	return in_stack_bound(addr, thread, esf);
    83000ce8:	b5c9                	j	83000baa <in_stack_bound>
		return false;
    83000cea:	4501                	li	a0,0
}
    83000cec:	8082                	ret

0000000083000cee <z_riscv_unwind_stack>:

void z_riscv_unwind_stack(const struct arch_esf *esf, const _callee_saved_t *csf)
{
    83000cee:	7179                	addi	sp,sp,-48
    83000cf0:	e42a                	sd	a0,8(sp)
    83000cf2:	e02e                	sd	a1,0(sp)
	int i = 0;

	LOG_ERR("call trace:");
    83000cf4:	00008517          	auipc	a0,0x8
    83000cf8:	cb450513          	addi	a0,a0,-844 # 830089a8 <CSWTCH.1972+0x1a8>
    83000cfc:	04500593          	li	a1,69
{
    83000d00:	f406                	sd	ra,40(sp)
	int i = 0;
    83000d02:	ce02                	sw	zero,28(sp)
	LOG_ERR("call trace:");
    83000d04:	a75ff0ef          	jal	83000778 <z_log_minimal_printk>
	walk_stackframe(print_trace_address, &i, _current, esf, in_fatal_stack_bound, csf);
    83000d08:	6782                	ld	a5,0(sp)
    83000d0a:	66a2                	ld	a3,8(sp)
    83000d0c:	00000717          	auipc	a4,0x0
    83000d10:	fac70713          	addi	a4,a4,-84 # 83000cb8 <in_fatal_stack_bound>
    83000d14:	0000b617          	auipc	a2,0xb
    83000d18:	c2c63603          	ld	a2,-980(a2) # 8300b940 <_kernel+0x10>
    83000d1c:	086c                	addi	a1,sp,28
    83000d1e:	00000517          	auipc	a0,0x0
    83000d22:	f7050513          	addi	a0,a0,-144 # 83000c8e <print_trace_address>
    83000d26:	ea5ff0ef          	jal	83000bca <walk_stackframe>
	LOG_ERR("");
    83000d2a:	04500593          	li	a1,69
    83000d2e:	00008517          	auipc	a0,0x8
    83000d32:	83250513          	addi	a0,a0,-1998 # 83008560 <__clz_tab+0x528>
    83000d36:	a43ff0ef          	jal	83000778 <z_log_minimal_printk>
}
    83000d3a:	70a2                	ld	ra,40(sp)
    83000d3c:	6145                	addi	sp,sp,48
    83000d3e:	8082                	ret

0000000083000d40 <z_impl_zephyr_fputc>:
#define LIBC_DATA	K_APP_DMEM(z_libc_partition)

static LIBC_DATA int (*_stdout_hook)(int);

int z_impl_zephyr_fputc(int a, FILE *out)
{
    83000d40:	1141                	addi	sp,sp,-16
    83000d42:	e406                	sd	ra,8(sp)
	(*_stdout_hook)(a);
    83000d44:	00008797          	auipc	a5,0x8
    83000d48:	7a47b783          	ld	a5,1956(a5) # 830094e8 <_stdout_hook>
    83000d4c:	9782                	jalr	a5
	return 0;
}
    83000d4e:	60a2                	ld	ra,8(sp)
    83000d50:	4501                	li	a0,0
    83000d52:	0141                	addi	sp,sp,16
    83000d54:	8082                	ret

0000000083000d56 <picolibc_put>:
}
#include <zephyr/syscalls/zephyr_fputc_mrsh.c>
#endif

static int picolibc_put(char a, FILE *f)
{
    83000d56:	1141                	addi	sp,sp,-16
    83000d58:	e406                	sd	ra,8(sp)
		union { uintptr_t x; FILE * val; } parm1 = { .val = stream };
		return (int) arch_syscall_invoke2(parm0.x, parm1.x, K_SYSCALL_ZEPHYR_FPUTC);
	}
#endif
	compiler_barrier();
	return z_impl_zephyr_fputc(c, stream);
    83000d5a:	fe7ff0ef          	jal	83000d40 <z_impl_zephyr_fputc>
	zephyr_fputc(a, f);
	return 0;
}
    83000d5e:	60a2                	ld	ra,8(sp)
    83000d60:	4501                	li	a0,0
    83000d62:	0141                	addi	sp,sp,16
    83000d64:	8082                	ret

0000000083000d66 <__stdout_hook_install>:
STDIO_ALIAS(stderr);

void __stdout_hook_install(int (*hook)(int))
{
	_stdout_hook = hook;
	__stdout.flags |= _FDEV_SETUP_WRITE;
    83000d66:	0001f717          	auipc	a4,0x1f
    83000d6a:	eca70713          	addi	a4,a4,-310 # 8301fc30 <__stdout>
	_stdout_hook = hook;
    83000d6e:	00008797          	auipc	a5,0x8
    83000d72:	76a7bd23          	sd	a0,1914(a5) # 830094e8 <_stdout_hook>
	__stdout.flags |= _FDEV_SETUP_WRITE;
    83000d76:	00474783          	lbu	a5,4(a4)
    83000d7a:	0027e793          	ori	a5,a5,2
    83000d7e:	00f70223          	sb	a5,4(a4)
}
    83000d82:	8082                	ret

0000000083000d84 <malloc_prepare>:
		if (POINTER_TO_UINT(heap_base) + heap_size <= HEAP_BASE + HEAP_SIZE)
			break;
		heap_size >>= 1;
	}
#else
	heap_base = UINT_TO_POINTER(HEAP_BASE);
    83000d84:	81b18593          	addi	a1,gp,-2021 # 8301fc93 <__data_region_end+0x7>
	z_malloc_partition.start = POINTER_TO_UINT(heap_base);
	z_malloc_partition.size = heap_size;
	z_malloc_partition.attr = K_MEM_PARTITION_P_RW_U_RW;
#endif

	sys_heap_init(&z_malloc_heap, heap_base, heap_size);
    83000d88:	00831637          	lui	a2,0x831
	heap_base = UINT_TO_POINTER(HEAP_BASE);
    83000d8c:	99e1                	andi	a1,a1,-8
	sys_heap_init(&z_malloc_heap, heap_base, heap_size);
    83000d8e:	0622                	slli	a2,a2,0x8
{
    83000d90:	1141                	addi	sp,sp,-16
	sys_heap_init(&z_malloc_heap, heap_base, heap_size);
    83000d92:	0000b517          	auipc	a0,0xb
    83000d96:	93650513          	addi	a0,a0,-1738 # 8300b6c8 <z_malloc_heap>
    83000d9a:	8e0d                	sub	a2,a2,a1
{
    83000d9c:	e406                	sd	ra,8(sp)
	sys_heap_init(&z_malloc_heap, heap_base, heap_size);
    83000d9e:	f4aff0ef          	jal	830004e8 <sys_heap_init>

	return 0;
}
    83000da2:	60a2                	ld	ra,8(sp)
    83000da4:	4501                	li	a0,0
    83000da6:	0141                	addi	sp,sp,16
    83000da8:	8082                	ret

0000000083000daa <plic_init>:
 * @param dev PLIC device struct
 *
 * @retval 0 on success.
 */
static int plic_init(const struct device *dev)
{
    83000daa:	1141                	addi	sp,sp,-16
    83000dac:	e406                	sd	ra,8(sp)
	/**for (uint32_t i = 0; i < config->num_irqs; i++) {*/
	/**        sys_write32(0U, prio_addr + (i * sizeof(uint32_t)));*/
	/**}*/

	/* Configure IRQ for PLIC driver */
	config->irq_config_func();
    83000dae:	651c                	ld	a5,8(a0)
    83000db0:	779c                	ld	a5,40(a5)
    83000db2:	9782                	jalr	a5

	return 0;
}
    83000db4:	60a2                	ld	ra,8(sp)
    83000db6:	4501                	li	a0,0
    83000db8:	0141                	addi	sp,sp,16
    83000dba:	8082                	ret

0000000083000dbc <plic_irq_config_func_0>:
	DEVICE_DT_INST_DEFINE(n, &plic_init, NULL,                                                 \
			      PLIC_INTC_DATA(n), &plic_config_##n,                                 \
			      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY,                             \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(PLIC_INTC_DEVICE_INIT)
    83000dbc:	1141                	addi	sp,sp,-16
    83000dbe:	452d                	li	a0,11
    83000dc0:	4601                	li	a2,0
    83000dc2:	4581                	li	a1,0
    83000dc4:	e406                	sd	ra,8(sp)
    83000dc6:	995ff0ef          	jal	8300075a <z_riscv_irq_priority_set>
    83000dca:	60a2                	ld	ra,8(sp)
    83000dcc:	452d                	li	a0,11
    83000dce:	0141                	addi	sp,sp,16
    83000dd0:	971ff06f          	j	83000740 <arch_irq_enable>

0000000083000dd4 <plic_irq_enable_set_state>:
#include "csr.h"

static ALWAYS_INLINE uint32_t arch_proc_id(void)
{
#if CONFIG_RISCV_S_MODE
	return _current_cpu->arch.hartid;
    83000dd4:	0000b797          	auipc	a5,0xb
    83000dd8:	b847b783          	ld	a5,-1148(a5) # 8300b958 <_kernel+0x28>
	return  config->irq_en + get_first_context(hartid) * CONTEXT_ENABLE_SIZE;
    83000ddc:	0087979b          	slliw	a5,a5,0x8
{
	if (IS_ENABLED(CONFIG_3RD_LEVEL_INTERRUPTS)) {
		return ((irq >> CONFIG_1ST_LEVEL_INTERRUPT_BITS) &
			BIT_MASK(CONFIG_2ND_LEVEL_INTERRUPT_BITS)) - 1;
	} else {
		return (irq >> CONFIG_1ST_LEVEL_INTERRUPT_BITS) - 1;
    83000de0:	0085551b          	srliw	a0,a0,0x8
    83000de4:	1782                	slli	a5,a5,0x20
    83000de6:	357d                	addiw	a0,a0,-1
    83000de8:	0c002737          	lui	a4,0xc002
    83000dec:	9381                	srli	a5,a5,0x20
    83000dee:	97ba                	add	a5,a5,a4
	return local_irq >> LOG2(PLIC_REG_SIZE);
    83000df0:	0055571b          	srliw	a4,a0,0x5
	return local_irq_to_reg_index(local_irq) * sizeof(uint32_t);
    83000df4:	0027171b          	slliw	a4,a4,0x2
			get_context_en_addr(dev, cpu_num) + local_irq_to_reg_offset(local_irq);
    83000df8:	1702                	slli	a4,a4,0x20
    83000dfa:	9301                	srli	a4,a4,0x20
		mem_addr_t en_addr =
    83000dfc:	97ba                	add	a5,a5,a4
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83000dfe:	10017673          	csrrci	a2,sstatus,2
	*(volatile uint16_t *)addr = data;
}

static ALWAYS_INLINE uint32_t sys_read32(mem_addr_t addr)
{
	return *(volatile uint32_t *)addr;
    83000e02:	4398                	lw	a4,0(a5)
		WRITE_BIT(en_value, local_irq & PLIC_REG_MASK, enable);
    83000e04:	4685                	li	a3,1
    83000e06:	2601                	sext.w	a2,a2
    83000e08:	2701                	sext.w	a4,a4
    83000e0a:	00a696bb          	sllw	a3,a3,a0
    83000e0e:	c981                	beqz	a1,83000e1e <plic_irq_enable_set_state+0x4a>
    83000e10:	8f55                	or	a4,a4,a3
}

static ALWAYS_INLINE void sys_write32(uint32_t data, mem_addr_t addr)
{
	*(volatile uint32_t *)addr = data;
    83000e12:	c398                	sw	a4,0(a5)
	csr_set(xstatus, (key & XSTATUS_IEN));
    83000e14:	00267793          	andi	a5,a2,2
    83000e18:	1007a073          	csrs	sstatus,a5
}
    83000e1c:	8082                	ret
		WRITE_BIT(en_value, local_irq & PLIC_REG_MASK, enable);
    83000e1e:	fff6c693          	not	a3,a3
    83000e22:	8f75                	and	a4,a4,a3
    83000e24:	b7fd                	j	83000e12 <plic_irq_enable_set_state+0x3e>

0000000083000e26 <plic_irq_handler>:
{
    83000e26:	1101                	addi	sp,sp,-32
    83000e28:	e822                	sd	s0,16(sp)
    83000e2a:	ec06                	sd	ra,24(sp)
    83000e2c:	e426                	sd	s1,8(sp)
    83000e2e:	e04a                	sd	s2,0(sp)
	const struct plic_config *config = dev->config;
    83000e30:	00853903          	ld	s2,8(a0)
    83000e34:	0000b417          	auipc	s0,0xb
    83000e38:	b2443403          	ld	s0,-1244(s0) # 8300b958 <_kernel+0x28>
	return config->reg + get_first_context(arch_proc_id()) * CONTEXT_SIZE +
    83000e3c:	00d4141b          	slliw	s0,s0,0xd
    83000e40:	01093783          	ld	a5,16(s2)
    83000e44:	1402                	slli	s0,s0,0x20
    83000e46:	9001                	srli	s0,s0,0x20
    83000e48:	0411                	addi	s0,s0,4
    83000e4a:	943e                	add	s0,s0,a5
	return *(volatile uint32_t *)addr;
    83000e4c:	4004                	lw	s1,0(s0)
	save_dev = dev;
    83000e4e:	00008797          	auipc	a5,0x8
    83000e52:	6aa7b123          	sd	a0,1698(a5) # 830094f0 <save_dev>
    83000e56:	2481                	sext.w	s1,s1
	save_irq = local_irq;
    83000e58:	00008797          	auipc	a5,0x8
    83000e5c:	6c97a423          	sw	s1,1736(a5) # 83009520 <save_irq>
	if (local_irq == 0U || local_irq >= config->num_irqs) {
    83000e60:	c489                	beqz	s1,83000e6a <plic_irq_handler+0x44>
    83000e62:	02492783          	lw	a5,36(s2)
    83000e66:	00f4e563          	bltu	s1,a5,83000e70 <plic_irq_handler+0x4a>
		z_irq_spurious(NULL);
    83000e6a:	4501                	li	a0,0
    83000e6c:	b73ff0ef          	jal	830009de <z_irq_spurious>
	ite = &config->isr_table[local_irq];
    83000e70:	02049793          	slli	a5,s1,0x20
    83000e74:	01c7d713          	srli	a4,a5,0x1c
    83000e78:	03093783          	ld	a5,48(s2)
    83000e7c:	97ba                	add	a5,a5,a4
	ite->isr(ite->arg);
    83000e7e:	6798                	ld	a4,8(a5)
    83000e80:	6388                	ld	a0,0(a5)
    83000e82:	9702                	jalr	a4
	*(volatile uint32_t *)addr = data;
    83000e84:	c004                	sw	s1,0(s0)
}
    83000e86:	60e2                	ld	ra,24(sp)
    83000e88:	6442                	ld	s0,16(sp)
    83000e8a:	64a2                	ld	s1,8(sp)
    83000e8c:	6902                	ld	s2,0(sp)
    83000e8e:	6105                	addi	sp,sp,32
    83000e90:	8082                	ret

0000000083000e92 <riscv_plic_irq_enable>:
	plic_irq_enable_set_state(irq, true);
    83000e92:	4585                	li	a1,1
    83000e94:	b781                	j	83000dd4 <plic_irq_enable_set_state>

0000000083000e96 <riscv_plic_set_priority>:
	if (priority > config->max_prio) {
    83000e96:	471d                	li	a4,7
    83000e98:	87ae                	mv	a5,a1
    83000e9a:	00b77363          	bgeu	a4,a1,83000ea0 <riscv_plic_set_priority+0xa>
    83000e9e:	479d                	li	a5,7
    83000ea0:	0085551b          	srliw	a0,a0,0x8
    83000ea4:	357d                	addiw	a0,a0,-1
	mem_addr_t prio_addr = config->prio + (local_irq * sizeof(uint32_t));
    83000ea6:	02051713          	slli	a4,a0,0x20
    83000eaa:	01e75513          	srli	a0,a4,0x1e
    83000eae:	0c000737          	lui	a4,0xc000
    83000eb2:	953a                	add	a0,a0,a4
    83000eb4:	c11c                	sw	a5,0(a0)
}
    83000eb6:	8082                	ret

0000000083000eb8 <riscv_plic_get_irq>:
}
    83000eb8:	00008517          	auipc	a0,0x8
    83000ebc:	66852503          	lw	a0,1640(a0) # 83009520 <save_irq>
    83000ec0:	8082                	ret

0000000083000ec2 <riscv_plic_get_dev>:
}
    83000ec2:	00008517          	auipc	a0,0x8
    83000ec6:	62e53503          	ld	a0,1582(a0) # 830094f0 <save_dev>
    83000eca:	8082                	ret

0000000083000ecc <uart_console_init>:
 * @brief Initialize one UART as the console/debug port
 *
 * @return 0 if successful, otherwise failed.
 */
static int uart_console_init(void)
{
    83000ecc:	1141                	addi	sp,sp,-16
    83000ece:	e406                	sd	ra,8(sp)
		union { uintptr_t x; const struct device * val; } parm0 = { .val = dev };
		return (bool) arch_syscall_invoke1(parm0.x, K_SYSCALL_DEVICE_IS_READY);
	}
#endif
	compiler_barrier();
	return z_impl_device_is_ready(dev);
    83000ed0:	00003517          	auipc	a0,0x3
    83000ed4:	04850513          	addi	a0,a0,72 # 83003f18 <__device_dts_ord_24>
    83000ed8:	6ce000ef          	jal	830015a6 <z_impl_device_is_ready>
    83000edc:	57b5                	li	a5,-19
	if (!device_is_ready(uart_console_dev)) {
    83000ede:	cd11                	beqz	a0,83000efa <uart_console_init+0x2e>
	__stdout_hook_install(console_out);
    83000ee0:	00000517          	auipc	a0,0x0
    83000ee4:	02250513          	addi	a0,a0,34 # 83000f02 <console_out>
    83000ee8:	e7fff0ef          	jal	83000d66 <__stdout_hook_install>
	__printk_hook_install(console_out);
    83000eec:	00000517          	auipc	a0,0x0
    83000ef0:	01650513          	addi	a0,a0,22 # 83000f02 <console_out>
    83000ef4:	fa2ff0ef          	jal	83000696 <__printk_hook_install>
		return -ENODEV;
	}

	uart_console_hook_install();

	return 0;
    83000ef8:	4781                	li	a5,0
}
    83000efa:	60a2                	ld	ra,8(sp)
    83000efc:	853e                	mv	a0,a5
    83000efe:	0141                	addi	sp,sp,16
    83000f00:	8082                	ret

0000000083000f02 <console_out>:
{
    83000f02:	1101                	addi	sp,sp,-32
    83000f04:	e822                	sd	s0,16(sp)
    83000f06:	e426                	sd	s1,8(sp)
    83000f08:	ec06                	sd	ra,24(sp)
	if ('\n' == c) {
    83000f0a:	47a9                	li	a5,10
{
    83000f0c:	842a                	mv	s0,a0
    83000f0e:	00003497          	auipc	s1,0x3
    83000f12:	00a48493          	addi	s1,s1,10 # 83003f18 <__device_dts_ord_24>
	if ('\n' == c) {
    83000f16:	00f51763          	bne	a0,a5,83000f24 <console_out+0x22>
					unsigned char out_char)
{
	const struct uart_driver_api *api =
		(const struct uart_driver_api *)dev->api;

	api->poll_out(dev, out_char);
    83000f1a:	689c                	ld	a5,16(s1)
    83000f1c:	45b5                	li	a1,13
    83000f1e:	8526                	mv	a0,s1
    83000f20:	679c                	ld	a5,8(a5)
    83000f22:	9782                	jalr	a5
    83000f24:	689c                	ld	a5,16(s1)
    83000f26:	0ff47593          	zext.b	a1,s0
    83000f2a:	00003517          	auipc	a0,0x3
    83000f2e:	fee50513          	addi	a0,a0,-18 # 83003f18 <__device_dts_ord_24>
    83000f32:	679c                	ld	a5,8(a5)
    83000f34:	9782                	jalr	a5
}
    83000f36:	60e2                	ld	ra,24(sp)
    83000f38:	8522                	mv	a0,s0
    83000f3a:	6442                	ld	s0,16(sp)
    83000f3c:	64a2                	ld	s1,8(sp)
    83000f3e:	6105                	addi	sp,sp,32
    83000f40:	8082                	ret

0000000083000f42 <uart_ns16550_config_get>:

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int uart_ns16550_config_get(const struct device *dev,
				   struct uart_config *cfg)
{
	struct uart_ns16550_dev_data *data = dev->data;
    83000f42:	711c                	ld	a5,32(a0)
	cfg->stop_bits = data->uart_config.stop_bits;
	cfg->data_bits = data->uart_config.data_bits;
	cfg->flow_ctrl = data->uart_config.flow_ctrl;

	return 0;
}
    83000f44:	4501                	li	a0,0
	cfg->baudrate = data->uart_config.baudrate;
    83000f46:	4398                	lw	a4,0(a5)
    83000f48:	c198                	sw	a4,0(a1)
	cfg->parity = data->uart_config.parity;
    83000f4a:	0047c703          	lbu	a4,4(a5)
    83000f4e:	00e58223          	sb	a4,4(a1)
	cfg->stop_bits = data->uart_config.stop_bits;
    83000f52:	0057c703          	lbu	a4,5(a5)
    83000f56:	00e582a3          	sb	a4,5(a1)
	cfg->data_bits = data->uart_config.data_bits;
    83000f5a:	0067c703          	lbu	a4,6(a5)
    83000f5e:	00e58323          	sb	a4,6(a1)
	cfg->flow_ctrl = data->uart_config.flow_ctrl;
    83000f62:	0077c783          	lbu	a5,7(a5)
    83000f66:	00f583a3          	sb	a5,7(a1)
}
    83000f6a:	8082                	ret

0000000083000f6c <uart_ns16550_poll_in>:
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int uart_ns16550_poll_in(const struct device *dev, unsigned char *c)
{
    83000f6c:	7179                	addi	sp,sp,-48
    83000f6e:	e84a                	sd	s2,16(sp)
    83000f70:	e44e                	sd	s3,8(sp)
    83000f72:	f406                	sd	ra,40(sp)
    83000f74:	f022                	sd	s0,32(sp)
    83000f76:	ec26                	sd	s1,24(sp)
	struct uart_ns16550_dev_data *data = dev->data;
	const struct uart_ns16550_device_config * const dev_cfg = dev->config;
	int ret = -1;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
    83000f78:	7100                	ld	s0,32(a0)
{
    83000f7a:	892a                	mv	s2,a0
    83000f7c:	89ae                	mv	s3,a1
	k_spinlock_key_t key = k_spin_lock(&data->lock);
    83000f7e:	0421                	addi	s0,s0,8
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83000f80:	100174f3          	csrrci	s1,sstatus,2

static ALWAYS_INLINE void z_spinlock_validate_pre(struct k_spinlock *l)
{
	ARG_UNUSED(l);
#ifdef CONFIG_SPIN_VALIDATE
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83000f84:	8522                	mv	a0,s0
    83000f86:	3ec020ef          	jal	83003372 <z_spin_lock_valid>
    83000f8a:	e121                	bnez	a0,83000fca <uart_ns16550_poll_in+0x5e>
    83000f8c:	08800693          	li	a3,136
    83000f90:	00008617          	auipc	a2,0x8
    83000f94:	a8860613          	addi	a2,a2,-1400 # 83008a18 <plic_config_0+0x38>
    83000f98:	00008597          	auipc	a1,0x8
    83000f9c:	ab058593          	addi	a1,a1,-1360 # 83008a48 <plic_config_0+0x68>
    83000fa0:	00007517          	auipc	a0,0x7
    83000fa4:	27850513          	addi	a0,a0,632 # 83008218 <__clz_tab+0x1e0>
    83000fa8:	f76ff0ef          	jal	8300071e <assert_print>
    83000fac:	85a2                	mv	a1,s0
    83000fae:	00008517          	auipc	a0,0x8
    83000fb2:	ab250513          	addi	a0,a0,-1358 # 83008a60 <plic_config_0+0x80>
    83000fb6:	f68ff0ef          	jal	8300071e <assert_print>
    83000fba:	08800593          	li	a1,136
static ALWAYS_INLINE void k_spin_unlock(struct k_spinlock *l,
					k_spinlock_key_t key)
{
	ARG_UNUSED(l);
#ifdef CONFIG_SPIN_VALIDATE
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83000fbe:	00008517          	auipc	a0,0x8
    83000fc2:	a5a50513          	addi	a0,a0,-1446 # 83008a18 <plic_config_0+0x38>
    83000fc6:	f4eff0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    83000fca:	8522                	mv	a0,s0
    83000fcc:	3ee020ef          	jal	830033ba <z_spin_lock_set_owner>
		port = DEVICE_MMIO_GET(dev);
    83000fd0:	00893783          	ld	a5,8(s2)

	if ((ns16550_inbyte(dev_cfg, LSR(dev)) & LSR_RXRDY) != 0) {
    83000fd4:	4695                	li	a3,5
    83000fd6:	2481                	sext.w	s1,s1
		port = DEVICE_MMIO_GET(dev);
    83000fd8:	6398                	ld	a4,0(a5)
	if ((ns16550_inbyte(dev_cfg, LSR(dev)) & LSR_RXRDY) != 0) {
    83000fda:	0207c783          	lbu	a5,32(a5)
	int ret = -1;
    83000fde:	597d                	li	s2,-1
	if ((ns16550_inbyte(dev_cfg, LSR(dev)) & LSR_RXRDY) != 0) {
    83000fe0:	02d787bb          	mulw	a5,a5,a3
    83000fe4:	97ba                	add	a5,a5,a4
	return *(volatile uint8_t *)addr;
    83000fe6:	0007c783          	lbu	a5,0(a5)
    83000fea:	8b85                	andi	a5,a5,1
    83000fec:	c791                	beqz	a5,83000ff8 <uart_ns16550_poll_in+0x8c>
    83000fee:	00074783          	lbu	a5,0(a4) # c000000 <CONFIG_TIMER_RANDOM_INITIAL_STATE+0x4a432eb>
		/* got a character */
		*c = ns16550_inbyte(dev_cfg, RDR(dev));
		ret = 0;
    83000ff2:	4901                	li	s2,0
		*c = ns16550_inbyte(dev_cfg, RDR(dev));
    83000ff4:	00f98023          	sb	a5,0(s3)
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83000ff8:	8522                	mv	a0,s0
    83000ffa:	392020ef          	jal	8300338c <z_spin_unlock_valid>
    83000ffe:	e91d                	bnez	a0,83001034 <uart_ns16550_poll_in+0xc8>
    83001000:	13100693          	li	a3,305
    83001004:	00008617          	auipc	a2,0x8
    83001008:	a1460613          	addi	a2,a2,-1516 # 83008a18 <plic_config_0+0x38>
    8300100c:	00008597          	auipc	a1,0x8
    83001010:	a6c58593          	addi	a1,a1,-1428 # 83008a78 <plic_config_0+0x98>
    83001014:	00007517          	auipc	a0,0x7
    83001018:	20450513          	addi	a0,a0,516 # 83008218 <__clz_tab+0x1e0>
    8300101c:	f02ff0ef          	jal	8300071e <assert_print>
    83001020:	85a2                	mv	a1,s0
    83001022:	00008517          	auipc	a0,0x8
    83001026:	a6e50513          	addi	a0,a0,-1426 # 83008a90 <plic_config_0+0xb0>
    8300102a:	ef4ff0ef          	jal	8300071e <assert_print>
    8300102e:	13100593          	li	a1,305
    83001032:	b771                	j	83000fbe <uart_ns16550_poll_in+0x52>
	csr_set(xstatus, (key & XSTATUS_IEN));
    83001034:	8889                	andi	s1,s1,2
    83001036:	1004a073          	csrs	sstatus,s1
	}

	k_spin_unlock(&data->lock, key);

	return ret;
}
    8300103a:	70a2                	ld	ra,40(sp)
    8300103c:	7402                	ld	s0,32(sp)
    8300103e:	64e2                	ld	s1,24(sp)
    83001040:	69a2                	ld	s3,8(sp)
    83001042:	854a                	mv	a0,s2
    83001044:	6942                	ld	s2,16(sp)
    83001046:	6145                	addi	sp,sp,48
    83001048:	8082                	ret

000000008300104a <uart_ns16550_poll_out>:
 * @param dev UART device struct
 * @param c Character to send
 */
static void uart_ns16550_poll_out(const struct device *dev,
					   unsigned char c)
{
    8300104a:	7179                	addi	sp,sp,-48
    8300104c:	e84a                	sd	s2,16(sp)
    8300104e:	e44e                	sd	s3,8(sp)
    83001050:	f406                	sd	ra,40(sp)
    83001052:	f022                	sd	s0,32(sp)
    83001054:	ec26                	sd	s1,24(sp)
	struct uart_ns16550_dev_data *data = dev->data;
	const struct uart_ns16550_device_config * const dev_cfg = dev->config;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
    83001056:	7100                	ld	s0,32(a0)
{
    83001058:	892a                	mv	s2,a0
    8300105a:	89ae                	mv	s3,a1
	k_spinlock_key_t key = k_spin_lock(&data->lock);
    8300105c:	0421                	addi	s0,s0,8
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    8300105e:	100174f3          	csrrci	s1,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83001062:	8522                	mv	a0,s0
    83001064:	30e020ef          	jal	83003372 <z_spin_lock_valid>
    83001068:	e121                	bnez	a0,830010a8 <uart_ns16550_poll_out+0x5e>
    8300106a:	08800693          	li	a3,136
    8300106e:	00008617          	auipc	a2,0x8
    83001072:	9aa60613          	addi	a2,a2,-1622 # 83008a18 <plic_config_0+0x38>
    83001076:	00008597          	auipc	a1,0x8
    8300107a:	9d258593          	addi	a1,a1,-1582 # 83008a48 <plic_config_0+0x68>
    8300107e:	00007517          	auipc	a0,0x7
    83001082:	19a50513          	addi	a0,a0,410 # 83008218 <__clz_tab+0x1e0>
    83001086:	e98ff0ef          	jal	8300071e <assert_print>
    8300108a:	85a2                	mv	a1,s0
    8300108c:	00008517          	auipc	a0,0x8
    83001090:	9d450513          	addi	a0,a0,-1580 # 83008a60 <plic_config_0+0x80>
    83001094:	e8aff0ef          	jal	8300071e <assert_print>
    83001098:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    8300109c:	00008517          	auipc	a0,0x8
    830010a0:	97c50513          	addi	a0,a0,-1668 # 83008a18 <plic_config_0+0x38>
    830010a4:	e70ff0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    830010a8:	8522                	mv	a0,s0
    830010aa:	310020ef          	jal	830033ba <z_spin_lock_set_owner>
		port = DEVICE_MMIO_GET(dev);
    830010ae:	00893783          	ld	a5,8(s2)

	while ((ns16550_inbyte(dev_cfg, LSR(dev)) & LSR_THRE) == 0) {
    830010b2:	4715                	li	a4,5
    830010b4:	2481                	sext.w	s1,s1
		port = DEVICE_MMIO_GET(dev);
    830010b6:	6394                	ld	a3,0(a5)
	while ((ns16550_inbyte(dev_cfg, LSR(dev)) & LSR_THRE) == 0) {
    830010b8:	0207c783          	lbu	a5,32(a5)
    830010bc:	02e787bb          	mulw	a5,a5,a4
    830010c0:	97b6                	add	a5,a5,a3
    830010c2:	0007c703          	lbu	a4,0(a5)
    830010c6:	02077713          	andi	a4,a4,32
    830010ca:	df65                	beqz	a4,830010c2 <uart_ns16550_poll_out+0x78>
	*(volatile uint8_t *)addr = data;
    830010cc:	01368023          	sb	s3,0(a3)
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    830010d0:	8522                	mv	a0,s0
    830010d2:	2ba020ef          	jal	8300338c <z_spin_unlock_valid>
    830010d6:	e91d                	bnez	a0,8300110c <uart_ns16550_poll_out+0xc2>
    830010d8:	13100693          	li	a3,305
    830010dc:	00008617          	auipc	a2,0x8
    830010e0:	93c60613          	addi	a2,a2,-1732 # 83008a18 <plic_config_0+0x38>
    830010e4:	00008597          	auipc	a1,0x8
    830010e8:	99458593          	addi	a1,a1,-1644 # 83008a78 <plic_config_0+0x98>
    830010ec:	00007517          	auipc	a0,0x7
    830010f0:	12c50513          	addi	a0,a0,300 # 83008218 <__clz_tab+0x1e0>
    830010f4:	e2aff0ef          	jal	8300071e <assert_print>
    830010f8:	85a2                	mv	a1,s0
    830010fa:	00008517          	auipc	a0,0x8
    830010fe:	99650513          	addi	a0,a0,-1642 # 83008a90 <plic_config_0+0xb0>
    83001102:	e1cff0ef          	jal	8300071e <assert_print>
    83001106:	13100593          	li	a1,305
    8300110a:	bf49                	j	8300109c <uart_ns16550_poll_out+0x52>
	csr_set(xstatus, (key & XSTATUS_IEN));
    8300110c:	8889                	andi	s1,s1,2
    8300110e:	1004a073          	csrs	sstatus,s1
	}

	ns16550_outbyte(dev_cfg, THR(dev), c);

	k_spin_unlock(&data->lock, key);
}
    83001112:	70a2                	ld	ra,40(sp)
    83001114:	7402                	ld	s0,32(sp)
    83001116:	64e2                	ld	s1,24(sp)
    83001118:	6942                	ld	s2,16(sp)
    8300111a:	69a2                	ld	s3,8(sp)
    8300111c:	6145                	addi	sp,sp,48
    8300111e:	8082                	ret

0000000083001120 <uart_ns16550_err_check>:
 *
 * @return one of UART_ERROR_OVERRUN, UART_ERROR_PARITY, UART_ERROR_FRAMING,
 * UART_BREAK if an error was detected, 0 otherwise.
 */
static int uart_ns16550_err_check(const struct device *dev)
{
    83001120:	1101                	addi	sp,sp,-32
    83001122:	e426                	sd	s1,8(sp)
    83001124:	ec06                	sd	ra,24(sp)
    83001126:	e822                	sd	s0,16(sp)
    83001128:	e04a                	sd	s2,0(sp)
	struct uart_ns16550_dev_data *data = dev->data;
	const struct uart_ns16550_device_config * const dev_cfg = dev->config;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
    8300112a:	7100                	ld	s0,32(a0)
{
    8300112c:	84aa                	mv	s1,a0
	k_spinlock_key_t key = k_spin_lock(&data->lock);
    8300112e:	0421                	addi	s0,s0,8
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83001130:	10017973          	csrrci	s2,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83001134:	8522                	mv	a0,s0
    83001136:	23c020ef          	jal	83003372 <z_spin_lock_valid>
    8300113a:	e121                	bnez	a0,8300117a <uart_ns16550_err_check+0x5a>
    8300113c:	08800693          	li	a3,136
    83001140:	00008617          	auipc	a2,0x8
    83001144:	8d860613          	addi	a2,a2,-1832 # 83008a18 <plic_config_0+0x38>
    83001148:	00008597          	auipc	a1,0x8
    8300114c:	90058593          	addi	a1,a1,-1792 # 83008a48 <plic_config_0+0x68>
    83001150:	00007517          	auipc	a0,0x7
    83001154:	0c850513          	addi	a0,a0,200 # 83008218 <__clz_tab+0x1e0>
    83001158:	dc6ff0ef          	jal	8300071e <assert_print>
    8300115c:	85a2                	mv	a1,s0
    8300115e:	00008517          	auipc	a0,0x8
    83001162:	90250513          	addi	a0,a0,-1790 # 83008a60 <plic_config_0+0x80>
    83001166:	db8ff0ef          	jal	8300071e <assert_print>
    8300116a:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    8300116e:	00008517          	auipc	a0,0x8
    83001172:	8aa50513          	addi	a0,a0,-1878 # 83008a18 <plic_config_0+0x38>
    83001176:	d9eff0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    8300117a:	8522                	mv	a0,s0
    8300117c:	23e020ef          	jal	830033ba <z_spin_lock_set_owner>
		port = DEVICE_MMIO_GET(dev);
    83001180:	6498                	ld	a4,8(s1)
	int check = (ns16550_inbyte(dev_cfg, LSR(dev)) & LSR_EOB_MASK);
    83001182:	4695                	li	a3,5
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83001184:	8522                	mv	a0,s0
    83001186:	02074783          	lbu	a5,32(a4)
    8300118a:	6318                	ld	a4,0(a4)
    8300118c:	02d787bb          	mulw	a5,a5,a3
    83001190:	97ba                	add	a5,a5,a4
	return *(volatile uint8_t *)addr;
    83001192:	0007c483          	lbu	s1,0(a5)
    83001196:	88f9                	andi	s1,s1,30
    83001198:	1f4020ef          	jal	8300338c <z_spin_unlock_valid>
    8300119c:	e91d                	bnez	a0,830011d2 <uart_ns16550_err_check+0xb2>
    8300119e:	13100693          	li	a3,305
    830011a2:	00008617          	auipc	a2,0x8
    830011a6:	87660613          	addi	a2,a2,-1930 # 83008a18 <plic_config_0+0x38>
    830011aa:	00008597          	auipc	a1,0x8
    830011ae:	8ce58593          	addi	a1,a1,-1842 # 83008a78 <plic_config_0+0x98>
    830011b2:	00007517          	auipc	a0,0x7
    830011b6:	06650513          	addi	a0,a0,102 # 83008218 <__clz_tab+0x1e0>
    830011ba:	d64ff0ef          	jal	8300071e <assert_print>
    830011be:	85a2                	mv	a1,s0
    830011c0:	00008517          	auipc	a0,0x8
    830011c4:	8d050513          	addi	a0,a0,-1840 # 83008a90 <plic_config_0+0xb0>
    830011c8:	d56ff0ef          	jal	8300071e <assert_print>
    830011cc:	13100593          	li	a1,305
    830011d0:	bf79                	j	8300116e <uart_ns16550_err_check+0x4e>
    830011d2:	2901                	sext.w	s2,s2
	csr_set(xstatus, (key & XSTATUS_IEN));
    830011d4:	00297913          	andi	s2,s2,2
    830011d8:	10092073          	csrs	sstatus,s2

	k_spin_unlock(&data->lock, key);

	return check >> 1;
}
    830011dc:	60e2                	ld	ra,24(sp)
    830011de:	6442                	ld	s0,16(sp)
    830011e0:	6902                	ld	s2,0(sp)
	return check >> 1;
    830011e2:	4014d51b          	sraiw	a0,s1,0x1
}
    830011e6:	64a2                	ld	s1,8(sp)
    830011e8:	6105                	addi	sp,sp,32
    830011ea:	8082                	ret

00000000830011ec <uart_ns16550_configure>:
{
    830011ec:	715d                	addi	sp,sp,-80
    830011ee:	e0a2                	sd	s0,64(sp)
    830011f0:	fc26                	sd	s1,56(sp)
    830011f2:	f44e                	sd	s3,40(sp)
    830011f4:	e486                	sd	ra,72(sp)
    830011f6:	f84a                	sd	s2,48(sp)
    830011f8:	f052                	sd	s4,32(sp)
    830011fa:	ec56                	sd	s5,24(sp)
	struct uart_ns16550_dev_data * const dev_data = dev->data;
    830011fc:	02053a03          	ld	s4,32(a0)
	const struct uart_ns16550_device_config * const dev_cfg = dev->config;
    83001200:	00853a83          	ld	s5,8(a0)
{
    83001204:	842a                	mv	s0,a0
    83001206:	84ae                	mv	s1,a1
	uint32_t pclk = 0U;
    83001208:	c602                	sw	zero,12(sp)
	k_spinlock_key_t key = k_spin_lock(&dev_data->lock);
    8300120a:	008a0993          	addi	s3,s4,8
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    8300120e:	10017973          	csrrci	s2,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83001212:	854e                	mv	a0,s3
    83001214:	15e020ef          	jal	83003372 <z_spin_lock_valid>
    83001218:	e121                	bnez	a0,83001258 <uart_ns16550_configure+0x6c>
    8300121a:	08800693          	li	a3,136
    8300121e:	00007617          	auipc	a2,0x7
    83001222:	7fa60613          	addi	a2,a2,2042 # 83008a18 <plic_config_0+0x38>
    83001226:	00008597          	auipc	a1,0x8
    8300122a:	82258593          	addi	a1,a1,-2014 # 83008a48 <plic_config_0+0x68>
    8300122e:	00007517          	auipc	a0,0x7
    83001232:	fea50513          	addi	a0,a0,-22 # 83008218 <__clz_tab+0x1e0>
    83001236:	ce8ff0ef          	jal	8300071e <assert_print>
    8300123a:	85ce                	mv	a1,s3
    8300123c:	00008517          	auipc	a0,0x8
    83001240:	82450513          	addi	a0,a0,-2012 # 83008a60 <plic_config_0+0x80>
    83001244:	cdaff0ef          	jal	8300071e <assert_print>
    83001248:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    8300124c:	00007517          	auipc	a0,0x7
    83001250:	7cc50513          	addi	a0,a0,1996 # 83008a18 <plic_config_0+0x38>
    83001254:	cc0ff0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    83001258:	854e                	mv	a0,s3
    8300125a:	160020ef          	jal	830033ba <z_spin_lock_set_owner>
	if (dev_cfg->sys_clk_freq != 0U) {
    8300125e:	008aa783          	lw	a5,8(s5)
    83001262:	2901                	sext.w	s2,s2
    83001264:	12078763          	beqz	a5,83001392 <uart_ns16550_configure+0x1a6>
		pclk = dev_cfg->sys_clk_freq;
    83001268:	c63e                	sw	a5,12(sp)
	set_baud_rate(dev, cfg->baudrate, pclk);
    8300126a:	4094                	lw	a3,0(s1)
	if ((baud_rate != 0U) && (pclk != 0U)) {
    8300126c:	c6b5                	beqz	a3,830012d8 <uart_ns16550_configure+0xec>
	set_baud_rate(dev, cfg->baudrate, pclk);
    8300126e:	4732                	lw	a4,12(sp)
	if ((baud_rate != 0U) && (pclk != 0U)) {
    83001270:	c725                	beqz	a4,830012d8 <uart_ns16550_configure+0xec>
	return ((pclk + (baud_rate << 3)) / baud_rate) >> 4;
    83001272:	0036979b          	slliw	a5,a3,0x3
    83001276:	9fb9                	addw	a5,a5,a4
    83001278:	02d7d7bb          	divuw	a5,a5,a3
	const struct uart_ns16550_device_config * const dev_cfg = dev->config;
    8300127c:	6410                	ld	a2,8(s0)
		lcr_cache = ns16550_inbyte(dev_cfg, LCR(dev));
    8300127e:	458d                	li	a1,3
	struct uart_ns16550_dev_data * const dev_data = dev->data;
    83001280:	7008                	ld	a0,32(s0)
		lcr_cache = ns16550_inbyte(dev_cfg, LCR(dev));
    83001282:	02064703          	lbu	a4,32(a2)
    83001286:	6210                	ld	a2,0(a2)
    83001288:	02b7073b          	mulw	a4,a4,a1
    8300128c:	9732                	add	a4,a4,a2
    8300128e:	00074603          	lbu	a2,0(a4)
    83001292:	0ff67813          	zext.b	a6,a2
		ns16550_outbyte(dev_cfg, LCR(dev), LCR_DLAB | lcr_cache);
    83001296:	08066613          	ori	a2,a2,128
	*(volatile uint8_t *)addr = data;
    8300129a:	00c70023          	sb	a2,0(a4)
		port = DEVICE_MMIO_GET(dev);
    8300129e:	6410                	ld	a2,8(s0)
	return ((pclk + (baud_rate << 3)) / baud_rate) >> 4;
    830012a0:	0047d71b          	srliw	a4,a5,0x4
		ns16550_outbyte(dev_cfg, BRDL(dev), (unsigned char)(divisor & 0xff));
    830012a4:	0ff77713          	zext.b	a4,a4
    830012a8:	6210                	ld	a2,0(a2)
		ns16550_outbyte(dev_cfg, BRDH(dev), (unsigned char)((divisor >> 8) & 0xff));
    830012aa:	00c7d79b          	srliw	a5,a5,0xc
    830012ae:	0ff7f793          	zext.b	a5,a5
    830012b2:	00e60023          	sb	a4,0(a2)
		port = DEVICE_MMIO_GET(dev);
    830012b6:	6410                	ld	a2,8(s0)
		ns16550_outbyte(dev_cfg, BRDH(dev), (unsigned char)((divisor >> 8) & 0xff));
    830012b8:	02064703          	lbu	a4,32(a2)
    830012bc:	6210                	ld	a2,0(a2)
    830012be:	9732                	add	a4,a4,a2
    830012c0:	00f70023          	sb	a5,0(a4)
		port = DEVICE_MMIO_GET(dev);
    830012c4:	6418                	ld	a4,8(s0)
		ns16550_outbyte(dev_cfg, LCR(dev), lcr_cache);
    830012c6:	02074783          	lbu	a5,32(a4)
    830012ca:	6318                	ld	a4,0(a4)
    830012cc:	02b787bb          	mulw	a5,a5,a1
    830012d0:	97ba                	add	a5,a5,a4
    830012d2:	01078023          	sb	a6,0(a5)
		dev_data->uart_config.baudrate = baud_rate;
    830012d6:	c114                	sw	a3,0(a0)
	switch (cfg->data_bits) {
    830012d8:	0064c703          	lbu	a4,6(s1)
    830012dc:	478d                	li	a5,3
    830012de:	f7a00a93          	li	s5,-134
    830012e2:	0ae7ee63          	bltu	a5,a4,8300139e <uart_ns16550_configure+0x1b2>
	switch (cfg->stop_bits) {
    830012e6:	0054c683          	lbu	a3,5(s1)
    830012ea:	4605                	li	a2,1
    830012ec:	10c68363          	beq	a3,a2,830013f2 <uart_ns16550_configure+0x206>
    830012f0:	4611                	li	a2,4
    830012f2:	0af69663          	bne	a3,a5,8300139e <uart_ns16550_configure+0x1b2>
	switch (cfg->parity) {
    830012f6:	0044c783          	lbu	a5,4(s1)
    830012fa:	c799                	beqz	a5,83001308 <uart_ns16550_configure+0x11c>
    830012fc:	4689                	li	a3,2
    830012fe:	f7a00a93          	li	s5,-134
    83001302:	08d79e63          	bne	a5,a3,8300139e <uart_ns16550_configure+0x1b2>
    83001306:	47c1                	li	a5,16
	dev_data->uart_config = *cfg;
    83001308:	4094                	lw	a3,0(s1)
	ns16550_outbyte(dev_cfg, LCR(dev),
    8300130a:	8f51                	or	a4,a4,a2
    8300130c:	8fd9                	or	a5,a5,a4
	dev_data->uart_config = *cfg;
    8300130e:	00da2023          	sw	a3,0(s4)
    83001312:	40d4                	lw	a3,4(s1)
	ns16550_outbyte(dev_cfg, LCR(dev),
    83001314:	460d                	li	a2,3
	dev_data->uart_config = *cfg;
    83001316:	00da2223          	sw	a3,4(s4)
		port = DEVICE_MMIO_GET(dev);
    8300131a:	6414                	ld	a3,8(s0)
	ns16550_outbyte(dev_cfg, LCR(dev),
    8300131c:	0206c703          	lbu	a4,32(a3)
    83001320:	6294                	ld	a3,0(a3)
    83001322:	02c7073b          	mulw	a4,a4,a2
    83001326:	9736                	add	a4,a4,a3
    83001328:	00f70023          	sb	a5,0(a4)
		port = DEVICE_MMIO_GET(dev);
    8300132c:	6418                	ld	a4,8(s0)
	if ((ns16550_inbyte(dev_cfg, IIR(dev)) & IIR_FE) == IIR_FE) {
    8300132e:	0c000693          	li	a3,192
	ns16550_outbyte(dev_cfg, MDC(dev), mdc);
    83001332:	02074783          	lbu	a5,32(a4)
    83001336:	6318                	ld	a4,0(a4)
    83001338:	0027979b          	slliw	a5,a5,0x2
    8300133c:	97ba                	add	a5,a5,a4
    8300133e:	472d                	li	a4,11
    83001340:	00e78023          	sb	a4,0(a5)
		port = DEVICE_MMIO_GET(dev);
    83001344:	6418                	ld	a4,8(s0)
	ns16550_outbyte(dev_cfg, FCR(dev),
    83001346:	02074783          	lbu	a5,32(a4)
    8300134a:	6318                	ld	a4,0(a4)
    8300134c:	0017979b          	slliw	a5,a5,0x1
    83001350:	97ba                	add	a5,a5,a4
    83001352:	f8700713          	li	a4,-121
    83001356:	00e78023          	sb	a4,0(a5)
		port = DEVICE_MMIO_GET(dev);
    8300135a:	6418                	ld	a4,8(s0)
	if ((ns16550_inbyte(dev_cfg, IIR(dev)) & IIR_FE) == IIR_FE) {
    8300135c:	02074783          	lbu	a5,32(a4)
    83001360:	6318                	ld	a4,0(a4)
    83001362:	0017979b          	slliw	a5,a5,0x1
    83001366:	97ba                	add	a5,a5,a4
	return *(volatile uint8_t *)addr;
    83001368:	0007c783          	lbu	a5,0(a5)
		dev_data->fifo_size = 1;
    8300136c:	4705                	li	a4,1
	if ((ns16550_inbyte(dev_cfg, IIR(dev)) & IIR_FE) == IIR_FE) {
    8300136e:	0c07f793          	andi	a5,a5,192
    83001372:	00d79363          	bne	a5,a3,83001378 <uart_ns16550_configure+0x18c>
		dev_data->fifo_size = 16;
    83001376:	4741                	li	a4,16
    83001378:	00ea0823          	sb	a4,16(s4)
	const struct uart_ns16550_device_config *config = dev->config;
    8300137c:	641c                	ld	a5,8(s0)
	int ret = 0;
    8300137e:	4a81                	li	s5,0
		port = DEVICE_MMIO_GET(dev);
    83001380:	6398                	ld	a4,0(a5)
	ns16550_outbyte(dev_cfg, IER(dev), 0x00);
    83001382:	0207c783          	lbu	a5,32(a5)
    83001386:	00074683          	lbu	a3,0(a4)
    8300138a:	97ba                	add	a5,a5,a4
	*(volatile uint8_t *)addr = data;
    8300138c:	00078023          	sb	zero,0(a5)
}
    83001390:	a039                	j	8300139e <uart_ns16550_configure+0x1b2>
		if (!device_is_ready(dev_cfg->clock_dev)) {
    83001392:	010ab503          	ld	a0,16(s5)
    83001396:	210000ef          	jal	830015a6 <z_impl_device_is_ready>
    8300139a:	e121                	bnez	a0,830013da <uart_ns16550_configure+0x1ee>
			ret = -EINVAL;
    8300139c:	5aa9                	li	s5,-22
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    8300139e:	854e                	mv	a0,s3
    830013a0:	7ed010ef          	jal	8300338c <z_spin_unlock_valid>
    830013a4:	e929                	bnez	a0,830013f6 <uart_ns16550_configure+0x20a>
    830013a6:	13100693          	li	a3,305
    830013aa:	00007617          	auipc	a2,0x7
    830013ae:	66e60613          	addi	a2,a2,1646 # 83008a18 <plic_config_0+0x38>
    830013b2:	00007597          	auipc	a1,0x7
    830013b6:	6c658593          	addi	a1,a1,1734 # 83008a78 <plic_config_0+0x98>
    830013ba:	00007517          	auipc	a0,0x7
    830013be:	e5e50513          	addi	a0,a0,-418 # 83008218 <__clz_tab+0x1e0>
    830013c2:	b5cff0ef          	jal	8300071e <assert_print>
    830013c6:	85ce                	mv	a1,s3
    830013c8:	00007517          	auipc	a0,0x7
    830013cc:	6c850513          	addi	a0,a0,1736 # 83008a90 <plic_config_0+0xb0>
    830013d0:	b4eff0ef          	jal	8300071e <assert_print>
    830013d4:	13100593          	li	a1,305
    830013d8:	bd95                	j	8300124c <uart_ns16550_configure+0x60>
		if (clock_control_get_rate(dev_cfg->clock_dev,
    830013da:	010ab503          	ld	a0,16(s5)
					   dev_cfg->clock_subsys,
    830013de:	018ab583          	ld	a1,24(s5)
					 uint32_t *rate)
{
	const struct clock_control_driver_api *api =
		(const struct clock_control_driver_api *)dev->api;

	if (api->get_rate == NULL) {
    830013e2:	691c                	ld	a5,16(a0)
    830013e4:	6f9c                	ld	a5,24(a5)
    830013e6:	dbdd                	beqz	a5,8300139c <uart_ns16550_configure+0x1b0>
		return -ENOSYS;
	}

	return api->get_rate(dev, sys, rate);
    830013e8:	0070                	addi	a2,sp,12
    830013ea:	9782                	jalr	a5
		if (clock_control_get_rate(dev_cfg->clock_dev,
    830013ec:	e6050fe3          	beqz	a0,8300126a <uart_ns16550_configure+0x7e>
    830013f0:	b775                	j	8300139c <uart_ns16550_configure+0x1b0>
		uart_cfg.stop_bits = LCR_1_STB;
    830013f2:	4601                	li	a2,0
    830013f4:	b709                	j	830012f6 <uart_ns16550_configure+0x10a>
	csr_set(xstatus, (key & XSTATUS_IEN));
    830013f6:	00297913          	andi	s2,s2,2
    830013fa:	10092073          	csrs	sstatus,s2
};
    830013fe:	60a6                	ld	ra,72(sp)
    83001400:	6406                	ld	s0,64(sp)
    83001402:	74e2                	ld	s1,56(sp)
    83001404:	7942                	ld	s2,48(sp)
    83001406:	79a2                	ld	s3,40(sp)
    83001408:	7a02                	ld	s4,32(sp)
    8300140a:	8556                	mv	a0,s5
    8300140c:	6ae2                	ld	s5,24(sp)
    8300140e:	6161                	addi	sp,sp,80
    83001410:	8082                	ret

0000000083001412 <uart_ns16550_init>:
	ret = uart_ns16550_configure(dev, &data->uart_config);
    83001412:	710c                	ld	a1,32(a0)
    83001414:	bbe1                	j	830011ec <uart_ns16550_configure>

0000000083001416 <sys_clock_set_timeout>:

/* Weak-linked noop defaults for optional driver interfaces*/

void __weak sys_clock_set_timeout(int32_t ticks, bool idle)
{
}
    83001416:	8082                	ret

0000000083001418 <sbi_set_timer.isra.0>:
static inline sbi_ret_t sbi_set_timer(uint64_t time_value)
    83001418:	862a                	mv	a2,a0
	return sbi_ecall(SBI_EXT_TIME, SBI_EXT_TIME_SET_TIMER, time_value, 0, 0, 0, 0, 0);
    8300141a:	54495537          	lui	a0,0x54495
    8300141e:	4881                	li	a7,0
    83001420:	4801                	li	a6,0
    83001422:	4781                	li	a5,0
    83001424:	4701                	li	a4,0
    83001426:	4681                	li	a3,0
    83001428:	4581                	li	a1,0
    8300142a:	d4550513          	addi	a0,a0,-699 # 54494d45 <CONFIG_TIMER_RANDOM_INITIAL_STATE+0x4ced8030>
    8300142e:	f42ff06f          	j	83000b70 <sbi_ecall>

0000000083001432 <timer_isr>:
{
	sbi_set_timer(next_time);
}

static void timer_isr(const void *arg)
{
    83001432:	1141                	addi	sp,sp,-16
    83001434:	e406                	sd	ra,8(sp)
    83001436:	e022                	sd	s0,0(sp)
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83001438:	10017473          	csrrci	s0,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    8300143c:	00008517          	auipc	a0,0x8
    83001440:	0bc50513          	addi	a0,a0,188 # 830094f8 <lock>
    83001444:	72f010ef          	jal	83003372 <z_spin_lock_valid>
    83001448:	e139                	bnez	a0,8300148e <timer_isr+0x5c>
    8300144a:	08800693          	li	a3,136
    8300144e:	00007617          	auipc	a2,0x7
    83001452:	5ca60613          	addi	a2,a2,1482 # 83008a18 <plic_config_0+0x38>
    83001456:	00007597          	auipc	a1,0x7
    8300145a:	5f258593          	addi	a1,a1,1522 # 83008a48 <plic_config_0+0x68>
    8300145e:	00007517          	auipc	a0,0x7
    83001462:	dba50513          	addi	a0,a0,-582 # 83008218 <__clz_tab+0x1e0>
    83001466:	ab8ff0ef          	jal	8300071e <assert_print>
    8300146a:	00008597          	auipc	a1,0x8
    8300146e:	08e58593          	addi	a1,a1,142 # 830094f8 <lock>
    83001472:	00007517          	auipc	a0,0x7
    83001476:	5ee50513          	addi	a0,a0,1518 # 83008a60 <plic_config_0+0x80>
    8300147a:	aa4ff0ef          	jal	8300071e <assert_print>
    8300147e:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83001482:	00007517          	auipc	a0,0x7
    83001486:	59650513          	addi	a0,a0,1430 # 83008a18 <plic_config_0+0x38>
    8300148a:	a8aff0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    8300148e:	00008517          	auipc	a0,0x8
    83001492:	06a50513          	addi	a0,a0,106 # 830094f8 <lock>
    83001496:	725010ef          	jal	830033ba <z_spin_lock_set_owner>
	__asm__ volatile("rdtime %0" : "=r"(current_time));
    8300149a:	c0102573          	rdtime	a0
	sbi_set_timer(next_time);
    8300149e:	67e1                	lui	a5,0x18
	return current_time;
    830014a0:	1502                	slli	a0,a0,0x20
	sbi_set_timer(next_time);
    830014a2:	6a078793          	addi	a5,a5,1696 # 186a0 <__kernel_ram_size+0x1efc>
	return current_time;
    830014a6:	9101                	srli	a0,a0,0x20
	sbi_set_timer(next_time);
    830014a8:	953e                	add	a0,a0,a5
    830014aa:	f6fff0ef          	jal	83001418 <sbi_set_timer.isra.0>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    830014ae:	00008517          	auipc	a0,0x8
    830014b2:	04a50513          	addi	a0,a0,74 # 830094f8 <lock>
    830014b6:	6d7010ef          	jal	8300338c <z_spin_unlock_valid>
    830014ba:	ed15                	bnez	a0,830014f6 <timer_isr+0xc4>
    830014bc:	13100693          	li	a3,305
    830014c0:	00007617          	auipc	a2,0x7
    830014c4:	55860613          	addi	a2,a2,1368 # 83008a18 <plic_config_0+0x38>
    830014c8:	00007597          	auipc	a1,0x7
    830014cc:	5b058593          	addi	a1,a1,1456 # 83008a78 <plic_config_0+0x98>
    830014d0:	00007517          	auipc	a0,0x7
    830014d4:	d4850513          	addi	a0,a0,-696 # 83008218 <__clz_tab+0x1e0>
    830014d8:	a46ff0ef          	jal	8300071e <assert_print>
    830014dc:	00008597          	auipc	a1,0x8
    830014e0:	01c58593          	addi	a1,a1,28 # 830094f8 <lock>
    830014e4:	00007517          	auipc	a0,0x7
    830014e8:	5ac50513          	addi	a0,a0,1452 # 83008a90 <plic_config_0+0xb0>
    830014ec:	a32ff0ef          	jal	8300071e <assert_print>
    830014f0:	13100593          	li	a1,305
    830014f4:	b779                	j	83001482 <timer_isr+0x50>
    830014f6:	2401                	sext.w	s0,s0
	csr_set(xstatus, (key & XSTATUS_IEN));
    830014f8:	8809                	andi	s0,s0,2
    830014fa:	10042073          	csrs	sstatus,s0

	set_xtimecmp(now + CYC_PER_TICK);

	k_spin_unlock(&lock, key);
	sys_clock_announce(1);
}
    830014fe:	6402                	ld	s0,0(sp)
    83001500:	60a2                	ld	ra,8(sp)
	sys_clock_announce(1);
    83001502:	4505                	li	a0,1
}
    83001504:	0141                	addi	sp,sp,16
	sys_clock_announce(1);
    83001506:	1e80206f          	j	830036ee <sys_clock_announce>

000000008300150a <sys_clock_driver_init>:
	k_spin_unlock(&lock, key);
	return dticks;
}

static int sys_clock_driver_init(void)
{
    8300150a:	1141                	addi	sp,sp,-16

	IRQ_CONNECT(TIMER_IRQN, 0, timer_isr, NULL, 0);
    8300150c:	4601                	li	a2,0
    8300150e:	4581                	li	a1,0
    83001510:	4515                	li	a0,5
{
    83001512:	e406                	sd	ra,8(sp)
	IRQ_CONNECT(TIMER_IRQN, 0, timer_isr, NULL, 0);
    83001514:	a46ff0ef          	jal	8300075a <z_riscv_irq_priority_set>
	__asm__ volatile("rdtime %0" : "=r"(current_time));
    83001518:	c0102573          	rdtime	a0
	sbi_set_timer(next_time);
    8300151c:	67e1                	lui	a5,0x18
	return current_time;
    8300151e:	1502                	slli	a0,a0,0x20
	sbi_set_timer(next_time);
    83001520:	6a078793          	addi	a5,a5,1696 # 186a0 <__kernel_ram_size+0x1efc>
	return current_time;
    83001524:	9101                	srli	a0,a0,0x20
	sbi_set_timer(next_time);
    83001526:	953e                	add	a0,a0,a5
    83001528:	ef1ff0ef          	jal	83001418 <sbi_set_timer.isra.0>

	last_ticks = xtime();
	set_xtimecmp(last_ticks + CYC_PER_TICK);
	irq_enable(TIMER_IRQN);
    8300152c:	4515                	li	a0,5
    8300152e:	a12ff0ef          	jal	83000740 <arch_irq_enable>
    83001532:	10016073          	csrsi	sstatus,2

	arch_irq_unlock(XSTATUS_IEN);

	return 0;
}
    83001536:	60a2                	ld	ra,8(sp)
    83001538:	4501                	li	a0,0
    8300153a:	0141                	addi	sp,sp,16
    8300153c:	8082                	ret

000000008300153e <sys_clock_cycle_get_32>:
	__asm__ volatile("rdtime %0" : "=r"(current_time));
    8300153e:	c0102573          	rdtime	a0
}
    83001542:	2501                	sext.w	a0,a0
    83001544:	8082                	ret

0000000083001546 <sys_clock_elapsed>:
}
    83001546:	4501                	li	a0,0
    83001548:	8082                	ret

000000008300154a <z_device_state_init>:
 * The state object is always zero-initialized, but this may not be
 * sufficient.
 */
void z_device_state_init(void)
{
	STRUCT_SECTION_FOREACH(device, dev) {
    8300154a:	00003797          	auipc	a5,0x3
    8300154e:	9a678793          	addi	a5,a5,-1626 # 83003ef0 <__device_dts_ord_23>
    83001552:	00003717          	auipc	a4,0x3
    83001556:	9ee70713          	addi	a4,a4,-1554 # 83003f40 <_sw_isr_table>
    8300155a:	04f77363          	bgeu	a4,a5,830015a0 <z_device_state_init+0x56>
{
    8300155e:	1141                	addi	sp,sp,-16
	STRUCT_SECTION_FOREACH(device, dev) {
    83001560:	00007597          	auipc	a1,0x7
    83001564:	5e058593          	addi	a1,a1,1504 # 83008b40 <uart_ns16550_driver_api+0x50>
    83001568:	46e1                	li	a3,24
    8300156a:	00007617          	auipc	a2,0x7
    8300156e:	5ae60613          	addi	a2,a2,1454 # 83008b18 <uart_ns16550_driver_api+0x28>
    83001572:	00007517          	auipc	a0,0x7
    83001576:	ca650513          	addi	a0,a0,-858 # 83008218 <__clz_tab+0x1e0>
{
    8300157a:	e406                	sd	ra,8(sp)
	STRUCT_SECTION_FOREACH(device, dev) {
    8300157c:	9a2ff0ef          	jal	8300071e <assert_print>
    83001580:	00007517          	auipc	a0,0x7
    83001584:	5d850513          	addi	a0,a0,1496 # 83008b58 <uart_ns16550_driver_api+0x68>
    83001588:	996ff0ef          	jal	8300071e <assert_print>
    8300158c:	45e1                	li	a1,24
    8300158e:	00007517          	auipc	a0,0x7
    83001592:	58a50513          	addi	a0,a0,1418 # 83008b18 <uart_ns16550_driver_api+0x28>
    83001596:	97eff0ef          	jal	83000714 <assert_post_action>
    8300159a:	02878793          	addi	a5,a5,40
    8300159e:	bf75                	j	8300155a <z_device_state_init+0x10>
    830015a0:	fee7ede3          	bltu	a5,a4,8300159a <z_device_state_init+0x50>
    830015a4:	8082                	ret

00000000830015a6 <z_impl_device_is_ready>:
{
	/*
	 * if an invalid device pointer is passed as argument, this call
	 * reports the `device` as not ready for usage.
	 */
	if (dev == NULL) {
    830015a6:	cd01                	beqz	a0,830015be <z_impl_device_is_ready+0x18>
		return false;
	}

	return dev->state->initialized && (dev->state->init_res == 0U);
    830015a8:	6d18                	ld	a4,24(a0)
    830015aa:	4501                	li	a0,0
    830015ac:	00174783          	lbu	a5,1(a4)
    830015b0:	8b85                	andi	a5,a5,1
    830015b2:	c799                	beqz	a5,830015c0 <z_impl_device_is_ready+0x1a>
    830015b4:	00074503          	lbu	a0,0(a4)
    830015b8:	00153513          	seqz	a0,a0
    830015bc:	8082                	ret
		return false;
    830015be:	4501                	li	a0,0
}
    830015c0:	8082                	ret

00000000830015c2 <arch_system_halt>:
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    830015c2:	100177f3          	csrrci	a5,sstatus,2
	/* TODO: What's the best way to totally halt the system if SMP
	 * is enabled?
	 */

	(void)arch_irq_lock();
	for (;;) {
    830015c6:	a001                	j	830015c6 <arch_system_halt+0x4>

00000000830015c8 <k_sys_fatal_error_handler>:
/* LCOV_EXCL_STOP */

/* LCOV_EXCL_START */
__weak void k_sys_fatal_error_handler(unsigned int reason,
				      const struct arch_esf *esf)
{
    830015c8:	1141                	addi	sp,sp,-16
    830015ca:	e022                	sd	s0,0(sp)
	ARG_UNUSED(esf);

	LOG_PANIC();
	LOG_ERR("Halting system");
    830015cc:	04500593          	li	a1,69
{
    830015d0:	842a                	mv	s0,a0
	LOG_ERR("Halting system");
    830015d2:	00007517          	auipc	a0,0x7
    830015d6:	5a650513          	addi	a0,a0,1446 # 83008b78 <uart_ns16550_driver_api+0x88>
{
    830015da:	e406                	sd	ra,8(sp)
	LOG_ERR("Halting system");
    830015dc:	99cff0ef          	jal	83000778 <z_log_minimal_printk>
	arch_system_halt(reason);
    830015e0:	8522                	mv	a0,s0
    830015e2:	fe1ff0ef          	jal	830015c2 <arch_system_halt>

00000000830015e6 <z_fatal_error>:
	arch_system_halt(reason);
}
/* LCOV_EXCL_STOP */

void z_fatal_error(unsigned int reason, const struct arch_esf *esf)
{
    830015e6:	7179                	addi	sp,sp,-48
    830015e8:	f022                	sd	s0,32(sp)
    830015ea:	e44e                	sd	s3,8(sp)
    830015ec:	f406                	sd	ra,40(sp)
    830015ee:	ec26                	sd	s1,24(sp)
    830015f0:	e84a                	sd	s2,16(sp)
    830015f2:	842a                	mv	s0,a0
    830015f4:	89ae                	mv	s3,a1
    830015f6:	100174f3          	csrrci	s1,sstatus,2
	/* We can't allow this code to be preempted, but don't need to
	 * synchronize between CPUs, so an arch-layer lock is
	 * appropriate.
	 */
	unsigned int key = arch_irq_lock();
	struct k_thread *thread = IS_ENABLED(CONFIG_MULTITHREADING) ?
    830015fa:	0000a717          	auipc	a4,0xa
    830015fe:	33670713          	addi	a4,a4,822 # 8300b930 <_kernel>
    83001602:	4791                	li	a5,4
    83001604:	01073903          	ld	s2,16(a4)
    83001608:	2481                	sext.w	s1,s1
	switch (reason) {
    8300160a:	00007697          	auipc	a3,0x7
    8300160e:	58668693          	addi	a3,a3,1414 # 83008b90 <uart_ns16550_driver_api+0xa0>
    83001612:	00a7ec63          	bltu	a5,a0,8300162a <z_fatal_error+0x44>
    83001616:	02051793          	slli	a5,a0,0x20
    8300161a:	01d7d693          	srli	a3,a5,0x1d
    8300161e:	00007797          	auipc	a5,0x7
    83001622:	62a78793          	addi	a5,a5,1578 # 83008c48 <CSWTCH.279>
    83001626:	97b6                	add	a5,a5,a3
    83001628:	6394                	ld	a3,0(a5)
			_current : NULL;

	/* twister looks for the "ZEPHYR FATAL ERROR" string, don't
	 * change it without also updating twister
	 */
	LOG_ERR(">>> ZEPHYR FATAL ERROR %d: %s on CPU %d", reason,
    8300162a:	02074703          	lbu	a4,32(a4)
    8300162e:	8622                	mv	a2,s0
    83001630:	04500593          	li	a1,69
    83001634:	00007517          	auipc	a0,0x7
    83001638:	56c50513          	addi	a0,a0,1388 # 83008ba0 <uart_ns16550_driver_api+0xb0>
    8300163c:	93cff0ef          	jal	83000778 <z_log_minimal_printk>
	const char *thread_name = (thread != NULL) ? k_thread_name_get(thread) : NULL;
    83001640:	00090a63          	beqz	s2,83001654 <z_fatal_error+0x6e>
    83001644:	854a                	mv	a0,s2
    83001646:	514000ef          	jal	83001b5a <k_thread_name_get>
    8300164a:	86aa                	mv	a3,a0
	if ((thread_name == NULL) || (thread_name[0] == '\0')) {
    8300164c:	c501                	beqz	a0,83001654 <z_fatal_error+0x6e>
    8300164e:	00054783          	lbu	a5,0(a0)
    83001652:	e789                	bnez	a5,8300165c <z_fatal_error+0x76>
		thread_name = "unknown";
    83001654:	00007697          	auipc	a3,0x7
    83001658:	00468693          	addi	a3,a3,4 # 83008658 <__clz_tab+0x620>
		LOG_ERR("Fault during interrupt handling\n");
	}
#endif /* CONFIG_ARCH_HAS_NESTED_EXCEPTION_DETECTION */

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		LOG_ERR("Current thread: %p (%s)", thread, thread_name_get(thread));
    8300165c:	864a                	mv	a2,s2
    8300165e:	04500593          	li	a1,69
    83001662:	00007517          	auipc	a0,0x7
    83001666:	56e50513          	addi	a0,a0,1390 # 83008bd0 <uart_ns16550_driver_api+0xe0>
    8300166a:	90eff0ef          	jal	83000778 <z_log_minimal_printk>
	}

	coredump(reason, esf, thread);

	k_sys_fatal_error_handler(reason, esf);
    8300166e:	85ce                	mv	a1,s3
    83001670:	8522                	mv	a0,s0
    83001672:	f57ff0ef          	jal	830015c8 <k_sys_fatal_error_handler>
	csr_set(xstatus, (key & XSTATUS_IEN));
    83001676:	8889                	andi	s1,s1,2
    83001678:	1004a073          	csrs	sstatus,s1
	arch_irq_unlock(key);

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		k_thread_abort(thread);
	}
}
    8300167c:	7402                	ld	s0,32(sp)
    8300167e:	70a2                	ld	ra,40(sp)
    83001680:	64e2                	ld	s1,24(sp)
    83001682:	69a2                	ld	s3,8(sp)
    83001684:	854a                	mv	a0,s2
    83001686:	6942                	ld	s2,16(sp)
    83001688:	6145                	addi	sp,sp,48
    8300168a:	2810106f          	j	8300310a <z_impl_k_thread_abort>

000000008300168e <do_device_init>:

__pinned_bss
bool z_sys_post_kernel;

static int do_device_init(const struct init_entry *entry)
{
    8300168e:	1141                	addi	sp,sp,-16
    83001690:	e406                	sd	ra,8(sp)
    83001692:	e022                	sd	s0,0(sp)
	const struct device *dev = entry->dev;
	int rc = 0;

	if (entry->init_fn.dev != NULL) {
    83001694:	611c                	ld	a5,0(a0)
	const struct device *dev = entry->dev;
    83001696:	6500                	ld	s0,8(a0)
	if (entry->init_fn.dev != NULL) {
    83001698:	ef89                	bnez	a5,830016b2 <do_device_init+0x24>
	int rc = 0;
    8300169a:	4501                	li	a0,0
			}
			dev->state->init_res = rc;
		}
	}

	dev->state->initialized = true;
    8300169c:	6c18                	ld	a4,24(s0)
    8300169e:	00174783          	lbu	a5,1(a4)
    830016a2:	0017e793          	ori	a5,a5,1
    830016a6:	00f700a3          	sb	a5,1(a4)
		/* Run automatic device runtime enablement */
		(void)pm_device_runtime_auto_enable(dev);
	}

	return rc;
}
    830016aa:	60a2                	ld	ra,8(sp)
    830016ac:	6402                	ld	s0,0(sp)
    830016ae:	0141                	addi	sp,sp,16
    830016b0:	8082                	ret
		rc = entry->init_fn.dev(dev);
    830016b2:	8522                	mv	a0,s0
    830016b4:	9782                	jalr	a5
		if (rc != 0) {
    830016b6:	d175                	beqz	a0,8300169a <do_device_init+0xc>
			if (rc < 0) {
    830016b8:	41f5579b          	sraiw	a5,a0,0x1f
    830016bc:	8d3d                	xor	a0,a0,a5
    830016be:	40f5073b          	subw	a4,a0,a5
			if (rc > UINT8_MAX) {
    830016c2:	0ff00793          	li	a5,255
			if (rc < 0) {
    830016c6:	853a                	mv	a0,a4
			if (rc > UINT8_MAX) {
    830016c8:	00e7d463          	bge	a5,a4,830016d0 <do_device_init+0x42>
    830016cc:	0ff00513          	li	a0,255
			dev->state->init_res = rc;
    830016d0:	6c1c                	ld	a5,24(s0)
    830016d2:	00a78023          	sb	a0,0(a5)
    830016d6:	b7d9                	j	8300169c <do_device_init+0xe>

00000000830016d8 <z_sys_init_run_level>:
		/* End marker */
		__init_end,
	};
	const struct init_entry *entry;

	for (entry = levels[level]; entry < levels[level+1]; entry++) {
    830016d8:	02051693          	slli	a3,a0,0x20
    830016dc:	00007797          	auipc	a5,0x7
    830016e0:	6c478793          	addi	a5,a5,1732 # 83008da0 <levels.0>
    830016e4:	01d6d713          	srli	a4,a3,0x1d
{
    830016e8:	1101                	addi	sp,sp,-32
	for (entry = levels[level]; entry < levels[level+1]; entry++) {
    830016ea:	973e                	add	a4,a4,a5
    830016ec:	2505                	addiw	a0,a0,1
{
    830016ee:	e822                	sd	s0,16(sp)
	for (entry = levels[level]; entry < levels[level+1]; entry++) {
    830016f0:	6300                	ld	s0,0(a4)
    830016f2:	02051713          	slli	a4,a0,0x20
    830016f6:	01d75513          	srli	a0,a4,0x1d
    830016fa:	97aa                	add	a5,a5,a0
{
    830016fc:	e426                	sd	s1,8(sp)
	for (entry = levels[level]; entry < levels[level+1]; entry++) {
    830016fe:	6384                	ld	s1,0(a5)
{
    83001700:	ec06                	sd	ra,24(sp)
	for (entry = levels[level]; entry < levels[level+1]; entry++) {
    83001702:	00946763          	bltu	s0,s1,83001710 <z_sys_init_run_level+0x38>
		} else {
			result = entry->init_fn.sys();
		}
		sys_trace_sys_init_exit(entry, level, result);
	}
}
    83001706:	60e2                	ld	ra,24(sp)
    83001708:	6442                	ld	s0,16(sp)
    8300170a:	64a2                	ld	s1,8(sp)
    8300170c:	6105                	addi	sp,sp,32
    8300170e:	8082                	ret
		if (dev != NULL) {
    83001710:	641c                	ld	a5,8(s0)
    83001712:	c791                	beqz	a5,8300171e <z_sys_init_run_level+0x46>
			result = do_device_init(entry);
    83001714:	8522                	mv	a0,s0
    83001716:	f79ff0ef          	jal	8300168e <do_device_init>
	for (entry = levels[level]; entry < levels[level+1]; entry++) {
    8300171a:	0441                	addi	s0,s0,16
    8300171c:	b7dd                	j	83001702 <z_sys_init_run_level+0x2a>
			result = entry->init_fn.sys();
    8300171e:	601c                	ld	a5,0(s0)
    83001720:	9782                	jalr	a5
    83001722:	bfe5                	j	8300171a <z_sys_init_run_level+0x42>

0000000083001724 <bg_thread_main>:
 * This routine completes kernel initialization by invoking the remaining
 * init functions, then invokes application's main() routine.
 */
__boot_func
static void bg_thread_main(void *unused1, void *unused2, void *unused3)
{
    83001724:	7139                	addi	sp,sp,-64
	 * may perform memory management tasks (except for
	 * k_mem_map_phys_bare() which is allowed at any time)
	 */
	z_mem_manage_init();
#endif /* CONFIG_MMU */
	z_sys_post_kernel = true;
    83001726:	4785                	li	a5,1

	z_sys_init_run_level(INIT_LEVEL_POST_KERNEL);
    83001728:	450d                	li	a0,3
{
    8300172a:	fc06                	sd	ra,56(sp)
	z_sys_post_kernel = true;
    8300172c:	00008717          	auipc	a4,0x8
    83001730:	e0f70223          	sb	a5,-508(a4) # 83009530 <z_sys_post_kernel>
{
    83001734:	f822                	sd	s0,48(sp)
    83001736:	f426                	sd	s1,40(sp)
    83001738:	f04a                	sd	s2,32(sp)
    8300173a:	ec4e                	sd	s3,24(sp)
	z_sys_init_run_level(INIT_LEVEL_POST_KERNEL);
    8300173c:	f9dff0ef          	jal	830016d8 <z_sys_init_run_level>
#if defined(CONFIG_STACK_POINTER_RANDOM) && (CONFIG_STACK_POINTER_RANDOM != 0)
	z_stack_adjust_initialized = 1;
#endif /* CONFIG_STACK_POINTER_RANDOM */
	boot_banner();
    83001740:	14c020ef          	jal	8300388c <boot_banner>

	void z_init_static(void);
	z_init_static();
    83001744:	3ca000ef          	jal	83001b0e <z_init_static>

	/* Final init level before app starts */
	z_sys_init_run_level(INIT_LEVEL_APPLICATION);
    83001748:	4511                	li	a0,4
	STRUCT_SECTION_FOREACH(_static_thread_data, thread_data) {
    8300174a:	00007497          	auipc	s1,0x7
    8300174e:	8a648493          	addi	s1,s1,-1882 # 83007ff0 <log_const_cbprintf_package>
	z_sys_init_run_level(INIT_LEVEL_APPLICATION);
    83001752:	f87ff0ef          	jal	830016d8 <z_sys_init_run_level>
	STRUCT_SECTION_FOREACH(_static_thread_data, thread_data) {
    83001756:	00007417          	auipc	s0,0x7
    8300175a:	89a40413          	addi	s0,s0,-1894 # 83007ff0 <log_const_cbprintf_package>
    8300175e:	8926                	mv	s2,s1
    83001760:	0684f663          	bgeu	s1,s0,830017cc <bg_thread_main+0xa8>
    83001764:	00007597          	auipc	a1,0x7
    83001768:	53458593          	addi	a1,a1,1332 # 83008c98 <CSWTCH.279+0x50>
    8300176c:	04400693          	li	a3,68
    83001770:	00007617          	auipc	a2,0x7
    83001774:	50060613          	addi	a2,a2,1280 # 83008c70 <CSWTCH.279+0x28>
    83001778:	00007517          	auipc	a0,0x7
    8300177c:	aa050513          	addi	a0,a0,-1376 # 83008218 <__clz_tab+0x1e0>
    83001780:	f9ffe0ef          	jal	8300071e <assert_print>
    83001784:	00007517          	auipc	a0,0x7
    83001788:	3d450513          	addi	a0,a0,980 # 83008b58 <uart_ns16550_driver_api+0x68>
    8300178c:	f93fe0ef          	jal	8300071e <assert_print>
    83001790:	04400593          	li	a1,68
	STRUCT_SECTION_FOREACH(_static_thread_data, thread_data) {
    83001794:	00007517          	auipc	a0,0x7
    83001798:	4dc50513          	addi	a0,a0,1244 # 83008c70 <CSWTCH.279+0x28>
    8300179c:	f79fe0ef          	jal	83000714 <assert_post_action>
		z_setup_new_thread(
    830017a0:	603c                	ld	a5,64(s0)
    830017a2:	e43e                	sd	a5,8(sp)
    830017a4:	5c5c                	lw	a5,60(s0)
    830017a6:	e03e                	sd	a5,0(sp)
    830017a8:	741c                	ld	a5,40(s0)
    830017aa:	03842883          	lw	a7,56(s0)
    830017ae:	03043803          	ld	a6,48(s0)
    830017b2:	7018                	ld	a4,32(s0)
    830017b4:	6c14                	ld	a3,24(s0)
    830017b6:	01046603          	lwu	a2,16(s0)
    830017ba:	640c                	ld	a1,8(s0)
    830017bc:	6008                	ld	a0,0(s0)
    830017be:	3d2000ef          	jal	83001b90 <z_setup_new_thread>
		thread_data->init_thread->init_data = thread_data;
    830017c2:	601c                	ld	a5,0(s0)
    830017c4:	ffc0                	sd	s0,184(a5)
	STRUCT_SECTION_FOREACH(_static_thread_data, thread_data) {
    830017c6:	05040413          	addi	s0,s0,80
    830017ca:	bf59                	j	83001760 <bg_thread_main+0x3c>
    830017cc:	fc946ae3          	bltu	s0,s1,830017a0 <bg_thread_main+0x7c>
	k_sched_lock();
    830017d0:	19c010ef          	jal	8300296c <k_sched_lock>
	STRUCT_SECTION_FOREACH(_static_thread_data, thread_data) {
    830017d4:	00007417          	auipc	s0,0x7
    830017d8:	81c40413          	addi	s0,s0,-2020 # 83007ff0 <log_const_cbprintf_package>
		if (!K_TIMEOUT_EQ(init_delay, K_FOREVER)) {
    830017dc:	54fd                	li	s1,-1

extern void z_thread_timeout(struct _timeout *timeout);

static inline void z_add_thread_timeout(struct k_thread *thread, k_timeout_t ticks)
{
	z_add_timeout(&thread->base.timeout, z_thread_timeout, ticks);
    830017de:	00001997          	auipc	s3,0x1
    830017e2:	ee498993          	addi	s3,s3,-284 # 830026c2 <z_thread_timeout>
	STRUCT_SECTION_FOREACH(_static_thread_data, thread_data) {
    830017e6:	04897b63          	bgeu	s2,s0,8300183c <bg_thread_main+0x118>
    830017ea:	00007597          	auipc	a1,0x7
    830017ee:	4ae58593          	addi	a1,a1,1198 # 83008c98 <CSWTCH.279+0x50>
    830017f2:	06800693          	li	a3,104
    830017f6:	00007617          	auipc	a2,0x7
    830017fa:	47a60613          	addi	a2,a2,1146 # 83008c70 <CSWTCH.279+0x28>
    830017fe:	00007517          	auipc	a0,0x7
    83001802:	a1a50513          	addi	a0,a0,-1510 # 83008218 <__clz_tab+0x1e0>
    83001806:	f19fe0ef          	jal	8300071e <assert_print>
    8300180a:	00007517          	auipc	a0,0x7
    8300180e:	34e50513          	addi	a0,a0,846 # 83008b58 <uart_ns16550_driver_api+0x68>
    83001812:	f0dfe0ef          	jal	8300071e <assert_print>
    83001816:	06800593          	li	a1,104
    8300181a:	bfad                	j	83001794 <bg_thread_main+0x70>
		k_timeout_t init_delay = Z_THREAD_INIT_DELAY(thread_data);
    8300181c:	6430                	ld	a2,72(s0)
		if (!K_TIMEOUT_EQ(init_delay, K_FOREVER)) {
    8300181e:	00960663          	beq	a2,s1,8300182a <bg_thread_main+0x106>
			thread_schedule_new(thread_data->init_thread,
    83001822:	6008                	ld	a0,0(s0)


static inline void thread_schedule_new(struct k_thread *thread, k_timeout_t delay)
{
#ifdef CONFIG_SYS_CLOCK_EXISTS
	if (K_TIMEOUT_EQ(delay, K_NO_WAIT)) {
    83001824:	e611                	bnez	a2,83001830 <bg_thread_main+0x10c>
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_K_THREAD_START);
		return;
	}
#endif
	compiler_barrier();
	z_impl_k_thread_start(thread);
    83001826:	366000ef          	jal	83001b8c <z_impl_k_thread_start>
	STRUCT_SECTION_FOREACH(_static_thread_data, thread_data) {
    8300182a:	05040413          	addi	s0,s0,80
    8300182e:	bf65                	j	830017e6 <bg_thread_main+0xc2>
    83001830:	85ce                	mv	a1,s3
    83001832:	02850513          	addi	a0,a0,40
    83001836:	421010ef          	jal	83003456 <z_add_timeout>
    8300183a:	bfc5                	j	8300182a <bg_thread_main+0x106>
    8300183c:	ff2460e3          	bltu	s0,s2,8300181c <bg_thread_main+0xf8>
	k_sched_unlock();
    83001840:	276010ef          	jal	83002ab6 <k_sched_unlock>
	z_mem_manage_boot_finish();
#endif /* CONFIG_MMU */

	extern int main(void);

	(void)main();
    83001844:	b03fe0ef          	jal	83000346 <main>
 * Exceptions raised by this thread may be recoverable.
 * (This is the default tag for a thread.)
 */
static inline void z_thread_essential_clear(struct k_thread *thread)
{
	thread->base.user_options &= ~K_ESSENTIAL;
    83001848:	0000a717          	auipc	a4,0xa
    8300184c:	fc070713          	addi	a4,a4,-64 # 8300b808 <z_main_thread>
    83001850:	01874783          	lbu	a5,24(a4)
    83001854:	9bf9                	andi	a5,a5,-2
    83001856:	00f70c23          	sb	a5,24(a4)

#ifdef CONFIG_COVERAGE_DUMP
	/* Dump coverage data once the main() has exited. */
	gcov_coverage_dump();
#endif /* CONFIG_COVERAGE_DUMP */
} /* LCOV_EXCL_LINE ... because we just dumped final coverage data */
    8300185a:	70e2                	ld	ra,56(sp)
    8300185c:	7442                	ld	s0,48(sp)
    8300185e:	74a2                	ld	s1,40(sp)
    83001860:	7902                	ld	s2,32(sp)
    83001862:	69e2                	ld	s3,24(sp)
    83001864:	6121                	addi	sp,sp,64
    83001866:	8082                	ret

0000000083001868 <z_early_memset>:
	(void) memset(dst, c, n);
    83001868:	6240206f          	j	83003e8c <memset>

000000008300186c <z_bss_zero>:
	z_early_memset(__bss_start, 0, __bss_end - __bss_start);
    8300186c:	00008517          	auipc	a0,0x8
    83001870:	c7c50513          	addi	a0,a0,-900 # 830094e8 <_stdout_hook>
    83001874:	0000a617          	auipc	a2,0xa
    83001878:	25460613          	addi	a2,a2,596 # 8300bac8 <resource_table>
    8300187c:	8e09                	sub	a2,a2,a0
    8300187e:	4581                	li	a1,0
    83001880:	b7e5                	j	83001868 <z_early_memset>

0000000083001882 <z_init_cpu>:
	thread->base.is_idle = 1U;
#endif /* CONFIG_SMP */
}

void z_init_cpu(int id)
{
    83001882:	7179                	addi	sp,sp,-48
    83001884:	e84a                	sd	s2,16(sp)
	struct k_thread *thread = &z_idle_threads[i];
    83001886:	12800913          	li	s2,296
{
    8300188a:	ec26                	sd	s1,24(sp)
	struct k_thread *thread = &z_idle_threads[i];
    8300188c:	03250933          	mul	s2,a0,s2
			  stack_size, idle, &_kernel.cpus[i],
    83001890:	03800493          	li	s1,56
	struct k_thread *thread = &z_idle_threads[i];
    83001894:	0000a797          	auipc	a5,0xa
    83001898:	e4c78793          	addi	a5,a5,-436 # 8300b6e0 <z_idle_threads>
{
    8300189c:	f022                	sd	s0,32(sp)
	k_thread_stack_t *stack = z_idle_stacks[i];
    8300189e:	00a51593          	slli	a1,a0,0xa
{
    830018a2:	842a                	mv	s0,a0
	z_setup_new_thread(thread, stack,
    830018a4:	48bd                	li	a7,15
    830018a6:	4801                	li	a6,0
    830018a8:	00000697          	auipc	a3,0x0
    830018ac:	26868693          	addi	a3,a3,616 # 83001b10 <idle>
    830018b0:	40000613          	li	a2,1024
			  stack_size, idle, &_kernel.cpus[i],
    830018b4:	029504b3          	mul	s1,a0,s1
	struct k_thread *thread = &z_idle_threads[i];
    830018b8:	993e                	add	s2,s2,a5
			  stack_size, idle, &_kernel.cpus[i],
    830018ba:	0000a797          	auipc	a5,0xa
    830018be:	07678793          	addi	a5,a5,118 # 8300b930 <_kernel>
	z_setup_new_thread(thread, stack,
    830018c2:	0001d517          	auipc	a0,0x1d
    830018c6:	f6e50513          	addi	a0,a0,-146 # 8301e830 <z_idle_stacks>
    830018ca:	95aa                	add	a1,a1,a0
    830018cc:	854a                	mv	a0,s2
{
    830018ce:	f406                	sd	ra,40(sp)
			  stack_size, idle, &_kernel.cpus[i],
    830018d0:	94be                	add	s1,s1,a5
	z_setup_new_thread(thread, stack,
    830018d2:	00007797          	auipc	a5,0x7
    830018d6:	3f678793          	addi	a5,a5,1014 # 83008cc8 <CSWTCH.279+0x80>
    830018da:	e43e                	sd	a5,8(sp)
    830018dc:	4785                	li	a5,1
    830018de:	e03e                	sd	a5,0(sp)
    830018e0:	8726                	mv	a4,s1
    830018e2:	4781                	li	a5,0
    830018e4:	2ac000ef          	jal	83001b90 <z_setup_new_thread>
	thread->base.thread_state &= ~_THREAD_PRESTART;
    830018e8:	01994783          	lbu	a5,25(s2)
	init_idle_thread(id);
	_kernel.cpus[id].idle_thread = &z_idle_threads[id];
	_kernel.cpus[id].id = id;
    830018ec:	02848023          	sb	s0,32(s1)
	_kernel.cpus[id].irq_stack =
		(K_KERNEL_STACK_BUFFER(z_interrupt_stacks[id]) +
    830018f0:	0405                	addi	s0,s0,1
    830018f2:	9bed                	andi	a5,a5,-5
    830018f4:	00f90ca3          	sb	a5,25(s2)
    830018f8:	042e                	slli	s0,s0,0xb
    830018fa:	0001c797          	auipc	a5,0x1c
    830018fe:	73678793          	addi	a5,a5,1846 # 8301e030 <z_interrupt_stacks>
    83001902:	943e                	add	s0,s0,a5
	_kernel.cpus[id].idle_thread = &z_idle_threads[id];
    83001904:	0124bc23          	sd	s2,24(s1)
	_kernel.cpus[id].irq_stack =
    83001908:	e480                	sd	s0,8(s1)
	k_obj_core_stats_register(K_OBJ_CORE(&_kernel.cpus[id]),
				  _kernel.cpus[id].usage,
				  sizeof(struct k_cycle_stats));
#endif
#endif
}
    8300190a:	70a2                	ld	ra,40(sp)
    8300190c:	7402                	ld	s0,32(sp)
    8300190e:	64e2                	ld	s1,24(sp)
    83001910:	6942                	ld	s2,16(sp)
    83001912:	6145                	addi	sp,sp,48
    83001914:	8082                	ret

0000000083001916 <z_cstart>:
 * @return Does not return
 */
__boot_func
FUNC_NO_STACK_PROTECTOR
FUNC_NORETURN void z_cstart(void)
{
    83001916:	7139                	addi	sp,sp,-64
	/* gcov hook needed to get the coverage report.*/
	gcov_static_init();

	/* initialize early init calls */
	z_sys_init_run_level(INIT_LEVEL_EARLY);
    83001918:	4501                	li	a0,0
{
    8300191a:	fc06                	sd	ra,56(sp)
    8300191c:	f822                	sd	s0,48(sp)
    8300191e:	f426                	sd	s1,40(sp)
    83001920:	f04a                	sd	s2,32(sp)
    83001922:	ec4e                	sd	s3,24(sp)
	z_sys_init_run_level(INIT_LEVEL_EARLY);
    83001924:	db5ff0ef          	jal	830016d8 <z_sys_init_run_level>
#ifndef _ASMLANGUAGE

static ALWAYS_INLINE void arch_kernel_init(void)
{
#ifdef CONFIG_THREAD_LOCAL_STORAGE
	__asm__ volatile ("li tp, 0");
    83001928:	4201                	li	tp,0
#endif
#if defined(CONFIG_SMP) || defined(CONFIG_USERSPACE)
	csr_write(xscratch, &_kernel.cpus[0]);
#endif
#if defined(CONFIG_SMP) || defined(CONFIG_RISCV_SBI_BOOT)
	_kernel.cpus[0].arch.hartid = CONFIG_RV_BOOT_HART;
    8300192a:	0000a417          	auipc	s0,0xa
    8300192e:	00640413          	addi	s0,s0,6 # 8300b930 <_kernel>
    83001932:	4789                	li	a5,2
{
	dummy_thread->base.thread_state = _THREAD_DUMMY;
#ifdef CONFIG_SCHED_CPU_MASK
	dummy_thread->base.cpu_mask = -1;
#endif /* CONFIG_SCHED_CPU_MASK */
	dummy_thread->base.user_options = K_ESSENTIAL;
    83001934:	10100713          	li	a4,257
    83001938:	f41c                	sd	a5,40(s0)
	_kernel.cpus[0].arch.online = true;
    8300193a:	4905                	li	s2,1
    8300193c:	0000a797          	auipc	a5,0xa
    83001940:	04478793          	addi	a5,a5,68 # 8300b980 <_thread_dummy>
    83001944:	00e79c23          	sh	a4,24(a5)

#ifdef CONFIG_TIMESLICE_PER_THREAD
	dummy_thread->base.slice_ticks = 0;
#endif /* CONFIG_TIMESLICE_PER_THREAD */

	_current_cpu->current = dummy_thread;
    83001948:	e81c                	sd	a5,16(s0)
    8300194a:	03240823          	sb	s2,48(s0)
	dummy_thread->stack_info.start = 0U;
    8300194e:	0000a717          	auipc	a4,0xa
    83001952:	12073123          	sd	zero,290(a4) # 8300ba70 <_thread_dummy+0xf0>
	dummy_thread->stack_info.size = 0U;
    83001956:	0000a717          	auipc	a4,0xa
    8300195a:	12073123          	sd	zero,290(a4) # 8300ba78 <_thread_dummy+0xf8>
	dummy_thread->resource_pool = NULL;
    8300195e:	0000a717          	auipc	a4,0xa
    83001962:	12073d23          	sd	zero,314(a4) # 8300ba98 <_thread_dummy+0x118>

#if defined(CONFIG_MULTITHREADING)
	z_dummy_thread_init(&_thread_dummy);
#endif /* CONFIG_MULTITHREADING */
	/* do any necessary initialization of static devices */
	z_device_state_init();
    83001966:	be5ff0ef          	jal	8300154a <z_device_state_init>

	/* perform basic hardware initialization */
	z_sys_init_run_level(INIT_LEVEL_PRE_KERNEL_1);
    8300196a:	4505                	li	a0,1
    8300196c:	d6dff0ef          	jal	830016d8 <z_sys_init_run_level>
	mica_init();
    83001970:	986ff0ef          	jal	83000af6 <mica_init>
#if defined(CONFIG_SMP)
	arch_smp_init();
#endif
	z_sys_init_run_level(INIT_LEVEL_PRE_KERNEL_2);
    83001974:	4509                	li	a0,2
    83001976:	d63ff0ef          	jal	830016d8 <z_sys_init_run_level>
	_kernel.ready_q.cache = &z_main_thread;
    8300197a:	0000a497          	auipc	s1,0xa
    8300197e:	e8e48493          	addi	s1,s1,-370 # 8300b808 <z_main_thread>
	z_sched_init();
    83001982:	2bc010ef          	jal	83002c3e <z_sched_init>
	stack_ptr = z_setup_new_thread(&z_main_thread, z_main_stack,
    83001986:	00007797          	auipc	a5,0x7
    8300198a:	34a78793          	addi	a5,a5,842 # 83008cd0 <CSWTCH.279+0x88>
	_kernel.ready_q.cache = &z_main_thread;
    8300198e:	fc04                	sd	s1,56(s0)
	stack_ptr = z_setup_new_thread(&z_main_thread, z_main_stack,
    83001990:	48ad                	li	a7,11
    83001992:	4801                	li	a6,0
    83001994:	4701                	li	a4,0
    83001996:	00000697          	auipc	a3,0x0
    8300199a:	d8e68693          	addi	a3,a3,-626 # 83001724 <bg_thread_main>
    8300199e:	6605                	lui	a2,0x1
    830019a0:	0001d597          	auipc	a1,0x1d
    830019a4:	29058593          	addi	a1,a1,656 # 8301ec30 <z_main_stack>
    830019a8:	e43e                	sd	a5,8(sp)
    830019aa:	e04a                	sd	s2,0(sp)
    830019ac:	4781                	li	a5,0
    830019ae:	8526                	mv	a0,s1
    830019b0:	1e0000ef          	jal	83001b90 <z_setup_new_thread>
    830019b4:	0194c783          	lbu	a5,25(s1)
	z_ready_thread(&z_main_thread);
    830019b8:	8526                	mv	a0,s1
    830019ba:	9bed                	andi	a5,a5,-5
    830019bc:	00f48ca3          	sb	a5,25(s1)
    830019c0:	255000ef          	jal	83002414 <z_ready_thread>
	z_init_cpu(0);
    830019c4:	4501                	li	a0,0
    830019c6:	ebdff0ef          	jal	83001882 <z_init_cpu>
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    830019ca:	10017973          	csrrci	s2,sstatus,2
	__ASSERT(arch_irq_unlocked(key) ||
    830019ce:	00297913          	andi	s2,s2,2
    830019d2:	04091563          	bnez	s2,83001a1c <z_cstart+0x106>
    830019d6:	681c                	ld	a5,16(s0)
    830019d8:	0197c783          	lbu	a5,25(a5)
    830019dc:	8ba5                	andi	a5,a5,9
    830019de:	ef9d                	bnez	a5,83001a1c <z_cstart+0x106>
    830019e0:	00007597          	auipc	a1,0x7
    830019e4:	32858593          	addi	a1,a1,808 # 83008d08 <CSWTCH.279+0xc0>
    830019e8:	06300693          	li	a3,99
    830019ec:	00007617          	auipc	a2,0x7
    830019f0:	2ec60613          	addi	a2,a2,748 # 83008cd8 <CSWTCH.279+0x90>
    830019f4:	00007517          	auipc	a0,0x7
    830019f8:	82450513          	addi	a0,a0,-2012 # 83008218 <__clz_tab+0x1e0>
    830019fc:	d23fe0ef          	jal	8300071e <assert_print>
    83001a00:	00007517          	auipc	a0,0x7
    83001a04:	37850513          	addi	a0,a0,888 # 83008d78 <CSWTCH.279+0x130>
    83001a08:	d17fe0ef          	jal	8300071e <assert_print>
    83001a0c:	06300593          	li	a1,99
    83001a10:	00007517          	auipc	a0,0x7
    83001a14:	2c850513          	addi	a0,a0,712 # 83008cd8 <CSWTCH.279+0x90>
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83001a18:	cfdfe0ef          	jal	83000714 <assert_post_action>
	old_thread = _current;
    83001a1c:	01043983          	ld	s3,16(s0)
	z_check_stack_sentinel();
    83001a20:	140000ef          	jal	83001b60 <z_check_stack_sentinel>
	old_thread->swap_retval = -EAGAIN;
    83001a24:	57d5                	li	a5,-11
    83001a26:	10f9a423          	sw	a5,264(s3)
    83001a2a:	100177f3          	csrrci	a5,sstatus,2
    83001a2e:	00008517          	auipc	a0,0x8
    83001a32:	ad250513          	addi	a0,a0,-1326 # 83009500 <_sched_spinlock>
    83001a36:	13d010ef          	jal	83003372 <z_spin_lock_valid>
    83001a3a:	e131                	bnez	a0,83001a7e <z_cstart+0x168>
    83001a3c:	08800693          	li	a3,136
    83001a40:	00007617          	auipc	a2,0x7
    83001a44:	fd860613          	addi	a2,a2,-40 # 83008a18 <plic_config_0+0x38>
    83001a48:	00007597          	auipc	a1,0x7
    83001a4c:	00058593          	mv	a1,a1
    83001a50:	00006517          	auipc	a0,0x6
    83001a54:	7c850513          	addi	a0,a0,1992 # 83008218 <__clz_tab+0x1e0>
    83001a58:	cc7fe0ef          	jal	8300071e <assert_print>
    83001a5c:	00008597          	auipc	a1,0x8
    83001a60:	aa458593          	addi	a1,a1,-1372 # 83009500 <_sched_spinlock>
    83001a64:	00007517          	auipc	a0,0x7
    83001a68:	ffc50513          	addi	a0,a0,-4 # 83008a60 <plic_config_0+0x80>
    83001a6c:	cb3fe0ef          	jal	8300071e <assert_print>
    83001a70:	08800593          	li	a1,136
    83001a74:	00007517          	auipc	a0,0x7
    83001a78:	fa450513          	addi	a0,a0,-92 # 83008a18 <plic_config_0+0x38>
    83001a7c:	bf71                	j	83001a18 <z_cstart+0x102>
	z_spin_lock_set_owner(l);
    83001a7e:	00008517          	auipc	a0,0x8
    83001a82:	a8250513          	addi	a0,a0,-1406 # 83009500 <_sched_spinlock>
    83001a86:	135010ef          	jal	830033ba <z_spin_lock_set_owner>
	new_thread = z_swap_next_thread();
    83001a8a:	180010ef          	jal	83002c0a <z_swap_next_thread>
    83001a8e:	84aa                	mv	s1,a0
	if (new_thread != old_thread) {
    83001a90:	06a98763          	beq	s3,a0,83001afe <z_cstart+0x1e8>
		_current_cpu->current = new_thread;
    83001a94:	e808                	sd	a0,16(s0)
		z_reset_time_slice(new_thread);
    83001a96:	774010ef          	jal	8300320a <z_reset_time_slice>
		z_spin_lock_set_owner(&_sched_spinlock);
    83001a9a:	00008517          	auipc	a0,0x8
    83001a9e:	a6650513          	addi	a0,a0,-1434 # 83009500 <_sched_spinlock>
    83001aa2:	119010ef          	jal	830033ba <z_spin_lock_set_owner>
/* Internal function: releases the lock, but leaves local interrupts disabled */
static ALWAYS_INLINE void k_spin_release(struct k_spinlock *l)
{
	ARG_UNUSED(l);
#ifdef CONFIG_SPIN_VALIDATE
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83001aa6:	00008517          	auipc	a0,0x8
    83001aaa:	a5a50513          	addi	a0,a0,-1446 # 83009500 <_sched_spinlock>
		void *newsh = new_thread->switch_handle;
    83001aae:	1104b403          	ld	s0,272(s1)
    83001ab2:	0db010ef          	jal	8300338c <z_spin_unlock_valid>
    83001ab6:	ed15                	bnez	a0,83001af2 <z_cstart+0x1dc>
    83001ab8:	16b00693          	li	a3,363
    83001abc:	00007617          	auipc	a2,0x7
    83001ac0:	f5c60613          	addi	a2,a2,-164 # 83008a18 <plic_config_0+0x38>
    83001ac4:	00007597          	auipc	a1,0x7
    83001ac8:	fb458593          	addi	a1,a1,-76 # 83008a78 <plic_config_0+0x98>
    83001acc:	00006517          	auipc	a0,0x6
    83001ad0:	74c50513          	addi	a0,a0,1868 # 83008218 <__clz_tab+0x1e0>
    83001ad4:	c4bfe0ef          	jal	8300071e <assert_print>
    83001ad8:	00008597          	auipc	a1,0x8
    83001adc:	a2858593          	addi	a1,a1,-1496 # 83009500 <_sched_spinlock>
    83001ae0:	00007517          	auipc	a0,0x7
    83001ae4:	fb050513          	addi	a0,a0,-80 # 83008a90 <plic_config_0+0xb0>
    83001ae8:	c37fe0ef          	jal	8300071e <assert_print>
    83001aec:	16b00593          	li	a1,363
    83001af0:	b751                	j	83001a74 <z_cstart+0x15e>
	struct k_thread *old = CONTAINER_OF(switched_from, struct k_thread,
					    switch_handle);
#ifdef CONFIG_RISCV_ALWAYS_SWITCH_THROUGH_ECALL
	arch_syscall_invoke2((uintptr_t)new, (uintptr_t)old, RV_ECALL_SCHEDULE);
#else
	z_riscv_switch(new, old);
    83001af2:	85ce                	mv	a1,s3
    83001af4:	8522                	mv	a0,s0
    83001af6:	eeafe0ef          	jal	830001e0 <z_riscv_switch>
	csr_set(xstatus, (key & XSTATUS_IEN));
    83001afa:	10092073          	csrs	sstatus,s2
    83001afe:	00008517          	auipc	a0,0x8
    83001b02:	a0250513          	addi	a0,a0,-1534 # 83009500 <_sched_spinlock>
    83001b06:	087010ef          	jal	8300338c <z_spin_unlock_valid>
    83001b0a:	f965                	bnez	a0,83001afa <z_cstart+0x1e4>
    83001b0c:	b775                	j	83001ab8 <z_cstart+0x1a2>

0000000083001b0e <z_init_static>:
	__do_global_ctors_aux();
	__do_init_array_aux();
#elif defined(__CCAC__) /* ARC MWDT */
	__do_global_ctors_aux();
#endif
}
    83001b0e:	8082                	ret

0000000083001b10 <idle>:
#include <wait_q.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

void idle(void *unused1, void *unused2, void *unused3)
{
    83001b10:	1141                	addi	sp,sp,-16
    83001b12:	e406                	sd	ra,8(sp)
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	__ASSERT_NO_MSG(_current->base.prio >= 0);
    83001b14:	0000a797          	auipc	a5,0xa
    83001b18:	e2c7b783          	ld	a5,-468(a5) # 8300b940 <_kernel+0x10>
    83001b1c:	01a78783          	lb	a5,26(a5)
    83001b20:	0207d863          	bgez	a5,83001b50 <idle+0x40>
    83001b24:	00007597          	auipc	a1,0x7
    83001b28:	2dc58593          	addi	a1,a1,732 # 83008e00 <levels.0+0x60>
    83001b2c:	00006517          	auipc	a0,0x6
    83001b30:	6ec50513          	addi	a0,a0,1772 # 83008218 <__clz_tab+0x1e0>
    83001b34:	46ed                	li	a3,27
    83001b36:	00007617          	auipc	a2,0x7
    83001b3a:	2a260613          	addi	a2,a2,674 # 83008dd8 <levels.0+0x38>
    83001b3e:	be1fe0ef          	jal	8300071e <assert_print>
    83001b42:	45ed                	li	a1,27
    83001b44:	00007517          	auipc	a0,0x7
    83001b48:	29450513          	addi	a0,a0,660 # 83008dd8 <levels.0+0x38>
    83001b4c:	bc9fe0ef          	jal	83000714 <assert_post_action>
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83001b50:	100177f3          	csrrci	a5,sstatus,2
 * @note In some architectures, before returning, the function unmasks interrupts
 * unconditionally.
 */
static inline void k_cpu_idle(void)
{
	arch_cpu_idle();
    83001b54:	c47fe0ef          	jal	8300079a <arch_cpu_idle>
    83001b58:	bfe5                	j	83001b50 <idle+0x40>

0000000083001b5a <k_thread_name_get>:
	return (const char *)thread->name;
#else
	ARG_UNUSED(thread);
	return NULL;
#endif /* CONFIG_THREAD_NAME */
}
    83001b5a:	0d050513          	addi	a0,a0,208
    83001b5e:	8082                	ret

0000000083001b60 <z_check_stack_sentinel>:
 */
void z_check_stack_sentinel(void)
{
	uint32_t *stack;

	if ((_current->base.thread_state & _THREAD_DUMMY) != 0) {
    83001b60:	0000a717          	auipc	a4,0xa
    83001b64:	de073703          	ld	a4,-544(a4) # 8300b940 <_kernel+0x10>
    83001b68:	01974783          	lbu	a5,25(a4)
    83001b6c:	8b85                	andi	a5,a5,1
    83001b6e:	ef91                	bnez	a5,83001b8a <z_check_stack_sentinel+0x2a>
		return;
	}

	stack = (uint32_t *)_current->stack_info.start;
    83001b70:	7b78                	ld	a4,240(a4)
	if (*stack != STACK_SENTINEL) {
    83001b72:	f0f0f7b7          	lui	a5,0xf0f0f
    83001b76:	0f078793          	addi	a5,a5,240 # fffffffff0f0f0f0 <CONFIG_RISCV_MCAUSE_EXCEPTION_MASK+0x7ffffffff0f0f0f1>
    83001b7a:	4314                	lw	a3,0(a4)
    83001b7c:	00f68763          	beq	a3,a5,83001b8a <z_check_stack_sentinel+0x2a>
		/* Restore it so further checks don't trigger this same error */
		*stack = STACK_SENTINEL;
    83001b80:	c31c                	sw	a5,0(a4)
	register unsigned long a0 __asm__ ("a0") = arg1;
    83001b82:	4509                	li	a0,2
	register unsigned long t0 __asm__ ("t0") = call_id;
    83001b84:	4281                	li	t0,0
	__asm__ volatile ("ecall"
    83001b86:	00000073          	ecall
		z_except_reason(K_ERR_STACK_CHK_FAIL);
	}
}
    83001b8a:	8082                	ret

0000000083001b8c <z_impl_k_thread_start>:

void z_impl_k_thread_start(k_tid_t thread)
{
	SYS_PORT_TRACING_OBJ_FUNC(k_thread, start, thread);

	z_sched_start(thread);
    83001b8c:	3c30006f          	j	8300274e <z_sched_start>

0000000083001b90 <z_setup_new_thread>:
char *z_setup_new_thread(struct k_thread *new_thread,
			 k_thread_stack_t *stack, size_t stack_size,
			 k_thread_entry_t entry,
			 void *p1, void *p2, void *p3,
			 int prio, uint32_t options, const char *name)
{
    83001b90:	715d                	addi	sp,sp,-80
    83001b92:	e0a2                	sd	s0,64(sp)
    83001b94:	fc26                	sd	s1,56(sp)
    83001b96:	f84a                	sd	s2,48(sp)
    83001b98:	e486                	sd	ra,72(sp)
    83001b9a:	892e                	mv	s2,a1
    83001b9c:	f44e                	sd	s3,40(sp)
	char *stack_ptr;

	Z_ASSERT_VALID_PRIO(prio, entry);
    83001b9e:	45bd                	li	a1,15
{
    83001ba0:	842a                	mv	s0,a0
    83001ba2:	84c6                	mv	s1,a7
	Z_ASSERT_VALID_PRIO(prio, entry);
    83001ba4:	0ab89f63          	bne	a7,a1,83001c62 <z_setup_new_thread+0xd2>
    83001ba8:	00000597          	auipc	a1,0x0
    83001bac:	f6858593          	addi	a1,a1,-152 # 83001b10 <idle>
    83001bb0:	0ab69e63          	bne	a3,a1,83001c6c <z_setup_new_thread+0xdc>
	SYS_DLIST_FOR_EACH_CONTAINER(&((wq)->waitq), thread_ptr, \
				     base.qnode_dlist)

static inline void z_waitq_init(_wait_q_t *w)
{
	sys_dlist_init(&w->waitq);
    83001bb4:	0c040593          	addi	a1,s0,192
 * @param list the doubly-linked list
 */

static inline void sys_dlist_init(sys_dlist_t *list)
{
	list->head = (sys_dnode_t *)list;
    83001bb8:	e06c                	sd	a1,192(s0)
	list->tail = (sys_dnode_t *)list;
    83001bba:	e46c                	sd	a1,200(s0)
void z_init_thread_base(struct _thread_base *thread_base, int priority,
		       uint32_t initial_state, unsigned int options)
{
	/* k_q_node is initialized upon first insertion in a list */
	thread_base->pended_on = NULL;
	thread_base->user_options = (uint8_t)options;
    83001bbc:	45c6                	lw	a1,80(sp)
		stack_obj_size = K_KERNEL_STACK_LEN(stack_size);
    83001bbe:	063d                	addi	a2,a2,15
	thread_base->thread_state = (uint8_t)initial_state;

	thread_base->prio = priority;
    83001bc0:	00940d23          	sb	s1,26(s0)
	thread_base->user_options = (uint8_t)options;
    83001bc4:	00b40c23          	sb	a1,24(s0)
		stack_obj_size = K_KERNEL_STACK_LEN(stack_size);
    83001bc8:	ff067493          	andi	s1,a2,-16
	thread_base->thread_state = (uint8_t)initial_state;
    83001bcc:	4591                	li	a1,4
	*((uint32_t *)stack_buf_start) = STACK_SENTINEL;
    83001bce:	f0f0f637          	lui	a2,0xf0f0f
	thread_base->thread_state = (uint8_t)initial_state;
    83001bd2:	00b40ca3          	sb	a1,25(s0)
	*((uint32_t *)stack_buf_start) = STACK_SENTINEL;
    83001bd6:	0f060613          	addi	a2,a2,240 # fffffffff0f0f0f0 <CONFIG_RISCV_MCAUSE_EXCEPTION_MASK+0x7ffffffff0f0f0f1>
	thread_base->pended_on = NULL;
    83001bda:	00043823          	sd	zero,16(s0)

	thread_base->sched_locked = 0U;
    83001bde:	00040da3          	sb	zero,27(s0)
 * @param node the node
 */

static inline void sys_dnode_init(sys_dnode_t *node)
{
	node->next = NULL;
    83001be2:	02043423          	sd	zero,40(s0)
	node->prev = NULL;
    83001be6:	02043823          	sd	zero,48(s0)
	stack_ptr = (char *)stack + stack_obj_size;
    83001bea:	009909b3          	add	s3,s2,s1
	delta += arch_tls_stack_setup(new_thread, stack_ptr);
    83001bee:	85ce                	mv	a1,s3
	*((uint32_t *)stack_buf_start) = STACK_SENTINEL;
    83001bf0:	00c92023          	sw	a2,0(s2)
	delta += arch_tls_stack_setup(new_thread, stack_ptr);
    83001bf4:	8522                	mv	a0,s0
    83001bf6:	ec42                	sd	a6,24(sp)
    83001bf8:	e83e                	sd	a5,16(sp)
    83001bfa:	e43a                	sd	a4,8(sp)
    83001bfc:	e036                	sd	a3,0(sp)
    83001bfe:	f21fe0ef          	jal	83000b1e <arch_tls_stack_setup>
	delta = ROUND_UP(delta, ARCH_STACK_PTR_ALIGN);
    83001c02:	053d                	addi	a0,a0,15
    83001c04:	9941                	andi	a0,a0,-16
	arch_new_thread(new_thread, stack, stack_ptr, entry, p1, p2, p3);
    83001c06:	67c2                	ld	a5,16(sp)
    83001c08:	6862                	ld	a6,24(sp)
    83001c0a:	6722                	ld	a4,8(sp)
    83001c0c:	6682                	ld	a3,0(sp)
	stack_ptr -= delta;
    83001c0e:	40a989b3          	sub	s3,s3,a0
	new_thread->stack_info.delta = delta;
    83001c12:	10a43023          	sd	a0,256(s0)
	new_thread->stack_info.start = (uintptr_t)stack_buf_start;
    83001c16:	0f243823          	sd	s2,240(s0)
	new_thread->stack_info.size = stack_buf_size;
    83001c1a:	fc64                	sd	s1,248(s0)
	arch_new_thread(new_thread, stack, stack_ptr, entry, p1, p2, p3);
    83001c1c:	864e                	mv	a2,s3
    83001c1e:	85ca                	mv	a1,s2
    83001c20:	8522                	mv	a0,s0
    83001c22:	e2bfe0ef          	jal	83000a4c <arch_new_thread>
	__ASSERT(new_thread->switch_handle != NULL,
    83001c26:	11043783          	ld	a5,272(s0)
	new_thread->init_data = NULL;
    83001c2a:	0a043c23          	sd	zero,184(s0)
	__ASSERT(new_thread->switch_handle != NULL,
    83001c2e:	e3c1                	bnez	a5,83001cae <z_setup_new_thread+0x11e>
    83001c30:	00007597          	auipc	a1,0x7
    83001c34:	2d858593          	addi	a1,a1,728 # 83008f08 <levels.0+0x168>
    83001c38:	24b00693          	li	a3,587
    83001c3c:	00007617          	auipc	a2,0x7
    83001c40:	1ec60613          	addi	a2,a2,492 # 83008e28 <levels.0+0x88>
    83001c44:	00006517          	auipc	a0,0x6
    83001c48:	5d450513          	addi	a0,a0,1492 # 83008218 <__clz_tab+0x1e0>
    83001c4c:	ad3fe0ef          	jal	8300071e <assert_print>
    83001c50:	00007517          	auipc	a0,0x7
    83001c54:	2e850513          	addi	a0,a0,744 # 83008f38 <levels.0+0x198>
    83001c58:	ac7fe0ef          	jal	8300071e <assert_print>
    83001c5c:	24b00593          	li	a1,587
    83001c60:	a089                	j	83001ca2 <z_setup_new_thread+0x112>
	Z_ASSERT_VALID_PRIO(prio, entry);
    83001c62:	0108859b          	addiw	a1,a7,16
    83001c66:	4579                	li	a0,30
    83001c68:	f4b576e3          	bgeu	a0,a1,83001bb4 <z_setup_new_thread+0x24>
    83001c6c:	21100693          	li	a3,529
    83001c70:	00007617          	auipc	a2,0x7
    83001c74:	1b860613          	addi	a2,a2,440 # 83008e28 <levels.0+0x88>
    83001c78:	00007597          	auipc	a1,0x7
    83001c7c:	1d858593          	addi	a1,a1,472 # 83008e50 <levels.0+0xb0>
    83001c80:	00006517          	auipc	a0,0x6
    83001c84:	59850513          	addi	a0,a0,1432 # 83008218 <__clz_tab+0x1e0>
    83001c88:	a97fe0ef          	jal	8300071e <assert_print>
    83001c8c:	85a6                	mv	a1,s1
    83001c8e:	56c1                	li	a3,-16
    83001c90:	4639                	li	a2,14
    83001c92:	00007517          	auipc	a0,0x7
    83001c96:	23e50513          	addi	a0,a0,574 # 83008ed0 <levels.0+0x130>
    83001c9a:	a85fe0ef          	jal	8300071e <assert_print>
    83001c9e:	21100593          	li	a1,529
	__ASSERT(new_thread->switch_handle != NULL,
    83001ca2:	00007517          	auipc	a0,0x7
    83001ca6:	18650513          	addi	a0,a0,390 # 83008e28 <levels.0+0x88>
    83001caa:	a6bfe0ef          	jal	83000714 <assert_post_action>
	if (name != NULL) {
    83001cae:	67e6                	ld	a5,88(sp)
    83001cb0:	cb8d                	beqz	a5,83001ce2 <z_setup_new_thread+0x152>
		strncpy(new_thread->name, name,
    83001cb2:	467d                	li	a2,31
    83001cb4:	85be                	mv	a1,a5
    83001cb6:	0d040513          	addi	a0,s0,208
    83001cba:	3df010ef          	jal	83003898 <strncpy>
		new_thread->name[CONFIG_THREAD_MAX_NAME_LEN - 1] = '\0';
    83001cbe:	0e0407a3          	sb	zero,239(s0)
	new_thread->resource_pool = _current->resource_pool;
    83001cc2:	0000a797          	auipc	a5,0xa
    83001cc6:	c7e7b783          	ld	a5,-898(a5) # 8300b940 <_kernel+0x10>
    83001cca:	1187b783          	ld	a5,280(a5)
}
    83001cce:	854e                	mv	a0,s3
	new_thread->resource_pool = _current->resource_pool;
    83001cd0:	10f43c23          	sd	a5,280(s0)
}
    83001cd4:	60a6                	ld	ra,72(sp)
    83001cd6:	6406                	ld	s0,64(sp)
    83001cd8:	74e2                	ld	s1,56(sp)
    83001cda:	7942                	ld	s2,48(sp)
    83001cdc:	79a2                	ld	s3,40(sp)
    83001cde:	6161                	addi	sp,sp,80
    83001ce0:	8082                	ret
		new_thread->name[0] = '\0';
    83001ce2:	0c040823          	sb	zero,208(s0)
    83001ce6:	bff1                	j	83001cc2 <z_setup_new_thread+0x132>

0000000083001ce8 <z_impl_k_thread_create>:
{
    83001ce8:	7179                	addi	sp,sp,-48
    83001cea:	ec26                	sd	s1,24(sp)
    83001cec:	f406                	sd	ra,40(sp)
    83001cee:	f022                	sd	s0,32(sp)
	__ASSERT(!arch_is_in_isr(), "Threads may not be created in ISRs");
    83001cf0:	0000a317          	auipc	t1,0xa
    83001cf4:	c4032303          	lw	t1,-960(t1) # 8300b930 <_kernel>
{
    83001cf8:	74e2                	ld	s1,56(sp)
	__ASSERT(!arch_is_in_isr(), "Threads may not be created in ISRs");
    83001cfa:	04030063          	beqz	t1,83001d3a <z_impl_k_thread_create+0x52>
    83001cfe:	00007597          	auipc	a1,0x7
    83001d02:	26a58593          	addi	a1,a1,618 # 83008f68 <levels.0+0x1c8>
    83001d06:	29e00693          	li	a3,670
    83001d0a:	00007617          	auipc	a2,0x7
    83001d0e:	11e60613          	addi	a2,a2,286 # 83008e28 <levels.0+0x88>
    83001d12:	00006517          	auipc	a0,0x6
    83001d16:	50650513          	addi	a0,a0,1286 # 83008218 <__clz_tab+0x1e0>
    83001d1a:	a05fe0ef          	jal	8300071e <assert_print>
    83001d1e:	00007517          	auipc	a0,0x7
    83001d22:	26250513          	addi	a0,a0,610 # 83008f80 <levels.0+0x1e0>
    83001d26:	9f9fe0ef          	jal	8300071e <assert_print>
    83001d2a:	29e00593          	li	a1,670
    83001d2e:	00007517          	auipc	a0,0x7
    83001d32:	0fa50513          	addi	a0,a0,250 # 83008e28 <levels.0+0x88>
    83001d36:	9dffe0ef          	jal	83000714 <assert_post_action>
	z_setup_new_thread(new_thread, stack, stack_size, entry, p1, p2, p3,
    83001d3a:	5342                	lw	t1,48(sp)
    83001d3c:	e402                	sd	zero,8(sp)
    83001d3e:	842a                	mv	s0,a0
    83001d40:	e01a                	sd	t1,0(sp)
    83001d42:	e4fff0ef          	jal	83001b90 <z_setup_new_thread>
	if (!K_TIMEOUT_EQ(delay, K_FOREVER)) {
    83001d46:	57fd                	li	a5,-1
    83001d48:	00f48663          	beq	s1,a5,83001d54 <z_impl_k_thread_create+0x6c>
	if (K_TIMEOUT_EQ(delay, K_NO_WAIT)) {
    83001d4c:	e891                	bnez	s1,83001d60 <z_impl_k_thread_create+0x78>
	z_sched_start(thread);
    83001d4e:	8522                	mv	a0,s0
    83001d50:	1ff000ef          	jal	8300274e <z_sched_start>
}
    83001d54:	70a2                	ld	ra,40(sp)
    83001d56:	8522                	mv	a0,s0
    83001d58:	7402                	ld	s0,32(sp)
    83001d5a:	64e2                	ld	s1,24(sp)
    83001d5c:	6145                	addi	sp,sp,48
    83001d5e:	8082                	ret
    83001d60:	8626                	mv	a2,s1
    83001d62:	00001597          	auipc	a1,0x1
    83001d66:	96058593          	addi	a1,a1,-1696 # 830026c2 <z_thread_timeout>
    83001d6a:	02840513          	addi	a0,s0,40
    83001d6e:	6e8010ef          	jal	83003456 <z_add_timeout>
    83001d72:	b7cd                	j	83001d54 <z_impl_k_thread_create+0x6c>

0000000083001d74 <sys_dlist_remove>:
 * @param node the node to remove
 */

static inline void sys_dlist_remove(sys_dnode_t *node)
{
	sys_dnode_t *const prev = node->prev;
    83001d74:	6518                	ld	a4,8(a0)
	sys_dnode_t *const next = node->next;
    83001d76:	611c                	ld	a5,0(a0)

	prev->next = next;
    83001d78:	e31c                	sd	a5,0(a4)
	next->prev = prev;
    83001d7a:	e798                	sd	a4,8(a5)
	node->next = NULL;
    83001d7c:	00053023          	sd	zero,0(a0)
	node->prev = NULL;
    83001d80:	00053423          	sd	zero,8(a0)
	sys_dnode_init(node);
}
    83001d84:	8082                	ret

0000000083001d86 <update_cache>:
 * CONFIG_NUM_COOP_PRIORITIES > CONFIG_NUM_METAIRQ_PRIORITIES
 */
}

static void update_cache(int preempt_ok)
{
    83001d86:	1101                	addi	sp,sp,-32
    83001d88:	e426                	sd	s1,8(sp)
    83001d8a:	ec06                	sd	ra,24(sp)
    83001d8c:	e822                	sd	s0,16(sp)
	return list->head == list;
    83001d8e:	0000a497          	auipc	s1,0xa
    83001d92:	ba248493          	addi	s1,s1,-1118 # 8300b930 <_kernel>
    83001d96:	60a0                	ld	s0,64(s1)
	return sys_dlist_is_empty(list) ? NULL : list->head;
    83001d98:	0000a797          	auipc	a5,0xa
    83001d9c:	bd878793          	addi	a5,a5,-1064 # 8300b970 <_kernel+0x40>
    83001da0:	00f40363          	beq	s0,a5,83001da6 <update_cache+0x20>
	return (thread != NULL) ? thread : _current_cpu->idle_thread;
    83001da4:	e011                	bnez	s0,83001da8 <update_cache+0x22>
    83001da6:	6c80                	ld	s0,24(s1)
	 */
	if (preempt_ok != 0) {
		return true;
	}

	__ASSERT(_current != NULL, "");
    83001da8:	689c                	ld	a5,16(s1)
	if (preempt_ok != 0) {
    83001daa:	e931                	bnez	a0,83001dfe <update_cache+0x78>
	__ASSERT(_current != NULL, "");
    83001dac:	ef9d                	bnez	a5,83001dea <update_cache+0x64>
    83001dae:	00007597          	auipc	a1,0x7
    83001db2:	22a58593          	addi	a1,a1,554 # 83008fd8 <levels.0+0x238>
    83001db6:	0c800693          	li	a3,200
    83001dba:	00007617          	auipc	a2,0x7
    83001dbe:	1ee60613          	addi	a2,a2,494 # 83008fa8 <levels.0+0x208>
    83001dc2:	00006517          	auipc	a0,0x6
    83001dc6:	45650513          	addi	a0,a0,1110 # 83008218 <__clz_tab+0x1e0>
    83001dca:	955fe0ef          	jal	8300071e <assert_print>
    83001dce:	00007517          	auipc	a0,0x7
    83001dd2:	23250513          	addi	a0,a0,562 # 83009000 <levels.0+0x260>
    83001dd6:	949fe0ef          	jal	8300071e <assert_print>
    83001dda:	0c800593          	li	a1,200
    83001dde:	00007517          	auipc	a0,0x7
    83001de2:	1ca50513          	addi	a0,a0,458 # 83008fa8 <levels.0+0x208>
    83001de6:	92ffe0ef          	jal	83000714 <assert_post_action>

	/* Or if we're pended/suspended/dummy (duh) */
	if (z_is_thread_prevented_from_running(_current)) {
    83001dea:	0197c703          	lbu	a4,25(a5)
    83001dee:	8b7d                	andi	a4,a4,31
    83001df0:	e719                	bnez	a4,83001dfe <update_cache+0x78>
	}

	/* Otherwise we have to be running a preemptible thread or
	 * switching to a metairq
	 */
	if (thread_is_preemptible(_current) || thread_is_metairq(thread)) {
    83001df2:	01a7d683          	lhu	a3,26(a5)
    83001df6:	07f00713          	li	a4,127
    83001dfa:	00d76863          	bltu	a4,a3,83001e0a <update_cache+0x84>
#ifndef CONFIG_SMP
	struct k_thread *thread = next_up();

	if (should_preempt(thread, preempt_ok)) {
#ifdef CONFIG_TIMESLICING
		if (thread != _current) {
    83001dfe:	00f40563          	beq	s0,a5,83001e08 <update_cache+0x82>
			z_reset_time_slice(thread);
    83001e02:	8522                	mv	a0,s0
    83001e04:	406010ef          	jal	8300320a <z_reset_time_slice>
		}
#endif /* CONFIG_TIMESLICING */
		update_metairq_preempt(thread);
		_kernel.ready_q.cache = thread;
    83001e08:	87a2                	mv	a5,s0
    83001e0a:	fc9c                	sd	a5,56(s1)
	 * thread because if the thread gets preempted for whatever
	 * reason the scheduler will make the same decision anyway.
	 */
	_current_cpu->swap_ok = preempt_ok;
#endif /* CONFIG_SMP */
}
    83001e0c:	60e2                	ld	ra,24(sp)
    83001e0e:	6442                	ld	s0,16(sp)
    83001e10:	64a2                	ld	s1,8(sp)
    83001e12:	6105                	addi	sp,sp,32
    83001e14:	8082                	ret

0000000083001e16 <unready_thread>:

	return thread->base.pended_on;
}

static void unready_thread(struct k_thread *thread)
{
    83001e16:	1141                	addi	sp,sp,-16
    83001e18:	e022                	sd	s0,0(sp)
    83001e1a:	e406                	sd	ra,8(sp)
	return (thread->base.thread_state & state) != 0U;
    83001e1c:	01954783          	lbu	a5,25(a0)
    83001e20:	842a                	mv	s0,a0
	if (z_is_thread_queued(thread)) {
    83001e22:	0187971b          	slliw	a4,a5,0x18
    83001e26:	4187571b          	sraiw	a4,a4,0x18
    83001e2a:	04075663          	bgez	a4,83001e76 <unready_thread+0x60>
	thread->base.thread_state &= ~_THREAD_QUEUED;
    83001e2e:	07f7f793          	andi	a5,a5,127
    83001e32:	00f50ca3          	sb	a5,25(a0)
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    83001e36:	0000a797          	auipc	a5,0xa
    83001e3a:	8aa78793          	addi	a5,a5,-1878 # 8300b6e0 <z_idle_threads>
    83001e3e:	02f51a63          	bne	a0,a5,83001e72 <unready_thread+0x5c>
    83001e42:	00007597          	auipc	a1,0x7
    83001e46:	1ee58593          	addi	a1,a1,494 # 83009030 <levels.0+0x290>
    83001e4a:	00006517          	auipc	a0,0x6
    83001e4e:	3ce50513          	addi	a0,a0,974 # 83008218 <__clz_tab+0x1e0>
    83001e52:	08000693          	li	a3,128
    83001e56:	00007617          	auipc	a2,0x7
    83001e5a:	1b260613          	addi	a2,a2,434 # 83009008 <levels.0+0x268>
    83001e5e:	8c1fe0ef          	jal	8300071e <assert_print>
    83001e62:	08000593          	li	a1,128
    83001e66:	00007517          	auipc	a0,0x7
    83001e6a:	1a250513          	addi	a0,a0,418 # 83009008 <levels.0+0x268>
    83001e6e:	8a7fe0ef          	jal	83000714 <assert_post_action>

static ALWAYS_INLINE void z_priq_dumb_remove(sys_dlist_t *pq, struct k_thread *thread)
{
	ARG_UNUSED(pq);

	sys_dlist_remove(&thread->base.qnode_dlist);
    83001e72:	f03ff0ef          	jal	83001d74 <sys_dlist_remove>
		dequeue_thread(thread);
	}
	update_cache(thread == _current);
    83001e76:	0000a517          	auipc	a0,0xa
    83001e7a:	aca53503          	ld	a0,-1334(a0) # 8300b940 <_kernel+0x10>
    83001e7e:	8d01                	sub	a0,a0,s0
}
    83001e80:	6402                	ld	s0,0(sp)
    83001e82:	60a2                	ld	ra,8(sp)
	update_cache(thread == _current);
    83001e84:	00153513          	seqz	a0,a0
}
    83001e88:	0141                	addi	sp,sp,16
	update_cache(thread == _current);
    83001e8a:	bdf5                	j	83001d86 <update_cache>

0000000083001e8c <unpend_thread_no_timeout>:
		pend_locked(thread, wait_q, timeout);
	}
}

static inline void unpend_thread_no_timeout(struct k_thread *thread)
{
    83001e8c:	1141                	addi	sp,sp,-16
    83001e8e:	e406                	sd	ra,8(sp)
    83001e90:	e022                	sd	s0,0(sp)
	__ASSERT_NO_MSG(thread->base.pended_on);
    83001e92:	691c                	ld	a5,16(a0)
    83001e94:	eb8d                	bnez	a5,83001ec6 <unpend_thread_no_timeout+0x3a>
    83001e96:	00007597          	auipc	a1,0x7
    83001e9a:	1c258593          	addi	a1,a1,450 # 83009058 <levels.0+0x2b8>
    83001e9e:	00006517          	auipc	a0,0x6
    83001ea2:	37a50513          	addi	a0,a0,890 # 83008218 <__clz_tab+0x1e0>
    83001ea6:	23600693          	li	a3,566
    83001eaa:	00007617          	auipc	a2,0x7
    83001eae:	15e60613          	addi	a2,a2,350 # 83009008 <levels.0+0x268>
    83001eb2:	86dfe0ef          	jal	8300071e <assert_print>
    83001eb6:	23600593          	li	a1,566
    83001eba:	00007517          	auipc	a0,0x7
    83001ebe:	14e50513          	addi	a0,a0,334 # 83009008 <levels.0+0x268>
    83001ec2:	853fe0ef          	jal	83000714 <assert_post_action>
    83001ec6:	842a                	mv	s0,a0
    83001ec8:	eadff0ef          	jal	83001d74 <sys_dlist_remove>
	thread->base.thread_state &= ~_THREAD_PENDING;
    83001ecc:	01944783          	lbu	a5,25(s0)
	_priq_wait_remove(&pended_on_thread(thread)->waitq, thread);
	z_mark_thread_as_not_pending(thread);
	thread->base.pended_on = NULL;
    83001ed0:	00043823          	sd	zero,16(s0)
    83001ed4:	9bf5                	andi	a5,a5,-3
    83001ed6:	00f40ca3          	sb	a5,25(s0)
}
    83001eda:	60a2                	ld	ra,8(sp)
    83001edc:	6402                	ld	s0,0(sp)
    83001ede:	0141                	addi	sp,sp,16
    83001ee0:	8082                	ret

0000000083001ee2 <z_swap>:
{
    83001ee2:	7179                	addi	sp,sp,-48
    83001ee4:	f022                	sd	s0,32(sp)
    83001ee6:	ec26                	sd	s1,24(sp)
    83001ee8:	e84a                	sd	s2,16(sp)
    83001eea:	f406                	sd	ra,40(sp)
    83001eec:	e44e                	sd	s3,8(sp)
	__ASSERT(arch_irq_unlocked(key) ||
    83001eee:	0000a497          	auipc	s1,0xa
    83001ef2:	a4248493          	addi	s1,s1,-1470 # 8300b930 <_kernel>
    83001ef6:	0025f913          	andi	s2,a1,2
    83001efa:	0104b983          	ld	s3,16(s1)
{
    83001efe:	842a                	mv	s0,a0
	__ASSERT(arch_irq_unlocked(key) ||
    83001f00:	04091463          	bnez	s2,83001f48 <z_swap+0x66>
    83001f04:	0199c783          	lbu	a5,25(s3)
    83001f08:	8ba5                	andi	a5,a5,9
    83001f0a:	ef9d                	bnez	a5,83001f48 <z_swap+0x66>
    83001f0c:	00007597          	auipc	a1,0x7
    83001f10:	dfc58593          	addi	a1,a1,-516 # 83008d08 <CSWTCH.279+0xc0>
    83001f14:	06300693          	li	a3,99
    83001f18:	00007617          	auipc	a2,0x7
    83001f1c:	dc060613          	addi	a2,a2,-576 # 83008cd8 <CSWTCH.279+0x90>
    83001f20:	00006517          	auipc	a0,0x6
    83001f24:	2f850513          	addi	a0,a0,760 # 83008218 <__clz_tab+0x1e0>
    83001f28:	ff6fe0ef          	jal	8300071e <assert_print>
    83001f2c:	00007517          	auipc	a0,0x7
    83001f30:	e4c50513          	addi	a0,a0,-436 # 83008d78 <CSWTCH.279+0x130>
    83001f34:	feafe0ef          	jal	8300071e <assert_print>
    83001f38:	06300593          	li	a1,99
    83001f3c:	00007517          	auipc	a0,0x7
    83001f40:	d9c50513          	addi	a0,a0,-612 # 83008cd8 <CSWTCH.279+0x90>
    83001f44:	fd0fe0ef          	jal	83000714 <assert_post_action>
	z_check_stack_sentinel();
    83001f48:	c19ff0ef          	jal	83001b60 <z_check_stack_sentinel>
	old_thread->swap_retval = -EAGAIN;
    83001f4c:	57d5                	li	a5,-11
    83001f4e:	10f9a423          	sw	a5,264(s3)
	if (is_spinlock && lock != NULL && lock != &_sched_spinlock) {
    83001f52:	c829                	beqz	s0,83001fa4 <z_swap+0xc2>
    83001f54:	00007797          	auipc	a5,0x7
    83001f58:	5ac78793          	addi	a5,a5,1452 # 83009500 <_sched_spinlock>
    83001f5c:	0af40063          	beq	s0,a5,83001ffc <z_swap+0x11a>
    83001f60:	8522                	mv	a0,s0
    83001f62:	42a010ef          	jal	8300338c <z_spin_unlock_valid>
    83001f66:	ed1d                	bnez	a0,83001fa4 <z_swap+0xc2>
    83001f68:	00007597          	auipc	a1,0x7
    83001f6c:	b1058593          	addi	a1,a1,-1264 # 83008a78 <plic_config_0+0x98>
    83001f70:	16b00693          	li	a3,363
    83001f74:	00007617          	auipc	a2,0x7
    83001f78:	aa460613          	addi	a2,a2,-1372 # 83008a18 <plic_config_0+0x38>
    83001f7c:	00006517          	auipc	a0,0x6
    83001f80:	29c50513          	addi	a0,a0,668 # 83008218 <__clz_tab+0x1e0>
    83001f84:	f9afe0ef          	jal	8300071e <assert_print>
    83001f88:	85a2                	mv	a1,s0
    83001f8a:	00007517          	auipc	a0,0x7
    83001f8e:	b0650513          	addi	a0,a0,-1274 # 83008a90 <plic_config_0+0xb0>
    83001f92:	f8cfe0ef          	jal	8300071e <assert_print>
    83001f96:	16b00593          	li	a1,363
    83001f9a:	00007517          	auipc	a0,0x7
    83001f9e:	a7e50513          	addi	a0,a0,-1410 # 83008a18 <plic_config_0+0x38>
    83001fa2:	b74d                	j	83001f44 <z_swap+0x62>
    83001fa4:	100177f3          	csrrci	a5,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83001fa8:	00007517          	auipc	a0,0x7
    83001fac:	55850513          	addi	a0,a0,1368 # 83009500 <_sched_spinlock>
    83001fb0:	3c2010ef          	jal	83003372 <z_spin_lock_valid>
    83001fb4:	ed15                	bnez	a0,83001ff0 <z_swap+0x10e>
    83001fb6:	08800693          	li	a3,136
    83001fba:	00007617          	auipc	a2,0x7
    83001fbe:	a5e60613          	addi	a2,a2,-1442 # 83008a18 <plic_config_0+0x38>
    83001fc2:	00007597          	auipc	a1,0x7
    83001fc6:	a8658593          	addi	a1,a1,-1402 # 83008a48 <plic_config_0+0x68>
    83001fca:	00006517          	auipc	a0,0x6
    83001fce:	24e50513          	addi	a0,a0,590 # 83008218 <__clz_tab+0x1e0>
    83001fd2:	f4cfe0ef          	jal	8300071e <assert_print>
    83001fd6:	00007597          	auipc	a1,0x7
    83001fda:	52a58593          	addi	a1,a1,1322 # 83009500 <_sched_spinlock>
    83001fde:	00007517          	auipc	a0,0x7
    83001fe2:	a8250513          	addi	a0,a0,-1406 # 83008a60 <plic_config_0+0x80>
    83001fe6:	f38fe0ef          	jal	8300071e <assert_print>
    83001fea:	08800593          	li	a1,136
    83001fee:	b775                	j	83001f9a <z_swap+0xb8>
	z_spin_lock_set_owner(l);
    83001ff0:	00007517          	auipc	a0,0x7
    83001ff4:	51050513          	addi	a0,a0,1296 # 83009500 <_sched_spinlock>
    83001ff8:	3c2010ef          	jal	830033ba <z_spin_lock_set_owner>
		 */
		signal_pending_ipi();
	}
	return ret;
#else
	return _kernel.ready_q.cache;
    83001ffc:	7c80                	ld	s0,56(s1)
	if (new_thread != old_thread) {
    83001ffe:	07340a63          	beq	s0,s3,83002072 <z_swap+0x190>
		z_reset_time_slice(new_thread);
    83002002:	8522                	mv	a0,s0
		_current_cpu->current = new_thread;
    83002004:	e880                	sd	s0,16(s1)
		z_reset_time_slice(new_thread);
    83002006:	204010ef          	jal	8300320a <z_reset_time_slice>
		z_spin_lock_set_owner(&_sched_spinlock);
    8300200a:	00007517          	auipc	a0,0x7
    8300200e:	4f650513          	addi	a0,a0,1270 # 83009500 <_sched_spinlock>
    83002012:	3a8010ef          	jal	830033ba <z_spin_lock_set_owner>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83002016:	00007517          	auipc	a0,0x7
    8300201a:	4ea50513          	addi	a0,a0,1258 # 83009500 <_sched_spinlock>
		void *newsh = new_thread->switch_handle;
    8300201e:	11043403          	ld	s0,272(s0)
    83002022:	36a010ef          	jal	8300338c <z_spin_unlock_valid>
    83002026:	e515                	bnez	a0,83002052 <z_swap+0x170>
    83002028:	00007597          	auipc	a1,0x7
    8300202c:	a5058593          	addi	a1,a1,-1456 # 83008a78 <plic_config_0+0x98>
    83002030:	16b00693          	li	a3,363
    83002034:	00007617          	auipc	a2,0x7
    83002038:	9e460613          	addi	a2,a2,-1564 # 83008a18 <plic_config_0+0x38>
    8300203c:	00006517          	auipc	a0,0x6
    83002040:	1dc50513          	addi	a0,a0,476 # 83008218 <__clz_tab+0x1e0>
    83002044:	edafe0ef          	jal	8300071e <assert_print>
    83002048:	00007597          	auipc	a1,0x7
    8300204c:	4b858593          	addi	a1,a1,1208 # 83009500 <_sched_spinlock>
    83002050:	bf2d                	j	83001f8a <z_swap+0xa8>
    83002052:	85ce                	mv	a1,s3
    83002054:	8522                	mv	a0,s0
    83002056:	98afe0ef          	jal	830001e0 <z_riscv_switch>
	csr_set(xstatus, (key & XSTATUS_IEN));
    8300205a:	10092073          	csrs	sstatus,s2
	return _current->swap_retval;
    8300205e:	689c                	ld	a5,16(s1)
}
    83002060:	70a2                	ld	ra,40(sp)
    83002062:	7402                	ld	s0,32(sp)
    83002064:	1087a503          	lw	a0,264(a5)
    83002068:	64e2                	ld	s1,24(sp)
    8300206a:	6942                	ld	s2,16(sp)
    8300206c:	69a2                	ld	s3,8(sp)
    8300206e:	6145                	addi	sp,sp,48
    83002070:	8082                	ret
    83002072:	00007517          	auipc	a0,0x7
    83002076:	48e50513          	addi	a0,a0,1166 # 83009500 <_sched_spinlock>
    8300207a:	312010ef          	jal	8300338c <z_spin_unlock_valid>
    8300207e:	fd71                	bnez	a0,8300205a <z_swap+0x178>
    83002080:	b765                	j	83002028 <z_swap+0x146>

0000000083002082 <ready_thread>:
	return (thread->base.thread_state & state) != 0U;
    83002082:	01954703          	lbu	a4,25(a0)
	if (!z_is_thread_queued(thread) && z_is_thread_ready(thread)) {
    83002086:	0187179b          	slliw	a5,a4,0x18
    8300208a:	4187d79b          	sraiw	a5,a5,0x18
    8300208e:	0a07c163          	bltz	a5,83002130 <ready_thread+0xae>
	return !((z_is_thread_prevented_from_running(thread)) != 0U ||
    83002092:	01f77793          	andi	a5,a4,31
    83002096:	efc9                	bnez	a5,83002130 <ready_thread+0xae>
	return node->next != NULL;
    83002098:	751c                	ld	a5,40(a0)
    8300209a:	ebd9                	bnez	a5,83002130 <ready_thread+0xae>
	thread->base.thread_state |= _THREAD_QUEUED;
    8300209c:	f8076713          	ori	a4,a4,-128
    830020a0:	00e50ca3          	sb	a4,25(a0)
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    830020a4:	00009717          	auipc	a4,0x9
    830020a8:	63c70713          	addi	a4,a4,1596 # 8300b6e0 <z_idle_threads>
    830020ac:	02e51c63          	bne	a0,a4,830020e4 <ready_thread+0x62>
{
    830020b0:	1141                	addi	sp,sp,-16
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    830020b2:	00007597          	auipc	a1,0x7
    830020b6:	f7e58593          	addi	a1,a1,-130 # 83009030 <levels.0+0x290>
    830020ba:	00006517          	auipc	a0,0x6
    830020be:	15e50513          	addi	a0,a0,350 # 83008218 <__clz_tab+0x1e0>
    830020c2:	07900693          	li	a3,121
    830020c6:	00007617          	auipc	a2,0x7
    830020ca:	f4260613          	addi	a2,a2,-190 # 83009008 <levels.0+0x268>
{
    830020ce:	e406                	sd	ra,8(sp)
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    830020d0:	e4efe0ef          	jal	8300071e <assert_print>
    830020d4:	07900593          	li	a1,121
    830020d8:	00007517          	auipc	a0,0x7
    830020dc:	f3050513          	addi	a0,a0,-208 # 83009008 <levels.0+0x268>
    830020e0:	e34fe0ef          	jal	83000714 <assert_post_action>
	return list->head == list;
    830020e4:	0000a717          	auipc	a4,0xa
    830020e8:	84c70713          	addi	a4,a4,-1972 # 8300b930 <_kernel>
    830020ec:	6334                	ld	a3,64(a4)
	return sys_dlist_is_empty(list) ? NULL : list->head;
    830020ee:	0000a617          	auipc	a2,0xa
    830020f2:	88260613          	addi	a2,a2,-1918 # 8300b970 <_kernel+0x40>
    830020f6:	00c68363          	beq	a3,a2,830020fc <ready_thread+0x7a>
    830020fa:	87b6                	mv	a5,a3
	return (node == list->tail) ? NULL : node->next;
    830020fc:	6734                	ld	a3,72(a4)
static ALWAYS_INLINE void z_priq_dumb_add(sys_dlist_t *pq,
					  struct k_thread *thread)
{
	struct k_thread *t;

	SYS_DLIST_FOR_EACH_CONTAINER(pq, t, base.qnode_dlist) {
    830020fe:	e791                	bnez	a5,8300210a <ready_thread+0x88>
	node->next = list;
    83002100:	e110                	sd	a2,0(a0)
	node->prev = tail;
    83002102:	e514                	sd	a3,8(a0)
	tail->next = node;
    83002104:	e288                	sd	a0,0(a3)
	list->tail = node;
    83002106:	e728                	sd	a0,72(a4)
}
    83002108:	a831                	j	83002124 <ready_thread+0xa2>
	int32_t b1 = thread_1->base.prio;
    8300210a:	01a50583          	lb	a1,26(a0)
	int32_t b2 = thread_2->base.prio;
    8300210e:	01a78803          	lb	a6,26(a5)
	if (b1 != b2) {
    83002112:	01058b63          	beq	a1,a6,83002128 <ready_thread+0xa6>
		if (z_sched_prio_cmp(thread, t) > 0) {
    83002116:	0105d963          	bge	a1,a6,83002128 <ready_thread+0xa6>
	sys_dnode_t *const prev = successor->prev;
    8300211a:	6798                	ld	a4,8(a5)
	node->next = successor;
    8300211c:	e11c                	sd	a5,0(a0)
	node->prev = prev;
    8300211e:	e518                	sd	a4,8(a0)
	prev->next = node;
    83002120:	e308                	sd	a0,0(a4)
	successor->prev = node;
    83002122:	e788                	sd	a0,8(a5)
		update_cache(0);
    83002124:	4501                	li	a0,0
    83002126:	b185                	j	83001d86 <update_cache>
	return (node == list->tail) ? NULL : node->next;
    83002128:	fcd78ce3          	beq	a5,a3,83002100 <ready_thread+0x7e>
    8300212c:	639c                	ld	a5,0(a5)
    8300212e:	bfc1                	j	830020fe <ready_thread+0x7c>
    83002130:	8082                	ret

0000000083002132 <z_thread_halt>:
{
    83002132:	7179                	addi	sp,sp,-48
    83002134:	f022                	sd	s0,32(sp)
    83002136:	ec26                	sd	s1,24(sp)
    83002138:	e84a                	sd	s2,16(sp)
    8300213a:	e44e                	sd	s3,8(sp)
    8300213c:	f406                	sd	ra,40(sp)
    8300213e:	e052                	sd	s4,0(sp)
    83002140:	842a                	mv	s0,a0
    83002142:	0005849b          	sext.w	s1,a1
    83002146:	89b2                	mv	s3,a2
		halt_thread(thread, terminate ? _THREAD_DEAD : _THREAD_SUSPENDED);
    83002148:	4941                	li	s2,16
    8300214a:	c211                	beqz	a2,8300214e <z_thread_halt+0x1c>
    8300214c:	4921                	li	s2,8
	bool dummify = false;

	/* We hold the lock, and the thread is known not to be running
	 * anywhere.
	 */
	if ((thread->base.thread_state & new_state) == 0U) {
    8300214e:	01944783          	lbu	a5,25(s0)
    83002152:	00f97733          	and	a4,s2,a5
    83002156:	e33d                	bnez	a4,830021bc <z_thread_halt+0x8a>
		thread->base.thread_state |= new_state;
    83002158:	00f967b3          	or	a5,s2,a5
		if (z_is_thread_queued(thread)) {
    8300215c:	0187971b          	slliw	a4,a5,0x18
    83002160:	4187571b          	sraiw	a4,a4,0x18
    83002164:	0a074863          	bltz	a4,83002214 <z_thread_halt+0xe2>
		thread->base.thread_state |= new_state;
    83002168:	00f40ca3          	sb	a5,25(s0)
			dequeue_thread(thread);
		}

		if (new_state == _THREAD_DEAD) {
    8300216c:	47a1                	li	a5,8
			}
		}
#ifdef CONFIG_SMP
		unpend_all(&thread->halt_queue);
#endif /* CONFIG_SMP */
		update_cache(1);
    8300216e:	4505                	li	a0,1
		if (new_state == _THREAD_DEAD) {
    83002170:	02f91c63          	bne	s2,a5,830021a8 <z_thread_halt+0x76>
			if (thread->base.pended_on != NULL) {
    83002174:	681c                	ld	a5,16(s0)
    83002176:	c781                	beqz	a5,8300217e <z_thread_halt+0x4c>
				unpend_thread_no_timeout(thread);
    83002178:	8522                	mv	a0,s0
    8300217a:	d13ff0ef          	jal	83001e8c <unpend_thread_no_timeout>
}

static inline int z_abort_thread_timeout(struct k_thread *thread)
{
	return z_abort_timeout(&thread->base.timeout);
    8300217e:	02840513          	addi	a0,s0,40
    83002182:	492010ef          	jal	83003614 <z_abort_timeout>
}

static inline struct k_thread *z_waitq_head(_wait_q_t *w)
{
	return (struct k_thread *)sys_dlist_peek_head(&w->waitq);
    83002186:	0c040a13          	addi	s4,s0,192
	return list->head == list;
    8300218a:	0c043903          	ld	s2,192(s0)
	return sys_dlist_is_empty(list) ? NULL : list->head;
    8300218e:	012a0463          	beq	s4,s2,83002196 <z_thread_halt+0x64>
	for (thread = z_waitq_head(wait_q); thread != NULL; thread = z_waitq_head(wait_q)) {
    83002192:	0c091763          	bnez	s2,83002260 <z_thread_halt+0x12e>
			if (thread == _current && arch_is_in_isr()) {
    83002196:	00009917          	auipc	s2,0x9
    8300219a:	79a90913          	addi	s2,s2,1946 # 8300b930 <_kernel>
    8300219e:	01093783          	ld	a5,16(s2)
		update_cache(1);
    830021a2:	4505                	li	a0,1
			if (thread == _current && arch_is_in_isr()) {
    830021a4:	0cf40b63          	beq	s0,a5,8300227a <z_thread_halt+0x148>
		update_cache(1);
    830021a8:	bdfff0ef          	jal	83001d86 <update_cache>

static ALWAYS_INLINE void z_barrier_dmem_fence_full(void)
{
#if defined(__GNUC__)
	/* GCC-ism */
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
    830021ac:	0ff0000f          	fence
	thread->base.thread_state &= ~(_THREAD_ABORTING | _THREAD_SUSPENDING);
    830021b0:	01944783          	lbu	a5,25(s0)
    830021b4:	f9f7f793          	andi	a5,a5,-97
    830021b8:	00f40ca3          	sb	a5,25(s0)
		if ((thread == _current) && !arch_is_in_isr()) {
    830021bc:	00009797          	auipc	a5,0x9
    830021c0:	77478793          	addi	a5,a5,1908 # 8300b930 <_kernel>
    830021c4:	6b98                	ld	a4,16(a5)
    830021c6:	0e871a63          	bne	a4,s0,830022ba <z_thread_halt+0x188>
    830021ca:	439c                	lw	a5,0(a5)
    830021cc:	0e079763          	bnez	a5,830022ba <z_thread_halt+0x188>
			z_swap(&_sched_spinlock, key);
    830021d0:	85a6                	mv	a1,s1
    830021d2:	00007517          	auipc	a0,0x7
    830021d6:	32e50513          	addi	a0,a0,814 # 83009500 <_sched_spinlock>
    830021da:	d09ff0ef          	jal	83001ee2 <z_swap>
			__ASSERT(!terminate, "aborted _current back from dead");
    830021de:	12098963          	beqz	s3,83002310 <z_thread_halt+0x1de>
    830021e2:	00007597          	auipc	a1,0x7
    830021e6:	e8e58593          	addi	a1,a1,-370 # 83009070 <levels.0+0x2d0>
    830021ea:	1ed00693          	li	a3,493
    830021ee:	00007617          	auipc	a2,0x7
    830021f2:	e1a60613          	addi	a2,a2,-486 # 83009008 <levels.0+0x268>
    830021f6:	00006517          	auipc	a0,0x6
    830021fa:	02250513          	addi	a0,a0,34 # 83008218 <__clz_tab+0x1e0>
    830021fe:	d20fe0ef          	jal	8300071e <assert_print>
    83002202:	00007517          	auipc	a0,0x7
    83002206:	e7e50513          	addi	a0,a0,-386 # 83009080 <levels.0+0x2e0>
    8300220a:	d14fe0ef          	jal	8300071e <assert_print>
    8300220e:	1ed00593          	li	a1,493
    83002212:	a82d                	j	8300224c <z_thread_halt+0x11a>
	thread->base.thread_state &= ~_THREAD_QUEUED;
    83002214:	07f7f793          	andi	a5,a5,127
    83002218:	00f40ca3          	sb	a5,25(s0)
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    8300221c:	00009797          	auipc	a5,0x9
    83002220:	4c478793          	addi	a5,a5,1220 # 8300b6e0 <z_idle_threads>
    83002224:	02f41a63          	bne	s0,a5,83002258 <z_thread_halt+0x126>
    83002228:	00007597          	auipc	a1,0x7
    8300222c:	e0858593          	addi	a1,a1,-504 # 83009030 <levels.0+0x290>
    83002230:	08000693          	li	a3,128
    83002234:	00007617          	auipc	a2,0x7
    83002238:	dd460613          	addi	a2,a2,-556 # 83009008 <levels.0+0x268>
    8300223c:	00006517          	auipc	a0,0x6
    83002240:	fdc50513          	addi	a0,a0,-36 # 83008218 <__clz_tab+0x1e0>
    83002244:	cdafe0ef          	jal	8300071e <assert_print>
    83002248:	08000593          	li	a1,128
			__ASSERT(!terminate, "aborted _current back from dead");
    8300224c:	00007517          	auipc	a0,0x7
    83002250:	dbc50513          	addi	a0,a0,-580 # 83009008 <levels.0+0x268>
    83002254:	cc0fe0ef          	jal	83000714 <assert_post_action>
	sys_dlist_remove(&thread->base.qnode_dlist);
    83002258:	8522                	mv	a0,s0
    8300225a:	b1bff0ef          	jal	83001d74 <sys_dlist_remove>
}
    8300225e:	b739                	j	8300216c <z_thread_halt+0x3a>
		unpend_thread_no_timeout(thread);
    83002260:	854a                	mv	a0,s2
    83002262:	c2bff0ef          	jal	83001e8c <unpend_thread_no_timeout>
    83002266:	02890513          	addi	a0,s2,40
    8300226a:	3aa010ef          	jal	83003614 <z_abort_timeout>
		ready_thread(thread);
    8300226e:	854a                	mv	a0,s2
 * z_swap() is in use it's a simple inline provided by the kernel.
 */
static ALWAYS_INLINE void
arch_thread_return_value_set(struct k_thread *thread, unsigned int value)
{
	thread->swap_retval = value;
    83002270:	10092423          	sw	zero,264(s2)
    83002274:	e0fff0ef          	jal	83002082 <ready_thread>
    83002278:	bf09                	j	8300218a <z_thread_halt+0x58>
			if (thread == _current && arch_is_in_isr()) {
    8300227a:	00092783          	lw	a5,0(s2)
    8300227e:	d78d                	beqz	a5,830021a8 <z_thread_halt+0x76>
		update_cache(1);
    83002280:	b07ff0ef          	jal	83001d86 <update_cache>
		 * never be used, as our thread is flagged dead, but
		 * it must not be NULL otherwise join can deadlock).
		 */
		if (dummify && !IS_ENABLED(CONFIG_ARCH_POSIX)) {
#ifdef CONFIG_USE_SWITCH
			_current->switch_handle = _current;
    83002284:	01093783          	ld	a5,16(s2)
	dummy_thread->base.user_options = K_ESSENTIAL;
    83002288:	10100713          	li	a4,257
    8300228c:	10f7b823          	sd	a5,272(a5)
    83002290:	00009797          	auipc	a5,0x9
    83002294:	6f078793          	addi	a5,a5,1776 # 8300b980 <_thread_dummy>
    83002298:	00e79c23          	sh	a4,24(a5)
	_current_cpu->current = dummy_thread;
    8300229c:	00f93823          	sd	a5,16(s2)
	dummy_thread->stack_info.start = 0U;
    830022a0:	00009717          	auipc	a4,0x9
    830022a4:	7c073823          	sd	zero,2000(a4) # 8300ba70 <_thread_dummy+0xf0>
	dummy_thread->stack_info.size = 0U;
    830022a8:	00009717          	auipc	a4,0x9
    830022ac:	7c073823          	sd	zero,2000(a4) # 8300ba78 <_thread_dummy+0xf8>
	dummy_thread->resource_pool = NULL;
    830022b0:	00009717          	auipc	a4,0x9
    830022b4:	7e073423          	sd	zero,2024(a4) # 8300ba98 <_thread_dummy+0x118>
}
    830022b8:	bdd5                	j	830021ac <z_thread_halt+0x7a>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    830022ba:	00007517          	auipc	a0,0x7
    830022be:	24650513          	addi	a0,a0,582 # 83009500 <_sched_spinlock>
    830022c2:	0ca010ef          	jal	8300338c <z_spin_unlock_valid>
    830022c6:	e131                	bnez	a0,8300230a <z_thread_halt+0x1d8>
    830022c8:	13100693          	li	a3,305
    830022cc:	00006617          	auipc	a2,0x6
    830022d0:	74c60613          	addi	a2,a2,1868 # 83008a18 <plic_config_0+0x38>
    830022d4:	00006597          	auipc	a1,0x6
    830022d8:	7a458593          	addi	a1,a1,1956 # 83008a78 <plic_config_0+0x98>
    830022dc:	00006517          	auipc	a0,0x6
    830022e0:	f3c50513          	addi	a0,a0,-196 # 83008218 <__clz_tab+0x1e0>
    830022e4:	c3afe0ef          	jal	8300071e <assert_print>
    830022e8:	00007597          	auipc	a1,0x7
    830022ec:	21858593          	addi	a1,a1,536 # 83009500 <_sched_spinlock>
    830022f0:	00006517          	auipc	a0,0x6
    830022f4:	7a050513          	addi	a0,a0,1952 # 83008a90 <plic_config_0+0xb0>
    830022f8:	c26fe0ef          	jal	8300071e <assert_print>
    830022fc:	13100593          	li	a1,305
    83002300:	00006517          	auipc	a0,0x6
    83002304:	71850513          	addi	a0,a0,1816 # 83008a18 <plic_config_0+0x38>
    83002308:	b7b1                	j	83002254 <z_thread_halt+0x122>
    8300230a:	8889                	andi	s1,s1,2
    8300230c:	1004a073          	csrs	sstatus,s1
}
    83002310:	70a2                	ld	ra,40(sp)
    83002312:	7402                	ld	s0,32(sp)
    83002314:	64e2                	ld	s1,24(sp)
    83002316:	6942                	ld	s2,16(sp)
    83002318:	69a2                	ld	s3,8(sp)
    8300231a:	6a02                	ld	s4,0(sp)
    8300231c:	6145                	addi	sp,sp,48
    8300231e:	8082                	ret

0000000083002320 <move_thread_to_end_of_prio_q>:
{
    83002320:	1101                	addi	sp,sp,-32
    83002322:	e822                	sd	s0,16(sp)
    83002324:	e426                	sd	s1,8(sp)
    83002326:	ec06                	sd	ra,24(sp)
	return (thread->base.thread_state & state) != 0U;
    83002328:	01954783          	lbu	a5,25(a0)
    8300232c:	842a                	mv	s0,a0
    8300232e:	00009497          	auipc	s1,0x9
    83002332:	3b248493          	addi	s1,s1,946 # 8300b6e0 <z_idle_threads>
	if (z_is_thread_queued(thread)) {
    83002336:	0187971b          	slliw	a4,a5,0x18
    8300233a:	4187571b          	sraiw	a4,a4,0x18
    8300233e:	04075263          	bgez	a4,83002382 <move_thread_to_end_of_prio_q+0x62>
	thread->base.thread_state &= ~_THREAD_QUEUED;
    83002342:	07f7f793          	andi	a5,a5,127
    83002346:	00f50ca3          	sb	a5,25(a0)
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    8300234a:	02951a63          	bne	a0,s1,8300237e <move_thread_to_end_of_prio_q+0x5e>
    8300234e:	00007597          	auipc	a1,0x7
    83002352:	ce258593          	addi	a1,a1,-798 # 83009030 <levels.0+0x290>
    83002356:	08000693          	li	a3,128
    8300235a:	00007617          	auipc	a2,0x7
    8300235e:	cae60613          	addi	a2,a2,-850 # 83009008 <levels.0+0x268>
    83002362:	00006517          	auipc	a0,0x6
    83002366:	eb650513          	addi	a0,a0,-330 # 83008218 <__clz_tab+0x1e0>
    8300236a:	bb4fe0ef          	jal	8300071e <assert_print>
    8300236e:	08000593          	li	a1,128
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    83002372:	00007517          	auipc	a0,0x7
    83002376:	c9650513          	addi	a0,a0,-874 # 83009008 <levels.0+0x268>
    8300237a:	b9afe0ef          	jal	83000714 <assert_post_action>
    8300237e:	9f7ff0ef          	jal	83001d74 <sys_dlist_remove>
	thread->base.thread_state |= _THREAD_QUEUED;
    83002382:	01944783          	lbu	a5,25(s0)
    83002386:	f807e793          	ori	a5,a5,-128
    8300238a:	00f40ca3          	sb	a5,25(s0)
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    8300238e:	02941563          	bne	s0,s1,830023b8 <move_thread_to_end_of_prio_q+0x98>
    83002392:	00007597          	auipc	a1,0x7
    83002396:	c9e58593          	addi	a1,a1,-866 # 83009030 <levels.0+0x290>
    8300239a:	07900693          	li	a3,121
    8300239e:	00007617          	auipc	a2,0x7
    830023a2:	c6a60613          	addi	a2,a2,-918 # 83009008 <levels.0+0x268>
    830023a6:	00006517          	auipc	a0,0x6
    830023aa:	e7250513          	addi	a0,a0,-398 # 83008218 <__clz_tab+0x1e0>
    830023ae:	b70fe0ef          	jal	8300071e <assert_print>
    830023b2:	07900593          	li	a1,121
    830023b6:	bf75                	j	83002372 <move_thread_to_end_of_prio_q+0x52>
	return list->head == list;
    830023b8:	00009717          	auipc	a4,0x9
    830023bc:	57870713          	addi	a4,a4,1400 # 8300b930 <_kernel>
    830023c0:	633c                	ld	a5,64(a4)
	return sys_dlist_is_empty(list) ? NULL : list->head;
    830023c2:	00009617          	auipc	a2,0x9
    830023c6:	5ae60613          	addi	a2,a2,1454 # 8300b970 <_kernel+0x40>
    830023ca:	00c79363          	bne	a5,a2,830023d0 <move_thread_to_end_of_prio_q+0xb0>
    830023ce:	4781                	li	a5,0
	return (node == list->tail) ? NULL : node->next;
    830023d0:	6734                	ld	a3,72(a4)
	SYS_DLIST_FOR_EACH_CONTAINER(pq, t, base.qnode_dlist) {
    830023d2:	e791                	bnez	a5,830023de <move_thread_to_end_of_prio_q+0xbe>
	node->next = list;
    830023d4:	e010                	sd	a2,0(s0)
	node->prev = tail;
    830023d6:	e414                	sd	a3,8(s0)
	tail->next = node;
    830023d8:	e280                	sd	s0,0(a3)
	list->tail = node;
    830023da:	e720                	sd	s0,72(a4)
}
    830023dc:	a831                	j	830023f8 <move_thread_to_end_of_prio_q+0xd8>
	int32_t b1 = thread_1->base.prio;
    830023de:	01a40583          	lb	a1,26(s0)
	int32_t b2 = thread_2->base.prio;
    830023e2:	01a78503          	lb	a0,26(a5)
	if (b1 != b2) {
    830023e6:	02a58363          	beq	a1,a0,8300240c <move_thread_to_end_of_prio_q+0xec>
		if (z_sched_prio_cmp(thread, t) > 0) {
    830023ea:	02a5d163          	bge	a1,a0,8300240c <move_thread_to_end_of_prio_q+0xec>
	sys_dnode_t *const prev = successor->prev;
    830023ee:	6794                	ld	a3,8(a5)
	node->next = successor;
    830023f0:	e01c                	sd	a5,0(s0)
	node->prev = prev;
    830023f2:	e414                	sd	a3,8(s0)
	prev->next = node;
    830023f4:	e280                	sd	s0,0(a3)
	successor->prev = node;
    830023f6:	e780                	sd	s0,8(a5)
	update_cache(thread == _current);
    830023f8:	6b08                	ld	a0,16(a4)
}
    830023fa:	60e2                	ld	ra,24(sp)
    830023fc:	64a2                	ld	s1,8(sp)
	update_cache(thread == _current);
    830023fe:	8d01                	sub	a0,a0,s0
}
    83002400:	6442                	ld	s0,16(sp)
	update_cache(thread == _current);
    83002402:	00153513          	seqz	a0,a0
}
    83002406:	6105                	addi	sp,sp,32
	update_cache(thread == _current);
    83002408:	97fff06f          	j	83001d86 <update_cache>
	return (node == list->tail) ? NULL : node->next;
    8300240c:	fcf684e3          	beq	a3,a5,830023d4 <move_thread_to_end_of_prio_q+0xb4>
    83002410:	639c                	ld	a5,0(a5)
    83002412:	b7c1                	j	830023d2 <move_thread_to_end_of_prio_q+0xb2>

0000000083002414 <z_ready_thread>:
{
    83002414:	1101                	addi	sp,sp,-32
    83002416:	e426                	sd	s1,8(sp)
    83002418:	ec06                	sd	ra,24(sp)
    8300241a:	e822                	sd	s0,16(sp)
    8300241c:	84aa                	mv	s1,a0
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    8300241e:	10017473          	csrrci	s0,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83002422:	00007517          	auipc	a0,0x7
    83002426:	0de50513          	addi	a0,a0,222 # 83009500 <_sched_spinlock>
    8300242a:	749000ef          	jal	83003372 <z_spin_lock_valid>
    8300242e:	e139                	bnez	a0,83002474 <z_ready_thread+0x60>
    83002430:	08800693          	li	a3,136
    83002434:	00006617          	auipc	a2,0x6
    83002438:	5e460613          	addi	a2,a2,1508 # 83008a18 <plic_config_0+0x38>
    8300243c:	00006597          	auipc	a1,0x6
    83002440:	60c58593          	addi	a1,a1,1548 # 83008a48 <plic_config_0+0x68>
    83002444:	00006517          	auipc	a0,0x6
    83002448:	dd450513          	addi	a0,a0,-556 # 83008218 <__clz_tab+0x1e0>
    8300244c:	ad2fe0ef          	jal	8300071e <assert_print>
    83002450:	00007597          	auipc	a1,0x7
    83002454:	0b058593          	addi	a1,a1,176 # 83009500 <_sched_spinlock>
    83002458:	00006517          	auipc	a0,0x6
    8300245c:	60850513          	addi	a0,a0,1544 # 83008a60 <plic_config_0+0x80>
    83002460:	abefe0ef          	jal	8300071e <assert_print>
    83002464:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83002468:	00006517          	auipc	a0,0x6
    8300246c:	5b050513          	addi	a0,a0,1456 # 83008a18 <plic_config_0+0x38>
    83002470:	aa4fe0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    83002474:	00007517          	auipc	a0,0x7
    83002478:	08c50513          	addi	a0,a0,140 # 83009500 <_sched_spinlock>
    8300247c:	73f000ef          	jal	830033ba <z_spin_lock_set_owner>
			ready_thread(thread);
    83002480:	8526                	mv	a0,s1
    83002482:	c01ff0ef          	jal	83002082 <ready_thread>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83002486:	00007517          	auipc	a0,0x7
    8300248a:	07a50513          	addi	a0,a0,122 # 83009500 <_sched_spinlock>
    8300248e:	6ff000ef          	jal	8300338c <z_spin_unlock_valid>
    83002492:	ed15                	bnez	a0,830024ce <z_ready_thread+0xba>
    83002494:	13100693          	li	a3,305
    83002498:	00006617          	auipc	a2,0x6
    8300249c:	58060613          	addi	a2,a2,1408 # 83008a18 <plic_config_0+0x38>
    830024a0:	00006597          	auipc	a1,0x6
    830024a4:	5d858593          	addi	a1,a1,1496 # 83008a78 <plic_config_0+0x98>
    830024a8:	00006517          	auipc	a0,0x6
    830024ac:	d7050513          	addi	a0,a0,-656 # 83008218 <__clz_tab+0x1e0>
    830024b0:	a6efe0ef          	jal	8300071e <assert_print>
    830024b4:	00007597          	auipc	a1,0x7
    830024b8:	04c58593          	addi	a1,a1,76 # 83009500 <_sched_spinlock>
    830024bc:	00006517          	auipc	a0,0x6
    830024c0:	5d450513          	addi	a0,a0,1492 # 83008a90 <plic_config_0+0xb0>
    830024c4:	a5afe0ef          	jal	8300071e <assert_print>
    830024c8:	13100593          	li	a1,305
    830024cc:	bf71                	j	83002468 <z_ready_thread+0x54>
    830024ce:	2401                	sext.w	s0,s0
	csr_set(xstatus, (key & XSTATUS_IEN));
    830024d0:	8809                	andi	s0,s0,2
    830024d2:	10042073          	csrs	sstatus,s0
}
    830024d6:	60e2                	ld	ra,24(sp)
    830024d8:	6442                	ld	s0,16(sp)
    830024da:	64a2                	ld	s1,8(sp)
    830024dc:	6105                	addi	sp,sp,32
    830024de:	8082                	ret

00000000830024e0 <z_impl_k_thread_suspend>:
{
    830024e0:	1101                	addi	sp,sp,-32
    830024e2:	e426                	sd	s1,8(sp)
    830024e4:	84aa                	mv	s1,a0
    830024e6:	02850513          	addi	a0,a0,40
    830024ea:	ec06                	sd	ra,24(sp)
    830024ec:	e822                	sd	s0,16(sp)
    830024ee:	126010ef          	jal	83003614 <z_abort_timeout>
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    830024f2:	10017473          	csrrci	s0,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    830024f6:	00007517          	auipc	a0,0x7
    830024fa:	00a50513          	addi	a0,a0,10 # 83009500 <_sched_spinlock>
    830024fe:	675000ef          	jal	83003372 <z_spin_lock_valid>
    83002502:	e139                	bnez	a0,83002548 <z_impl_k_thread_suspend+0x68>
    83002504:	08800693          	li	a3,136
    83002508:	00006617          	auipc	a2,0x6
    8300250c:	51060613          	addi	a2,a2,1296 # 83008a18 <plic_config_0+0x38>
    83002510:	00006597          	auipc	a1,0x6
    83002514:	53858593          	addi	a1,a1,1336 # 83008a48 <plic_config_0+0x68>
    83002518:	00006517          	auipc	a0,0x6
    8300251c:	d0050513          	addi	a0,a0,-768 # 83008218 <__clz_tab+0x1e0>
    83002520:	9fefe0ef          	jal	8300071e <assert_print>
    83002524:	00007597          	auipc	a1,0x7
    83002528:	fdc58593          	addi	a1,a1,-36 # 83009500 <_sched_spinlock>
    8300252c:	00006517          	auipc	a0,0x6
    83002530:	53450513          	addi	a0,a0,1332 # 83008a60 <plic_config_0+0x80>
    83002534:	9eafe0ef          	jal	8300071e <assert_print>
    83002538:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    8300253c:	00006517          	auipc	a0,0x6
    83002540:	4dc50513          	addi	a0,a0,1244 # 83008a18 <plic_config_0+0x38>
    83002544:	9d0fe0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    83002548:	00007517          	auipc	a0,0x7
    8300254c:	fb850513          	addi	a0,a0,-72 # 83009500 <_sched_spinlock>
    83002550:	66b000ef          	jal	830033ba <z_spin_lock_set_owner>
	if ((thread->base.thread_state & _THREAD_SUSPENDED) != 0U) {
    83002554:	0194c783          	lbu	a5,25(s1)
    83002558:	2401                	sext.w	s0,s0
    8300255a:	8bc1                	andi	a5,a5,16
    8300255c:	cfa9                	beqz	a5,830025b6 <z_impl_k_thread_suspend+0xd6>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    8300255e:	00007517          	auipc	a0,0x7
    83002562:	fa250513          	addi	a0,a0,-94 # 83009500 <_sched_spinlock>
    83002566:	627000ef          	jal	8300338c <z_spin_unlock_valid>
    8300256a:	ed15                	bnez	a0,830025a6 <z_impl_k_thread_suspend+0xc6>
    8300256c:	13100693          	li	a3,305
    83002570:	00006617          	auipc	a2,0x6
    83002574:	4a860613          	addi	a2,a2,1192 # 83008a18 <plic_config_0+0x38>
    83002578:	00006597          	auipc	a1,0x6
    8300257c:	50058593          	addi	a1,a1,1280 # 83008a78 <plic_config_0+0x98>
    83002580:	00006517          	auipc	a0,0x6
    83002584:	c9850513          	addi	a0,a0,-872 # 83008218 <__clz_tab+0x1e0>
    83002588:	996fe0ef          	jal	8300071e <assert_print>
    8300258c:	00007597          	auipc	a1,0x7
    83002590:	f7458593          	addi	a1,a1,-140 # 83009500 <_sched_spinlock>
    83002594:	00006517          	auipc	a0,0x6
    83002598:	4fc50513          	addi	a0,a0,1276 # 83008a90 <plic_config_0+0xb0>
    8300259c:	982fe0ef          	jal	8300071e <assert_print>
    830025a0:	13100593          	li	a1,305
    830025a4:	bf61                	j	8300253c <z_impl_k_thread_suspend+0x5c>
	csr_set(xstatus, (key & XSTATUS_IEN));
    830025a6:	8809                	andi	s0,s0,2
    830025a8:	10042073          	csrs	sstatus,s0
}
    830025ac:	60e2                	ld	ra,24(sp)
    830025ae:	6442                	ld	s0,16(sp)
    830025b0:	64a2                	ld	s1,8(sp)
    830025b2:	6105                	addi	sp,sp,32
    830025b4:	8082                	ret
	z_thread_halt(thread, key, false);
    830025b6:	85a2                	mv	a1,s0
}
    830025b8:	6442                	ld	s0,16(sp)
    830025ba:	60e2                	ld	ra,24(sp)
	z_thread_halt(thread, key, false);
    830025bc:	8526                	mv	a0,s1
}
    830025be:	64a2                	ld	s1,8(sp)
	z_thread_halt(thread, key, false);
    830025c0:	4601                	li	a2,0
}
    830025c2:	6105                	addi	sp,sp,32
	z_thread_halt(thread, key, false);
    830025c4:	b6fff06f          	j	83002132 <z_thread_halt>

00000000830025c8 <z_sched_wake_thread>:
{
    830025c8:	1101                	addi	sp,sp,-32
    830025ca:	e822                	sd	s0,16(sp)
    830025cc:	e04a                	sd	s2,0(sp)
    830025ce:	ec06                	sd	ra,24(sp)
    830025d0:	e426                	sd	s1,8(sp)
    830025d2:	842a                	mv	s0,a0
    830025d4:	892e                	mv	s2,a1
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    830025d6:	100174f3          	csrrci	s1,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    830025da:	00007517          	auipc	a0,0x7
    830025de:	f2650513          	addi	a0,a0,-218 # 83009500 <_sched_spinlock>
    830025e2:	591000ef          	jal	83003372 <z_spin_lock_valid>
    830025e6:	e139                	bnez	a0,8300262c <z_sched_wake_thread+0x64>
    830025e8:	08800693          	li	a3,136
    830025ec:	00006617          	auipc	a2,0x6
    830025f0:	42c60613          	addi	a2,a2,1068 # 83008a18 <plic_config_0+0x38>
    830025f4:	00006597          	auipc	a1,0x6
    830025f8:	45458593          	addi	a1,a1,1108 # 83008a48 <plic_config_0+0x68>
    830025fc:	00006517          	auipc	a0,0x6
    83002600:	c1c50513          	addi	a0,a0,-996 # 83008218 <__clz_tab+0x1e0>
    83002604:	91afe0ef          	jal	8300071e <assert_print>
    83002608:	00007597          	auipc	a1,0x7
    8300260c:	ef858593          	addi	a1,a1,-264 # 83009500 <_sched_spinlock>
    83002610:	00006517          	auipc	a0,0x6
    83002614:	45050513          	addi	a0,a0,1104 # 83008a60 <plic_config_0+0x80>
    83002618:	906fe0ef          	jal	8300071e <assert_print>
    8300261c:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83002620:	00006517          	auipc	a0,0x6
    83002624:	3f850513          	addi	a0,a0,1016 # 83008a18 <plic_config_0+0x38>
    83002628:	8ecfe0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    8300262c:	00007517          	auipc	a0,0x7
    83002630:	ed450513          	addi	a0,a0,-300 # 83009500 <_sched_spinlock>
    83002634:	587000ef          	jal	830033ba <z_spin_lock_set_owner>
		if (!killed) {
    83002638:	01944783          	lbu	a5,25(s0)
    8300263c:	2481                	sext.w	s1,s1
    8300263e:	0287f793          	andi	a5,a5,40
    83002642:	e39d                	bnez	a5,83002668 <z_sched_wake_thread+0xa0>
			if (thread->base.pended_on != NULL) {
    83002644:	681c                	ld	a5,16(s0)
    83002646:	c781                	beqz	a5,8300264e <z_sched_wake_thread+0x86>
				unpend_thread_no_timeout(thread);
    83002648:	8522                	mv	a0,s0
    8300264a:	843ff0ef          	jal	83001e8c <unpend_thread_no_timeout>
	thread->base.thread_state &= ~_THREAD_PRESTART;
    8300264e:	01944703          	lbu	a4,25(s0)
	thread->base.thread_state &= ~_THREAD_SUSPENDED;
    83002652:	0eb77793          	andi	a5,a4,235
			if (is_timeout) {
    83002656:	00091463          	bnez	s2,8300265e <z_sched_wake_thread+0x96>
	thread->base.thread_state &= ~_THREAD_PRESTART;
    8300265a:	0fb77793          	andi	a5,a4,251
    8300265e:	00f40ca3          	sb	a5,25(s0)
			ready_thread(thread);
    83002662:	8522                	mv	a0,s0
    83002664:	a1fff0ef          	jal	83002082 <ready_thread>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83002668:	00007517          	auipc	a0,0x7
    8300266c:	e9850513          	addi	a0,a0,-360 # 83009500 <_sched_spinlock>
    83002670:	51d000ef          	jal	8300338c <z_spin_unlock_valid>
    83002674:	ed15                	bnez	a0,830026b0 <z_sched_wake_thread+0xe8>
    83002676:	13100693          	li	a3,305
    8300267a:	00006617          	auipc	a2,0x6
    8300267e:	39e60613          	addi	a2,a2,926 # 83008a18 <plic_config_0+0x38>
    83002682:	00006597          	auipc	a1,0x6
    83002686:	3f658593          	addi	a1,a1,1014 # 83008a78 <plic_config_0+0x98>
    8300268a:	00006517          	auipc	a0,0x6
    8300268e:	b8e50513          	addi	a0,a0,-1138 # 83008218 <__clz_tab+0x1e0>
    83002692:	88cfe0ef          	jal	8300071e <assert_print>
    83002696:	00007597          	auipc	a1,0x7
    8300269a:	e6a58593          	addi	a1,a1,-406 # 83009500 <_sched_spinlock>
    8300269e:	00006517          	auipc	a0,0x6
    830026a2:	3f250513          	addi	a0,a0,1010 # 83008a90 <plic_config_0+0xb0>
    830026a6:	878fe0ef          	jal	8300071e <assert_print>
    830026aa:	13100593          	li	a1,305
    830026ae:	bf8d                	j	83002620 <z_sched_wake_thread+0x58>
	csr_set(xstatus, (key & XSTATUS_IEN));
    830026b0:	8889                	andi	s1,s1,2
    830026b2:	1004a073          	csrs	sstatus,s1
}
    830026b6:	60e2                	ld	ra,24(sp)
    830026b8:	6442                	ld	s0,16(sp)
    830026ba:	64a2                	ld	s1,8(sp)
    830026bc:	6902                	ld	s2,0(sp)
    830026be:	6105                	addi	sp,sp,32
    830026c0:	8082                	ret

00000000830026c2 <z_thread_timeout>:
	z_sched_wake_thread(thread, true);
    830026c2:	4585                	li	a1,1
    830026c4:	fd850513          	addi	a0,a0,-40
    830026c8:	b701                	j	830025c8 <z_sched_wake_thread>

00000000830026ca <z_reschedule>:
{
    830026ca:	1101                	addi	sp,sp,-32
    830026cc:	e822                	sd	s0,16(sp)
    830026ce:	e426                	sd	s1,8(sp)
    830026d0:	ec06                	sd	ra,24(sp)
    830026d2:	0025f413          	andi	s0,a1,2
    830026d6:	84aa                	mv	s1,a0
	return arch_irq_unlocked(key) && !arch_is_in_isr();
    830026d8:	c00d                	beqz	s0,830026fa <z_reschedule+0x30>
	bool ret = arch_curr_cpu()->nested != 0U;

	arch_irq_unlock(key);
	return ret;
#else
	return _kernel.cpus[0].nested != 0U;
    830026da:	00009797          	auipc	a5,0x9
    830026de:	25678793          	addi	a5,a5,598 # 8300b930 <_kernel>
    830026e2:	4398                	lw	a4,0(a5)
    830026e4:	eb19                	bnez	a4,830026fa <z_reschedule+0x30>
	if (resched(key.key) && need_swap()) {
    830026e6:	7f98                	ld	a4,56(a5)
    830026e8:	6b9c                	ld	a5,16(a5)
    830026ea:	00f70863          	beq	a4,a5,830026fa <z_reschedule+0x30>
}
    830026ee:	6442                	ld	s0,16(sp)
    830026f0:	60e2                	ld	ra,24(sp)
    830026f2:	64a2                	ld	s1,8(sp)
    830026f4:	6105                	addi	sp,sp,32
		z_swap(lock, key);
    830026f6:	fecff06f          	j	83001ee2 <z_swap>
    830026fa:	8526                	mv	a0,s1
    830026fc:	491000ef          	jal	8300338c <z_spin_unlock_valid>
    83002700:	e121                	bnez	a0,83002740 <z_reschedule+0x76>
    83002702:	13100693          	li	a3,305
    83002706:	00006617          	auipc	a2,0x6
    8300270a:	31260613          	addi	a2,a2,786 # 83008a18 <plic_config_0+0x38>
    8300270e:	00006597          	auipc	a1,0x6
    83002712:	36a58593          	addi	a1,a1,874 # 83008a78 <plic_config_0+0x98>
    83002716:	00006517          	auipc	a0,0x6
    8300271a:	b0250513          	addi	a0,a0,-1278 # 83008218 <__clz_tab+0x1e0>
    8300271e:	800fe0ef          	jal	8300071e <assert_print>
    83002722:	85a6                	mv	a1,s1
    83002724:	00006517          	auipc	a0,0x6
    83002728:	36c50513          	addi	a0,a0,876 # 83008a90 <plic_config_0+0xb0>
    8300272c:	ff3fd0ef          	jal	8300071e <assert_print>
    83002730:	13100593          	li	a1,305
    83002734:	00006517          	auipc	a0,0x6
    83002738:	2e450513          	addi	a0,a0,740 # 83008a18 <plic_config_0+0x38>
    8300273c:	fd9fd0ef          	jal	83000714 <assert_post_action>
    83002740:	10042073          	csrs	sstatus,s0
}
    83002744:	60e2                	ld	ra,24(sp)
    83002746:	6442                	ld	s0,16(sp)
    83002748:	64a2                	ld	s1,8(sp)
    8300274a:	6105                	addi	sp,sp,32
    8300274c:	8082                	ret

000000008300274e <z_sched_start>:
{
    8300274e:	1101                	addi	sp,sp,-32
    83002750:	e426                	sd	s1,8(sp)
    83002752:	ec06                	sd	ra,24(sp)
    83002754:	e822                	sd	s0,16(sp)
    83002756:	84aa                	mv	s1,a0
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83002758:	10017473          	csrrci	s0,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    8300275c:	00007517          	auipc	a0,0x7
    83002760:	da450513          	addi	a0,a0,-604 # 83009500 <_sched_spinlock>
    83002764:	40f000ef          	jal	83003372 <z_spin_lock_valid>
    83002768:	e139                	bnez	a0,830027ae <z_sched_start+0x60>
    8300276a:	08800693          	li	a3,136
    8300276e:	00006617          	auipc	a2,0x6
    83002772:	2aa60613          	addi	a2,a2,682 # 83008a18 <plic_config_0+0x38>
    83002776:	00006597          	auipc	a1,0x6
    8300277a:	2d258593          	addi	a1,a1,722 # 83008a48 <plic_config_0+0x68>
    8300277e:	00006517          	auipc	a0,0x6
    83002782:	a9a50513          	addi	a0,a0,-1382 # 83008218 <__clz_tab+0x1e0>
    83002786:	f99fd0ef          	jal	8300071e <assert_print>
    8300278a:	00007597          	auipc	a1,0x7
    8300278e:	d7658593          	addi	a1,a1,-650 # 83009500 <_sched_spinlock>
    83002792:	00006517          	auipc	a0,0x6
    83002796:	2ce50513          	addi	a0,a0,718 # 83008a60 <plic_config_0+0x80>
    8300279a:	f85fd0ef          	jal	8300071e <assert_print>
    8300279e:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    830027a2:	00006517          	auipc	a0,0x6
    830027a6:	27650513          	addi	a0,a0,630 # 83008a18 <plic_config_0+0x38>
    830027aa:	f6bfd0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    830027ae:	00007517          	auipc	a0,0x7
    830027b2:	d5250513          	addi	a0,a0,-686 # 83009500 <_sched_spinlock>
    830027b6:	405000ef          	jal	830033ba <z_spin_lock_set_owner>
	return (thread->base.thread_state & _THREAD_PRESTART) == 0U;
    830027ba:	0194c783          	lbu	a5,25(s1)
    830027be:	2401                	sext.w	s0,s0
	if (z_has_thread_started(thread)) {
    830027c0:	0047f713          	andi	a4,a5,4
    830027c4:	ef29                	bnez	a4,8300281e <z_sched_start+0xd0>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    830027c6:	00007517          	auipc	a0,0x7
    830027ca:	d3a50513          	addi	a0,a0,-710 # 83009500 <_sched_spinlock>
    830027ce:	3bf000ef          	jal	8300338c <z_spin_unlock_valid>
    830027d2:	ed15                	bnez	a0,8300280e <z_sched_start+0xc0>
    830027d4:	13100693          	li	a3,305
    830027d8:	00006617          	auipc	a2,0x6
    830027dc:	24060613          	addi	a2,a2,576 # 83008a18 <plic_config_0+0x38>
    830027e0:	00006597          	auipc	a1,0x6
    830027e4:	29858593          	addi	a1,a1,664 # 83008a78 <plic_config_0+0x98>
    830027e8:	00006517          	auipc	a0,0x6
    830027ec:	a3050513          	addi	a0,a0,-1488 # 83008218 <__clz_tab+0x1e0>
    830027f0:	f2ffd0ef          	jal	8300071e <assert_print>
    830027f4:	00007597          	auipc	a1,0x7
    830027f8:	d0c58593          	addi	a1,a1,-756 # 83009500 <_sched_spinlock>
    830027fc:	00006517          	auipc	a0,0x6
    83002800:	29450513          	addi	a0,a0,660 # 83008a90 <plic_config_0+0xb0>
    83002804:	f1bfd0ef          	jal	8300071e <assert_print>
    83002808:	13100593          	li	a1,305
    8300280c:	bf59                	j	830027a2 <z_sched_start+0x54>
	csr_set(xstatus, (key & XSTATUS_IEN));
    8300280e:	8809                	andi	s0,s0,2
    83002810:	10042073          	csrs	sstatus,s0
}
    83002814:	60e2                	ld	ra,24(sp)
    83002816:	6442                	ld	s0,16(sp)
    83002818:	64a2                	ld	s1,8(sp)
    8300281a:	6105                	addi	sp,sp,32
    8300281c:	8082                	ret
	thread->base.thread_state &= ~_THREAD_PRESTART;
    8300281e:	9bed                	andi	a5,a5,-5
	ready_thread(thread);
    83002820:	8526                	mv	a0,s1
    83002822:	00f48ca3          	sb	a5,25(s1)
    83002826:	85dff0ef          	jal	83002082 <ready_thread>
	z_reschedule(&_sched_spinlock, key);
    8300282a:	85a2                	mv	a1,s0
}
    8300282c:	6442                	ld	s0,16(sp)
    8300282e:	60e2                	ld	ra,24(sp)
    83002830:	64a2                	ld	s1,8(sp)
	z_reschedule(&_sched_spinlock, key);
    83002832:	00007517          	auipc	a0,0x7
    83002836:	cce50513          	addi	a0,a0,-818 # 83009500 <_sched_spinlock>
}
    8300283a:	6105                	addi	sp,sp,32
	z_reschedule(&_sched_spinlock, key);
    8300283c:	b579                	j	830026ca <z_reschedule>

000000008300283e <z_reschedule_irqlock>:
{
    8300283e:	7179                	addi	sp,sp,-48
    83002840:	f022                	sd	s0,32(sp)
    83002842:	00257793          	andi	a5,a0,2
    83002846:	f406                	sd	ra,40(sp)
    83002848:	ec26                	sd	s1,24(sp)
    8300284a:	e84a                	sd	s2,16(sp)
    8300284c:	e44e                	sd	s3,8(sp)
    8300284e:	843e                	mv	s0,a5
	return arch_irq_unlocked(key) && !arch_is_in_isr();
    83002850:	0e078d63          	beqz	a5,8300294a <z_reschedule_irqlock+0x10c>
    83002854:	00009497          	auipc	s1,0x9
    83002858:	0dc48493          	addi	s1,s1,220 # 8300b930 <_kernel>
    8300285c:	409c                	lw	a5,0(s1)
    8300285e:	0e079663          	bnez	a5,8300294a <z_reschedule_irqlock+0x10c>
	return new_thread != _current;
    83002862:	0104b903          	ld	s2,16(s1)
	if (resched(key) && need_swap()) {
    83002866:	7c9c                	ld	a5,56(s1)
    83002868:	0f278163          	beq	a5,s2,8300294a <z_reschedule_irqlock+0x10c>
	z_check_stack_sentinel();
    8300286c:	af4ff0ef          	jal	83001b60 <z_check_stack_sentinel>
	old_thread->swap_retval = -EAGAIN;
    83002870:	57d5                	li	a5,-11
    83002872:	10f92423          	sw	a5,264(s2)
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83002876:	100177f3          	csrrci	a5,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    8300287a:	00007517          	auipc	a0,0x7
    8300287e:	c8650513          	addi	a0,a0,-890 # 83009500 <_sched_spinlock>
    83002882:	2f1000ef          	jal	83003372 <z_spin_lock_valid>
    83002886:	e139                	bnez	a0,830028cc <z_reschedule_irqlock+0x8e>
    83002888:	08800693          	li	a3,136
    8300288c:	00006617          	auipc	a2,0x6
    83002890:	18c60613          	addi	a2,a2,396 # 83008a18 <plic_config_0+0x38>
    83002894:	00006597          	auipc	a1,0x6
    83002898:	1b458593          	addi	a1,a1,436 # 83008a48 <plic_config_0+0x68>
    8300289c:	00006517          	auipc	a0,0x6
    830028a0:	97c50513          	addi	a0,a0,-1668 # 83008218 <__clz_tab+0x1e0>
    830028a4:	e7bfd0ef          	jal	8300071e <assert_print>
    830028a8:	00007597          	auipc	a1,0x7
    830028ac:	c5858593          	addi	a1,a1,-936 # 83009500 <_sched_spinlock>
    830028b0:	00006517          	auipc	a0,0x6
    830028b4:	1b050513          	addi	a0,a0,432 # 83008a60 <plic_config_0+0x80>
    830028b8:	e67fd0ef          	jal	8300071e <assert_print>
    830028bc:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    830028c0:	00006517          	auipc	a0,0x6
    830028c4:	15850513          	addi	a0,a0,344 # 83008a18 <plic_config_0+0x38>
    830028c8:	e4dfd0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    830028cc:	00007517          	auipc	a0,0x7
    830028d0:	c3450513          	addi	a0,a0,-972 # 83009500 <_sched_spinlock>
    830028d4:	2e7000ef          	jal	830033ba <z_spin_lock_set_owner>
	return _kernel.ready_q.cache;
    830028d8:	0384b983          	ld	s3,56(s1)
	if (new_thread != old_thread) {
    830028dc:	09390063          	beq	s2,s3,8300295c <z_reschedule_irqlock+0x11e>
		z_reset_time_slice(new_thread);
    830028e0:	854e                	mv	a0,s3
		_current_cpu->current = new_thread;
    830028e2:	0134b823          	sd	s3,16(s1)
		z_reset_time_slice(new_thread);
    830028e6:	125000ef          	jal	8300320a <z_reset_time_slice>
		z_spin_lock_set_owner(&_sched_spinlock);
    830028ea:	00007517          	auipc	a0,0x7
    830028ee:	c1650513          	addi	a0,a0,-1002 # 83009500 <_sched_spinlock>
    830028f2:	2c9000ef          	jal	830033ba <z_spin_lock_set_owner>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    830028f6:	00007517          	auipc	a0,0x7
    830028fa:	c0a50513          	addi	a0,a0,-1014 # 83009500 <_sched_spinlock>
		void *newsh = new_thread->switch_handle;
    830028fe:	1109b483          	ld	s1,272(s3)
    83002902:	28b000ef          	jal	8300338c <z_spin_unlock_valid>
    83002906:	ed15                	bnez	a0,83002942 <z_reschedule_irqlock+0x104>
    83002908:	16b00693          	li	a3,363
    8300290c:	00006617          	auipc	a2,0x6
    83002910:	10c60613          	addi	a2,a2,268 # 83008a18 <plic_config_0+0x38>
    83002914:	00006597          	auipc	a1,0x6
    83002918:	16458593          	addi	a1,a1,356 # 83008a78 <plic_config_0+0x98>
    8300291c:	00006517          	auipc	a0,0x6
    83002920:	8fc50513          	addi	a0,a0,-1796 # 83008218 <__clz_tab+0x1e0>
    83002924:	dfbfd0ef          	jal	8300071e <assert_print>
    83002928:	00007597          	auipc	a1,0x7
    8300292c:	bd858593          	addi	a1,a1,-1064 # 83009500 <_sched_spinlock>
    83002930:	00006517          	auipc	a0,0x6
    83002934:	16050513          	addi	a0,a0,352 # 83008a90 <plic_config_0+0xb0>
    83002938:	de7fd0ef          	jal	8300071e <assert_print>
    8300293c:	16b00593          	li	a1,363
    83002940:	b741                	j	830028c0 <z_reschedule_irqlock+0x82>
	z_riscv_switch(new, old);
    83002942:	85ca                	mv	a1,s2
    83002944:	8526                	mv	a0,s1
    83002946:	89bfd0ef          	jal	830001e0 <z_riscv_switch>
	csr_set(xstatus, (key & XSTATUS_IEN));
    8300294a:	10042073          	csrs	sstatus,s0
}
    8300294e:	70a2                	ld	ra,40(sp)
    83002950:	7402                	ld	s0,32(sp)
    83002952:	64e2                	ld	s1,24(sp)
    83002954:	6942                	ld	s2,16(sp)
    83002956:	69a2                	ld	s3,8(sp)
    83002958:	6145                	addi	sp,sp,48
    8300295a:	8082                	ret
    8300295c:	00007517          	auipc	a0,0x7
    83002960:	ba450513          	addi	a0,a0,-1116 # 83009500 <_sched_spinlock>
    83002964:	229000ef          	jal	8300338c <z_spin_unlock_valid>
    83002968:	f16d                	bnez	a0,8300294a <z_reschedule_irqlock+0x10c>
    8300296a:	bf79                	j	83002908 <z_reschedule_irqlock+0xca>

000000008300296c <k_sched_lock>:
{
    8300296c:	1141                	addi	sp,sp,-16
    8300296e:	e406                	sd	ra,8(sp)
    83002970:	e022                	sd	s0,0(sp)
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83002972:	10017473          	csrrci	s0,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83002976:	00007517          	auipc	a0,0x7
    8300297a:	b8a50513          	addi	a0,a0,-1142 # 83009500 <_sched_spinlock>
    8300297e:	1f5000ef          	jal	83003372 <z_spin_lock_valid>
    83002982:	e131                	bnez	a0,830029c6 <k_sched_lock+0x5a>
    83002984:	08800693          	li	a3,136
    83002988:	00006617          	auipc	a2,0x6
    8300298c:	09060613          	addi	a2,a2,144 # 83008a18 <plic_config_0+0x38>
    83002990:	00006597          	auipc	a1,0x6
    83002994:	0b858593          	addi	a1,a1,184 # 83008a48 <plic_config_0+0x68>
    83002998:	00006517          	auipc	a0,0x6
    8300299c:	88050513          	addi	a0,a0,-1920 # 83008218 <__clz_tab+0x1e0>
    830029a0:	d7ffd0ef          	jal	8300071e <assert_print>
    830029a4:	00007597          	auipc	a1,0x7
    830029a8:	b5c58593          	addi	a1,a1,-1188 # 83009500 <_sched_spinlock>
    830029ac:	00006517          	auipc	a0,0x6
    830029b0:	0b450513          	addi	a0,a0,180 # 83008a60 <plic_config_0+0x80>
    830029b4:	d6bfd0ef          	jal	8300071e <assert_print>
    830029b8:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    830029bc:	00006517          	auipc	a0,0x6
    830029c0:	05c50513          	addi	a0,a0,92 # 83008a18 <plic_config_0+0x38>
    830029c4:	a889                	j	83002a16 <k_sched_lock+0xaa>
	z_spin_lock_set_owner(l);
    830029c6:	00007517          	auipc	a0,0x7
    830029ca:	b3a50513          	addi	a0,a0,-1222 # 83009500 <_sched_spinlock>
    830029ce:	1ed000ef          	jal	830033ba <z_spin_lock_set_owner>
	return _kernel.cpus[0].nested != 0U;
    830029d2:	00009797          	auipc	a5,0x9
    830029d6:	f5e78793          	addi	a5,a5,-162 # 8300b930 <_kernel>
	return true;
}

static inline void z_sched_lock(void)
{
	__ASSERT(!arch_is_in_isr(), "");
    830029da:	4398                	lw	a4,0(a5)
    830029dc:	cf1d                	beqz	a4,83002a1a <k_sched_lock+0xae>
    830029de:	00006597          	auipc	a1,0x6
    830029e2:	58a58593          	addi	a1,a1,1418 # 83008f68 <levels.0+0x1c8>
    830029e6:	08a00693          	li	a3,138
    830029ea:	00006617          	auipc	a2,0x6
    830029ee:	6be60613          	addi	a2,a2,1726 # 830090a8 <levels.0+0x308>
    830029f2:	00006517          	auipc	a0,0x6
    830029f6:	82650513          	addi	a0,a0,-2010 # 83008218 <__clz_tab+0x1e0>
    830029fa:	d25fd0ef          	jal	8300071e <assert_print>
    830029fe:	00006517          	auipc	a0,0x6
    83002a02:	60250513          	addi	a0,a0,1538 # 83009000 <levels.0+0x260>
    83002a06:	d19fd0ef          	jal	8300071e <assert_print>
    83002a0a:	08a00593          	li	a1,138
    83002a0e:	00006517          	auipc	a0,0x6
    83002a12:	69a50513          	addi	a0,a0,1690 # 830090a8 <levels.0+0x308>
    83002a16:	cfffd0ef          	jal	83000714 <assert_post_action>
	__ASSERT(_current->base.sched_locked != 1U, "");
    83002a1a:	6b98                	ld	a4,16(a5)
    83002a1c:	4685                	li	a3,1
    83002a1e:	01b74783          	lbu	a5,27(a4)
    83002a22:	02d79b63          	bne	a5,a3,83002a58 <k_sched_lock+0xec>
    83002a26:	00006597          	auipc	a1,0x6
    83002a2a:	6b258593          	addi	a1,a1,1714 # 830090d8 <levels.0+0x338>
    83002a2e:	08b00693          	li	a3,139
    83002a32:	00006617          	auipc	a2,0x6
    83002a36:	67660613          	addi	a2,a2,1654 # 830090a8 <levels.0+0x308>
    83002a3a:	00005517          	auipc	a0,0x5
    83002a3e:	7de50513          	addi	a0,a0,2014 # 83008218 <__clz_tab+0x1e0>
    83002a42:	cddfd0ef          	jal	8300071e <assert_print>
    83002a46:	00006517          	auipc	a0,0x6
    83002a4a:	5ba50513          	addi	a0,a0,1466 # 83009000 <levels.0+0x260>
    83002a4e:	cd1fd0ef          	jal	8300071e <assert_print>
    83002a52:	08b00593          	li	a1,139
    83002a56:	bf65                	j	83002a0e <k_sched_lock+0xa2>

	--_current->base.sched_locked;
    83002a58:	37fd                	addiw	a5,a5,-1
    83002a5a:	00f70da3          	sb	a5,27(a4)
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83002a5e:	00007517          	auipc	a0,0x7
    83002a62:	aa250513          	addi	a0,a0,-1374 # 83009500 <_sched_spinlock>
    83002a66:	127000ef          	jal	8300338c <z_spin_unlock_valid>
    83002a6a:	ed15                	bnez	a0,83002aa6 <k_sched_lock+0x13a>
    83002a6c:	13100693          	li	a3,305
    83002a70:	00006617          	auipc	a2,0x6
    83002a74:	fa860613          	addi	a2,a2,-88 # 83008a18 <plic_config_0+0x38>
    83002a78:	00006597          	auipc	a1,0x6
    83002a7c:	00058593          	mv	a1,a1
    83002a80:	00005517          	auipc	a0,0x5
    83002a84:	79850513          	addi	a0,a0,1944 # 83008218 <__clz_tab+0x1e0>
    83002a88:	c97fd0ef          	jal	8300071e <assert_print>
    83002a8c:	00007597          	auipc	a1,0x7
    83002a90:	a7458593          	addi	a1,a1,-1420 # 83009500 <_sched_spinlock>
    83002a94:	00006517          	auipc	a0,0x6
    83002a98:	ffc50513          	addi	a0,a0,-4 # 83008a90 <plic_config_0+0xb0>
    83002a9c:	c83fd0ef          	jal	8300071e <assert_print>
    83002aa0:	13100593          	li	a1,305
    83002aa4:	bf21                	j	830029bc <k_sched_lock+0x50>
    83002aa6:	2401                	sext.w	s0,s0
	csr_set(xstatus, (key & XSTATUS_IEN));
    83002aa8:	8809                	andi	s0,s0,2
    83002aaa:	10042073          	csrs	sstatus,s0
}
    83002aae:	60a2                	ld	ra,8(sp)
    83002ab0:	6402                	ld	s0,0(sp)
    83002ab2:	0141                	addi	sp,sp,16
    83002ab4:	8082                	ret

0000000083002ab6 <k_sched_unlock>:
{
    83002ab6:	1141                	addi	sp,sp,-16
    83002ab8:	e406                	sd	ra,8(sp)
    83002aba:	e022                	sd	s0,0(sp)
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83002abc:	10017473          	csrrci	s0,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83002ac0:	00007517          	auipc	a0,0x7
    83002ac4:	a4050513          	addi	a0,a0,-1472 # 83009500 <_sched_spinlock>
    83002ac8:	0ab000ef          	jal	83003372 <z_spin_lock_valid>
    83002acc:	e131                	bnez	a0,83002b10 <k_sched_unlock+0x5a>
    83002ace:	08800693          	li	a3,136
    83002ad2:	00006617          	auipc	a2,0x6
    83002ad6:	f4660613          	addi	a2,a2,-186 # 83008a18 <plic_config_0+0x38>
    83002ada:	00006597          	auipc	a1,0x6
    83002ade:	f6e58593          	addi	a1,a1,-146 # 83008a48 <plic_config_0+0x68>
    83002ae2:	00005517          	auipc	a0,0x5
    83002ae6:	73650513          	addi	a0,a0,1846 # 83008218 <__clz_tab+0x1e0>
    83002aea:	c35fd0ef          	jal	8300071e <assert_print>
    83002aee:	00007597          	auipc	a1,0x7
    83002af2:	a1258593          	addi	a1,a1,-1518 # 83009500 <_sched_spinlock>
    83002af6:	00006517          	auipc	a0,0x6
    83002afa:	f6a50513          	addi	a0,a0,-150 # 83008a60 <plic_config_0+0x80>
    83002afe:	c21fd0ef          	jal	8300071e <assert_print>
    83002b02:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83002b06:	00006517          	auipc	a0,0x6
    83002b0a:	f1250513          	addi	a0,a0,-238 # 83008a18 <plic_config_0+0x38>
    83002b0e:	a899                	j	83002b64 <k_sched_unlock+0xae>
	z_spin_lock_set_owner(l);
    83002b10:	00007517          	auipc	a0,0x7
    83002b14:	9f050513          	addi	a0,a0,-1552 # 83009500 <_sched_spinlock>
    83002b18:	0a3000ef          	jal	830033ba <z_spin_lock_set_owner>
		__ASSERT(_current->base.sched_locked != 0U, "");
    83002b1c:	00009697          	auipc	a3,0x9
    83002b20:	e1468693          	addi	a3,a3,-492 # 8300b930 <_kernel>
    83002b24:	6a98                	ld	a4,16(a3)
    83002b26:	01b74783          	lbu	a5,27(a4)
    83002b2a:	ef9d                	bnez	a5,83002b68 <k_sched_unlock+0xb2>
    83002b2c:	00006597          	auipc	a1,0x6
    83002b30:	5e458593          	addi	a1,a1,1508 # 83009110 <levels.0+0x370>
    83002b34:	34c00693          	li	a3,844
    83002b38:	00006617          	auipc	a2,0x6
    83002b3c:	4d060613          	addi	a2,a2,1232 # 83009008 <levels.0+0x268>
    83002b40:	00005517          	auipc	a0,0x5
    83002b44:	6d850513          	addi	a0,a0,1752 # 83008218 <__clz_tab+0x1e0>
    83002b48:	bd7fd0ef          	jal	8300071e <assert_print>
    83002b4c:	00006517          	auipc	a0,0x6
    83002b50:	4b450513          	addi	a0,a0,1204 # 83009000 <levels.0+0x260>
    83002b54:	bcbfd0ef          	jal	8300071e <assert_print>
    83002b58:	34c00593          	li	a1,844
    83002b5c:	00006517          	auipc	a0,0x6
    83002b60:	4ac50513          	addi	a0,a0,1196 # 83009008 <levels.0+0x268>
    83002b64:	bb1fd0ef          	jal	83000714 <assert_post_action>
		__ASSERT(!arch_is_in_isr(), "");
    83002b68:	4294                	lw	a3,0(a3)
    83002b6a:	ca95                	beqz	a3,83002b9e <k_sched_unlock+0xe8>
    83002b6c:	00006597          	auipc	a1,0x6
    83002b70:	3fc58593          	addi	a1,a1,1020 # 83008f68 <levels.0+0x1c8>
    83002b74:	34d00693          	li	a3,845
    83002b78:	00006617          	auipc	a2,0x6
    83002b7c:	49060613          	addi	a2,a2,1168 # 83009008 <levels.0+0x268>
    83002b80:	00005517          	auipc	a0,0x5
    83002b84:	69850513          	addi	a0,a0,1688 # 83008218 <__clz_tab+0x1e0>
    83002b88:	b97fd0ef          	jal	8300071e <assert_print>
    83002b8c:	00006517          	auipc	a0,0x6
    83002b90:	47450513          	addi	a0,a0,1140 # 83009000 <levels.0+0x260>
    83002b94:	b8bfd0ef          	jal	8300071e <assert_print>
    83002b98:	34d00593          	li	a1,845
    83002b9c:	b7c1                	j	83002b5c <k_sched_unlock+0xa6>
		++_current->base.sched_locked;
    83002b9e:	2785                	addiw	a5,a5,1
		update_cache(0);
    83002ba0:	4501                	li	a0,0
		++_current->base.sched_locked;
    83002ba2:	00f70da3          	sb	a5,27(a4)
		update_cache(0);
    83002ba6:	9e0ff0ef          	jal	83001d86 <update_cache>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83002baa:	00007517          	auipc	a0,0x7
    83002bae:	95650513          	addi	a0,a0,-1706 # 83009500 <_sched_spinlock>
    83002bb2:	7da000ef          	jal	8300338c <z_spin_unlock_valid>
    83002bb6:	ed15                	bnez	a0,83002bf2 <k_sched_unlock+0x13c>
    83002bb8:	13100693          	li	a3,305
    83002bbc:	00006617          	auipc	a2,0x6
    83002bc0:	e5c60613          	addi	a2,a2,-420 # 83008a18 <plic_config_0+0x38>
    83002bc4:	00006597          	auipc	a1,0x6
    83002bc8:	eb458593          	addi	a1,a1,-332 # 83008a78 <plic_config_0+0x98>
    83002bcc:	00005517          	auipc	a0,0x5
    83002bd0:	64c50513          	addi	a0,a0,1612 # 83008218 <__clz_tab+0x1e0>
    83002bd4:	b4bfd0ef          	jal	8300071e <assert_print>
    83002bd8:	00007597          	auipc	a1,0x7
    83002bdc:	92858593          	addi	a1,a1,-1752 # 83009500 <_sched_spinlock>
    83002be0:	00006517          	auipc	a0,0x6
    83002be4:	eb050513          	addi	a0,a0,-336 # 83008a90 <plic_config_0+0xb0>
    83002be8:	b37fd0ef          	jal	8300071e <assert_print>
    83002bec:	13100593          	li	a1,305
    83002bf0:	bf19                	j	83002b06 <k_sched_unlock+0x50>
    83002bf2:	2401                	sext.w	s0,s0
	csr_set(xstatus, (key & XSTATUS_IEN));
    83002bf4:	8809                	andi	s0,s0,2
    83002bf6:	10042073          	csrs	sstatus,s0
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83002bfa:	10017573          	csrrci	a0,sstatus,2
}
    83002bfe:	6402                	ld	s0,0(sp)
    83002c00:	60a2                	ld	ra,8(sp)
	(void) z_reschedule_irqlock(arch_irq_lock());
    83002c02:	2501                	sext.w	a0,a0
    83002c04:	0141                	addi	sp,sp,16
    83002c06:	c39ff06f          	j	8300283e <z_reschedule_irqlock>

0000000083002c0a <z_swap_next_thread>:
}
    83002c0a:	00009517          	auipc	a0,0x9
    83002c0e:	d5e53503          	ld	a0,-674(a0) # 8300b968 <_kernel+0x38>
    83002c12:	8082                	ret

0000000083002c14 <z_get_next_switch_handle>:
{
    83002c14:	1141                	addi	sp,sp,-16
    83002c16:	e022                	sd	s0,0(sp)
    83002c18:	e406                	sd	ra,8(sp)
    83002c1a:	842a                	mv	s0,a0
	z_check_stack_sentinel();
    83002c1c:	f45fe0ef          	jal	83001b60 <z_check_stack_sentinel>
	z_sched_usage_switch(_kernel.ready_q.cache);
    83002c20:	00009797          	auipc	a5,0x9
    83002c24:	d1078793          	addi	a5,a5,-752 # 8300b930 <_kernel>
    83002c28:	7f98                	ld	a4,56(a5)
	_current->switch_handle = interrupted;
    83002c2a:	6b94                	ld	a3,16(a5)
	_current_cpu->current = new_thread;
    83002c2c:	eb98                	sd	a4,16(a5)
	_current->switch_handle = interrupted;
    83002c2e:	1086b823          	sd	s0,272(a3)
}
    83002c32:	60a2                	ld	ra,8(sp)
    83002c34:	6402                	ld	s0,0(sp)
    83002c36:	11073503          	ld	a0,272(a4)
    83002c3a:	0141                	addi	sp,sp,16
    83002c3c:	8082                	ret

0000000083002c3e <z_sched_init>:
	list->head = (sys_dnode_t *)list;
    83002c3e:	00009797          	auipc	a5,0x9
    83002c42:	cf278793          	addi	a5,a5,-782 # 8300b930 <_kernel>
    83002c46:	00009717          	auipc	a4,0x9
    83002c4a:	d2a70713          	addi	a4,a4,-726 # 8300b970 <_kernel+0x40>
    83002c4e:	e3b8                	sd	a4,64(a5)
	list->tail = (sys_dnode_t *)list;
    83002c50:	e7b8                	sd	a4,72(a5)
}
    83002c52:	8082                	ret

0000000083002c54 <z_impl_k_yield>:
{
    83002c54:	1101                	addi	sp,sp,-32
    83002c56:	e822                	sd	s0,16(sp)
    83002c58:	ec06                	sd	ra,24(sp)
    83002c5a:	e426                	sd	s1,8(sp)
    83002c5c:	e04a                	sd	s2,0(sp)
    83002c5e:	00009417          	auipc	s0,0x9
    83002c62:	cd240413          	addi	s0,s0,-814 # 8300b930 <_kernel>
	__ASSERT(!arch_is_in_isr(), "");
    83002c66:	401c                	lw	a5,0(s0)
    83002c68:	cf95                	beqz	a5,83002ca4 <z_impl_k_yield+0x50>
    83002c6a:	00006597          	auipc	a1,0x6
    83002c6e:	2fe58593          	addi	a1,a1,766 # 83008f68 <levels.0+0x1c8>
    83002c72:	45c00693          	li	a3,1116
    83002c76:	00006617          	auipc	a2,0x6
    83002c7a:	39260613          	addi	a2,a2,914 # 83009008 <levels.0+0x268>
    83002c7e:	00005517          	auipc	a0,0x5
    83002c82:	59a50513          	addi	a0,a0,1434 # 83008218 <__clz_tab+0x1e0>
    83002c86:	a99fd0ef          	jal	8300071e <assert_print>
    83002c8a:	00006517          	auipc	a0,0x6
    83002c8e:	37650513          	addi	a0,a0,886 # 83009000 <levels.0+0x260>
    83002c92:	a8dfd0ef          	jal	8300071e <assert_print>
    83002c96:	45c00593          	li	a1,1116
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    83002c9a:	00006517          	auipc	a0,0x6
    83002c9e:	36e50513          	addi	a0,a0,878 # 83009008 <levels.0+0x268>
    83002ca2:	a899                	j	83002cf8 <z_impl_k_yield+0xa4>
    83002ca4:	100174f3          	csrrci	s1,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83002ca8:	00007517          	auipc	a0,0x7
    83002cac:	85850513          	addi	a0,a0,-1960 # 83009500 <_sched_spinlock>
	k.key = arch_irq_lock();
    83002cb0:	2481                	sext.w	s1,s1
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83002cb2:	6c0000ef          	jal	83003372 <z_spin_lock_valid>
    83002cb6:	e139                	bnez	a0,83002cfc <z_impl_k_yield+0xa8>
    83002cb8:	08800693          	li	a3,136
    83002cbc:	00006617          	auipc	a2,0x6
    83002cc0:	d5c60613          	addi	a2,a2,-676 # 83008a18 <plic_config_0+0x38>
    83002cc4:	00006597          	auipc	a1,0x6
    83002cc8:	d8458593          	addi	a1,a1,-636 # 83008a48 <plic_config_0+0x68>
    83002ccc:	00005517          	auipc	a0,0x5
    83002cd0:	54c50513          	addi	a0,a0,1356 # 83008218 <__clz_tab+0x1e0>
    83002cd4:	a4bfd0ef          	jal	8300071e <assert_print>
    83002cd8:	00007597          	auipc	a1,0x7
    83002cdc:	82858593          	addi	a1,a1,-2008 # 83009500 <_sched_spinlock>
    83002ce0:	00006517          	auipc	a0,0x6
    83002ce4:	d8050513          	addi	a0,a0,-640 # 83008a60 <plic_config_0+0x80>
    83002ce8:	a37fd0ef          	jal	8300071e <assert_print>
    83002cec:	08800593          	li	a1,136
    83002cf0:	00006517          	auipc	a0,0x6
    83002cf4:	d2850513          	addi	a0,a0,-728 # 83008a18 <plic_config_0+0x38>
    83002cf8:	a1dfd0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    83002cfc:	00007517          	auipc	a0,0x7
    83002d00:	80450513          	addi	a0,a0,-2044 # 83009500 <_sched_spinlock>
    83002d04:	6b6000ef          	jal	830033ba <z_spin_lock_set_owner>
		dequeue_thread(_current);
    83002d08:	6808                	ld	a0,16(s0)
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    83002d0a:	00009917          	auipc	s2,0x9
    83002d0e:	9d690913          	addi	s2,s2,-1578 # 8300b6e0 <z_idle_threads>
	thread->base.thread_state &= ~_THREAD_QUEUED;
    83002d12:	01954783          	lbu	a5,25(a0)
    83002d16:	07f7f793          	andi	a5,a5,127
    83002d1a:	00f50ca3          	sb	a5,25(a0)
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    83002d1e:	03251563          	bne	a0,s2,83002d48 <z_impl_k_yield+0xf4>
    83002d22:	00006597          	auipc	a1,0x6
    83002d26:	30e58593          	addi	a1,a1,782 # 83009030 <levels.0+0x290>
    83002d2a:	08000693          	li	a3,128
    83002d2e:	00006617          	auipc	a2,0x6
    83002d32:	2da60613          	addi	a2,a2,730 # 83009008 <levels.0+0x268>
    83002d36:	00005517          	auipc	a0,0x5
    83002d3a:	4e250513          	addi	a0,a0,1250 # 83008218 <__clz_tab+0x1e0>
    83002d3e:	9e1fd0ef          	jal	8300071e <assert_print>
    83002d42:	08000593          	li	a1,128
    83002d46:	bf91                	j	83002c9a <z_impl_k_yield+0x46>
	sys_dlist_remove(&thread->base.qnode_dlist);
    83002d48:	82cff0ef          	jal	83001d74 <sys_dlist_remove>
	queue_thread(_current);
    83002d4c:	681c                	ld	a5,16(s0)
	thread->base.thread_state |= _THREAD_QUEUED;
    83002d4e:	0197c703          	lbu	a4,25(a5)
    83002d52:	f8076713          	ori	a4,a4,-128
    83002d56:	00e78ca3          	sb	a4,25(a5)
	__ASSERT_NO_MSG(!z_is_idle_thread_object(thread));
    83002d5a:	03279563          	bne	a5,s2,83002d84 <z_impl_k_yield+0x130>
    83002d5e:	00006597          	auipc	a1,0x6
    83002d62:	2d258593          	addi	a1,a1,722 # 83009030 <levels.0+0x290>
    83002d66:	07900693          	li	a3,121
    83002d6a:	00006617          	auipc	a2,0x6
    83002d6e:	29e60613          	addi	a2,a2,670 # 83009008 <levels.0+0x268>
    83002d72:	00005517          	auipc	a0,0x5
    83002d76:	4a650513          	addi	a0,a0,1190 # 83008218 <__clz_tab+0x1e0>
    83002d7a:	9a5fd0ef          	jal	8300071e <assert_print>
    83002d7e:	07900593          	li	a1,121
    83002d82:	bf21                	j	83002c9a <z_impl_k_yield+0x46>
	return list->head == list;
    83002d84:	6038                	ld	a4,64(s0)
	return sys_dlist_is_empty(list) ? NULL : list->head;
    83002d86:	00009617          	auipc	a2,0x9
    83002d8a:	bea60613          	addi	a2,a2,-1046 # 8300b970 <_kernel+0x40>
    83002d8e:	00c71363          	bne	a4,a2,83002d94 <z_impl_k_yield+0x140>
    83002d92:	4701                	li	a4,0
	return (node == list->tail) ? NULL : node->next;
    83002d94:	6434                	ld	a3,72(s0)
	SYS_DLIST_FOR_EACH_CONTAINER(pq, t, base.qnode_dlist) {
    83002d96:	e711                	bnez	a4,83002da2 <z_impl_k_yield+0x14e>
	node->next = list;
    83002d98:	e390                	sd	a2,0(a5)
	node->prev = tail;
    83002d9a:	e794                	sd	a3,8(a5)
	tail->next = node;
    83002d9c:	e29c                	sd	a5,0(a3)
	list->tail = node;
    83002d9e:	e43c                	sd	a5,72(s0)
}
    83002da0:	a831                	j	83002dbc <z_impl_k_yield+0x168>
	int32_t b1 = thread_1->base.prio;
    83002da2:	01a78583          	lb	a1,26(a5)
	int32_t b2 = thread_2->base.prio;
    83002da6:	01a70503          	lb	a0,26(a4)
	if (b1 != b2) {
    83002daa:	02a58863          	beq	a1,a0,83002dda <z_impl_k_yield+0x186>
		if (z_sched_prio_cmp(thread, t) > 0) {
    83002dae:	02a5d663          	bge	a1,a0,83002dda <z_impl_k_yield+0x186>
	sys_dnode_t *const prev = successor->prev;
    83002db2:	6714                	ld	a3,8(a4)
	node->next = successor;
    83002db4:	e398                	sd	a4,0(a5)
	node->prev = prev;
    83002db6:	e794                	sd	a3,8(a5)
	prev->next = node;
    83002db8:	e29c                	sd	a5,0(a3)
	successor->prev = node;
    83002dba:	e71c                	sd	a5,8(a4)
	update_cache(1);
    83002dbc:	4505                	li	a0,1
    83002dbe:	fc9fe0ef          	jal	83001d86 <update_cache>
}
    83002dc2:	6442                	ld	s0,16(sp)
    83002dc4:	60e2                	ld	ra,24(sp)
    83002dc6:	6902                	ld	s2,0(sp)
	z_swap(&_sched_spinlock, key);
    83002dc8:	85a6                	mv	a1,s1
}
    83002dca:	64a2                	ld	s1,8(sp)
	z_swap(&_sched_spinlock, key);
    83002dcc:	00006517          	auipc	a0,0x6
    83002dd0:	73450513          	addi	a0,a0,1844 # 83009500 <_sched_spinlock>
}
    83002dd4:	6105                	addi	sp,sp,32
	z_swap(&_sched_spinlock, key);
    83002dd6:	90cff06f          	j	83001ee2 <z_swap>
	return (node == list->tail) ? NULL : node->next;
    83002dda:	fad70fe3          	beq	a4,a3,83002d98 <z_impl_k_yield+0x144>
    83002dde:	6318                	ld	a4,0(a4)
    83002de0:	bf5d                	j	83002d96 <z_impl_k_yield+0x142>

0000000083002de2 <z_tick_sleep>:
{
    83002de2:	7179                	addi	sp,sp,-48
    83002de4:	e84a                	sd	s2,16(sp)
    83002de6:	f406                	sd	ra,40(sp)
    83002de8:	f022                	sd	s0,32(sp)
    83002dea:	ec26                	sd	s1,24(sp)
    83002dec:	e44e                	sd	s3,8(sp)
    83002dee:	00009917          	auipc	s2,0x9
    83002df2:	b4290913          	addi	s2,s2,-1214 # 8300b930 <_kernel>
	__ASSERT(!arch_is_in_isr(), "");
    83002df6:	00092783          	lw	a5,0(s2)
    83002dfa:	cf95                	beqz	a5,83002e36 <z_tick_sleep+0x54>
    83002dfc:	00006597          	auipc	a1,0x6
    83002e00:	16c58593          	addi	a1,a1,364 # 83008f68 <levels.0+0x1c8>
    83002e04:	47700693          	li	a3,1143
    83002e08:	00006617          	auipc	a2,0x6
    83002e0c:	20060613          	addi	a2,a2,512 # 83009008 <levels.0+0x268>
    83002e10:	00005517          	auipc	a0,0x5
    83002e14:	40850513          	addi	a0,a0,1032 # 83008218 <__clz_tab+0x1e0>
    83002e18:	907fd0ef          	jal	8300071e <assert_print>
    83002e1c:	00006517          	auipc	a0,0x6
    83002e20:	1e450513          	addi	a0,a0,484 # 83009000 <levels.0+0x260>
    83002e24:	8fbfd0ef          	jal	8300071e <assert_print>
    83002e28:	47700593          	li	a1,1143
	__ASSERT(!z_is_thread_state_set(_current, _THREAD_SUSPENDED), "");
    83002e2c:	00006517          	auipc	a0,0x6
    83002e30:	1dc50513          	addi	a0,a0,476 # 83009008 <levels.0+0x268>
    83002e34:	a88d                	j	83002ea6 <z_tick_sleep+0xc4>
    83002e36:	84aa                	mv	s1,a0
	if (ticks == 0) {
    83002e38:	e509                	bnez	a0,83002e42 <z_tick_sleep+0x60>
	z_impl_k_yield();
    83002e3a:	e1bff0ef          	jal	83002c54 <z_impl_k_yield>
		return 0;
    83002e3e:	4501                	li	a0,0
    83002e40:	a239                	j	83002f4e <z_tick_sleep+0x16c>
	if (Z_TICK_ABS(ticks) <= 0) {
    83002e42:	57f9                	li	a5,-2
		expected_wakeup_ticks = ticks + sys_clock_tick_get_32();
    83002e44:	0005041b          	sext.w	s0,a0
	if (Z_TICK_ABS(ticks) <= 0) {
    83002e48:	06f54163          	blt	a0,a5,83002eaa <z_tick_sleep+0xc8>
		expected_wakeup_ticks = ticks + sys_clock_tick_get_32();
    83002e4c:	237000ef          	jal	83003882 <sys_clock_tick_get_32>
    83002e50:	9c29                	addw	s0,s0,a0
    83002e52:	100179f3          	csrrci	s3,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83002e56:	00006517          	auipc	a0,0x6
    83002e5a:	6aa50513          	addi	a0,a0,1706 # 83009500 <_sched_spinlock>
	k.key = arch_irq_lock();
    83002e5e:	2981                	sext.w	s3,s3
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83002e60:	512000ef          	jal	83003372 <z_spin_lock_valid>
    83002e64:	e539                	bnez	a0,83002eb2 <z_tick_sleep+0xd0>
    83002e66:	08800693          	li	a3,136
    83002e6a:	00006617          	auipc	a2,0x6
    83002e6e:	bae60613          	addi	a2,a2,-1106 # 83008a18 <plic_config_0+0x38>
    83002e72:	00006597          	auipc	a1,0x6
    83002e76:	bd658593          	addi	a1,a1,-1066 # 83008a48 <plic_config_0+0x68>
    83002e7a:	00005517          	auipc	a0,0x5
    83002e7e:	39e50513          	addi	a0,a0,926 # 83008218 <__clz_tab+0x1e0>
    83002e82:	89dfd0ef          	jal	8300071e <assert_print>
    83002e86:	00006597          	auipc	a1,0x6
    83002e8a:	67a58593          	addi	a1,a1,1658 # 83009500 <_sched_spinlock>
    83002e8e:	00006517          	auipc	a0,0x6
    83002e92:	bd250513          	addi	a0,a0,-1070 # 83008a60 <plic_config_0+0x80>
    83002e96:	889fd0ef          	jal	8300071e <assert_print>
    83002e9a:	08800593          	li	a1,136
    83002e9e:	00006517          	auipc	a0,0x6
    83002ea2:	b7a50513          	addi	a0,a0,-1158 # 83008a18 <plic_config_0+0x38>
    83002ea6:	86ffd0ef          	jal	83000714 <assert_post_action>
		expected_wakeup_ticks = Z_TICK_ABS(ticks);
    83002eaa:	57f9                	li	a5,-2
    83002eac:	4087843b          	subw	s0,a5,s0
    83002eb0:	b74d                	j	83002e52 <z_tick_sleep+0x70>
	z_spin_lock_set_owner(l);
    83002eb2:	00006517          	auipc	a0,0x6
    83002eb6:	64e50513          	addi	a0,a0,1614 # 83009500 <_sched_spinlock>
    83002eba:	500000ef          	jal	830033ba <z_spin_lock_set_owner>
	unready_thread(_current);
    83002ebe:	01093503          	ld	a0,16(s2)
    83002ec2:	f55fe0ef          	jal	83001e16 <unready_thread>
	z_add_thread_timeout(_current, timeout);
    83002ec6:	01093503          	ld	a0,16(s2)
	z_add_timeout(&thread->base.timeout, z_thread_timeout, ticks);
    83002eca:	8626                	mv	a2,s1
    83002ecc:	fffff597          	auipc	a1,0xfffff
    83002ed0:	7f658593          	addi	a1,a1,2038 # 830026c2 <z_thread_timeout>
    83002ed4:	02850513          	addi	a0,a0,40
    83002ed8:	57e000ef          	jal	83003456 <z_add_timeout>
	z_mark_thread_as_suspended(_current);
    83002edc:	01093703          	ld	a4,16(s2)
	(void)z_swap(&_sched_spinlock, key);
    83002ee0:	85ce                	mv	a1,s3
    83002ee2:	00006517          	auipc	a0,0x6
    83002ee6:	61e50513          	addi	a0,a0,1566 # 83009500 <_sched_spinlock>
	thread->base.thread_state |= _THREAD_SUSPENDED;
    83002eea:	01974783          	lbu	a5,25(a4)
    83002eee:	0107e793          	ori	a5,a5,16
    83002ef2:	00f70ca3          	sb	a5,25(a4)
    83002ef6:	fedfe0ef          	jal	83001ee2 <z_swap>
	return (thread->base.thread_state & state) != 0U;
    83002efa:	01093783          	ld	a5,16(s2)
	__ASSERT(!z_is_thread_state_set(_current, _THREAD_SUSPENDED), "");
    83002efe:	0197c783          	lbu	a5,25(a5)
    83002f02:	8bc1                	andi	a5,a5,16
    83002f04:	cb95                	beqz	a5,83002f38 <z_tick_sleep+0x156>
    83002f06:	00006597          	auipc	a1,0x6
    83002f0a:	24258593          	addi	a1,a1,578 # 83009148 <levels.0+0x3a8>
    83002f0e:	49300693          	li	a3,1171
    83002f12:	00006617          	auipc	a2,0x6
    83002f16:	0f660613          	addi	a2,a2,246 # 83009008 <levels.0+0x268>
    83002f1a:	00005517          	auipc	a0,0x5
    83002f1e:	2fe50513          	addi	a0,a0,766 # 83008218 <__clz_tab+0x1e0>
    83002f22:	ffcfd0ef          	jal	8300071e <assert_print>
    83002f26:	00006517          	auipc	a0,0x6
    83002f2a:	0da50513          	addi	a0,a0,218 # 83009000 <levels.0+0x260>
    83002f2e:	ff0fd0ef          	jal	8300071e <assert_print>
    83002f32:	49300593          	li	a1,1171
    83002f36:	bddd                	j	83002e2c <z_tick_sleep+0x4a>
	ticks = (k_ticks_t)expected_wakeup_ticks - sys_clock_tick_get_32();
    83002f38:	14b000ef          	jal	83003882 <sys_clock_tick_get_32>
    83002f3c:	1402                	slli	s0,s0,0x20
    83002f3e:	1502                	slli	a0,a0,0x20
    83002f40:	9101                	srli	a0,a0,0x20
    83002f42:	9001                	srli	s0,s0,0x20
    83002f44:	8c09                	sub	s0,s0,a0
		return ticks;
    83002f46:	0004051b          	sext.w	a0,s0
	if (ticks > 0) {
    83002f4a:	ee805ae3          	blez	s0,83002e3e <z_tick_sleep+0x5c>
}
    83002f4e:	70a2                	ld	ra,40(sp)
    83002f50:	7402                	ld	s0,32(sp)
    83002f52:	64e2                	ld	s1,24(sp)
    83002f54:	6942                	ld	s2,16(sp)
    83002f56:	69a2                	ld	s3,8(sp)
    83002f58:	6145                	addi	sp,sp,48
    83002f5a:	8082                	ret

0000000083002f5c <z_impl_k_sleep>:
{
    83002f5c:	1141                	addi	sp,sp,-16
    83002f5e:	e406                	sd	ra,8(sp)
    83002f60:	00009717          	auipc	a4,0x9
    83002f64:	9d070713          	addi	a4,a4,-1584 # 8300b930 <_kernel>
	__ASSERT(!arch_is_in_isr(), "");
    83002f68:	4314                	lw	a3,0(a4)
    83002f6a:	ce9d                	beqz	a3,83002fa8 <z_impl_k_sleep+0x4c>
    83002f6c:	00006597          	auipc	a1,0x6
    83002f70:	ffc58593          	addi	a1,a1,-4 # 83008f68 <levels.0+0x1c8>
    83002f74:	4a100693          	li	a3,1185
    83002f78:	00006617          	auipc	a2,0x6
    83002f7c:	09060613          	addi	a2,a2,144 # 83009008 <levels.0+0x268>
    83002f80:	00005517          	auipc	a0,0x5
    83002f84:	29850513          	addi	a0,a0,664 # 83008218 <__clz_tab+0x1e0>
    83002f88:	f96fd0ef          	jal	8300071e <assert_print>
    83002f8c:	00006517          	auipc	a0,0x6
    83002f90:	07450513          	addi	a0,a0,116 # 83009000 <levels.0+0x260>
    83002f94:	f8afd0ef          	jal	8300071e <assert_print>
    83002f98:	4a100593          	li	a1,1185
    83002f9c:	00006517          	auipc	a0,0x6
    83002fa0:	06c50513          	addi	a0,a0,108 # 83009008 <levels.0+0x268>
    83002fa4:	f70fd0ef          	jal	83000714 <assert_post_action>
	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
    83002fa8:	56fd                	li	a3,-1
    83002faa:	00d51963          	bne	a0,a3,83002fbc <z_impl_k_sleep+0x60>
		k_thread_suspend(_current);
    83002fae:	6b08                	ld	a0,16(a4)
		(void) arch_syscall_invoke1(parm0.x, K_SYSCALL_K_THREAD_SUSPEND);
		return;
	}
#endif
	compiler_barrier();
	z_impl_k_thread_suspend(thread);
    83002fb0:	d30ff0ef          	jal	830024e0 <z_impl_k_thread_suspend>
		return (int32_t) K_TICKS_FOREVER;
    83002fb4:	557d                	li	a0,-1
}
    83002fb6:	60a2                	ld	ra,8(sp)
    83002fb8:	0141                	addi	sp,sp,16
    83002fba:	8082                	ret
	ticks = z_tick_sleep(ticks);
    83002fbc:	e27ff0ef          	jal	83002de2 <z_tick_sleep>
	int32_t ret = k_ticks_to_ms_ceil64(ticks);
    83002fc0:	47a9                	li	a5,10
    83002fc2:	02a7853b          	mulw	a0,a5,a0
	return ret;
    83002fc6:	bfc5                	j	83002fb6 <z_impl_k_sleep+0x5a>

0000000083002fc8 <z_impl_k_sched_current_thread_query>:
}
    83002fc8:	00009517          	auipc	a0,0x9
    83002fcc:	97853503          	ld	a0,-1672(a0) # 8300b940 <_kernel+0x10>
    83002fd0:	8082                	ret

0000000083002fd2 <z_thread_abort>:
		clear_halting(thread);
	}
}

void z_thread_abort(struct k_thread *thread)
{
    83002fd2:	1101                	addi	sp,sp,-32
    83002fd4:	e426                	sd	s1,8(sp)
    83002fd6:	ec06                	sd	ra,24(sp)
    83002fd8:	e822                	sd	s0,16(sp)
    83002fda:	84aa                	mv	s1,a0
    83002fdc:	10017473          	csrrci	s0,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83002fe0:	00006517          	auipc	a0,0x6
    83002fe4:	52050513          	addi	a0,a0,1312 # 83009500 <_sched_spinlock>
    83002fe8:	38a000ef          	jal	83003372 <z_spin_lock_valid>
    83002fec:	e139                	bnez	a0,83003032 <z_thread_abort+0x60>
    83002fee:	08800693          	li	a3,136
    83002ff2:	00006617          	auipc	a2,0x6
    83002ff6:	a2660613          	addi	a2,a2,-1498 # 83008a18 <plic_config_0+0x38>
    83002ffa:	00006597          	auipc	a1,0x6
    83002ffe:	a4e58593          	addi	a1,a1,-1458 # 83008a48 <plic_config_0+0x68>
    83003002:	00005517          	auipc	a0,0x5
    83003006:	21650513          	addi	a0,a0,534 # 83008218 <__clz_tab+0x1e0>
    8300300a:	f14fd0ef          	jal	8300071e <assert_print>
    8300300e:	00006597          	auipc	a1,0x6
    83003012:	4f258593          	addi	a1,a1,1266 # 83009500 <_sched_spinlock>
    83003016:	00006517          	auipc	a0,0x6
    8300301a:	a4a50513          	addi	a0,a0,-1462 # 83008a60 <plic_config_0+0x80>
    8300301e:	f00fd0ef          	jal	8300071e <assert_print>
    83003022:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83003026:	00006517          	auipc	a0,0x6
    8300302a:	9f250513          	addi	a0,a0,-1550 # 83008a18 <plic_config_0+0x38>
    8300302e:	ee6fd0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    83003032:	00006517          	auipc	a0,0x6
    83003036:	4ce50513          	addi	a0,a0,1230 # 83009500 <_sched_spinlock>
    8300303a:	380000ef          	jal	830033ba <z_spin_lock_set_owner>
	return (thread->base.user_options & K_ESSENTIAL) == K_ESSENTIAL;
    8300303e:	0184c783          	lbu	a5,24(s1)
    83003042:	2401                	sext.w	s0,s0
	k_spinlock_key_t key = k_spin_lock(&_sched_spinlock);

	if (z_is_thread_essential(thread)) {
    83003044:	8b85                	andi	a5,a5,1
    83003046:	c7d1                	beqz	a5,830030d2 <z_thread_abort+0x100>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83003048:	00006517          	auipc	a0,0x6
    8300304c:	4b850513          	addi	a0,a0,1208 # 83009500 <_sched_spinlock>
    83003050:	33c000ef          	jal	8300338c <z_spin_unlock_valid>
    83003054:	ed15                	bnez	a0,83003090 <z_thread_abort+0xbe>
    83003056:	13100693          	li	a3,305
    8300305a:	00006617          	auipc	a2,0x6
    8300305e:	9be60613          	addi	a2,a2,-1602 # 83008a18 <plic_config_0+0x38>
    83003062:	00006597          	auipc	a1,0x6
    83003066:	a1658593          	addi	a1,a1,-1514 # 83008a78 <plic_config_0+0x98>
    8300306a:	00005517          	auipc	a0,0x5
    8300306e:	1ae50513          	addi	a0,a0,430 # 83008218 <__clz_tab+0x1e0>
    83003072:	eacfd0ef          	jal	8300071e <assert_print>
    83003076:	00006597          	auipc	a1,0x6
    8300307a:	48a58593          	addi	a1,a1,1162 # 83009500 <_sched_spinlock>
    8300307e:	00006517          	auipc	a0,0x6
    83003082:	a1250513          	addi	a0,a0,-1518 # 83008a90 <plic_config_0+0xb0>
    83003086:	e98fd0ef          	jal	8300071e <assert_print>
    8300308a:	13100593          	li	a1,305
    8300308e:	bf61                	j	83003026 <z_thread_abort+0x54>
	csr_set(xstatus, (key & XSTATUS_IEN));
    83003090:	8809                	andi	s0,s0,2
    83003092:	10042073          	csrs	sstatus,s0
		k_spin_unlock(&_sched_spinlock, key);
		__ASSERT(false, "aborting essential thread %p", thread);
    83003096:	59600693          	li	a3,1430
    8300309a:	00006617          	auipc	a2,0x6
    8300309e:	f6e60613          	addi	a2,a2,-146 # 83009008 <levels.0+0x268>
    830030a2:	00006597          	auipc	a1,0x6
    830030a6:	13658593          	addi	a1,a1,310 # 830091d8 <levels.0+0x438>
    830030aa:	00005517          	auipc	a0,0x5
    830030ae:	16e50513          	addi	a0,a0,366 # 83008218 <__clz_tab+0x1e0>
    830030b2:	e6cfd0ef          	jal	8300071e <assert_print>
    830030b6:	85a6                	mv	a1,s1
    830030b8:	00006517          	auipc	a0,0x6
    830030bc:	0d050513          	addi	a0,a0,208 # 83009188 <levels.0+0x3e8>
    830030c0:	e5efd0ef          	jal	8300071e <assert_print>
    830030c4:	59600593          	li	a1,1430
    830030c8:	00006517          	auipc	a0,0x6
    830030cc:	f4050513          	addi	a0,a0,-192 # 83009008 <levels.0+0x268>
    830030d0:	bfb9                	j	8300302e <z_thread_abort+0x5c>
		k_panic();
		return;
	}

	if ((thread->base.thread_state & _THREAD_DEAD) != 0U) {
    830030d2:	0194c783          	lbu	a5,25(s1)
    830030d6:	8ba1                	andi	a5,a5,8
    830030d8:	c385                	beqz	a5,830030f8 <z_thread_abort+0x126>
    830030da:	00006517          	auipc	a0,0x6
    830030de:	42650513          	addi	a0,a0,1062 # 83009500 <_sched_spinlock>
    830030e2:	2aa000ef          	jal	8300338c <z_spin_unlock_valid>
    830030e6:	d925                	beqz	a0,83003056 <z_thread_abort+0x84>
    830030e8:	8809                	andi	s0,s0,2
    830030ea:	10042073          	csrs	sstatus,s0
		k_spin_unlock(&_sched_spinlock, key);
		return;
	}

	z_thread_halt(thread, key, true);
}
    830030ee:	60e2                	ld	ra,24(sp)
    830030f0:	6442                	ld	s0,16(sp)
    830030f2:	64a2                	ld	s1,8(sp)
    830030f4:	6105                	addi	sp,sp,32
    830030f6:	8082                	ret
	z_thread_halt(thread, key, true);
    830030f8:	85a2                	mv	a1,s0
}
    830030fa:	6442                	ld	s0,16(sp)
    830030fc:	60e2                	ld	ra,24(sp)
	z_thread_halt(thread, key, true);
    830030fe:	8526                	mv	a0,s1
}
    83003100:	64a2                	ld	s1,8(sp)
	z_thread_halt(thread, key, true);
    83003102:	4605                	li	a2,1
}
    83003104:	6105                	addi	sp,sp,32
	z_thread_halt(thread, key, true);
    83003106:	82cff06f          	j	83002132 <z_thread_halt>

000000008300310a <z_impl_k_thread_abort>:

#if !defined(CONFIG_ARCH_HAS_THREAD_ABORT)
void z_impl_k_thread_abort(k_tid_t thread)
{
    8300310a:	1141                	addi	sp,sp,-16
    8300310c:	e022                	sd	s0,0(sp)
    8300310e:	e406                	sd	ra,8(sp)
    83003110:	842a                	mv	s0,a0
	SYS_PORT_TRACING_OBJ_FUNC_ENTER(k_thread, abort, thread);

	z_thread_abort(thread);
    83003112:	ec1ff0ef          	jal	83002fd2 <z_thread_abort>

	__ASSERT_NO_MSG((thread->base.thread_state & _THREAD_DEAD) != 0);
    83003116:	01944783          	lbu	a5,25(s0)
    8300311a:	8ba1                	andi	a5,a5,8
    8300311c:	eb8d                	bnez	a5,8300314e <z_impl_k_thread_abort+0x44>
    8300311e:	00006597          	auipc	a1,0x6
    83003122:	08a58593          	addi	a1,a1,138 # 830091a8 <levels.0+0x408>
    83003126:	00005517          	auipc	a0,0x5
    8300312a:	0f250513          	addi	a0,a0,242 # 83008218 <__clz_tab+0x1e0>
    8300312e:	5aa00693          	li	a3,1450
    83003132:	00006617          	auipc	a2,0x6
    83003136:	ed660613          	addi	a2,a2,-298 # 83009008 <levels.0+0x268>
    8300313a:	de4fd0ef          	jal	8300071e <assert_print>
    8300313e:	5aa00593          	li	a1,1450
    83003142:	00006517          	auipc	a0,0x6
    83003146:	ec650513          	addi	a0,a0,-314 # 83009008 <levels.0+0x268>
    8300314a:	dcafd0ef          	jal	83000714 <assert_post_action>

	SYS_PORT_TRACING_OBJ_FUNC_EXIT(k_thread, abort, thread);
}
    8300314e:	60a2                	ld	ra,8(sp)
    83003150:	6402                	ld	s0,0(sp)
    83003152:	0141                	addi	sp,sp,16
    83003154:	8082                	ret

0000000083003156 <slice_timeout>:
	return ret;
}

static void slice_timeout(struct _timeout *timeout)
{
	int cpu = ARRAY_INDEX(slice_timeouts, timeout);
    83003156:	c10d                	beqz	a0,83003178 <slice_timeout+0x22>
    83003158:	00009797          	auipc	a5,0x9
    8300315c:	95078793          	addi	a5,a5,-1712 # 8300baa8 <slice_timeouts>
    83003160:	00f56c63          	bltu	a0,a5,83003178 <slice_timeout+0x22>
    83003164:	00009717          	auipc	a4,0x9
    83003168:	96470713          	addi	a4,a4,-1692 # 8300bac8 <resource_table>
    8300316c:	00e57663          	bgeu	a0,a4,83003178 <slice_timeout+0x22>
    83003170:	8d1d                	sub	a0,a0,a5
    83003172:	01f57793          	andi	a5,a0,31
    83003176:	cb9d                	beqz	a5,830031ac <slice_timeout+0x56>
{
    83003178:	1141                	addi	sp,sp,-16
	int cpu = ARRAY_INDEX(slice_timeouts, timeout);
    8300317a:	00006597          	auipc	a1,0x6
    8300317e:	08e58593          	addi	a1,a1,142 # 83009208 <levels.0+0x468>
    83003182:	00005517          	auipc	a0,0x5
    83003186:	09650513          	addi	a0,a0,150 # 83008218 <__clz_tab+0x1e0>
    8300318a:	03800693          	li	a3,56
    8300318e:	00006617          	auipc	a2,0x6
    83003192:	05260613          	addi	a2,a2,82 # 830091e0 <levels.0+0x440>
{
    83003196:	e406                	sd	ra,8(sp)
	int cpu = ARRAY_INDEX(slice_timeouts, timeout);
    83003198:	d86fd0ef          	jal	8300071e <assert_print>
    8300319c:	03800593          	li	a1,56
    830031a0:	00006517          	auipc	a0,0x6
    830031a4:	04050513          	addi	a0,a0,64 # 830091e0 <levels.0+0x440>
    830031a8:	d6cfd0ef          	jal	83000714 <assert_post_action>
    830031ac:	8515                	srai	a0,a0,0x5

	slice_expired[cpu] = true;
    830031ae:	2501                	sext.w	a0,a0
    830031b0:	00006797          	auipc	a5,0x6
    830031b4:	35878793          	addi	a5,a5,856 # 83009508 <slice_expired>
    830031b8:	97aa                	add	a5,a5,a0
    830031ba:	4705                	li	a4,1
    830031bc:	00e78023          	sb	a4,0(a5)
    830031c0:	8082                	ret

00000000830031c2 <thread_is_sliceable>:
		&& !z_is_idle_thread_object(thread);
    830031c2:	01a55703          	lhu	a4,26(a0)
    830031c6:	07f00793          	li	a5,127
    830031ca:	02e7ee63          	bltu	a5,a4,83003206 <thread_is_sliceable+0x44>
	int ret = slice_ticks;
    830031ce:	00006797          	auipc	a5,0x6
    830031d2:	35a7a783          	lw	a5,858(a5) # 83009528 <slice_ticks>
		&& slice_time(thread) != 0
    830031d6:	c78d                	beqz	a5,83003200 <thread_is_sliceable+0x3e>
		&& !z_is_prio_higher(thread->base.prio, slice_max_prio)
    830031d8:	01a50683          	lb	a3,26(a0)
    830031dc:	00006717          	auipc	a4,0x6
    830031e0:	34872703          	lw	a4,840(a4) # 83009524 <slice_max_prio>
		&& !z_is_idle_thread_object(thread);
    830031e4:	4781                	li	a5,0
		&& !z_is_prio_higher(thread->base.prio, slice_max_prio)
    830031e6:	00e6cd63          	blt	a3,a4,83003200 <thread_is_sliceable+0x3e>
		&& !z_is_thread_prevented_from_running(thread)
    830031ea:	01954703          	lbu	a4,25(a0)
    830031ee:	8b7d                	andi	a4,a4,31
    830031f0:	eb01                	bnez	a4,83003200 <thread_is_sliceable+0x3e>
		&& !z_is_idle_thread_object(thread);
    830031f2:	00008797          	auipc	a5,0x8
    830031f6:	4ee78793          	addi	a5,a5,1262 # 8300b6e0 <z_idle_threads>
    830031fa:	8d1d                	sub	a0,a0,a5
    830031fc:	00a037b3          	snez	a5,a0
}
    83003200:	0017f513          	andi	a0,a5,1
    83003204:	8082                	ret
		&& !z_is_idle_thread_object(thread);
    83003206:	4781                	li	a5,0
    83003208:	bfe5                	j	83003200 <thread_is_sliceable+0x3e>

000000008300320a <z_reset_time_slice>:
		flag_ipi(IPI_CPU_MASK(cpu));
	}
}

void z_reset_time_slice(struct k_thread *thread)
{
    8300320a:	1101                	addi	sp,sp,-32
    8300320c:	e04a                	sd	s2,0(sp)
	int cpu = _current_cpu->id;
    8300320e:	00008917          	auipc	s2,0x8
    83003212:	74294903          	lbu	s2,1858(s2) # 8300b950 <_kernel+0x20>
{
    83003216:	e822                	sd	s0,16(sp)

	z_abort_timeout(&slice_timeouts[cpu]);
    83003218:	00009797          	auipc	a5,0x9
    8300321c:	89078793          	addi	a5,a5,-1904 # 8300baa8 <slice_timeouts>
    83003220:	00591413          	slli	s0,s2,0x5
    83003224:	943e                	add	s0,s0,a5
{
    83003226:	e426                	sd	s1,8(sp)
    83003228:	84aa                	mv	s1,a0
	z_abort_timeout(&slice_timeouts[cpu]);
    8300322a:	8522                	mv	a0,s0
{
    8300322c:	ec06                	sd	ra,24(sp)
	z_abort_timeout(&slice_timeouts[cpu]);
    8300322e:	3e6000ef          	jal	83003614 <z_abort_timeout>
	slice_expired[cpu] = false;
    83003232:	00006797          	auipc	a5,0x6
    83003236:	2d678793          	addi	a5,a5,726 # 83009508 <slice_expired>
    8300323a:	97ca                	add	a5,a5,s2
	if (thread_is_sliceable(thread)) {
    8300323c:	8526                	mv	a0,s1
	slice_expired[cpu] = false;
    8300323e:	00078023          	sb	zero,0(a5)
	if (thread_is_sliceable(thread)) {
    83003242:	f81ff0ef          	jal	830031c2 <thread_is_sliceable>
    83003246:	c10d                	beqz	a0,83003268 <z_reset_time_slice+0x5e>
		z_add_timeout(&slice_timeouts[cpu], slice_timeout,
    83003248:	8522                	mv	a0,s0
			      K_TICKS(slice_time(thread) - 1));
	}
}
    8300324a:	6442                	ld	s0,16(sp)
    8300324c:	60e2                	ld	ra,24(sp)
    8300324e:	64a2                	ld	s1,8(sp)
    83003250:	6902                	ld	s2,0(sp)
			      K_TICKS(slice_time(thread) - 1));
    83003252:	00006617          	auipc	a2,0x6
    83003256:	2d662603          	lw	a2,726(a2) # 83009528 <slice_ticks>
		z_add_timeout(&slice_timeouts[cpu], slice_timeout,
    8300325a:	367d                	addiw	a2,a2,-1
    8300325c:	00000597          	auipc	a1,0x0
    83003260:	efa58593          	addi	a1,a1,-262 # 83003156 <slice_timeout>
}
    83003264:	6105                	addi	sp,sp,32
		z_add_timeout(&slice_timeouts[cpu], slice_timeout,
    83003266:	aac5                	j	83003456 <z_add_timeout>
}
    83003268:	60e2                	ld	ra,24(sp)
    8300326a:	6442                	ld	s0,16(sp)
    8300326c:	64a2                	ld	s1,8(sp)
    8300326e:	6902                	ld	s2,0(sp)
    83003270:	6105                	addi	sp,sp,32
    83003272:	8082                	ret

0000000083003274 <z_time_slice>:
}
#endif

/* Called out of each timer interrupt */
void z_time_slice(void)
{
    83003274:	1101                	addi	sp,sp,-32
    83003276:	ec06                	sd	ra,24(sp)
    83003278:	e822                	sd	s0,16(sp)
    8300327a:	e426                	sd	s1,8(sp)
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    8300327c:	10017473          	csrrci	s0,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83003280:	00006517          	auipc	a0,0x6
    83003284:	28050513          	addi	a0,a0,640 # 83009500 <_sched_spinlock>
    83003288:	0ea000ef          	jal	83003372 <z_spin_lock_valid>
    8300328c:	e139                	bnez	a0,830032d2 <z_time_slice+0x5e>
    8300328e:	08800693          	li	a3,136
    83003292:	00005617          	auipc	a2,0x5
    83003296:	78660613          	addi	a2,a2,1926 # 83008a18 <plic_config_0+0x38>
    8300329a:	00005597          	auipc	a1,0x5
    8300329e:	7ae58593          	addi	a1,a1,1966 # 83008a48 <plic_config_0+0x68>
    830032a2:	00005517          	auipc	a0,0x5
    830032a6:	f7650513          	addi	a0,a0,-138 # 83008218 <__clz_tab+0x1e0>
    830032aa:	c74fd0ef          	jal	8300071e <assert_print>
    830032ae:	00006597          	auipc	a1,0x6
    830032b2:	25258593          	addi	a1,a1,594 # 83009500 <_sched_spinlock>
    830032b6:	00005517          	auipc	a0,0x5
    830032ba:	7aa50513          	addi	a0,a0,1962 # 83008a60 <plic_config_0+0x80>
    830032be:	c60fd0ef          	jal	8300071e <assert_print>
    830032c2:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    830032c6:	00005517          	auipc	a0,0x5
    830032ca:	75250513          	addi	a0,a0,1874 # 83008a18 <plic_config_0+0x38>
    830032ce:	c46fd0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    830032d2:	00006517          	auipc	a0,0x6
    830032d6:	22e50513          	addi	a0,a0,558 # 83009500 <_sched_spinlock>
    830032da:	0e0000ef          	jal	830033ba <z_spin_lock_set_owner>
		return;
	}
	pending_current = NULL;
#endif

	if (slice_expired[_current_cpu->id] && thread_is_sliceable(curr)) {
    830032de:	00008717          	auipc	a4,0x8
    830032e2:	65270713          	addi	a4,a4,1618 # 8300b930 <_kernel>
    830032e6:	02074683          	lbu	a3,32(a4)
    830032ea:	00006797          	auipc	a5,0x6
    830032ee:	21e78793          	addi	a5,a5,542 # 83009508 <slice_expired>
    830032f2:	2401                	sext.w	s0,s0
    830032f4:	97b6                	add	a5,a5,a3
    830032f6:	0007c783          	lbu	a5,0(a5)
    830032fa:	c385                	beqz	a5,8300331a <z_time_slice+0xa6>
	struct k_thread *curr = _current;
    830032fc:	6b04                	ld	s1,16(a4)
	if (slice_expired[_current_cpu->id] && thread_is_sliceable(curr)) {
    830032fe:	8526                	mv	a0,s1
    83003300:	ec3ff0ef          	jal	830031c2 <thread_is_sliceable>
    83003304:	c919                	beqz	a0,8300331a <z_time_slice+0xa6>
			k_spin_unlock(&_sched_spinlock, key);
			curr->base.slice_expired(curr, curr->base.slice_data);
			key = k_spin_lock(&_sched_spinlock);
		}
#endif
		if (!z_is_thread_prevented_from_running(curr)) {
    83003306:	0194c783          	lbu	a5,25(s1)
    8300330a:	8bfd                	andi	a5,a5,31
    8300330c:	e781                	bnez	a5,83003314 <z_time_slice+0xa0>
			move_thread_to_end_of_prio_q(curr);
    8300330e:	8526                	mv	a0,s1
    83003310:	810ff0ef          	jal	83002320 <move_thread_to_end_of_prio_q>
		}
		z_reset_time_slice(curr);
    83003314:	8526                	mv	a0,s1
    83003316:	ef5ff0ef          	jal	8300320a <z_reset_time_slice>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    8300331a:	00006517          	auipc	a0,0x6
    8300331e:	1e650513          	addi	a0,a0,486 # 83009500 <_sched_spinlock>
    83003322:	06a000ef          	jal	8300338c <z_spin_unlock_valid>
    83003326:	ed15                	bnez	a0,83003362 <z_time_slice+0xee>
    83003328:	13100693          	li	a3,305
    8300332c:	00005617          	auipc	a2,0x5
    83003330:	6ec60613          	addi	a2,a2,1772 # 83008a18 <plic_config_0+0x38>
    83003334:	00005597          	auipc	a1,0x5
    83003338:	74458593          	addi	a1,a1,1860 # 83008a78 <plic_config_0+0x98>
    8300333c:	00005517          	auipc	a0,0x5
    83003340:	edc50513          	addi	a0,a0,-292 # 83008218 <__clz_tab+0x1e0>
    83003344:	bdafd0ef          	jal	8300071e <assert_print>
    83003348:	00006597          	auipc	a1,0x6
    8300334c:	1b858593          	addi	a1,a1,440 # 83009500 <_sched_spinlock>
    83003350:	00005517          	auipc	a0,0x5
    83003354:	74050513          	addi	a0,a0,1856 # 83008a90 <plic_config_0+0xb0>
    83003358:	bc6fd0ef          	jal	8300071e <assert_print>
    8300335c:	13100593          	li	a1,305
    83003360:	b79d                	j	830032c6 <z_time_slice+0x52>
	csr_set(xstatus, (key & XSTATUS_IEN));
    83003362:	8809                	andi	s0,s0,2
    83003364:	10042073          	csrs	sstatus,s0
	}
	k_spin_unlock(&_sched_spinlock, key);
}
    83003368:	60e2                	ld	ra,24(sp)
    8300336a:	6442                	ld	s0,16(sp)
    8300336c:	64a2                	ld	s1,8(sp)
    8300336e:	6105                	addi	sp,sp,32
    83003370:	8082                	ret

0000000083003372 <z_spin_lock_valid>:
#include <kernel_internal.h>
#include <zephyr/spinlock.h>

bool z_spin_lock_valid(struct k_spinlock *l)
{
	uintptr_t thread_cpu = l->thread_cpu;
    83003372:	6108                	ld	a0,0(a0)

	if (thread_cpu != 0U) {
    83003374:	c911                	beqz	a0,83003388 <z_spin_lock_valid+0x16>
		if ((thread_cpu & 3U) == _current_cpu->id) {
    83003376:	00008797          	auipc	a5,0x8
    8300337a:	5da7c783          	lbu	a5,1498(a5) # 8300b950 <_kernel+0x20>
    8300337e:	890d                	andi	a0,a0,3
    83003380:	8d1d                	sub	a0,a0,a5
    83003382:	00a03533          	snez	a0,a0
    83003386:	8082                	ret
			return false;
		}
	}
	return true;
    83003388:	4505                	li	a0,1
}
    8300338a:	8082                	ret

000000008300338c <z_spin_unlock_valid>:

bool z_spin_unlock_valid(struct k_spinlock *l)
{
	uintptr_t tcpu = l->thread_cpu;
    8300338c:	6114                	ld	a3,0(a0)
    8300338e:	00008797          	auipc	a5,0x8
    83003392:	5a278793          	addi	a5,a5,1442 # 8300b930 <_kernel>

	l->thread_cpu = 0;
    83003396:	00053023          	sd	zero,0(a0)

	if (arch_is_in_isr() && _current->base.thread_state & _THREAD_DUMMY) {
    8300339a:	4398                	lw	a4,0(a5)
    8300339c:	c719                	beqz	a4,830033aa <z_spin_unlock_valid+0x1e>
    8300339e:	6b98                	ld	a4,16(a5)
		/* Edge case where an ISR aborted _current */
		return true;
    830033a0:	4505                	li	a0,1
	if (arch_is_in_isr() && _current->base.thread_state & _THREAD_DUMMY) {
    830033a2:	01974703          	lbu	a4,25(a4)
    830033a6:	8b05                	andi	a4,a4,1
    830033a8:	eb01                	bnez	a4,830033b8 <z_spin_unlock_valid+0x2c>
	}
	if (tcpu != (_current_cpu->id | (uintptr_t)_current)) {
    830033aa:	0207c703          	lbu	a4,32(a5)
    830033ae:	6b88                	ld	a0,16(a5)
    830033b0:	8d59                	or	a0,a0,a4
    830033b2:	8d15                	sub	a0,a0,a3
    830033b4:	00153513          	seqz	a0,a0
		return false;
	}
	return true;
}
    830033b8:	8082                	ret

00000000830033ba <z_spin_lock_set_owner>:

void z_spin_lock_set_owner(struct k_spinlock *l)
{
	l->thread_cpu = _current_cpu->id | (uintptr_t)_current;
    830033ba:	00008797          	auipc	a5,0x8
    830033be:	57678793          	addi	a5,a5,1398 # 8300b930 <_kernel>
    830033c2:	0207c703          	lbu	a4,32(a5)
    830033c6:	6b9c                	ld	a5,16(a5)
    830033c8:	8fd9                	or	a5,a5,a4
    830033ca:	e11c                	sd	a5,0(a0)
}
    830033cc:	8082                	ret

00000000830033ce <elapsed>:
	 *
	 * The distinction is implemented by looking at announce_remaining which
	 * will be non-zero while sys_clock_announce() is executing and zero
	 * otherwise.
	 */
	return announce_remaining == 0 ? sys_clock_elapsed() : 0U;
    830033ce:	00006797          	auipc	a5,0x6
    830033d2:	15e7a783          	lw	a5,350(a5) # 8300952c <announce_remaining>
    830033d6:	e399                	bnez	a5,830033dc <elapsed+0xe>
    830033d8:	96efe06f          	j	83001546 <sys_clock_elapsed>
}
    830033dc:	4501                	li	a0,0
    830033de:	8082                	ret

00000000830033e0 <remove_timeout>:
	return (node != NULL) ? sys_dlist_peek_next_no_check(list, node) : NULL;
    830033e0:	cd09                	beqz	a0,830033fa <remove_timeout+0x1a>
	return (node == list->tail) ? NULL : node->next;
    830033e2:	0001d797          	auipc	a5,0x1d
    830033e6:	88e7b783          	ld	a5,-1906(a5) # 8301fc70 <timeout_list+0x8>
    830033ea:	00f50863          	beq	a0,a5,830033fa <remove_timeout+0x1a>
    830033ee:	611c                	ld	a5,0(a0)
	if (next(t) != NULL) {
    830033f0:	c789                	beqz	a5,830033fa <remove_timeout+0x1a>
		next(t)->dticks += t->dticks;
    830033f2:	6f98                	ld	a4,24(a5)
    830033f4:	6d14                	ld	a3,24(a0)
    830033f6:	9736                	add	a4,a4,a3
    830033f8:	ef98                	sd	a4,24(a5)
	sys_dnode_t *const prev = node->prev;
    830033fa:	6518                	ld	a4,8(a0)
	sys_dnode_t *const next = node->next;
    830033fc:	611c                	ld	a5,0(a0)
	prev->next = next;
    830033fe:	e31c                	sd	a5,0(a4)
	next->prev = prev;
    83003400:	e798                	sd	a4,8(a5)
	node->next = NULL;
    83003402:	00053023          	sd	zero,0(a0)
	node->prev = NULL;
    83003406:	00053423          	sd	zero,8(a0)
}
    8300340a:	8082                	ret

000000008300340c <next_timeout>:

static int32_t next_timeout(void)
{
    8300340c:	1141                	addi	sp,sp,-16
    8300340e:	e406                	sd	ra,8(sp)
    83003410:	e022                	sd	s0,0(sp)
	return list->head == list;
    83003412:	0001d797          	auipc	a5,0x1d
    83003416:	85678793          	addi	a5,a5,-1962 # 8301fc68 <timeout_list>
    8300341a:	6380                	ld	s0,0(a5)
	return sys_dlist_is_empty(list) ? NULL : list->head;
    8300341c:	00f41c63          	bne	s0,a5,83003434 <next_timeout+0x28>
	struct _timeout *to = first();
	int32_t ticks_elapsed = elapsed();
    83003420:	fafff0ef          	jal	830033ce <elapsed>
	int32_t ret;

	if ((to == NULL) ||
	    ((int64_t)(to->dticks - ticks_elapsed) > (int64_t)INT_MAX)) {
		ret = MAX_WAIT;
    83003424:	80000537          	lui	a0,0x80000
    83003428:	fff54513          	not	a0,a0
	} else {
		ret = MAX(0, to->dticks - ticks_elapsed);
	}

	return ret;
}
    8300342c:	60a2                	ld	ra,8(sp)
    8300342e:	6402                	ld	s0,0(sp)
    83003430:	0141                	addi	sp,sp,16
    83003432:	8082                	ret
	int32_t ticks_elapsed = elapsed();
    83003434:	f9bff0ef          	jal	830033ce <elapsed>
	if ((to == NULL) ||
    83003438:	d475                	beqz	s0,83003424 <next_timeout+0x18>
	    ((int64_t)(to->dticks - ticks_elapsed) > (int64_t)INT_MAX)) {
    8300343a:	6c1c                	ld	a5,24(s0)
    8300343c:	40a78533          	sub	a0,a5,a0
	if ((to == NULL) ||
    83003440:	800007b7          	lui	a5,0x80000
    83003444:	fff7c793          	not	a5,a5
    83003448:	fca7cee3          	blt	a5,a0,83003424 <next_timeout+0x18>
		ret = MAX(0, to->dticks - ticks_elapsed);
    8300344c:	00055363          	bgez	a0,83003452 <next_timeout+0x46>
    83003450:	4501                	li	a0,0
    83003452:	2501                	sext.w	a0,a0
    83003454:	bfe1                	j	8300342c <next_timeout+0x20>

0000000083003456 <z_add_timeout>:

void z_add_timeout(struct _timeout *to, _timeout_func_t fn,
		   k_timeout_t timeout)
{
	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
    83003456:	57fd                	li	a5,-1
    83003458:	1af60d63          	beq	a2,a5,83003612 <z_add_timeout+0x1bc>
{
    8300345c:	7179                	addi	sp,sp,-48
    8300345e:	f022                	sd	s0,32(sp)
    83003460:	f406                	sd	ra,40(sp)
    83003462:	ec26                	sd	s1,24(sp)
    83003464:	e84a                	sd	s2,16(sp)
    83003466:	e44e                	sd	s3,8(sp)
	return node->next != NULL;
    83003468:	00053903          	ld	s2,0(a0) # ffffffff80000000 <CONFIG_RISCV_MCAUSE_EXCEPTION_MASK+0x7fffffff80000001>
    8300346c:	842a                	mv	s0,a0

#ifdef CONFIG_KERNEL_COHERENCE
	__ASSERT_NO_MSG(arch_mem_coherent(to));
#endif /* CONFIG_KERNEL_COHERENCE */

	__ASSERT(!sys_dnode_is_linked(&to->node), "");
    8300346e:	04090063          	beqz	s2,830034ae <z_add_timeout+0x58>
    83003472:	00006597          	auipc	a1,0x6
    83003476:	f7658593          	addi	a1,a1,-138 # 830093e8 <levels.0+0x648>
    8300347a:	06e00693          	li	a3,110
    8300347e:	00006617          	auipc	a2,0x6
    83003482:	f4260613          	addi	a2,a2,-190 # 830093c0 <levels.0+0x620>
    83003486:	00005517          	auipc	a0,0x5
    8300348a:	d9250513          	addi	a0,a0,-622 # 83008218 <__clz_tab+0x1e0>
    8300348e:	a90fd0ef          	jal	8300071e <assert_print>
    83003492:	00006517          	auipc	a0,0x6
    83003496:	b6e50513          	addi	a0,a0,-1170 # 83009000 <levels.0+0x260>
    8300349a:	a84fd0ef          	jal	8300071e <assert_print>
    8300349e:	06e00593          	li	a1,110
    830034a2:	00006517          	auipc	a0,0x6
    830034a6:	f1e50513          	addi	a0,a0,-226 # 830093c0 <levels.0+0x620>
    830034aa:	a6afd0ef          	jal	83000714 <assert_post_action>
    830034ae:	84b2                	mv	s1,a2
	to->fn = fn;
    830034b0:	e90c                	sd	a1,16(a0)
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    830034b2:	100179f3          	csrrci	s3,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    830034b6:	00006517          	auipc	a0,0x6
    830034ba:	05a50513          	addi	a0,a0,90 # 83009510 <timeout_lock>
	k.key = arch_irq_lock();
    830034be:	2981                	sext.w	s3,s3
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    830034c0:	eb3ff0ef          	jal	83003372 <z_spin_lock_valid>
    830034c4:	e139                	bnez	a0,8300350a <z_add_timeout+0xb4>
    830034c6:	08800693          	li	a3,136
    830034ca:	00005617          	auipc	a2,0x5
    830034ce:	54e60613          	addi	a2,a2,1358 # 83008a18 <plic_config_0+0x38>
    830034d2:	00005597          	auipc	a1,0x5
    830034d6:	57658593          	addi	a1,a1,1398 # 83008a48 <plic_config_0+0x68>
    830034da:	00005517          	auipc	a0,0x5
    830034de:	d3e50513          	addi	a0,a0,-706 # 83008218 <__clz_tab+0x1e0>
    830034e2:	a3cfd0ef          	jal	8300071e <assert_print>
    830034e6:	00006597          	auipc	a1,0x6
    830034ea:	02a58593          	addi	a1,a1,42 # 83009510 <timeout_lock>
    830034ee:	00005517          	auipc	a0,0x5
    830034f2:	57250513          	addi	a0,a0,1394 # 83008a60 <plic_config_0+0x80>
    830034f6:	a28fd0ef          	jal	8300071e <assert_print>
    830034fa:	08800593          	li	a1,136
    830034fe:	00005517          	auipc	a0,0x5
    83003502:	51a50513          	addi	a0,a0,1306 # 83008a18 <plic_config_0+0x38>
    83003506:	a0efd0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    8300350a:	00006517          	auipc	a0,0x6
    8300350e:	00650513          	addi	a0,a0,6 # 83009510 <timeout_lock>
    83003512:	ea9ff0ef          	jal	830033ba <z_spin_lock_set_owner>

	K_SPINLOCK(&timeout_lock) {
		struct _timeout *t;

		if (IS_ENABLED(CONFIG_TIMEOUT_64BIT) &&
    83003516:	57f9                	li	a5,-2
    83003518:	0297ce63          	blt	a5,s1,83003554 <z_add_timeout+0xfe>
		    (Z_TICK_ABS(timeout.ticks) >= 0)) {
			k_ticks_t ticks = Z_TICK_ABS(timeout.ticks) - curr_tick;
    8300351c:	00006717          	auipc	a4,0x6
    83003520:	ffc73703          	ld	a4,-4(a4) # 83009518 <curr_tick>
    83003524:	8f99                	sub	a5,a5,a4
    83003526:	409784b3          	sub	s1,a5,s1

			to->dticks = MAX(1, ticks);
    8300352a:	00904363          	bgtz	s1,83003530 <z_add_timeout+0xda>
    8300352e:	4485                	li	s1,1
	return list->head == list;
    83003530:	0001c797          	auipc	a5,0x1c
    83003534:	73878793          	addi	a5,a5,1848 # 8301fc68 <timeout_list>
    83003538:	6398                	ld	a4,0(a5)
    8300353a:	ec04                	sd	s1,24(s0)
    8300353c:	85be                	mv	a1,a5
	return sys_dlist_is_empty(list) ? NULL : list->head;
    8300353e:	00f70363          	beq	a4,a5,83003544 <z_add_timeout+0xee>
    83003542:	893a                	mv	s2,a4
	return (node == list->tail) ? NULL : node->next;
    83003544:	6790                	ld	a2,8(a5)
		} else {
			to->dticks = timeout.ticks + 1 + elapsed();
		}

		for (t = first(); t != NULL; t = next(t)) {
    83003546:	00091c63          	bnez	s2,8300355e <z_add_timeout+0x108>
	node->next = list;
    8300354a:	e01c                	sd	a5,0(s0)
	node->prev = tail;
    8300354c:	e410                	sd	a2,8(s0)
	tail->next = node;
    8300354e:	e200                	sd	s0,0(a2)
	list->tail = node;
    83003550:	e780                	sd	s0,8(a5)
}
    83003552:	a035                	j	8300357e <z_add_timeout+0x128>
			to->dticks = timeout.ticks + 1 + elapsed();
    83003554:	e7bff0ef          	jal	830033ce <elapsed>
    83003558:	0485                	addi	s1,s1,1
    8300355a:	94aa                	add	s1,s1,a0
    8300355c:	bfd1                	j	83003530 <z_add_timeout+0xda>
			if (t->dticks > to->dticks) {
    8300355e:	01893683          	ld	a3,24(s2)
    83003562:	6c18                	ld	a4,24(s0)
    83003564:	08d75563          	bge	a4,a3,830035ee <z_add_timeout+0x198>
				t->dticks -= to->dticks;
    83003568:	8e99                	sub	a3,a3,a4
	sys_dnode_t *const prev = successor->prev;
    8300356a:	00893703          	ld	a4,8(s2)
    8300356e:	00d93c23          	sd	a3,24(s2)
	node->next = successor;
    83003572:	01243023          	sd	s2,0(s0)
	node->prev = prev;
    83003576:	e418                	sd	a4,8(s0)
	prev->next = node;
    83003578:	e300                	sd	s0,0(a4)
	successor->prev = node;
    8300357a:	00893423          	sd	s0,8(s2)
	return list->head == list;
    8300357e:	639c                	ld	a5,0(a5)
	return sys_dlist_is_empty(list) ? NULL : list->head;
    83003580:	00b78e63          	beq	a5,a1,8300359c <z_add_timeout+0x146>

		if (t == NULL) {
			sys_dlist_append(&timeout_list, &to->node);
		}

		if (to == first() && announce_remaining == 0) {
    83003584:	00f41c63          	bne	s0,a5,8300359c <z_add_timeout+0x146>
    83003588:	00006797          	auipc	a5,0x6
    8300358c:	fa47a783          	lw	a5,-92(a5) # 8300952c <announce_remaining>
    83003590:	e791                	bnez	a5,8300359c <z_add_timeout+0x146>
			sys_clock_set_timeout(next_timeout(), false);
    83003592:	e7bff0ef          	jal	8300340c <next_timeout>
    83003596:	4581                	li	a1,0
    83003598:	e7ffd0ef          	jal	83001416 <sys_clock_set_timeout>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    8300359c:	00006517          	auipc	a0,0x6
    830035a0:	f7450513          	addi	a0,a0,-140 # 83009510 <timeout_lock>
    830035a4:	de9ff0ef          	jal	8300338c <z_spin_unlock_valid>
    830035a8:	e931                	bnez	a0,830035fc <z_add_timeout+0x1a6>
    830035aa:	13100693          	li	a3,305
    830035ae:	00005617          	auipc	a2,0x5
    830035b2:	46a60613          	addi	a2,a2,1130 # 83008a18 <plic_config_0+0x38>
    830035b6:	00005597          	auipc	a1,0x5
    830035ba:	4c258593          	addi	a1,a1,1218 # 83008a78 <plic_config_0+0x98>
    830035be:	00005517          	auipc	a0,0x5
    830035c2:	c5a50513          	addi	a0,a0,-934 # 83008218 <__clz_tab+0x1e0>
    830035c6:	958fd0ef          	jal	8300071e <assert_print>
    830035ca:	00006597          	auipc	a1,0x6
    830035ce:	f4658593          	addi	a1,a1,-186 # 83009510 <timeout_lock>
    830035d2:	00005517          	auipc	a0,0x5
    830035d6:	4be50513          	addi	a0,a0,1214 # 83008a90 <plic_config_0+0xb0>
    830035da:	944fd0ef          	jal	8300071e <assert_print>
    830035de:	13100593          	li	a1,305
    830035e2:	00005517          	auipc	a0,0x5
    830035e6:	43650513          	addi	a0,a0,1078 # 83008a18 <plic_config_0+0x38>
    830035ea:	92afd0ef          	jal	83000714 <assert_post_action>
			to->dticks -= t->dticks;
    830035ee:	8f15                	sub	a4,a4,a3
    830035f0:	ec18                	sd	a4,24(s0)
	return (node == list->tail) ? NULL : node->next;
    830035f2:	f4c90ce3          	beq	s2,a2,8300354a <z_add_timeout+0xf4>
    830035f6:	00093903          	ld	s2,0(s2)
    830035fa:	b7b1                	j	83003546 <z_add_timeout+0xf0>
	csr_set(xstatus, (key & XSTATUS_IEN));
    830035fc:	0029f993          	andi	s3,s3,2
    83003600:	1009a073          	csrs	sstatus,s3
		}
	}
}
    83003604:	70a2                	ld	ra,40(sp)
    83003606:	7402                	ld	s0,32(sp)
    83003608:	64e2                	ld	s1,24(sp)
    8300360a:	6942                	ld	s2,16(sp)
    8300360c:	69a2                	ld	s3,8(sp)
    8300360e:	6145                	addi	sp,sp,48
    83003610:	8082                	ret
    83003612:	8082                	ret

0000000083003614 <z_abort_timeout>:

int z_abort_timeout(struct _timeout *to)
{
    83003614:	1101                	addi	sp,sp,-32
    83003616:	e426                	sd	s1,8(sp)
    83003618:	ec06                	sd	ra,24(sp)
    8300361a:	e822                	sd	s0,16(sp)
    8300361c:	e04a                	sd	s2,0(sp)
    8300361e:	84aa                	mv	s1,a0
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83003620:	10017473          	csrrci	s0,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83003624:	00006517          	auipc	a0,0x6
    83003628:	eec50513          	addi	a0,a0,-276 # 83009510 <timeout_lock>
    8300362c:	d47ff0ef          	jal	83003372 <z_spin_lock_valid>
    83003630:	e139                	bnez	a0,83003676 <z_abort_timeout+0x62>
    83003632:	08800693          	li	a3,136
    83003636:	00005617          	auipc	a2,0x5
    8300363a:	3e260613          	addi	a2,a2,994 # 83008a18 <plic_config_0+0x38>
    8300363e:	00005597          	auipc	a1,0x5
    83003642:	40a58593          	addi	a1,a1,1034 # 83008a48 <plic_config_0+0x68>
    83003646:	00005517          	auipc	a0,0x5
    8300364a:	bd250513          	addi	a0,a0,-1070 # 83008218 <__clz_tab+0x1e0>
    8300364e:	8d0fd0ef          	jal	8300071e <assert_print>
    83003652:	00006597          	auipc	a1,0x6
    83003656:	ebe58593          	addi	a1,a1,-322 # 83009510 <timeout_lock>
    8300365a:	00005517          	auipc	a0,0x5
    8300365e:	40650513          	addi	a0,a0,1030 # 83008a60 <plic_config_0+0x80>
    83003662:	8bcfd0ef          	jal	8300071e <assert_print>
    83003666:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    8300366a:	00005517          	auipc	a0,0x5
    8300366e:	3ae50513          	addi	a0,a0,942 # 83008a18 <plic_config_0+0x38>
    83003672:	8a2fd0ef          	jal	83000714 <assert_post_action>
	z_spin_lock_set_owner(l);
    83003676:	00006517          	auipc	a0,0x6
    8300367a:	e9a50513          	addi	a0,a0,-358 # 83009510 <timeout_lock>
    8300367e:	d3dff0ef          	jal	830033ba <z_spin_lock_set_owner>
	int ret = -EINVAL;

	K_SPINLOCK(&timeout_lock) {
		if (sys_dnode_is_linked(&to->node)) {
    83003682:	609c                	ld	a5,0(s1)
    83003684:	2401                	sext.w	s0,s0
	int ret = -EINVAL;
    83003686:	5929                	li	s2,-22
		if (sys_dnode_is_linked(&to->node)) {
    83003688:	c789                	beqz	a5,83003692 <z_abort_timeout+0x7e>
			remove_timeout(to);
    8300368a:	8526                	mv	a0,s1
    8300368c:	d55ff0ef          	jal	830033e0 <remove_timeout>
			ret = 0;
    83003690:	4901                	li	s2,0
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83003692:	00006517          	auipc	a0,0x6
    83003696:	e7e50513          	addi	a0,a0,-386 # 83009510 <timeout_lock>
    8300369a:	cf3ff0ef          	jal	8300338c <z_spin_unlock_valid>
    8300369e:	ed15                	bnez	a0,830036da <z_abort_timeout+0xc6>
    830036a0:	13100693          	li	a3,305
    830036a4:	00005617          	auipc	a2,0x5
    830036a8:	37460613          	addi	a2,a2,884 # 83008a18 <plic_config_0+0x38>
    830036ac:	00005597          	auipc	a1,0x5
    830036b0:	3cc58593          	addi	a1,a1,972 # 83008a78 <plic_config_0+0x98>
    830036b4:	00005517          	auipc	a0,0x5
    830036b8:	b6450513          	addi	a0,a0,-1180 # 83008218 <__clz_tab+0x1e0>
    830036bc:	862fd0ef          	jal	8300071e <assert_print>
    830036c0:	00006597          	auipc	a1,0x6
    830036c4:	e5058593          	addi	a1,a1,-432 # 83009510 <timeout_lock>
    830036c8:	00005517          	auipc	a0,0x5
    830036cc:	3c850513          	addi	a0,a0,968 # 83008a90 <plic_config_0+0xb0>
    830036d0:	84efd0ef          	jal	8300071e <assert_print>
    830036d4:	13100593          	li	a1,305
    830036d8:	bf49                	j	8300366a <z_abort_timeout+0x56>
	csr_set(xstatus, (key & XSTATUS_IEN));
    830036da:	8809                	andi	s0,s0,2
    830036dc:	10042073          	csrs	sstatus,s0
		}
	}

	return ret;
}
    830036e0:	60e2                	ld	ra,24(sp)
    830036e2:	6442                	ld	s0,16(sp)
    830036e4:	64a2                	ld	s1,8(sp)
    830036e6:	854a                	mv	a0,s2
    830036e8:	6902                	ld	s2,0(sp)
    830036ea:	6105                	addi	sp,sp,32
    830036ec:	8082                	ret

00000000830036ee <sys_clock_announce>:
	}
	return ret;
}

void sys_clock_announce(int32_t ticks)
{
    830036ee:	715d                	addi	sp,sp,-80
    830036f0:	e0a2                	sd	s0,64(sp)
    830036f2:	e486                	sd	ra,72(sp)
    830036f4:	fc26                	sd	s1,56(sp)
    830036f6:	f84a                	sd	s2,48(sp)
    830036f8:	f44e                	sd	s3,40(sp)
    830036fa:	f052                	sd	s4,32(sp)
    830036fc:	ec56                	sd	s5,24(sp)
    830036fe:	e85a                	sd	s6,16(sp)
    83003700:	e45e                	sd	s7,8(sp)
    83003702:	842a                	mv	s0,a0
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83003704:	100174f3          	csrrci	s1,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83003708:	00006517          	auipc	a0,0x6
    8300370c:	e0850513          	addi	a0,a0,-504 # 83009510 <timeout_lock>
    83003710:	c63ff0ef          	jal	83003372 <z_spin_lock_valid>
    83003714:	e139                	bnez	a0,8300375a <sys_clock_announce+0x6c>
    83003716:	08800693          	li	a3,136
    8300371a:	00005617          	auipc	a2,0x5
    8300371e:	2fe60613          	addi	a2,a2,766 # 83008a18 <plic_config_0+0x38>
    83003722:	00005597          	auipc	a1,0x5
    83003726:	32658593          	addi	a1,a1,806 # 83008a48 <plic_config_0+0x68>
    8300372a:	00005517          	auipc	a0,0x5
    8300372e:	aee50513          	addi	a0,a0,-1298 # 83008218 <__clz_tab+0x1e0>
    83003732:	fedfc0ef          	jal	8300071e <assert_print>
    83003736:	00006597          	auipc	a1,0x6
    8300373a:	dda58593          	addi	a1,a1,-550 # 83009510 <timeout_lock>
    8300373e:	00005517          	auipc	a0,0x5
    83003742:	32250513          	addi	a0,a0,802 # 83008a60 <plic_config_0+0x80>
    83003746:	fd9fc0ef          	jal	8300071e <assert_print>
    8300374a:	08800593          	li	a1,136
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    8300374e:	00005517          	auipc	a0,0x5
    83003752:	2ca50513          	addi	a0,a0,714 # 83008a18 <plic_config_0+0x38>
    83003756:	fbffc0ef          	jal	83000714 <assert_post_action>
	return list->head == list;
    8300375a:	0001ca97          	auipc	s5,0x1c
    8300375e:	50ea8a93          	addi	s5,s5,1294 # 8301fc68 <timeout_list>
	z_spin_lock_set_owner(l);
    83003762:	00006517          	auipc	a0,0x6
    83003766:	dae50513          	addi	a0,a0,-594 # 83009510 <timeout_lock>
		announce_remaining += ticks;
		k_spin_unlock(&timeout_lock, key);
		return;
	}

	announce_remaining = ticks;
    8300376a:	00006a17          	auipc	s4,0x6
    8300376e:	dc2a0a13          	addi	s4,s4,-574 # 8300952c <announce_remaining>
    83003772:	c49ff0ef          	jal	830033ba <z_spin_lock_set_owner>
    83003776:	008a2023          	sw	s0,0(s4)
    8300377a:	000ab403          	ld	s0,0(s5)
    8300377e:	2481                	sext.w	s1,s1
    83003780:	8bd6                	mv	s7,s5
	return sys_dlist_is_empty(list) ? NULL : list->head;
    83003782:	01541363          	bne	s0,s5,83003788 <sys_clock_announce+0x9a>
    83003786:	4401                	li	s0,0
	for (t = first();
	     (t != NULL) && (t->dticks <= announce_remaining);
	     t = first()) {
		int dt = t->dticks;

		curr_tick += dt;
    83003788:	00006997          	auipc	s3,0x6
    8300378c:	d9098993          	addi	s3,s3,-624 # 83009518 <curr_tick>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83003790:	00006b17          	auipc	s6,0x6
    83003794:	d80b0b13          	addi	s6,s6,-640 # 83009510 <timeout_lock>
    83003798:	0009b683          	ld	a3,0(s3)
	     (t != NULL) && (t->dticks <= announce_remaining);
    8300379c:	000a2703          	lw	a4,0(s4)
    830037a0:	cc59                	beqz	s0,8300383e <sys_clock_announce+0x150>
    830037a2:	6c1c                	ld	a5,24(s0)
    830037a4:	00f75563          	bge	a4,a5,830037ae <sys_clock_announce+0xc0>
		key = k_spin_lock(&timeout_lock);
		announce_remaining -= dt;
	}

	if (t != NULL) {
		t->dticks -= announce_remaining;
    830037a8:	8f99                	sub	a5,a5,a4
    830037aa:	ec1c                	sd	a5,24(s0)
    830037ac:	a849                	j	8300383e <sys_clock_announce+0x150>
		int dt = t->dticks;
    830037ae:	0007891b          	sext.w	s2,a5
		curr_tick += dt;
    830037b2:	96ca                	add	a3,a3,s2
		remove_timeout(t);
    830037b4:	8522                	mv	a0,s0
		t->dticks = 0;
    830037b6:	00043c23          	sd	zero,24(s0)
		curr_tick += dt;
    830037ba:	00d9b023          	sd	a3,0(s3)
		remove_timeout(t);
    830037be:	c23ff0ef          	jal	830033e0 <remove_timeout>
    830037c2:	855a                	mv	a0,s6
    830037c4:	bc9ff0ef          	jal	8300338c <z_spin_unlock_valid>
    830037c8:	ed15                	bnez	a0,83003804 <sys_clock_announce+0x116>
    830037ca:	13100693          	li	a3,305
    830037ce:	00005617          	auipc	a2,0x5
    830037d2:	24a60613          	addi	a2,a2,586 # 83008a18 <plic_config_0+0x38>
    830037d6:	00005597          	auipc	a1,0x5
    830037da:	2a258593          	addi	a1,a1,674 # 83008a78 <plic_config_0+0x98>
    830037de:	00005517          	auipc	a0,0x5
    830037e2:	a3a50513          	addi	a0,a0,-1478 # 83008218 <__clz_tab+0x1e0>
    830037e6:	f39fc0ef          	jal	8300071e <assert_print>
    830037ea:	00006597          	auipc	a1,0x6
    830037ee:	d2658593          	addi	a1,a1,-730 # 83009510 <timeout_lock>
    830037f2:	00005517          	auipc	a0,0x5
    830037f6:	29e50513          	addi	a0,a0,670 # 83008a90 <plic_config_0+0xb0>
    830037fa:	f25fc0ef          	jal	8300071e <assert_print>
    830037fe:	13100593          	li	a1,305
    83003802:	b7b1                	j	8300374e <sys_clock_announce+0x60>
	csr_set(xstatus, (key & XSTATUS_IEN));
    83003804:	8889                	andi	s1,s1,2
    83003806:	1004a073          	csrs	sstatus,s1
		t->fn(t);
    8300380a:	681c                	ld	a5,16(s0)
    8300380c:	8522                	mv	a0,s0
    8300380e:	9782                	jalr	a5
	key = csr_read_clear(xstatus, XSTATUS_IEN);
    83003810:	100174f3          	csrrci	s1,sstatus,2
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83003814:	855a                	mv	a0,s6
	k.key = arch_irq_lock();
    83003816:	2481                	sext.w	s1,s1
	__ASSERT(z_spin_lock_valid(l), "Invalid spinlock %p", l);
    83003818:	b5bff0ef          	jal	83003372 <z_spin_lock_valid>
    8300381c:	ee050de3          	beqz	a0,83003716 <sys_clock_announce+0x28>
	z_spin_lock_set_owner(l);
    83003820:	855a                	mv	a0,s6
    83003822:	b99ff0ef          	jal	830033ba <z_spin_lock_set_owner>
		announce_remaining -= dt;
    83003826:	000a2783          	lw	a5,0(s4)
	return list->head == list;
    8300382a:	000ab403          	ld	s0,0(s5)
    8300382e:	4127873b          	subw	a4,a5,s2
    83003832:	00ea2023          	sw	a4,0(s4)
	return sys_dlist_is_empty(list) ? NULL : list->head;
    83003836:	f77411e3          	bne	s0,s7,83003798 <sys_clock_announce+0xaa>
		curr_tick += dt;
    8300383a:	0009b683          	ld	a3,0(s3)
	}

	curr_tick += announce_remaining;
    8300383e:	9736                	add	a4,a4,a3
    83003840:	00e9b023          	sd	a4,0(s3)
	announce_remaining = 0;
    83003844:	00006797          	auipc	a5,0x6
    83003848:	ce07a423          	sw	zero,-792(a5) # 8300952c <announce_remaining>

	sys_clock_set_timeout(next_timeout(), false);
    8300384c:	bc1ff0ef          	jal	8300340c <next_timeout>
    83003850:	4581                	li	a1,0
    83003852:	bc5fd0ef          	jal	83001416 <sys_clock_set_timeout>
	__ASSERT(z_spin_unlock_valid(l), "Not my spinlock %p", l);
    83003856:	00006517          	auipc	a0,0x6
    8300385a:	cba50513          	addi	a0,a0,-838 # 83009510 <timeout_lock>
    8300385e:	b2fff0ef          	jal	8300338c <z_spin_unlock_valid>
    83003862:	d525                	beqz	a0,830037ca <sys_clock_announce+0xdc>
	csr_set(xstatus, (key & XSTATUS_IEN));
    83003864:	8889                	andi	s1,s1,2
    83003866:	1004a073          	csrs	sstatus,s1
	k_spin_unlock(&timeout_lock, key);

#ifdef CONFIG_TIMESLICING
	z_time_slice();
#endif /* CONFIG_TIMESLICING */
}
    8300386a:	6406                	ld	s0,64(sp)
    8300386c:	60a6                	ld	ra,72(sp)
    8300386e:	74e2                	ld	s1,56(sp)
    83003870:	7942                	ld	s2,48(sp)
    83003872:	79a2                	ld	s3,40(sp)
    83003874:	7a02                	ld	s4,32(sp)
    83003876:	6ae2                	ld	s5,24(sp)
    83003878:	6b42                	ld	s6,16(sp)
    8300387a:	6ba2                	ld	s7,8(sp)
    8300387c:	6161                	addi	sp,sp,80
	z_time_slice();
    8300387e:	9f7ff06f          	j	83003274 <z_time_slice>

0000000083003882 <sys_clock_tick_get_32>:
#ifdef CONFIG_TICKLESS_KERNEL
	return (uint32_t)sys_clock_tick_get();
#else
	return (uint32_t)curr_tick;
#endif /* CONFIG_TICKLESS_KERNEL */
}
    83003882:	00006517          	auipc	a0,0x6
    83003886:	c9652503          	lw	a0,-874(a0) # 83009518 <curr_tick>
    8300388a:	8082                	ret

000000008300388c <boot_banner>:
	  */
	printk("\x1b[3J\x1b[2J\x1b[H");
#endif /* CONFIG_BOOT_CLEAR_SCREEN */

#ifdef CONFIG_BOOT_BANNER
	printk("*** " CONFIG_BOOT_BANNER_STRING " " BANNER_VERSION BANNER_POSTFIX " ***\n");
    8300388c:	00006517          	auipc	a0,0x6
    83003890:	b7c50513          	addi	a0,a0,-1156 # 83009408 <levels.0+0x668>
    83003894:	e37fc06f          	j	830006ca <printk>

0000000083003898 <strncpy>:
    83003898:	1141                	addi	sp,sp,-16
    8300389a:	e022                	sd	s0,0(sp)
    8300389c:	e406                	sd	ra,8(sp)
    8300389e:	842a                	mv	s0,a0
    830038a0:	ca09                	beqz	a2,830038b2 <strncpy+0x1a>
    830038a2:	0005c783          	lbu	a5,0(a1)
    830038a6:	0585                	addi	a1,a1,1
    830038a8:	0505                	addi	a0,a0,1
    830038aa:	fef50fa3          	sb	a5,-1(a0)
    830038ae:	167d                	addi	a2,a2,-1
    830038b0:	fbe5                	bnez	a5,830038a0 <strncpy+0x8>
    830038b2:	4581                	li	a1,0
    830038b4:	5d8000ef          	jal	83003e8c <memset>
    830038b8:	60a2                	ld	ra,8(sp)
    830038ba:	8522                	mv	a0,s0
    830038bc:	6402                	ld	s0,0(sp)
    830038be:	0141                	addi	sp,sp,16
    830038c0:	8082                	ret

00000000830038c2 <strnlen>:
    830038c2:	95aa                	add	a1,a1,a0
    830038c4:	87aa                	mv	a5,a0
    830038c6:	00b78563          	beq	a5,a1,830038d0 <strnlen+0xe>
    830038ca:	0007c703          	lbu	a4,0(a5)
    830038ce:	e701                	bnez	a4,830038d6 <strnlen+0x14>
    830038d0:	40a78533          	sub	a0,a5,a0
    830038d4:	8082                	ret
    830038d6:	0785                	addi	a5,a5,1
    830038d8:	b7fd                	j	830038c6 <strnlen+0x4>

00000000830038da <sprintf>:
    830038da:	7175                	addi	sp,sp,-144
    830038dc:	e8a2                	sd	s0,80(sp)
    830038de:	e4a6                	sd	s1,72(sp)
    830038e0:	842a                	mv	s0,a0
    830038e2:	84ae                	mv	s1,a1
    830038e4:	f0b2                	sd	a2,96(sp)
    830038e6:	4581                	li	a1,0
    830038e8:	03800613          	li	a2,56
    830038ec:	0028                	addi	a0,sp,8
    830038ee:	ec86                	sd	ra,88(sp)
    830038f0:	fcbe                	sd	a5,120(sp)
    830038f2:	f4b6                	sd	a3,104(sp)
    830038f4:	f8ba                	sd	a4,112(sp)
    830038f6:	e142                	sd	a6,128(sp)
    830038f8:	e546                	sd	a7,136(sp)
    830038fa:	592000ef          	jal	83003e8c <memset>
    830038fe:	4789                	li	a5,2
    83003900:	00f10623          	sb	a5,12(sp)
    83003904:	00000797          	auipc	a5,0x0
    83003908:	55e78793          	addi	a5,a5,1374 # 83003e62 <__file_str_put>
    8300390c:	e83e                	sd	a5,16(sp)
    8300390e:	800007b7          	lui	a5,0x80000
    83003912:	fff7c793          	not	a5,a5
    83003916:	97a2                	add	a5,a5,s0
    83003918:	1090                	addi	a2,sp,96
    8300391a:	85a6                	mv	a1,s1
    8300391c:	0028                	addi	a0,sp,8
    8300391e:	f422                	sd	s0,40(sp)
    83003920:	f83e                	sd	a5,48(sp)
    83003922:	e032                	sd	a2,0(sp)
    83003924:	052000ef          	jal	83003976 <__l_vfprintf>
    83003928:	00054563          	bltz	a0,83003932 <sprintf+0x58>
    8300392c:	942a                	add	s0,s0,a0
    8300392e:	00040023          	sb	zero,0(s0)
    83003932:	60e6                	ld	ra,88(sp)
    83003934:	6446                	ld	s0,80(sp)
    83003936:	64a6                	ld	s1,72(sp)
    83003938:	6149                	addi	sp,sp,144
    8300393a:	8082                	ret

000000008300393c <__ultoa_invert>:
    8300393c:	03700693          	li	a3,55
    83003940:	9e91                	subw	a3,a3,a2
    83003942:	872a                	mv	a4,a0
    83003944:	0ff6f693          	zext.b	a3,a3
    83003948:	8a7d                	andi	a2,a2,31
    8300394a:	852e                	mv	a0,a1
    8300394c:	48a5                	li	a7,9
    8300394e:	02c775b3          	remu	a1,a4,a2
    83003952:	883a                	mv	a6,a4
    83003954:	0ff5f793          	zext.b	a5,a1
    83003958:	02c75733          	divu	a4,a4,a2
    8300395c:	00b8f563          	bgeu	a7,a1,83003966 <__ultoa_invert+0x2a>
    83003960:	9fb5                	addw	a5,a5,a3
    83003962:	0ff7f793          	zext.b	a5,a5
    83003966:	0505                	addi	a0,a0,1
    83003968:	0307879b          	addiw	a5,a5,48 # ffffffff80000030 <CONFIG_RISCV_MCAUSE_EXCEPTION_MASK+0x7fffffff80000031>
    8300396c:	fef50fa3          	sb	a5,-1(a0)
    83003970:	fcc87fe3          	bgeu	a6,a2,8300394e <__ultoa_invert+0x12>
    83003974:	8082                	ret

0000000083003976 <__l_vfprintf>:
    83003976:	7135                	addi	sp,sp,-160
    83003978:	f0da                	sd	s6,96(sp)
    8300397a:	ed06                	sd	ra,152(sp)
    8300397c:	e922                	sd	s0,144(sp)
    8300397e:	e526                	sd	s1,136(sp)
    83003980:	e14a                	sd	s2,128(sp)
    83003982:	fcce                	sd	s3,120(sp)
    83003984:	f8d2                	sd	s4,112(sp)
    83003986:	f4d6                	sd	s5,104(sp)
    83003988:	ecde                	sd	s7,88(sp)
    8300398a:	e8e2                	sd	s8,80(sp)
    8300398c:	e4e6                	sd	s9,72(sp)
    8300398e:	e0ea                	sd	s10,64(sp)
    83003990:	fc6e                	sd	s11,56(sp)
    83003992:	00454783          	lbu	a5,4(a0)
    83003996:	00853b03          	ld	s6,8(a0)
    8300399a:	8b89                	andi	a5,a5,2
    8300399c:	24078f63          	beqz	a5,83003bfa <__l_vfprintf+0x284>
    830039a0:	8a2a                	mv	s4,a0
    830039a2:	84b2                	mv	s1,a2
    830039a4:	4901                	li	s2,0
    830039a6:	02500c13          	li	s8,37
    830039aa:	0005c983          	lbu	s3,0(a1)
    830039ae:	24098763          	beqz	s3,83003bfc <__l_vfprintf+0x286>
    830039b2:	00158b93          	addi	s7,a1,1
    830039b6:	01899863          	bne	s3,s8,830039c6 <__l_vfprintf+0x50>
    830039ba:	0015c983          	lbu	s3,1(a1)
    830039be:	00258b93          	addi	s7,a1,2
    830039c2:	01899a63          	bne	s3,s8,830039d6 <__l_vfprintf+0x60>
    830039c6:	2905                	addiw	s2,s2,1
    830039c8:	85d2                	mv	a1,s4
    830039ca:	854e                	mv	a0,s3
    830039cc:	9b02                	jalr	s6
    830039ce:	22054063          	bltz	a0,83003bee <__l_vfprintf+0x278>
    830039d2:	85de                	mv	a1,s7
    830039d4:	bfd9                	j	830039aa <__l_vfprintf+0x34>
    830039d6:	4d81                	li	s11,0
    830039d8:	4a81                	li	s5,0
    830039da:	4401                	li	s0,0
    830039dc:	467d                	li	a2,31
    830039de:	06c00713          	li	a4,108
    830039e2:	07400593          	li	a1,116
    830039e6:	07a00513          	li	a0,122
    830039ea:	06800813          	li	a6,104
    830039ee:	06a00893          	li	a7,106
    830039f2:	04c00313          	li	t1,76
    830039f6:	02a00e13          	li	t3,42
    830039fa:	02e00e93          	li	t4,46
    830039fe:	07f00f13          	li	t5,127
    83003a02:	4fc1                	li	t6,16
    83003a04:	42a5                	li	t0,9
    83003a06:	00006697          	auipc	a3,0x6
    83003a0a:	a5268693          	addi	a3,a3,-1454 # 83009458 <levels.0+0x6b8>
    83003a0e:	0004079b          	sext.w	a5,s0
    83003a12:	06866563          	bltu	a2,s0,83003a7c <__l_vfprintf+0x106>
    83003a16:	fe09879b          	addiw	a5,s3,-32
    83003a1a:	0007839b          	sext.w	t2,a5
    83003a1e:	007fea63          	bltu	t6,t2,83003a32 <__l_vfprintf+0xbc>
    83003a22:	02079393          	slli	t2,a5,0x20
    83003a26:	01e3d793          	srli	a5,t2,0x1e
    83003a2a:	97b6                	add	a5,a5,a3
    83003a2c:	439c                	lw	a5,0(a5)
    83003a2e:	97b6                	add	a5,a5,a3
    83003a30:	8782                	jr	a5
    83003a32:	fd09879b          	addiw	a5,s3,-48
    83003a36:	06f2f563          	bgeu	t0,a5,83003aa0 <__l_vfprintf+0x12a>
    83003a3a:	09c99063          	bne	s3,t3,83003aba <__l_vfprintf+0x144>
    83003a3e:	04047393          	andi	t2,s0,64
    83003a42:	409c                	lw	a5,0(s1)
    83003a44:	04a1                	addi	s1,s1,8
    83003a46:	10039763          	bnez	t2,83003b54 <__l_vfprintf+0x1de>
    83003a4a:	0607c363          	bltz	a5,83003ab0 <__l_vfprintf+0x13a>
    83003a4e:	02046413          	ori	s0,s0,32
    83003a52:	8abe                	mv	s5,a5
    83003a54:	a019                	j	83003a5a <__l_vfprintf+0xe4>
    83003a56:	00146413          	ori	s0,s0,1
    83003a5a:	000bc983          	lbu	s3,0(s7)
    83003a5e:	0b85                	addi	s7,s7,1
    83003a60:	fa0997e3          	bnez	s3,83003a0e <__l_vfprintf+0x98>
    83003a64:	a8b5                	j	83003ae0 <__l_vfprintf+0x16a>
    83003a66:	00246413          	ori	s0,s0,2
    83003a6a:	00446413          	ori	s0,s0,4
    83003a6e:	b7f5                	j	83003a5a <__l_vfprintf+0xe4>
    83003a70:	00846413          	ori	s0,s0,8
    83003a74:	b7dd                	j	83003a5a <__l_vfprintf+0xe4>
    83003a76:	01046413          	ori	s0,s0,16
    83003a7a:	b7c5                	j	83003a5a <__l_vfprintf+0xe4>
    83003a7c:	04ff6863          	bltu	t5,a5,83003acc <__l_vfprintf+0x156>
    83003a80:	fd09879b          	addiw	a5,s3,-48
    83003a84:	4ca5                	li	s9,9
    83003a86:	83be                	mv	t2,a5
    83003a88:	fafce9e3          	bltu	s9,a5,83003a3a <__l_vfprintf+0xc4>
    83003a8c:	04047993          	andi	s3,s0,64
    83003a90:	00098863          	beqz	s3,83003aa0 <__l_vfprintf+0x12a>
    83003a94:	47a9                	li	a5,10
    83003a96:	03b787bb          	mulw	a5,a5,s11
    83003a9a:	00778dbb          	addw	s11,a5,t2
    83003a9e:	bf75                	j	83003a5a <__l_vfprintf+0xe4>
    83003aa0:	43a9                	li	t2,10
    83003aa2:	035383bb          	mulw	t2,t2,s5
    83003aa6:	02046413          	ori	s0,s0,32
    83003aaa:	00f38abb          	addw	s5,t2,a5
    83003aae:	b775                	j	83003a5a <__l_vfprintf+0xe4>
    83003ab0:	40f00abb          	negw	s5,a5
    83003ab4:	02846413          	ori	s0,s0,40
    83003ab8:	b74d                	j	83003a5a <__l_vfprintf+0xe4>
    83003aba:	01d99963          	bne	s3,t4,83003acc <__l_vfprintf+0x156>
    83003abe:	04047793          	andi	a5,s0,64
    83003ac2:	12079d63          	bnez	a5,83003bfc <__l_vfprintf+0x286>
    83003ac6:	04046413          	ori	s0,s0,64
    83003aca:	bf41                	j	83003a5a <__l_vfprintf+0xe4>
    83003acc:	06e98363          	beq	s3,a4,83003b32 <__l_vfprintf+0x1bc>
    83003ad0:	05376a63          	bltu	a4,s3,83003b24 <__l_vfprintf+0x1ae>
    83003ad4:	07098563          	beq	s3,a6,83003b3e <__l_vfprintf+0x1c8>
    83003ad8:	05198a63          	beq	s3,a7,83003b2c <__l_vfprintf+0x1b6>
    83003adc:	06698963          	beq	s3,t1,83003b4e <__l_vfprintf+0x1d8>
    83003ae0:	000dd763          	bgez	s11,83003aee <__l_vfprintf+0x178>
    83003ae4:	fbf47413          	andi	s0,s0,-65
    83003ae8:	1442                	slli	s0,s0,0x30
    83003aea:	9041                	srli	s0,s0,0x30
    83003aec:	4d81                	li	s11,0
    83003aee:	0209e713          	ori	a4,s3,32
    83003af2:	f9b7069b          	addiw	a3,a4,-101
    83003af6:	4789                	li	a5,2
    83003af8:	00d7f663          	bgeu	a5,a3,83003b04 <__l_vfprintf+0x18e>
    83003afc:	06100793          	li	a5,97
    83003b00:	04f71c63          	bne	a4,a5,83003b58 <__l_vfprintf+0x1e2>
    83003b04:	04a1                	addi	s1,s1,8
    83003b06:	499d                	li	s3,7
    83003b08:	00006c97          	auipc	s9,0x6
    83003b0c:	948c8c93          	addi	s9,s9,-1720 # 83009450 <levels.0+0x6b0>
    83003b10:	8821                	andi	s0,s0,8
    83003b12:	c079                	beqz	s0,83003bd8 <__l_vfprintf+0x262>
    83003b14:	4401                	li	s0,0
    83003b16:	0c899363          	bne	s3,s0,83003bdc <__l_vfprintf+0x266>
    83003b1a:	0129843b          	addw	s0,s3,s2
    83003b1e:	413a8abb          	subw	s5,s5,s3
    83003b22:	a4e9                	j	83003dec <__l_vfprintf+0x476>
    83003b24:	00b98463          	beq	s3,a1,83003b2c <__l_vfprintf+0x1b6>
    83003b28:	faa99ce3          	bne	s3,a0,83003ae0 <__l_vfprintf+0x16a>
    83003b2c:	08046413          	ori	s0,s0,128
    83003b30:	b72d                	j	83003a5a <__l_vfprintf+0xe4>
    83003b32:	08047793          	andi	a5,s0,128
    83003b36:	dbfd                	beqz	a5,83003b2c <__l_vfprintf+0x1b6>
    83003b38:	20046413          	ori	s0,s0,512
    83003b3c:	bfc5                	j	83003b2c <__l_vfprintf+0x1b6>
    83003b3e:	10047793          	andi	a5,s0,256
    83003b42:	c399                	beqz	a5,83003b48 <__l_vfprintf+0x1d2>
    83003b44:	20046413          	ori	s0,s0,512
    83003b48:	10046413          	ori	s0,s0,256
    83003b4c:	b739                	j	83003a5a <__l_vfprintf+0xe4>
    83003b4e:	28046413          	ori	s0,s0,640
    83003b52:	b721                	j	83003a5a <__l_vfprintf+0xe4>
    83003b54:	8dbe                	mv	s11,a5
    83003b56:	b711                	j	83003a5a <__l_vfprintf+0xe4>
    83003b58:	f9d9879b          	addiw	a5,s3,-99
    83003b5c:	0007861b          	sext.w	a2,a5
    83003b60:	46c9                	li	a3,18
    83003b62:	1ac6ef63          	bltu	a3,a2,83003d20 <__l_vfprintf+0x3aa>
    83003b66:	02079693          	slli	a3,a5,0x20
    83003b6a:	01e6d793          	srli	a5,a3,0x1e
    83003b6e:	00006697          	auipc	a3,0x6
    83003b72:	92e68693          	addi	a3,a3,-1746 # 8300949c <levels.0+0x6fc>
    83003b76:	97b6                	add	a5,a5,a3
    83003b78:	439c                	lw	a5,0(a5)
    83003b7a:	97b6                	add	a5,a5,a3
    83003b7c:	8782                	jr	a5
    83003b7e:	409c                	lw	a5,0(s1)
    83003b80:	4985                	li	s3,1
    83003b82:	04a1                	addi	s1,s1,8
    83003b84:	00f10c23          	sb	a5,24(sp)
    83003b88:	01810c93          	addi	s9,sp,24
    83003b8c:	b751                	j	83003b10 <__l_vfprintf+0x19a>
    83003b8e:	0004bc83          	ld	s9,0(s1)
    83003b92:	00848d13          	addi	s10,s1,8
    83003b96:	000c9663          	bnez	s9,83003ba2 <__l_vfprintf+0x22c>
    83003b9a:	00006c97          	auipc	s9,0x6
    83003b9e:	8aec8c93          	addi	s9,s9,-1874 # 83009448 <levels.0+0x6a8>
    83003ba2:	04047793          	andi	a5,s0,64
    83003ba6:	55fd                	li	a1,-1
    83003ba8:	c391                	beqz	a5,83003bac <__l_vfprintf+0x236>
    83003baa:	85ee                	mv	a1,s11
    83003bac:	8566                	mv	a0,s9
    83003bae:	d15ff0ef          	jal	830038c2 <strnlen>
    83003bb2:	89aa                	mv	s3,a0
    83003bb4:	84ea                	mv	s1,s10
    83003bb6:	bfa9                	j	83003b10 <__l_vfprintf+0x19a>
    83003bb8:	85d2                	mv	a1,s4
    83003bba:	02000513          	li	a0,32
    83003bbe:	9b02                	jalr	s6
    83003bc0:	02054763          	bltz	a0,83003bee <__l_vfprintf+0x278>
    83003bc4:	0405                	addi	s0,s0,1
    83003bc6:	408a87b3          	sub	a5,s5,s0
    83003bca:	fef9e7e3          	bltu	s3,a5,83003bb8 <__l_vfprintf+0x242>
    83003bce:	0124093b          	addw	s2,s0,s2
    83003bd2:	408a8abb          	subw	s5,s5,s0
    83003bd6:	bf3d                	j	83003b14 <__l_vfprintf+0x19e>
    83003bd8:	4401                	li	s0,0
    83003bda:	b7f5                	j	83003bc6 <__l_vfprintf+0x250>
    83003bdc:	008c87b3          	add	a5,s9,s0
    83003be0:	0007c503          	lbu	a0,0(a5)
    83003be4:	85d2                	mv	a1,s4
    83003be6:	0405                	addi	s0,s0,1
    83003be8:	9b02                	jalr	s6
    83003bea:	f20556e3          	bgez	a0,83003b16 <__l_vfprintf+0x1a0>
    83003bee:	004a4783          	lbu	a5,4(s4)
    83003bf2:	0047e793          	ori	a5,a5,4
    83003bf6:	00fa0223          	sb	a5,4(s4)
    83003bfa:	597d                	li	s2,-1
    83003bfc:	60ea                	ld	ra,152(sp)
    83003bfe:	644a                	ld	s0,144(sp)
    83003c00:	64aa                	ld	s1,136(sp)
    83003c02:	79e6                	ld	s3,120(sp)
    83003c04:	7a46                	ld	s4,112(sp)
    83003c06:	7aa6                	ld	s5,104(sp)
    83003c08:	7b06                	ld	s6,96(sp)
    83003c0a:	6be6                	ld	s7,88(sp)
    83003c0c:	6c46                	ld	s8,80(sp)
    83003c0e:	6ca6                	ld	s9,72(sp)
    83003c10:	6d06                	ld	s10,64(sp)
    83003c12:	7de2                	ld	s11,56(sp)
    83003c14:	854a                	mv	a0,s2
    83003c16:	690a                	ld	s2,128(sp)
    83003c18:	610d                	addi	sp,sp,160
    83003c1a:	8082                	ret
    83003c1c:	08047793          	andi	a5,s0,128
    83003c20:	00848c93          	addi	s9,s1,8
    83003c24:	c7a9                	beqz	a5,83003c6e <__l_vfprintf+0x2f8>
    83003c26:	6088                	ld	a0,0(s1)
    83003c28:	fef47493          	andi	s1,s0,-17
    83003c2c:	14c2                	slli	s1,s1,0x30
    83003c2e:	90c1                	srli	s1,s1,0x30
    83003c30:	06055063          	bgez	a0,83003c90 <__l_vfprintf+0x31a>
    83003c34:	40a00533          	neg	a0,a0
    83003c38:	4004e493          	ori	s1,s1,1024
    83003c3c:	0820                	addi	s0,sp,24
    83003c3e:	4629                	li	a2,10
    83003c40:	85a2                	mv	a1,s0
    83003c42:	cfbff0ef          	jal	8300393c <__ultoa_invert>
    83003c46:	0404f793          	andi	a5,s1,64
    83003c4a:	4085043b          	subw	s0,a0,s0
    83003c4e:	cfc1                	beqz	a5,83003ce6 <__l_vfprintf+0x370>
    83003c50:	ffe4f713          	andi	a4,s1,-2
    83003c54:	1742                	slli	a4,a4,0x30
    83003c56:	9341                	srli	a4,a4,0x30
    83003c58:	13b45663          	bge	s0,s11,83003d84 <__l_vfprintf+0x40e>
    83003c5c:	10098f63          	beqz	s3,83003d7a <__l_vfprintf+0x404>
    83003c60:	0104f793          	andi	a5,s1,16
    83003c64:	8d6e                	mv	s10,s11
    83003c66:	84ba                	mv	s1,a4
    83003c68:	c3a1                	beqz	a5,83003ca8 <__l_vfprintf+0x332>
    83003c6a:	2d09                	addiw	s10,s10,2
    83003c6c:	a061                	j	83003cf4 <__l_vfprintf+0x37e>
    83003c6e:	10047793          	andi	a5,s0,256
    83003c72:	4088                	lw	a0,0(s1)
    83003c74:	dbd5                	beqz	a5,83003c28 <__l_vfprintf+0x2b2>
    83003c76:	20047793          	andi	a5,s0,512
    83003c7a:	c791                	beqz	a5,83003c86 <__l_vfprintf+0x310>
    83003c7c:	0185151b          	slliw	a0,a0,0x18
    83003c80:	4185551b          	sraiw	a0,a0,0x18
    83003c84:	b755                	j	83003c28 <__l_vfprintf+0x2b2>
    83003c86:	0105151b          	slliw	a0,a0,0x10
    83003c8a:	4105551b          	sraiw	a0,a0,0x10
    83003c8e:	bf69                	j	83003c28 <__l_vfprintf+0x2b2>
    83003c90:	f555                	bnez	a0,83003c3c <__l_vfprintf+0x2c6>
    83003c92:	04047793          	andi	a5,s0,64
    83003c96:	d3dd                	beqz	a5,83003c3c <__l_vfprintf+0x2c6>
    83003c98:	fa0d92e3          	bnez	s11,83003c3c <__l_vfprintf+0x2c6>
    83003c9c:	fee47493          	andi	s1,s0,-18
    83003ca0:	14c2                	slli	s1,s1,0x30
    83003ca2:	90c1                	srli	s1,s1,0x30
    83003ca4:	4d01                	li	s10,0
    83003ca6:	4401                	li	s0,0
    83003ca8:	4064f793          	andi	a5,s1,1030
    83003cac:	c7a1                	beqz	a5,83003cf4 <__l_vfprintf+0x37e>
    83003cae:	2d05                	addiw	s10,s10,1
    83003cb0:	4781                	li	a5,0
    83003cb2:	a089                	j	83003cf4 <__l_vfprintf+0x37e>
    83003cb4:	983d                	andi	s0,s0,-17
    83003cb6:	1442                	slli	s0,s0,0x30
    83003cb8:	9041                	srli	s0,s0,0x30
    83003cba:	4629                	li	a2,10
    83003cbc:	08047793          	andi	a5,s0,128
    83003cc0:	00848c93          	addi	s9,s1,8
    83003cc4:	c7c9                	beqz	a5,83003d4e <__l_vfprintf+0x3d8>
    83003cc6:	6088                	ld	a0,0(s1)
    83003cc8:	e15d                	bnez	a0,83003d6e <__l_vfprintf+0x3f8>
    83003cca:	fe947493          	andi	s1,s0,-23
    83003cce:	14c2                	slli	s1,s1,0x30
    83003cd0:	04047793          	andi	a5,s0,64
    83003cd4:	90c1                	srli	s1,s1,0x30
    83003cd6:	c3c5                	beqz	a5,83003d76 <__l_vfprintf+0x400>
    83003cd8:	080d9f63          	bnez	s11,83003d76 <__l_vfprintf+0x400>
    83003cdc:	fe847493          	andi	s1,s0,-24
    83003ce0:	14c2                	slli	s1,s1,0x30
    83003ce2:	90c1                	srli	s1,s1,0x30
    83003ce4:	4401                	li	s0,0
    83003ce6:	0104f793          	andi	a5,s1,16
    83003cea:	c3cd                	beqz	a5,83003d8c <__l_vfprintf+0x416>
    83003cec:	08099e63          	bnez	s3,83003d88 <__l_vfprintf+0x412>
    83003cf0:	00140d1b          	addiw	s10,s0,1
    83003cf4:	0084f713          	andi	a4,s1,8
    83003cf8:	ef55                	bnez	a4,83003db4 <__l_vfprintf+0x43e>
    83003cfa:	0014f713          	andi	a4,s1,1
    83003cfe:	cb09                	beqz	a4,83003d10 <__l_vfprintf+0x39a>
    83003d00:	8da2                	mv	s11,s0
    83003d02:	015d5763          	bge	s10,s5,83003d10 <__l_vfprintf+0x39a>
    83003d06:	41aa8d3b          	subw	s10,s5,s10
    83003d0a:	008d0dbb          	addw	s11,s10,s0
    83003d0e:	8d56                	mv	s10,s5
    83003d10:	876a                	mv	a4,s10
    83003d12:	a851                	j	83003da6 <__l_vfprintf+0x430>
    83003d14:	09046413          	ori	s0,s0,144
    83003d18:	4641                	li	a2,16
    83003d1a:	07800993          	li	s3,120
    83003d1e:	bf79                	j	83003cbc <__l_vfprintf+0x346>
    83003d20:	07800793          	li	a5,120
    83003d24:	00f71a63          	bne	a4,a5,83003d38 <__l_vfprintf+0x3c2>
    83003d28:	07800613          	li	a2,120
    83003d2c:	4136063b          	subw	a2,a2,s3
    83003d30:	01066613          	ori	a2,a2,16
    83003d34:	2601                	sext.w	a2,a2
    83003d36:	b759                	j	83003cbc <__l_vfprintf+0x346>
    83003d38:	85d2                	mv	a1,s4
    83003d3a:	02500513          	li	a0,37
    83003d3e:	9b02                	jalr	s6
    83003d40:	ea0547e3          	bltz	a0,83003bee <__l_vfprintf+0x278>
    83003d44:	2909                	addiw	s2,s2,2
    83003d46:	b149                	j	830039c8 <__l_vfprintf+0x52>
    83003d48:	4621                	li	a2,8
    83003d4a:	4981                	li	s3,0
    83003d4c:	bf85                	j	83003cbc <__l_vfprintf+0x346>
    83003d4e:	10047793          	andi	a5,s0,256
    83003d52:	4088                	lw	a0,0(s1)
    83003d54:	e781                	bnez	a5,83003d5c <__l_vfprintf+0x3e6>
    83003d56:	1502                	slli	a0,a0,0x20
    83003d58:	9101                	srli	a0,a0,0x20
    83003d5a:	b7bd                	j	83003cc8 <__l_vfprintf+0x352>
    83003d5c:	20047793          	andi	a5,s0,512
    83003d60:	c781                	beqz	a5,83003d68 <__l_vfprintf+0x3f2>
    83003d62:	0ff57513          	zext.b	a0,a0
    83003d66:	b78d                	j	83003cc8 <__l_vfprintf+0x352>
    83003d68:	1542                	slli	a0,a0,0x30
    83003d6a:	9141                	srli	a0,a0,0x30
    83003d6c:	bfb1                	j	83003cc8 <__l_vfprintf+0x352>
    83003d6e:	9865                	andi	s0,s0,-7
    83003d70:	03041493          	slli	s1,s0,0x30
    83003d74:	90c1                	srli	s1,s1,0x30
    83003d76:	0820                	addi	s0,sp,24
    83003d78:	b5e1                	j	83003c40 <__l_vfprintf+0x2ca>
    83003d7a:	98b9                	andi	s1,s1,-18
    83003d7c:	14c2                	slli	s1,s1,0x30
    83003d7e:	90c1                	srli	s1,s1,0x30
    83003d80:	8d6e                	mv	s10,s11
    83003d82:	b71d                	j	83003ca8 <__l_vfprintf+0x332>
    83003d84:	84ba                	mv	s1,a4
    83003d86:	b785                	j	83003ce6 <__l_vfprintf+0x370>
    83003d88:	8d22                	mv	s10,s0
    83003d8a:	b5c5                	j	83003c6a <__l_vfprintf+0x2f4>
    83003d8c:	8d22                	mv	s10,s0
    83003d8e:	bf29                	j	83003ca8 <__l_vfprintf+0x332>
    83003d90:	85d2                	mv	a1,s4
    83003d92:	02000513          	li	a0,32
    83003d96:	e43e                	sd	a5,8(sp)
    83003d98:	e03a                	sd	a4,0(sp)
    83003d9a:	9b02                	jalr	s6
    83003d9c:	e40549e3          	bltz	a0,83003bee <__l_vfprintf+0x278>
    83003da0:	6702                	ld	a4,0(sp)
    83003da2:	67a2                	ld	a5,8(sp)
    83003da4:	2705                	addiw	a4,a4,1
    83003da6:	ff5745e3          	blt	a4,s5,83003d90 <__l_vfprintf+0x41a>
    83003daa:	41a90d3b          	subw	s10,s2,s10
    83003dae:	00ed093b          	addw	s2,s10,a4
    83003db2:	8d3a                	mv	s10,a4
    83003db4:	c7b9                	beqz	a5,83003e02 <__l_vfprintf+0x48c>
    83003db6:	85d2                	mv	a1,s4
    83003db8:	03000513          	li	a0,48
    83003dbc:	9b02                	jalr	s6
    83003dbe:	e20548e3          	bltz	a0,83003bee <__l_vfprintf+0x278>
    83003dc2:	02099963          	bnez	s3,83003df4 <__l_vfprintf+0x47e>
    83003dc6:	2905                	addiw	s2,s2,1
    83003dc8:	012d8dbb          	addw	s11,s11,s2
    83003dcc:	412d87bb          	subw	a5,s11,s2
    83003dd0:	0009099b          	sext.w	s3,s2
    83003dd4:	04f44963          	blt	s0,a5,83003e26 <__l_vfprintf+0x4b0>
    83003dd8:	84a2                	mv	s1,s0
    83003dda:	0004879b          	sext.w	a5,s1
    83003dde:	efa1                	bnez	a5,83003e36 <__l_vfprintf+0x4c0>
    83003de0:	0134043b          	addw	s0,s0,s3
    83003de4:	9c05                	subw	s0,s0,s1
    83003de6:	41aa8abb          	subw	s5,s5,s10
    83003dea:	84e6                	mv	s1,s9
    83003dec:	8922                	mv	s2,s0
    83003dee:	01540abb          	addw	s5,s0,s5
    83003df2:	a09d                	j	83003e58 <__l_vfprintf+0x4e2>
    83003df4:	2909                	addiw	s2,s2,2
    83003df6:	85d2                	mv	a1,s4
    83003df8:	854e                	mv	a0,s3
    83003dfa:	9b02                	jalr	s6
    83003dfc:	fc0556e3          	bgez	a0,83003dc8 <__l_vfprintf+0x452>
    83003e00:	b3fd                	j	83003bee <__l_vfprintf+0x278>
    83003e02:	4064f793          	andi	a5,s1,1030
    83003e06:	d3e9                	beqz	a5,83003dc8 <__l_vfprintf+0x452>
    83003e08:	0024f793          	andi	a5,s1,2
    83003e0c:	02000513          	li	a0,32
    83003e10:	c399                	beqz	a5,83003e16 <__l_vfprintf+0x4a0>
    83003e12:	02b00513          	li	a0,43
    83003e16:	4004f493          	andi	s1,s1,1024
    83003e1a:	c099                	beqz	s1,83003e20 <__l_vfprintf+0x4aa>
    83003e1c:	02d00513          	li	a0,45
    83003e20:	2905                	addiw	s2,s2,1
    83003e22:	85d2                	mv	a1,s4
    83003e24:	bfd9                	j	83003dfa <__l_vfprintf+0x484>
    83003e26:	85d2                	mv	a1,s4
    83003e28:	03000513          	li	a0,48
    83003e2c:	2905                	addiw	s2,s2,1
    83003e2e:	9b02                	jalr	s6
    83003e30:	f8055ee3          	bgez	a0,83003dcc <__l_vfprintf+0x456>
    83003e34:	bb6d                	j	83003bee <__l_vfprintf+0x278>
    83003e36:	083c                	addi	a5,sp,24
    83003e38:	97a6                	add	a5,a5,s1
    83003e3a:	fff7c503          	lbu	a0,-1(a5)
    83003e3e:	85d2                	mv	a1,s4
    83003e40:	14fd                	addi	s1,s1,-1
    83003e42:	9b02                	jalr	s6
    83003e44:	f8055be3          	bgez	a0,83003dda <__l_vfprintf+0x464>
    83003e48:	b35d                	j	83003bee <__l_vfprintf+0x278>
    83003e4a:	85d2                	mv	a1,s4
    83003e4c:	02000513          	li	a0,32
    83003e50:	2905                	addiw	s2,s2,1
    83003e52:	9b02                	jalr	s6
    83003e54:	d8054de3          	bltz	a0,83003bee <__l_vfprintf+0x278>
    83003e58:	412a87bb          	subw	a5,s5,s2
    83003e5c:	fef047e3          	bgtz	a5,83003e4a <__l_vfprintf+0x4d4>
    83003e60:	be8d                	j	830039d2 <__l_vfprintf+0x5c>

0000000083003e62 <__file_str_put>:
    83003e62:	719c                	ld	a5,32(a1)
    83003e64:	7598                	ld	a4,40(a1)
    83003e66:	00e78763          	beq	a5,a4,83003e74 <__file_str_put+0x12>
    83003e6a:	00178713          	addi	a4,a5,1
    83003e6e:	f198                	sd	a4,32(a1)
    83003e70:	00a78023          	sb	a0,0(a5)
    83003e74:	8082                	ret

0000000083003e76 <memcpy>:
    83003e76:	832a                	mv	t1,a0
    83003e78:	ca09                	beqz	a2,83003e8a <memcpy+0x14>
    83003e7a:	00058383          	lb	t2,0(a1)
    83003e7e:	00730023          	sb	t2,0(t1)
    83003e82:	167d                	addi	a2,a2,-1
    83003e84:	0305                	addi	t1,t1,1
    83003e86:	0585                	addi	a1,a1,1
    83003e88:	fa6d                	bnez	a2,83003e7a <memcpy+0x4>
    83003e8a:	8082                	ret

0000000083003e8c <memset>:
    83003e8c:	832a                	mv	t1,a0
    83003e8e:	c611                	beqz	a2,83003e9a <memset+0xe>
    83003e90:	00b30023          	sb	a1,0(t1)
    83003e94:	167d                	addi	a2,a2,-1
    83003e96:	0305                	addi	t1,t1,1
    83003e98:	fe65                	bnez	a2,83003e90 <memset+0x4>
    83003e9a:	8082                	ret
