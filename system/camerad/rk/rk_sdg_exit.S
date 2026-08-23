	.text
	.align 4
	.global rk_sdg_exit_continue
	.hidden rk_sdg_exit_resume
	.hidden g_cfg_slots

rk_sdg_exit_continue:
	mrs	x16, tpidr_el0
	lsr	x16, x16, #12
	and	x16, x16, #15
	adrp	x17, g_cfg_slots
	add	x17, x17, #:lo12:g_cfg_slots
	add	x17, x17, x16, lsl #3
	ldr	x0, [x17]
	cbz	x0, 1f
	bl	rk_isp3x_patch_sdg_cfg
1:	mov	w0, #1
	adrp	x17, rk_sdg_exit_resume
	ldr	x17, [x17, #:lo12:rk_sdg_exit_resume]
	br	x17
