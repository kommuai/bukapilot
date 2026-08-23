	.text
	.align 4
	.global rk_sdg_entry_continue
	.hidden rk_sdg_entry_resume
	.hidden g_cfg_slots

rk_sdg_entry_continue:
	mrs	x16, tpidr_el0
	lsr	x16, x16, #12
	and	x16, x16, #15
	adrp	x17, g_cfg_slots
	add	x17, x17, #:lo12:g_cfg_slots
	add	x17, x17, x16, lsl #3
	str	x2, [x17]
	mov	x19, x2
	adrp	x17, rk_sdg_entry_resume
	ldr	x17, [x17, #:lo12:rk_sdg_entry_resume]
	br	x17
