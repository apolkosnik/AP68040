; Dependent, cache-resident integer throughput. The inner body performs
; 256 ALU instructions per pass, 256 passes = 65,536 ALU instructions.
; D0 feeds D2 and D2 feeds D0, so stale operand reads cannot look fast.
; Reference recurrence, modulo 2^32 (16,384 iterations, d0=1, d2=0):
;   d0 += 3; d2 += d0; d0 ^= d2; d0 -= 3
; Final d0=$82792B15, d2=$2FEADE94.
	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	failed
	endr
	org	$400
start:
	move.l	#$80008000,d0
	movec	d0,cacr
	moveq	#1,d0
	moveq	#3,d1
	moveq	#0,d2
	move.w	#255,d3
inner:
	rept	64
	add.l	d1,d0
	add.l	d0,d2
	eor.l	d2,d0
	sub.l	d1,d0
	endr
	dbra	d3,inner
	cmp.l	#$82792B15,d0
	bne.s	failed
	cmp.l	#$2FEADE94,d2
	bne.s	failed
	move.w	#$600D,($F102).l
	stop	#$2700
failed:
	move.w	#1,($F100).l
	move.w	#$BAD0,($F102).l
	stop	#$2700
