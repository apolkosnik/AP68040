; AP68040: an interrupt at every clock across second-slot loads.  The
; Minimig with only the load class in the second slot read garbage from the
; hard disk (LoadSeg: "not enough memory" for every command); interrupts
; are constant there and a companion load that misses the cache is retried
; alone (s1_rep) -- the boundary an interrupt can be taken at must then be
; the load's own PC, or the load is skipped.
; assembled with vasmm68k_mot -Fbin -m68040
;   1    MOVE.L (A1),D3 beside an ADDQ, the line flushed first (a miss:
;        retried alone), a level 2 interrupt raised 1..150 clocks into the
;        loop; 16 loads per pass, checked
;   2    the same with the lines resident (the fast path answers)
;   3    ADD.L (A1),D2 companions (a running sum), missing, interrupted

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
IPLDLY	equ	$F148
DAT	equ	$8000
nirq	equ	$3604

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	24
	dc.l	unexp
	endr
	dc.l	h_irq2		; 26 level 2 autovector
	rept	37
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$80008000,d0
	movec	d0,cacr
	; the data: $D0000000 + i
	lea	(DAT).l,a0
	moveq	#0,d0
	moveq	#15,d1
.d:	move.l	d0,d2
	add.l	#$D0000000,d2
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,.d
	cpusha	dc
	move.w	#$2000,sr

;------------------------------------------------ 1 misses, interrupted
	moveq	#1,d5
.p1:	cpusha	dc
	clr.l	(nirq).l
	move.w	d5,(IPLDLY).l
	bsr	body
	tst.l	d7
	beq.s	.k1
	failt	1
.k1:	bsr	waitirq
	addq.l	#1,d5
	cmp.l	#151,d5
	bne.s	.p1

;------------------------------------------------ 2 hits, interrupted
	bsr	body			; (the lines resident)
	moveq	#1,d5
.p2:	clr.l	(nirq).l
	move.w	d5,(IPLDLY).l
	bsr	body
	tst.l	d7
	beq.s	.k2
	failt	2
.k2:	bsr	waitirq
	addq.l	#1,d5
	cmp.l	#151,d5
	bne.s	.p2

;------------------------------------------------ 3 ADD.L (A1),D2, missing
	moveq	#1,d5
.p3:	cpusha	dc
	clr.l	(nirq).l
	move.w	d5,(IPLDLY).l
	lea	(DAT).l,a1
	moveq	#15,d4
	moveq	#0,d2
	moveq	#0,d6
.l3:	addq.l	#1,d6
	add.l	(a1),d2			; companion: ADD (Ay),Dx
	addq.l	#4,a1
	dbra	d4,.l3
	cmp.l	#$D0000000*16+120,d2
	beq.s	.k3
	failt	3
.k3:	bsr	waitirq
	addq.l	#1,d5
	cmp.l	#151,d5
	bne.s	.p3

	move.w	#$600D,(DONEREG).l
	bra.s	*

; 16 companion loads of $D0000000 + i: D7 = 0 when all right
body:	lea	(DAT).l,a1
	moveq	#15,d4
	moveq	#0,d6
	moveq	#0,d7
.l:	move.l	d6,d2
	addq.l	#1,d6			; first slot: no memory access
	move.l	(a1),d3			; companion: load from (Ay)
	add.l	#$D0000000,d2
	cmp.l	d2,d3
	beq.s	.n
	moveq	#1,d7
.n:	addq.l	#4,a1
	dbra	d4,.l
	rts

; the pass's interrupt must have been taken (and only one)
waitirq:
	move.l	#3000,d0
.w:	tst.l	(nirq).l
	bne.s	.g
	subq.l	#1,d0
	bne.s	.w
	failt	9
.g:	cmp.l	#1,(nirq).l
	beq.s	.r
	failt	10
.r:	rts

h_irq2:
	clr.w	(IPLREG).l
	addq.l	#1,(nirq).l
	rte

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99
