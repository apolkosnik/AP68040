; AP68040: second-slot loads (DUAL).  A one-word (Ay) load beside a uop
; with no memory access of its own takes the pair's access.  Only the
; data cache's fast path answers it there; anything else (a miss, a
; noncachable access, a fault) makes WB commit the first slot and run the
; load again alone, so the bus and the ATC see exactly what they would
; without dual issue.  The bench counts the CPU's data reads in a window
; ($F2D0-$F2DC).
; assembled with vasmm68k_mot -Fbin -m68040
;   1    a noncachable load at a taken loop branch's target: one bus read
;        per iteration, the sum right
;   2    the same beside an ADDQ (no change of flow)
;   3    beside a branch that falls through
;   4    a bus error on a second slot's load: the first slot completes
;        once, the frame names the load, its restart reads the data
;   5    copyback: the loads hit (and miss once), the sums right
;   6    a second slot's ALU, CMP, TST and MOVEA.L loads

FAILREG	equ	$F100
DONEREG	equ	$F102
BERRCTL	equ	$F142
BERRADR	equ	$F140
WLO	equ	$F2D0
WHI	equ	$F2D4
WRD	equ	$F2D8
WWR	equ	$F2DC
L0	equ	$9C00
cnt_be	equ	$3600
be_pc	equ	$3604
be_d1	equ	$3608

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro
	cmp.l	#\2,\1
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	h_buserr
	rept	253
	dc.l	unexp
	endr

	org	$400
start:
	clr.l	(cnt_be).l
	move.l	#$01010101,(L0).l
	move.l	#$01000100,(L0+4).l
	move.l	#$80008000,d0
	movec	d0,cacr
	move.l	#$0000C040,d0		; DTT0: everything noncachable serialized
	movec	d0,dtt0
	move.l	#L0,(WLO).l
	move.l	#L0+4,(WHI).l

;------------------------- 1 the load at a taken loop branch's target
	clr.l	(WRD).l
	lea	(L0).l,a0
	moveq	#20,d2
	moveq	#0,d3
	bra.w	.l1
	cnop	0,4
.l1:	move.l	(a0),d0
	add.l	d0,d3
	subq.l	#1,d2
	bne.s	.l1
	chkl	d3,20*$01010101,1
	move.l	(WRD).l,d1
	chkl	d1,20,1

;-------------------------------------------- 2 beside an ADDQ, no branch
	clr.l	(WRD).l
	moveq	#0,d3
	moveq	#0,d4
	moveq	#10,d2
.l2:	addq.l	#1,d4
	move.l	(a0),d0
	add.l	d0,d3
	subq.l	#1,d2
	bne.s	.l2
	chkl	d3,10*$01010101,2
	chkl	d4,10,2
	move.l	(WRD).l,d1
	chkl	d1,10,2

;----------------------------------- 3 beside a branch that falls through
	clr.l	(WRD).l
	moveq	#1,d2
	moveq	#0,d3
	subq.l	#1,d2
	bne.s	.l3x
	move.l	(a0),d0
	add.l	d0,d3
.l3x:	move.l	(WRD).l,d1
	chkl	d1,1,3
	chkl	d3,$01010101,3

;------------------------------------------ 4 a bus error on the second slot
	move.w	#1,(BERRCTL).l		; (armed by reset already: again anyway)
	lea	(BERRADR).l,a1
	moveq	#0,d1
	bra.w	.t4
.t4:	moveq	#7,d1			; (the load its companion)
ld4:	move.w	(a1),d0
	move.l	(cnt_be).l,d5
	chkl	d5,1,4
	move.l	(be_pc).l,d5
	chkl	d5,ld4,4
	move.l	(be_d1).l,d5
	chkl	d5,7,4
	chkl	d1,7,4

;----------------------------------------------- 5 copyback: the loads hit
	move.l	#$0000C020,d0
	movec	d0,dtt0
	cinva	dc
	moveq	#0,d3
	moveq	#0,d4
	moveq	#30,d2
	bra.w	.l5
	cnop	0,4
.l5:	move.l	(a0),d0
	add.l	d0,d3
	addq.l	#4,d4
	move.b	(a0),d0
	add.l	d0,d3
	subq.l	#1,d2
	bne.s	.l5
	chkl	d3,60*$01010101,5
	chkl	d4,120,5

;------------------------------------------- 6 ALU, CMP, TST, MOVEA loads
	lea	(L0+4).l,a2
	moveq	#5,d0
	moveq	#1,d1
	add.l	(a2),d0
	chkl	d0,$01000105,6
	moveq	#0,d1
	sub.w	(a2),d0
	chkl	d0,$01000005,6
	moveq	#3,d1
	cmp.l	(a0),d1			; 3 - $01010101: N, C
	bcc	f6
	bpl	f6
	moveq	#0,d1
	tst.l	(a2)
	beq	f6
	moveq	#-1,d1
	and.w	(a0),d1
	chkl	d1,$FFFF0101,6
	moveq	#2,d1
	or.b	(a0),d1
	chkl	d1,$03,6
	moveq	#0,d2
	movea.l	(a2),a3
	move.l	a3,d2
	chkl	d2,$01000100,6

	bsr	nocache
	clr.l	(WLO).l
	clr.l	(WHI).l
	move.w	#$600D,(DONEREG).l
	bra.s	*
f6:	failt	6

; access error: count it, keep the stacked PC and D1, return (the read
; is restarted; the bench rejects it only once)
h_buserr:
	addq.l	#1,(cnt_be).l
	move.l	2(sp),(be_pc).l
	move.l	d1,(be_d1).l
	rte

; the bench's registers see a write only on the bus
nocache:
	cpusha	dc
	move.l	#$0000C040,d0
	movec	d0,dtt0
	rts

fail_all:
	bsr	nocache
	clr.l	(WLO).l
	clr.l	(WHI).l
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99
