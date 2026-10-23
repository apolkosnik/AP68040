; AP68040: a second-slot load never reads a device on a discarded path.
; The Minimig's hard-disk boot (68040.library: I/O space noncachable
; serialized) lost an interrupt and waited for ever with dual issue on: a
; companion load beside a branch went to the bus before its own pair's
; branch was resolved (dm_older counts only the uops older than the pair),
; so a mispredicted branch read a device register (CIA ICR: read clears)
; on the path it then discarded.
; assembled with vasmm68k_mot -Fbin -m68040
;
; $F178 counts its reads (tb_ap68040).  DTT0 makes $00xxxxxx noncachable
; serialized data.
;   1    a DBRA loop whose target's first word reads the device (the
;        BTB-taken target's companion): 20 passes of 8; the exit is
;        mispredicted every pass.  The values read are 0, 1, 2 ... and the
;        count is 160
;   2    the same with a Bcc (BNE) closing the loop

FAILREG	equ	$F100
DONEREG	equ	$F102
DEV	equ	$F178

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	62
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$80008000,d0
	movec	d0,cacr
	move.l	#$0000C040,d0		; $00xxxxxx, enabled, either mode, serialized
	movec	d0,dtt0
	lea	(DEV).l,a1

;------------------------------------------------ 1 DBRA
	clr.l	(a1)
	moveq	#0,d1			; the value expected next
	moveq	#19,d5
.p1:	moveq	#7,d4
.l1:	move.l	(a1),d3			; the device (the DBRA's target companion)
	cmp.l	d1,d3
	beq.s	.n1
	failt	1
.n1:	addq.l	#1,d1
	dbra	d4,.l1
	dbra	d5,.p1
	move.l	(a1),d3
	cmp.l	#160,d3
	beq.s	.k1
	failt	2

;------------------------------------------------ 2 BNE
.k1:	clr.l	(a1)
	moveq	#0,d1
	moveq	#19,d5
.p2:	moveq	#8,d4
.l2:	move.l	(a1),d3
	cmp.l	d1,d3
	beq.s	.n2
	failt	3
.n2:	addq.l	#1,d1
	subq.l	#1,d4
	bne.s	.l2
	dbra	d5,.p2
	move.l	(a1),d3
	cmp.l	#160,d3
	beq.s	.k2
	failt	4

.k2:	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99
