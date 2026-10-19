; AP68040: no interrupt inside a section that MOVE to SR raised to mask 7
; assembled with vasmm68k_mot -Fbin -m68040
;
; A level 2 request (bench $F148: IPL 2 after n clocks) is swept, n = 1 to
; 400, across a block of MOVE #$2000,SR / MOVE #$2700,SR pairs.  Each pair
; is followed by MOVEQ #1,D4 and the next starts with MOVEQ #0,D4: the
; request may be taken at the mask-0 boundary or at the boundary of the
; MOVE #$2700,SR itself (a claim made against mask 0), never later, so the
; handler must always see D4 = 0.  Kickstart's dispatcher relies on this
; (RemHead on TaskReady right after MOVE #$2700,SR).
;   1   the interrupt ran inside the mask-7 section (D4 = 1)
;   2   an iteration took no interrupt, or more than one
;   98  an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
IPLDLY	equ	$F148
nirq	equ	$E100
bad	equ	$E102

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

	org	0
	dc.l	$E000
	dc.l	start
	rept	24
	dc.l	unexp
	endr
	dc.l	h_irq2		; 26 level 2 autovector
	rept	229
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#1,d6		; n
iter:
	move.w	#$2700,sr
	clr.w	(nirq).l
	clr.w	(bad).l
	moveq	#0,d4
	move.w	d6,(IPLDLY).l	; IPL 2 in n clocks
	rept	48
	moveq	#0,d4
	move.w	#$2000,sr
	move.w	#$2700,sr
	moveq	#1,d4
	endr
	moveq	#0,d4
	move.w	#$2000,sr	; take it now if the window missed it
	move.l	#300,d0
w1:	subq.l	#1,d0
	bne.s	w1
	move.w	#$2700,sr
	tst.w	(bad).l
	beq.s	ok1
	failt	1
ok1:	cmpi.w	#1,(nirq).l
	beq.s	ok2
	failt	2
ok2:	addq.w	#1,d6
	cmpi.w	#400,d6
	bls	iter
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	#$2700,sr
	move.w	#0,(IPLREG).l
	move.w	d6,($E104).l	; n at the failure
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

h_irq2:
	addq.w	#1,(nirq).l
	tst.w	d4
	beq.s	h1
	move.w	#1,(bad).l
h1:	move.w	#0,(IPLREG).l
	nop			; the release is on the bus before the RTE
	rte

unexp:
	failt	98
