; AP68060 level 7 interrupts held by the device (MC68060UM 8.2.9, fig. 8-3)
; assembled with vasmm68k_mot -Fbin -m68040
;
; Level 7 is recognised on its edge -- even at mask 7 -- and also by level
; comparison like any other level: while IPL stays at 7, an RTE or MOVE to
; SR that lowers the mask below 7 takes it again.  The handler here leaves
; IPL at 7 for the first interrupt and releases it in the second, so each
; case takes exactly two.
;   1-2  mask 5: the edge, then the handler's RTE back to mask 5 with the
;        request still at 7: the second by level comparison
;   3-5  mask 7: the edge only (the RTE restores mask 7: nothing more);
;        MOVE #$2600,SR: the second
;   6    a third never comes once IPL is released
;
;   98   an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
cnt7	equ	$3600		; level 7 interrupts taken
stk_sr	equ	$3602		; the stacked SR of the last one

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkw	macro			; memory word, value, test
	cmp.w	#\2,(\1).l
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	29
	dc.l	unexp
	endr
	dc.l	h_l7		; 31 level 7 autovector
	rept	224
	dc.l	unexp
	endr

	org	$400
start:
;------------------------------------------------ 1-2 mask 5, RTE back to 5
	clr.w	(cnt7).l
	move.w	#$2500,sr
	move.w	#7,(IPLREG).l
	move.l	#500,d0
w1:	subq.l	#1,d0
	bne.s	w1
	chkw	cnt7,2,1
	moveq	#0,d0
	move.w	(stk_sr).l,d0
	and.w	#$0700,d0
	cmp.w	#$0500,d0	; the second interrupted mask 5 again
	beq.s	ok2
	failt	2
ok2:

;------------------------------------------------ 3-5 mask 7, MOVE to SR
	clr.w	(cnt7).l
	move.w	#$2700,sr
	move.w	#7,(IPLREG).l
	move.l	#500,d0
w3:	subq.l	#1,d0
	bne.s	w3
	chkw	cnt7,1,3	; the edge, once
	move.w	#$2600,sr	; mask 6: level 7 by comparison
	nop
	nop
	chkw	cnt7,2,4
	moveq	#0,d0
	move.w	(stk_sr).l,d0
	and.w	#$0700,d0
	cmp.w	#$0600,d0
	beq.s	ok5
	failt	5
ok5:
	move.l	#500,d0
w6:	subq.l	#1,d0
	bne.s	w6
	chkw	cnt7,2,6

	move.w	#$2700,sr
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	#$2700,sr
	move.w	#0,(IPLREG).l
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

; level 7: count; the second releases the request
h_l7:
	cmpi.w	#$007C,6(sp)
	bne.s	h_bad
	addq.w	#1,(cnt7).l
	move.w	(sp),(stk_sr).l
	cmpi.w	#2,(cnt7).l
	bne.s	h_rte
	move.w	#0,(IPLREG).l
	nop			; the release is on the bus before the RTE
h_rte:	rte
h_bad:
	failt	98

unexp:
	failt	98
