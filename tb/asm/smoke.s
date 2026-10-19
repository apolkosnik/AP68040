; AP68040-60 bring-up smoke test: a handful of instructions, then pass
FAILREG	equ	$F100
DONEREG	equ	$F102

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	moveq	#5,d0
	moveq	#0,d1
loop:	add.l	d0,d1
	subq.l	#1,d0
	bne.s	loop
	cmp.l	#15,d1
	bne	bad1
	lea	buf,a0
	move.l	#$12345678,(a0)+
	move.w	#$9ABC,(a0)
	move.l	buf,d2
	cmp.l	#$12345678,d2
	bne	bad2
	move.w	-(a0),d3
	cmp.w	#$5678,d3
	bne	bad3
	bsr	sub1
	cmp.l	#42,d4
	bne	bad4
	move.l	#$11112222,d5
	move.l	d5,-(sp)
	move.l	(sp)+,d6
	cmp.l	d5,d6
	bne	bad5
	move.w	#$600D,DONEREG
stop:	bra.s	stop

sub1:	moveq	#42,d4
	rts

bad1:	move.w	#1,FAILREG
	bra.s	fail
bad2:	move.w	#2,FAILREG
	bra.s	fail
bad3:	move.w	#3,FAILREG
	bra.s	fail
bad4:	move.w	#4,FAILREG
	bra.s	fail
bad5:	move.w	#5,FAILREG
	bra.s	fail
unexp:	move.w	#99,FAILREG
fail:	move.w	#$BAD0,DONEREG
	bra.s	fail

	even
buf:	dc.l	0,0
