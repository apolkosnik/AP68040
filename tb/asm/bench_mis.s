; cycles of loads by alignment (copyback, all hits): STAMP after each loop
STAMP	equ	$F108
DONEREG	equ	$F102
BUF	equ	$9800
	org	0
	dc.l	$3400
	dc.l	start
	org	$400
start:
	move.l	#$80008000,d0
	movec	d0,cacr
	lea	(BUF).l,a0
	move.l	(a0),d1
	move.l	16(a0),d1
	move.w	#1,(STAMP).l
	move.w	#999,d7
.a:	move.l	(a0),d1
	move.l	4(a0),d2
	move.l	8(a0),d3
	move.l	12(a0),d4
	dbra	d7,.a
	move.w	#2,(STAMP).l
	move.w	#999,d7
.b:	move.l	1(a0),d1
	move.l	5(a0),d2
	move.l	2(a0),d3
	move.l	7(a0),d4
	dbra	d7,.b
	move.w	#3,(STAMP).l
	move.w	#999,d7
.c:	move.l	13(a0),d1
	move.l	14(a0),d2
	move.l	15(a0),d3
	move.w	15(a0),d4
	dbra	d7,.c
	move.w	#4,(STAMP).l
	move.w	#$600D,(DONEREG).l
	bra.s	*
