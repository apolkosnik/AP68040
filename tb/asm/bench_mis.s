; cycles of loads and stores by alignment, copyback, all hits: a STAMP
; after each loop (user mode: user data is copyback through DTT0; the
; stamps are written by the TRAP #2 handler, supervisor data noncachable)
STAMP	equ	$F108
DONEREG	equ	$F102
BUF	equ	$9800

stamp	macro
	move.w	#\1,d6
	trap	#2
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	32
	dc.l	0
	endr
	dc.l	h_stamp			; 34: TRAP #2
	dc.l	h_done			; 35: TRAP #3
	org	$400
start:
	move.l	#$0000A040,d0		; DTT1: supervisor data noncachable
	movec	d0,dtt1
	move.l	#$00008020,d0		; DTT0: user data copyback
	movec	d0,dtt0
	move.l	#$80008000,d0
	movec	d0,cacr
	lea	($3000).l,a1
	move	a1,usp
	move.w	#$0000,sr
	lea	(BUF).l,a0
	move.l	(a0),d1
	move.l	16(a0),d1
	move.l	d1,(a0)
	move.l	d1,16(a0)
	stamp	1
	move.w	#999,d7
.a:	move.l	(a0),d1			; aligned loads
	move.l	4(a0),d2
	move.l	8(a0),d3
	move.l	12(a0),d4
	dbra	d7,.a
	stamp	2
	move.w	#999,d7
.b:	move.l	1(a0),d1		; misaligned loads within a line
	move.l	5(a0),d2
	move.l	2(a0),d3
	move.l	7(a0),d4
	dbra	d7,.b
	stamp	3
	move.w	#999,d7
.c:	move.l	13(a0),d1		; loads across a line
	move.l	14(a0),d2
	move.l	15(a0),d3
	move.w	15(a0),d4
	dbra	d7,.c
	stamp	4
	move.w	#999,d7
.f:	move.l	d1,(a0)			; aligned stores
	move.l	d2,4(a0)
	move.l	d3,8(a0)
	move.l	d4,12(a0)
	dbra	d7,.f
	stamp	5
	move.w	#999,d7
.d:	move.l	d1,1(a0)		; misaligned stores within a line
	move.l	d2,5(a0)
	move.l	d3,2(a0)
	move.l	d4,7(a0)
	dbra	d7,.d
	stamp	6
	move.w	#999,d7
.e:	move.l	d1,13(a0)		; stores across a line
	move.l	d2,14(a0)
	move.l	d3,15(a0)
	move.w	d4,15(a0)
	dbra	d7,.e
	stamp	7
	trap	#3

h_stamp:
	move.w	d6,(STAMP).l
	rte
h_done:
	move.w	#$600D,(DONEREG).l
	bra.s	*
