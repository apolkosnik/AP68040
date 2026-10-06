; cycles of common kernels, copyback user data, warm caches: a STAMP after
; each (user mode; the TRAP #2 handler writes the stamps)
;   1 move.l (a0)+,(a1)+ x 256 (1 KB)       5 move16 (a0)+,(a1)+ x 64 (1 KB)
;   2 move.b (a0)+,(a1)+ x 1024             6 fmul.x/fadd.x x 256
;   3 clr.l (a0)+ x 256                     7 strlen-style byte scan x 1024
;   4 movem.l 12 regs in, 12 out x 21
STAMP	equ	$F108
DONEREG	equ	$F102
SRC	equ	$9000
DST	equ	$A000

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
	moveq	#1,d5			; two passes: the second is measured
pass:
	lea	(SRC).l,a0		; fill the source (and warm both buffers)
	lea	(DST).l,a1
	move.w	#255,d7
.w:	move.l	d7,(a0)+
	clr.l	(a1)+
	dbra	d7,.w
	clr.b	(SRC+1023).l
	stamp	1
	lea	(SRC).l,a0
	lea	(DST).l,a1
	move.w	#255,d7
.k1:	move.l	(a0)+,(a1)+
	dbra	d7,.k1
	stamp	2
	lea	(SRC).l,a0
	lea	(DST).l,a1
	move.w	#1023,d7
.k2:	move.b	(a0)+,(a1)+
	dbra	d7,.k2
	stamp	3
	lea	(DST).l,a1
	move.w	#255,d7
.k3:	clr.l	(a1)+
	dbra	d7,.k3
	stamp	4
	lea	(SRC).l,a0
	lea	(DST).l,a1
	moveq	#20,d7
.k4:	movem.l	(a0)+,d0-d4/a2-a6/d6/a3
	movem.l	d0-d4/a2-a6/d6/a3,(a1)
	lea	48(a1),a1
	dbra	d7,.k4
	stamp	5
	lea	(SRC).l,a0
	lea	(DST).l,a1
	moveq	#63,d7
.k5:	move16	(a0)+,(a1)+
	dbra	d7,.k5
	stamp	6
	fmove.l	#3,fp0
	fmove.l	#1,fp1
	fmove.l	#0,fp2
	move.w	#255,d7
.k6:	fmul.x	fp0,fp1
	fadd.x	fp1,fp2
	dbra	d7,.k6
	stamp	7
	lea	(SRC+1).l,a0		; bytes 1..1022 are nonzero low bytes...
	move.w	#1023,d7
.k7:	tst.b	(a0)+
	dbra	d7,.k7
	stamp	8
	dbra	d5,pass
	trap	#3

h_stamp:
	move.w	d6,(STAMP).l
	rte
h_done:
	move.w	#$600D,(DONEREG).l
	bra.s	*
