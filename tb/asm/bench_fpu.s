; FPU latency and throughput: dependent and independent chains (supervisor,
; no data memory in the loops)
STAMP	equ	$F108
DONEREG	equ	$F102
	org	0
	dc.l	$3400
	dc.l	start
	org	$400
start:
	move.l	#$80008000,d0
	movec	d0,cacr
	fmove.l	#3,fp0
	fmove.l	#1,fp1
	fmove.l	#2,fp2
	fmove.l	#5,fp3
	moveq	#1,d5
.p:	move.w	#1,(STAMP).l
	move.w	#99,d7
.a:	fadd.x	fp0,fp1			; dependent adds
	fadd.x	fp0,fp1
	fadd.x	fp0,fp1
	fadd.x	fp0,fp1
	dbra	d7,.a
	move.w	#2,(STAMP).l
	move.w	#99,d7
.b:	fadd.x	fp0,fp1			; independent adds
	fadd.x	fp0,fp2
	fadd.x	fp0,fp3
	fadd.x	fp0,fp4
	dbra	d7,.b
	move.w	#3,(STAMP).l
	move.w	#99,d7
.c:	fmul.x	fp0,fp1			; dependent multiplies
	fmul.x	fp0,fp1
	fmul.x	fp0,fp1
	fmul.x	fp0,fp1
	dbra	d7,.c
	move.w	#4,(STAMP).l
	move.w	#99,d7
.d:	fadd.x	fp0,fp1			; integer work beside the FPU
	addq.l	#1,d1
	addq.l	#1,d2
	addq.l	#1,d3
	dbra	d7,.d
	move.w	#5,(STAMP).l
	dbra	d5,.p
	move.w	#$600D,(DONEREG).l
	bra.s	*
