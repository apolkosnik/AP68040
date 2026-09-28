; pipe_bench: a measurement program for the sequencer's control-flow paths
; (2026-09-17): BSR.W/RTS, LINK/UNLK, MOVEM push/pop, a forward taken Bcc.B
; whose target lies outside the 32-byte refill sector, and a DBcc loop.
; Verifies its own checksum.  Same protocol as bench_loop.s.
;
; assembled with vasmm68k_mot -Fbin -m68040 -no-opt

FAILREG	equ	$F100
DONEREG	equ	$F102
ITERS	equ	1000

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$80008000,d0
	movec	d0,cacr
	moveq	#0,d2
	move.w	#ITERS-1,d3
loop:
	move.l	d3,d0
	bsr.w	sub1
	add.l	d0,d2
	move.l	d3,d0
	and.l	#1,d0
	beq.s	even
	addq.l	#3,d2
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
even:
	addq.l	#1,d2
	dbra	d3,loop

	cmp.l	#1001500,d2
	beq.s	done
	move.w	#1,d7
	bra.s	fail_all

done:
	move.w	#$600D,(DONEREG).l
	stop	#$2700

	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
sub1:
	link	a6,#-8
	movem.l	d4-d5/a2-a3,-(sp)
	move.l	d0,-4(a6)
	move.l	-4(a6),d4
	lsl.l	#1,d4
	move.l	d4,d0
	movem.l	(sp)+,d4-d5/a2-a3
	unlk	a6
	rts

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1

unexp:
	move.w	#$0099,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2
