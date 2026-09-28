; branch_bench: a measurement program for the sequencer's unconditional
; transfers (2026-09-18): BSR.W/RTS, JSR abs.L/RTS, JSR d16(PC)/RTS, a
; forward BRA.W and a JMP abs.L whose targets lie outside the 32-byte
; refill sector, plus two controls that the early redirect does not cover
; (JSR (An) and a Bcc.W taken every other iteration), closed by DBcc.
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
	bsr.w	sub_w
	add.l	d0,d2
	moveq	#1,d1
	jsr	(sub_l).l
	add.l	d1,d2
	moveq	#2,d1
	jsr	sub_p(pc)
	add.l	d1,d2
	moveq	#3,d1
	bra.w	fwd1
	rept	20
	nop
	endr
fwd1:
	add.l	d1,d2
	moveq	#4,d1
	jmp	(fwd2).l
	rept	20
	nop
	endr
fwd2:
	add.l	d1,d2
	lea	sub_w(pc),a0
	moveq	#5,d1
	jsr	(a0)
	add.l	d1,d2
	move.l	d3,d0
	and.l	#1,d0
	beq.w	even
	addq.l	#3,d2
even:
	addq.l	#1,d2
	dbra	d3,loop

	cmp.l	#517000,d2
	beq.s	done
	move.w	#1,d7
	bra.s	fail_all

done:
	move.w	#$600D,(DONEREG).l
	stop	#$2700

	rept	8
	nop
	endr
sub_w:
	rts
	rept	8
	nop
	endr
sub_l:
	rts
	rept	8
	nop
	endr
sub_p:
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
