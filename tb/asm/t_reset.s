; AP040 RESET on the pipelined core (caches stage F: the card's wrapper)
; assembled with vasmm68k_mot -Fbin -m68040 -no-opt
;
; testbench protocol: $F100 fail number, $F102 result magic; $F166 r: how
; many times RESET has driven the reset line
;
; MC68040UM RESET: in supervisor mode the reset line is driven -- once every
; earlier bus cycle has finished -- and the instruction then completes;
; in user mode it is a privilege violation and nothing is driven. The bench
; checks the line itself: it rose with no write pending, the bus stayed
; idle while it was up, and it was up for 128 of the core's cycles. RESET
; leaves the MMU alone: a translation in the ATC is still used after it,
; though the descriptor in memory has been changed behind the CPU.
;
; $F134 w: a poke's address, $F136 w: poke the word there behind the CPU

FAILREG		equ	$F100
DONEREG		equ	$F102
RSTCNT		equ	$F166
POKEA		equ	$F134
POKED		equ	$F136

; poke <address>,<word>: memory changed behind the CPU
poke	macro
	move.w	#\1,(POKEA).l
	move.w	#\2,(POKED).l
	endm

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	unexp,unexp,unexp,unexp,unexp,unexp
	dc.l	priv			; 8: privilege violation
	rept	247
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#$2700,sr
	moveq	#0,d6
	move.l	#$12345678,($5000).l	; a write RESET must wait for
	reset
	addq.l	#1,d6			; runs after it
	cmp.l	#$12345678,($5000).l
	beq.s	t1
	failt	1
t1:	cmp.w	#1,(RSTCNT).l		; the bench saw the line driven once
	beq.s	t2
	failt	2
t2:	cmp.l	#1,d6
	beq.s	t3
	failt	3
; the MMU is left alone: page 5 translated once (its entry in the ATC), its
; descriptor then changed in memory to physical page 6; a read after RESET
; still goes to page 5. D0 and A0 hold the page's address: a RESET that
; flushed (An) would have it to flush.
t3:	lea	($4400).l,a0
	moveq	#0,d0
	moveq	#63,d1
tloop:	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#4,d2
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,tloop
	move.l	#$00004203,($4000).l
	move.l	#$00004403,($4200).l
	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	pflusha
	move.l	#$8000,d0
	movec	d0,tc
	poke	$5000,$AAAA
	poke	$5002,$5555
	poke	$6000,$BBBB
	poke	$6002,$6666
	move.l	($5000).l,d1		; translated: page 5 in the ATC
	poke	$4414,$0000		; page 5's descriptor: physical $6000 now
	poke	$4416,$6003
	move.l	#$5000,d0
	lea	($5000).l,a0
	reset
	move.l	($5000).l,d2
	cmp.l	d1,d2			; the ATC's entry, still
	beq.s	t6
	failt	6
t6:	pflusha
	move.l	($5000).l,d2		; walked again: the new descriptor
	cmp.l	#$BBBB6666,d2
	beq.s	t7
	failt	7
t7:	moveq	#0,d0
	movec	d0,tc
	pflusha
; user mode: a privilege violation, nothing driven
	lea	(back).l,a0
	move.l	a0,($3500).l
	move.w	#$0000,sr		; user mode
	reset
	failt	4			; not reached
back:	cmp.w	#2,(RSTCNT).l		; driven twice in all, not by the user-mode RESET
	beq.s	t5
	failt	5
t5:	move.w	#$600D,(DONEREG).l
	stop	#$2700

; the privilege violation: back to supervisor code at 'back'
priv:
	move.l	($3500).l,2(sp)
	ori.w	#$2000,(sp)		; return in supervisor mode
	rte

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
