; AP68040: the Minimig demo hang of 2026-10-23.  The board (DUAL=1 BTIC=1
; LOOP_PRED=0) livelocked in this inner loop, run from fast RAM at
; $41E45CBA: the back end redirected to $..CC8 (the DBF's displacement
; word) every 14 clocks, D1 sent the fetch back to the loop head, and
; nothing retired again.  The loop sits at the same address bits here
; ($5CBA: the BTB index and the cache set match).
; assembled with vasmm68k_mot -Fbin -m68040
;   1    the loop run for counts 0..40, 20 passes each: D1 = count + 1
;   2    the same with a level 2 interrupt raised 1..120 clocks after the
;        call: D1 and the interrupt count
;   3    the board's trigger: a BTB entry left by the code that was there
;        before (BRA.S ending at the DBF's first word, to the routine's
;        start), the new code written over it and CPUSHA.  The fetch hands
;        the DBF's first word and the old target's words to D1; the DBF
;        runs across the flag and is refetched at WB (EXC_SNR), and the
;        wrong path behind it, its PCs a word off, must not write the
;        entry back (it did: WB refetched $..CC8 for ever)

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
IPLDLY	equ	$F148
LOOP	equ	$5CBA
NEW	equ	$6CB8		; case 3: the board's $41E45CB8 (the same index bits)
nirq	equ	$3604

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

	org	0
	dc.l	$3400
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
	move.l	#$80008000,d0
	movec	d0,cacr
	clr.l	(nirq).l
	move.w	#$2000,sr

;------------------------------------------------ 1 counts 0..40
	moveq	#0,d6			; the count
.c1:	moveq	#19,d5			; passes
.p1:	bsr	call
	move.l	d6,d3
	addq.l	#1,d3
	cmp.l	d3,d1
	beq.s	.k1
	failt	1
.k1:	dbra	d5,.p1
	addq.l	#1,d6
	cmp.l	#41,d6
	bne.s	.c1

;------------------------------------------------ 2 with an interrupt
	moveq	#1,d6			; the delay
.c2:	clr.l	(nirq).l
	move.w	d6,(IPLDLY).l
	move.l	d6,d0
	and.l	#15,d0
	bsr	call0
	move.l	d6,d3
	and.l	#15,d3
	addq.l	#1,d3
	cmp.l	d3,d1
	beq.s	.k2
	failt	2
.k2:	move.l	#2000,d0
.w2:	tst.l	(nirq).l
	bne.s	.i2
	subq.l	#1,d0
	bne.s	.w2
	failt	3
.i2:	addq.l	#1,d6
	cmp.l	#121,d6
	bne.s	.c2


;------------------------------------------------ 3 a stale BTB entry
	; the old code: at NEW+$10 BRA.S NEW, at NEW RTS; called three times
	lea	(NEW).l,a1
	move.w	#$4E75,(a1)
	move.w	#$60EE,$10(a1)
	cpusha	bc
	moveq	#2,d5
.o3:	jsr	$10(a1)
	dbra	d5,.o3
	; the new code over it, as the demo's
	lea	newc(pc),a2
	moveq	#(newe-newc)/2-1,d5
.n3:	move.w	(a2)+,(a1)+
	dbra	d5,.n3
	cpusha	bc
	moveq	#0,d6
.c3:	move.l	d6,d0
	addq.l	#1,d0
	neg.l	d0			; NEG then SUBQ: the count again
	moveq	#0,d1
	moveq	#0,d2
	moveq	#0,d4
	moveq	#0,d7
	move.l	#1,a4
	move.l	#$10000,a5
	move.l	#$123,a0
	jsr	(NEW).l
	move.l	d6,d3
	addq.l	#1,d3
	cmp.l	d3,d1
	beq.s	.k3
	failt	4
.k3:	addq.l	#1,d6
	cmp.l	#20,d6
	bne.s	.c3

	move.w	#$600D,(DONEREG).l
	bra.s	*

; the demo's routine, as the board fetched it at $41E45CB8 (the loop at
; NEW+4, the DBF at NEW+$10)
newc:	neg.w	d0
	subq.w	#1,d0
.l:	add.l	a4,d1
	addx.w	d7,d1
	add.l	a5,d2
	addx.w	d7,d2
	add.l	a0,d4
	addx.w	d7,d4
	dbf	d0,.l
	moveq	#0,d0
	rts
newe:

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

; the loop with D0 = D6
call:	move.l	d6,d0
; the loop with D0 as given: D1 = D0 + 1
call0:	moveq	#0,d1
	moveq	#0,d2
	moveq	#0,d4
	moveq	#0,d7
	move.l	#1,a4
	move.l	#$10000,a5
	move.l	#$123,a0
	jmp	(LOOP).l

h_irq2:
	clr.w	(IPLREG).l
	addq.l	#1,(nirq).l
	rte

unexp:
	failt	99

	org	LOOP
	add.l	a4,d1
	addx.w	d7,d1
	add.l	a5,d2
	addx.w	d7,d2
	add.l	a0,d4
	addx.w	d7,d4
	dbf	d0,LOOP
	moveq	#0,d0
	move.w	.w(pc),d3
	rts
.w:	dc.w	0
