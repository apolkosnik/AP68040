; AP68040-60 front end: the loop exit predictor
; assembled with vasmm68k_mot -Fbin -m68040
;
; D1 predicts a backward Bcc/DBcc to fall through when it has been taken
; as many times as on its last two exits (the trip).  EX trains the count,
; D1 counts ahead of it, and a flush puts D1 back on EX's count.  The
; predictor is a prediction: every case checks the architectural result;
; the cases with a fixed trip also check, through the bench's Bcc/DBcc
; misprediction counter ($F2E4), that the exits are predicted:
;   1    Bcc loop, trip 20, 40 passes: the sum, and one misprediction (the
;        outer loop's own exit) after the same code learned for 8 passes
;   2    the same with DBRA, trip 17
;   3    a one-instruction loop (the branch itself and a SUBQ): many
;        instances of the branch in flight at once, trip 9
;   4    the trip changes every pass (1..5 in turn): the sum
;   5    trip 0 (a backward branch never taken) and trip 300 (past the
;        8-bit count): the sums
;   6    nested loops, trips 3 inside 4, 30 passes: the sum, one
;        misprediction after learning
;   7    a TRAP inside the loop body on every pass (a flush in each
;        pass, at a different point in the count): the sum and the trap
;        count, and the exits still predicted
;   8    a forward branch inside the body that alternates direction
;        (mispredicted flushes mid-loop): the sum
;   9    the trip grows by one after it was learned: predicted exit at the
;        old trip is wrong (the branch is taken), relearned after two
;        exits: the sum, and at most 4 mispredicted over 12 passes
;  10    twelve loops in turn (more than the 8 entries): the sums
;  11    two loops 8 KB apart (the same PC[12:1] tag), alternating: sums
;  12    a level 2 interrupt arriving at every clock from 1 to 60 across a
;        loop that is being predicted: the sum each time, one interrupt

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
IPLDLY	equ	$F148
MISP	equ	$F2E4		; Bcc/DBcc mispredicted (bench counter)

ntrap	equ	$3600		; TRAP #0 taken
nirq	equ	$3604		; level 2 interrupts taken

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro
	cmp.l	#\2,\1
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

chkm	macro
	cmp.l	#\2,(\1).l
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

; at most \1 mispredicted since MISP was cleared, else fail \2
chkmp	macro
	cmp.l	#\1,(MISP).l
	bls.s	.ok\@
	failt	\2
.ok\@:
	endm

; test 10: a loop of trip \1 adding 1
lp10	macro
	moveq	#\1,d1
.l\@:	addq.l	#1,d0
	subq.l	#1,d1
	bne.s	.l\@
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	24
	dc.l	unexp
	endr
	dc.l	h_irq2		; 26 level 2 autovector
	rept	5
	dc.l	unexp
	endr
	dc.l	h_trap0		; 32 TRAP #0
	rept	223
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$0000A040,d0		; DTT1: supervisor data noncachable
	movec	d0,dtt1			; (the bench registers)
	move.l	#$80008000,d0
	movec	d0,cacr
	clr.l	(ntrap).l
	clr.l	(nirq).l
	move.w	#$2700,sr

;------------------------------------------------- 1 Bcc loop, trip 20
; the same loop code learns (8 passes), then runs 40 passes counted: its
; exits predicted, the one misprediction the outer DBRA's own exit.
; inner: d1 = 20 .. 1, sum 210 a pass
	moveq	#0,d0
	moveq	#7,d6
	bsr	t1loop
	clr.l	(MISP).l
	moveq	#39,d6
	bsr	t1loop
	chkmp	1,1
	chkl	d0,48*210,1
	bra	t2

t1loop:	moveq	#20,d1
t1b:	add.l	d1,d0
	subq.l	#1,d1
	bne.s	t1b
	dbra	d6,t1loop
	rts

;------------------------------------------------- 2 DBRA loop, trip 17
; inner: d1 = 16 .. -1, the body 17 times: sum 16..0 = 136 a pass
t2:	moveq	#0,d0
	moveq	#7,d6
	bsr	t2loop
	clr.l	(MISP).l
	moveq	#39,d6
	bsr	t2loop
	chkmp	1,2
	chkl	d0,48*136,2
	bra	t3

t2loop:	moveq	#16,d1
t2b:	add.l	d1,d0
	dbra	d1,t2b
	dbra	d6,t2loop
	rts

;------------------------------------------------- 3 one-instruction body
; d1 = 9 .. 1: the loop is SUBQ and BNE, d2 counts the passes
t3:	moveq	#0,d2
	moveq	#7,d6
	bsr	t3loop
	clr.l	(MISP).l
	moveq	#39,d6
	bsr	t3loop
	chkmp	1,3
	chkl	d2,48,3
	bra	t4

t3loop:	moveq	#9,d1
t3b:	subq.l	#1,d1
	bne.s	t3b
	add.l	d1,d2			; d1 = 0 here
	addq.l	#1,d2
	dbra	d6,t3loop
	rts

;------------------------------------------------- 4 changing trip
; pass k (0..49): trip (k mod 5) + 1; sum of the trips' sums:
; 10 rounds of (1 + 3 + 6 + 10 + 15) = 350
t4:	moveq	#0,d0
	moveq	#0,d3			; k mod 5
	moveq	#49,d6
t4a:	move.l	d3,d1
	addq.l	#1,d1
t4b:	add.l	d1,d0
	subq.l	#1,d1
	bne.s	t4b
	addq.l	#1,d3
	cmp.l	#5,d3
	bne.s	t4c
	moveq	#0,d3
t4c:	dbra	d6,t4a
	chkl	d0,350,4

;------------------------------------------------- 5 trip 0 and trip 300
; trip 0: BMI back to the ADD is never taken (d1 stays positive); 30
; passes add 1 each.  trip 300: d1 = 300 .. 1, sum 45150, 4 passes
	moveq	#0,d0
	moveq	#29,d6
t5a:	moveq	#1,d1
t5b:	add.l	d1,d0
	tst.l	d1
	bmi.s	t5b
	dbra	d6,t5a
	chkl	d0,30,5
	moveq	#0,d0
	moveq	#3,d6
t5c:	move.l	#300,d1
t5d:	add.l	d1,d0
	subq.l	#1,d1
	bne.s	t5d
	dbra	d6,t5c
	chkl	d0,4*45150,5

;------------------------------------------------- 6 nested loops
; middle: d2 = 4 .. 1, inner d1 = 3 .. 1: each pass adds 4 * 6 = 24;
; both loops' exits predicted, the outer DBRA's exit mispredicted
	moveq	#0,d0
	moveq	#7,d6
	bsr	t6loop
	clr.l	(MISP).l
	moveq	#29,d6
	bsr	t6loop
	chkmp	1,6
	chkl	d0,38*24,6
	bra	t7

t6loop:	moveq	#4,d2
t6b:	moveq	#3,d1
t6c:	add.l	d1,d0
	subq.l	#1,d1
	bne.s	t6c
	subq.l	#1,d2
	bne.s	t6b
	dbra	d6,t6loop
	rts

;------------------------------------------------- 7 TRAP in the body
; trip 10 (d1 = 10 .. 1); a TRAP #0 on the iteration where d1 equals
; d3, which walks 1..10 over the passes: each pass flushes at a different
; count.  sum 55 a pass, one trap a pass.  The BNE past the TRAP is not
; taken once in ten (mispredicted once a pass: 30), the BNE past the d3
; wrap once in ten passes (3); the loop exits are predicted (the outer
; DBRA's exit: 1 more)
t7:	clr.l	(ntrap).l
	moveq	#0,d0
	moveq	#1,d3
	moveq	#7,d6
	bsr	t7loop
	clr.l	(MISP).l
	moveq	#29,d6
	bsr	t7loop
	chkmp	34,7
	chkl	d0,38*55,7
	chkm	ntrap,38,7
	bra	t8

t7loop:	moveq	#10,d1
t7b:	add.l	d1,d0
	cmp.l	d1,d3
	bne.s	t7c
	trap	#0
t7c:	subq.l	#1,d1
	bne.s	t7b
	addq.l	#1,d3
	cmp.l	#11,d3
	bne.s	t7d
	moveq	#1,d3
t7d:	dbra	d6,t7loop
	rts

;------------------------------------------------- 8 alternating branch
; trip 12 (d1 = 12 .. 1): odd d1 adds 100, even d1 adds 1 (the BEQ
; alternates); 6 odd + 6 even a pass = 606, 20 passes
t8:	moveq	#0,d0
	moveq	#19,d6
t8a:	moveq	#12,d1
t8b:	btst	#0,d1
	beq.s	t8c
	add.l	#99,d0
t8c:	addq.l	#1,d0
	subq.l	#1,d1
	bne.s	t8b
	dbra	d6,t8a
	chkl	d0,20*606,8

;------------------------------------------------- 9 trip grows by one
; 10 passes of trip 10 (sum 55), then 12 passes of trip 11 (sum 66).
; The same loop for both: d4 holds the trip
	moveq	#0,d0
	moveq	#10,d4
	moveq	#9,d6
	bsr	t9loop
	moveq	#11,d4
	moveq	#11,d6
	clr.l	(MISP).l
	bsr	t9loop
	; old-trip exit predicted (taken: 1), relearning exits (2), the
	; outer DBRA's exit (1)
	chkmp	4,9
	chkl	d0,10*55+12*66,9
	bra	t10

t9loop:	move.l	d4,d1
t9b:	add.l	d1,d0
	subq.l	#1,d1
	bne.s	t9b
	dbra	d6,t9loop
	rts

;------------------------------------------------- 10 twelve loops
; loop j (1..12) runs j times adding 1; 10 rounds of all twelve:
; 10 * 78 = 780
t10:	moveq	#0,d0
	moveq	#9,d6
t10a:
	lp10	1
	lp10	2
	lp10	3
	lp10	4
	lp10	5
	lp10	6
	lp10	7
	lp10	8
	lp10	9
	lp10	10
	lp10	11
	lp10	12
	dbra	d6,t10a
	chkl	d0,780,10

;------------------------------------------------- 11 the same tag
; near ($1000) and far ($3000, the same PC[12:1]) loops alternate:
; near trip 5 (sum 15), far trip 7 (sum 28), 20 rounds
	moveq	#0,d0
	moveq	#19,d6
t11a:	bsr	near
	jsr	far
	dbra	d6,t11a
	chkl	d0,20*43,11
	bra	t12


;------------------------------------------------- 12 interrupts
; trip 8 (sum 36) learned; then for each delay 1..60, the interrupt is
; armed and the loop run once: the sum each time, one interrupt each
t12:	moveq	#0,d0
	moveq	#9,d6
t12a:	moveq	#8,d1
t12b:	add.l	d1,d0
	subq.l	#1,d1
	bne.s	t12b
	dbra	d6,t12a
	chkl	d0,360,12
	moveq	#1,d5
t12c:	clr.l	(nirq).l
	moveq	#0,d0
	move.w	#$2000,sr		; interrupts on
	move.w	d5,(IPLDLY).l
	moveq	#8,d1
t12d:	add.l	d1,d0
	subq.l	#1,d1
	bne.s	t12d
	moveq	#8,d1
t12e:	add.l	d1,d0
	subq.l	#1,d1
	bne.s	t12e
	moveq	#99,d2			; let a late interrupt arrive
t12f:	dbra	d2,t12f
	move.w	#$2700,sr
	chkl	d0,72,12
	chkm	nirq,1,12
	addq.l	#1,d5
	cmp.l	#61,d5
	bne.s	t12c

;----------------------------------------------------------------- done
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	#$2700,sr
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

h_trap0:
	addq.l	#1,(ntrap).l
	rte

h_irq2:
	clr.w	(IPLREG).l
	addq.l	#1,(nirq).l
	rte

unexp:
	failt	99

; test 11's loops, 8 KB apart: the same PC[12:1]
	org	$1000
near:	moveq	#5,d1
nr1:	add.l	d1,d0
	subq.l	#1,d1
	bne.s	nr1
	rts

	org	$3000
far:	moveq	#7,d1
fr1:	add.l	d1,d0
	subq.l	#1,d1
	bne.s	fr1
	rts
