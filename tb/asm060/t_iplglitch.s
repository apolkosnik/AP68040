; AP68060 IPL level changes whose pins settle at different times
; assembled with vasmm68k_mot -Fbin -m68040
;
; IPL2-IPL0 are asynchronous inputs synchronized to CLK, and a request is
; valid once it has been held for two consecutive CLK periods (MC68060UM
; 7.8.1).  On an FPGA the three pins (or the three synchronizer flip-flops)
; do not change in the same clock: going from level 0 to level 3, the
; processor can see level 1 for one clock.  A processor idling in STOP takes
; the first level above its mask, so without the two-clock rule it takes
; that level 1, which no device raised (HRTmon's red screen on Minimig).
;
; The bench's $F16E makes chosen IPL bits change one clock after the others
; and raises a level some clocks later.  Each case holds its starting level
; under mask 7, arms the raise, STOPs with the case's mask, and checks that
; the handler that ran is the raised level's, and only once:
;   case n: test 2n-1 the first handler's level, 2n the number taken
;   1   0 -> 3, bit 1 late (1 seen for a clock)    mask 0
;   2   0 -> 3, bit 0 late (2)                     mask 0
;   3   0 -> 6, bit 2 late (2)                     mask 0
;   4   0 -> 6, bit 1 late (4)                     mask 0
;   5   0 -> 5, bit 0 late (4)                     mask 0
;   6   0 -> 5, bit 2 late (1)                     mask 0
;   7   0 -> 7, bits 2 and 1 late (1)              mask 0
;   8   2 -> 5, bit 0 late (4)                     mask 2
;   9   3 -> 4, bits 1 and 0 late (7, the NMI)     mask 3
;   10  3 -> 4, bit 2 late (0: the request drops for a clock; the raise
;       is still taken once)                       mask 3
;
;   98   an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
IPLSKEW	equ	$F16E

first	equ	$3600		; level of the first handler that ran (0: none)
count	equ	$3602		; handlers run

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	22
	dc.l	unexp
	endr
	dc.l	unexp		; 24 spurious
	dc.l	h_l1		; 25-31 autovectors
	dc.l	h_l2
	dc.l	h_l3
	dc.l	h_l4
	dc.l	h_l5
	dc.l	h_l6
	dc.l	h_l7
	rept	224
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#$2700,sr
	lea	cases(pc),a0
	moveq	#1,d6			; test number of the case's first check
case_loop:
	move.w	(a0)+,d0		; starting level ($FFFF: done)
	bmi	all_done
	move.w	(a0)+,d1		; IPLSKEW word: delay, level, late bits
	move.w	(a0)+,d2		; STOP's SR
	move.w	(a0)+,d3		; the level that must be taken
	move.w	#$2700,sr
	move.w	#0,(IPLSKEW).l		; no skew while setting up
	move.w	d0,(IPLREG).l		; the starting level, masked
	clr.w	(first).l
	clr.w	(count).l
	moveq	#50,d4
settle:	dbra	d4,settle
	move.w	d1,(IPLSKEW).l		; arm the skewed raise
	cmp.w	#$2000,d2		; STOP needs its mask as an immediate
	beq.s	stop0
	cmp.w	#$2200,d2
	beq.s	stop2
	stop	#$2300
	bra.s	woke
stop0:	stop	#$2000
	bra.s	woke
stop2:	stop	#$2200
woke:
	move.l	#200,d4			; room for a second (wrong) interrupt
linger:	subq.l	#1,d4
	bne.s	linger
	move.w	#$2700,sr
	cmp.w	(first).l,d3
	beq.s	lvl_ok
	move.w	d6,d7
	bra	fail_all
lvl_ok:
	addq.w	#1,d6
	cmp.w	#1,(count).l
	beq.s	cnt_ok
	move.w	d6,d7
	bra	fail_all
cnt_ok:
	addq.w	#1,d6
	bra	case_loop

all_done:
	move.w	#0,(IPLSKEW).l
	move.w	#0,(IPLREG).l
	move.w	#$600D,(DONEREG).l
	bra.s	*

; start level, IPLSKEW (delay << 8 | level << 4 | late bits), STOP SR,
; expected level
cases:
	dc.w	0,$C832,$2000,3
	dc.w	0,$C831,$2000,3
	dc.w	0,$C864,$2000,6
	dc.w	0,$C862,$2000,6
	dc.w	0,$C851,$2000,5
	dc.w	0,$C854,$2000,5
	dc.w	0,$C876,$2000,7
	dc.w	2,$C851,$2200,5
	dc.w	3,$C843,$2300,4
	dc.w	3,$C844,$2300,4
	dc.w	$FFFF

; each handler records its level if it is the first, counts, and releases
; the request (the device lets go once acknowledged)
hl	macro
h_l\1:
	tst.w	(first).l
	bne.s	.n\@
	move.w	#\1,(first).l
.n\@:	addq.w	#1,(count).l
	move.w	#0,(IPLREG).l
	nop
	rte
	endm

	hl	1
	hl	2
	hl	3
	hl	4
	hl	5
	hl	6
	hl	7

fail_all:
	move.w	#$2700,sr
	move.w	#0,(IPLSKEW).l
	move.w	#0,(IPLREG).l
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	98
