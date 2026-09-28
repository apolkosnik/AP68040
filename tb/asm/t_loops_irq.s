; AP040 loops under an interrupt storm (2026-09-19)
;
; Build 9 (the loop-top record cache, increment 12) returned impossible
; Speedometer loop-test times in 2 of 5 runs on the board: a loop that
; "finished" forty times sooner is a loop that did not run.  This program
; runs checksummed loop workloads of every shape the record cache replays
; -- a bubble sort with data-dependent branches, DBcc loops whose counter
; is the loop top's index register, loops whose top has an immediate or is
; a descriptor class, nested loops sharing the one cache entry, a body
; longer than the refill seed -- once quietly, then again under 400
; level-2 interrupts arriving at rotating intervals, and compares the
; two runs' checksums.  It also checks fixed properties (the sort's
; order and sum).  Protocol as t_integer.s.
;
;   word write to $F100 = failing test number
;   word write to $F102 = $BAD0 on failure, $600D when all tests passed
;   word write to $F110 = request interrupt level (0 releases)
;   word write to $F148 = raise level 2 N cycles from now
;
; assembled with vasmm68k_mot -Fbin -m68040 -no-opt

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
IPLDLY	equ	$F148

NIRQ	equ	400

cnt_int2	equ	$3600
irq_on		equ	$3602
ref_a		equ	$3610	; the quiet run's checksums
ref_b		equ	$3614
ref_c		equ	$3618
ref_d		equ	$361C
ref_e		equ	$3620
ref_f		equ	$3624
sum_init	equ	$3628
ARR		equ	$4000	; 64 words
ARR2		equ	$4100	; 16 longs
CELL		equ	$4200

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro
	cmp.l	#\2,\1
	beq.s	ok\@
	failt	\3
ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	24
	dc.l	unexp		; 2-25
	endr
	dc.l	h_int2		; 26 level 2 autovector
	rept	229
	dc.l	unexp		; 27-255
	endr

	org	$400
start:
	move.l	#$80008000,d0
	movec	d0,cacr
	clr.w	(cnt_int2).l
	clr.w	(irq_on).l
	move.w	#$2000,sr	; supervisor, mask 0

;----------------------------------------------- the quiet reference run
	bsr.w	workloads
	move.l	d4,(ref_a).l
	move.l	d5,(ref_b).l
	move.l	d6,(ref_c).l
	move.l	d7,(ref_d).l
	move.l	a5,(ref_e).l
	move.l	a6,(ref_f).l

;----------------------------------------------- the same under interrupts
	move.w	#1,(irq_on).l
	move.w	#7,(IPLDLY).l	; the first one; the handler re-arms the rest
	bsr.w	workloads
	move.w	#0,(irq_on).l
	move.w	#0,(IPLREG).l
	cmp.l	(ref_a).l,d4
	bne	f_a
	cmp.l	(ref_b).l,d5
	bne	f_b
	cmp.l	(ref_c).l,d6
	bne	f_c
	cmp.l	(ref_d).l,d7
	bne	f_d
	cmpa.l	(ref_e).l,a5
	bne	f_e
	cmpa.l	(ref_f).l,a6
	bne	f_f
	; enough interrupts actually landed inside the workloads
	move.w	(cnt_int2).l,d0
	cmp.w	#NIRQ,d0
	bne	f_n

	move.w	#$600D,(DONEREG).l
	stop	#$2700

f_a:	failt	1
f_b:	failt	2
f_c:	failt	3
f_d:	failt	4
f_e:	failt	5
f_f:	failt	6
f_n:	failt	7

;----------------------------------------------- the workloads
; results: d4 (sort), d5 (indexed DBcc), d6 (immediate/descriptor tops),
; d7 (nested), a5 (long body), a6 (Bcc.W loop with a memory top)
workloads:
	; A: fill ARR with an LFSR pattern, keep its sum, bubble sort, check
	lea	(ARR).l,a0
	move.w	#$ACE1,d0
	moveq	#0,d4
	moveq	#63,d1
fill:
	move.w	d0,(a0)+
	add.w	d0,d4
	lsr.w	#1,d0
	bcc.s	fill_nx
	eor.w	#$B400,d0
fill_nx:
	dbra	d1,fill
	move.l	d4,(sum_init).l
	lea	(ARR).l,a0
	moveq	#62,d2
outer:
	lea	(ARR).l,a1
	move.w	d2,d3
inner:
	move.w	(a1),d0
	cmp.w	2(a1),d0
	ble.s	noswap
	move.w	2(a1),d1
	move.w	d0,2(a1)
	move.w	d1,(a1)
noswap:
	addq.l	#2,a1
	dbra	d3,inner
	dbra	d2,outer
	; sorted ascending, sum preserved
	lea	(ARR).l,a0
	moveq	#0,d4
	moveq	#62,d1
chk_sorted:
	move.w	(a0)+,d0
	cmp.w	(a0),d0
	bgt	f_sort		; the sort is signed (ble)
	add.w	d0,d4
	dbra	d1,chk_sorted
	add.w	(a0),d4
	cmp.l	(sum_init).l,d4
	bne	f_sum
	; a weighted checksum of the sorted order
	lea	(ARR).l,a0
	moveq	#0,d4
	moveq	#63,d1
wsum:
	move.w	(a0)+,d0
	mulu	d1,d0
	add.l	d0,d4
	dbra	d1,wsum

	; B: DBcc loops whose counter is the loop top's index register, and
	; whose top's base is written by the instruction that closes the loop
	lea	(ARR).l,a0
	moveq	#0,d5
	moveq	#31,d1
idx_loop:
	move.w	0(a0,d1.w),d0
	add.l	d0,d5
	dbra	d1,idx_loop
	lea	(ARR).l,a1
	movea.l	a1,a0
	moveq	#31,d1
base_loop:
	move.w	(a0),d0
	add.l	d0,d5
	addq.l	#2,a1
	movea.l	a1,a0
	dbra	d1,base_loop
	lea	(ARR).l,a1
	movea.l	a1,a0
	moveq	#8,d1
base_loop2:
	move.l	(a0),d0
	add.l	d0,d5
	addq.l	#4,a1
	subq.l	#1,d1
	movea.l	a1,a0
	bne.s	base_loop2

	; C: an immediate loop top under a Bcc.W, a descriptor loop top
	lea	(CELL).l,a0
	move.l	#0,(a0)
	moveq	#9,d1
imm_loop:
	add.l	#7,(a0)
	subq.l	#1,d1
	bne.w	imm_loop
	move.l	(a0),d6
	moveq	#9,d1
q_loop:
	addq.l	#3,d6
	dbra	d1,q_loop
	moveq	#5,d1
mq_loop:
	move.l	#$1000,d0
	add.l	d0,d6
	dbra	d1,mq_loop

	; D: nested loops whose tops alternate in the one cache entry
	lea	(ARR2).l,a2
	moveq	#15,d1
fill2:
	move.l	d1,(a2)+
	dbra	d1,fill2
	moveq	#0,d7
	moveq	#7,d2
outer2:
	lea	(ARR2).l,a2
	moveq	#15,d1
inner2:
	move.l	(a2)+,d0
	add.l	d0,d7
	dbra	d1,inner2
	eor.l	d2,d7
	dbra	d2,outer2

	; E: a body longer than the refill seed, closed by a Bcc.B
	lea	(ARR).l,a0
	lea	(ARR+128).l,a3
	moveq	#0,d1
	suba.l	a5,a5
long_loop:
	move.l	(a0)+,d0
	adda.l	d0,a5
	eor.l	d1,d0
	rol.l	#3,d0
	add.l	#$1234,d0
	sub.l	d0,d1
	add.l	d1,d1
	nop
	adda.l	d0,a5
	cmpa.l	a3,a0
	bne.s	long_loop

	; F: a Bcc.W-closed loop with a memory-source top and a swap inside
	lea	(ARR).l,a0
	suba.l	a6,a6
	moveq	#30,d1
f_loop:
	move.w	(a0)+,d0
	adda.w	d0,a6
	btst	#0,d0
	beq.s	f_even
	adda.w	#3,a6
f_even:
	subq.l	#1,d1
	bne.w	f_loop

	; G: memory-destination loop tops -- the Sieve shape (an indexed byte
	; clear), a fill with (An)+, a memory-to-memory move, ADDQ to memory,
	; a store of the DBcc counter itself; folded into a6
	lea	(ARR).l,a0
	moveq	#0,d0
	moveq	#2,d1
	moveq	#40,d2
sieve_loop:
	clr.b	0(a0,d0.w)
	add.w	d1,d0
	cmp.w	d2,d0
	ble.s	sieve_loop
	lea	(ARR2).l,a2
	move.l	#$5A5A0001,d0
	moveq	#15,d1
fill3:
	move.l	d0,(a2)+
	addq.l	#3,d0
	dbra	d1,fill3
	lea	(ARR2).l,a2
	lea	(ARR).l,a0
	moveq	#15,d1
m2m:
	move.l	(a2)+,(a0)+
	dbra	d1,m2m
	lea	(ARR).l,a0
	moveq	#7,d1
addq_loop:
	addq.l	#1,(a0)+
	dbra	d1,addq_loop
	lea	(ARR2).l,a2
	moveq	#15,d1
cnt_store:
	move.w	d1,(a2)+
	dbra	d1,cnt_store
	lea	(ARR).l,a0
	moveq	#31,d1
g_sum:
	move.l	(a0)+,d0
	adda.l	d0,a6
	dbra	d1,g_sum
	lea	(ARR2).l,a2
	moveq	#15,d1
g_sum2:
	move.l	(a2)+,d0
	adda.l	d0,a6
	dbra	d1,g_sum2
	rts

f_sort:	failt	8
f_sum:	failt	9

;----------------------------------------------- the interrupt handler
; releases the request and re-arms the next one 5 + (k * 13 mod 37)
; cycles out, for NIRQ interrupts; touches nothing of the workloads
h_int2:
	move.w	#0,(IPLREG).l
	addq.w	#1,(cnt_int2).l
	move.l	d0,-(sp)
	move.w	(irq_on).l,d0
	beq.s	h_done
	move.w	(cnt_int2).l,d0
	cmp.w	#NIRQ,d0
	bge.s	h_done
	mulu	#13,d0
	divu	#37,d0
	swap	d0
	and.l	#$FFFF,d0
	add.w	#5,d0
	move.w	d0,(IPLDLY).l
h_done:
	move.l	(sp)+,d0
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
