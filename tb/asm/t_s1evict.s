; AP68040: second-slot loads while the data cache churns.  The Minimig's
; hard-disk boot failed with only the load class in the second slot; the
; tests touched a handful of lines.  Here every access of a pass lands in
; the same set (addresses 2 KB apart, 8 KB 4-way cache: 128 sets of 16
; bytes), more lines than ways, stores make them dirty (copyback pushes),
; and companion loads hit lines an older access's fill may be replacing.
; assembled with vasmm68k_mot -Fbin -m68040
;   1    store to line k, ADDQ + MOVE.L (A1),D3 companion from line k-1
;        (loaded just before), 8 aliasing lines, 64 rounds: D3 checked
;   2    the same with the companion's line the one most likely evicted
;        (k-4)
;   3    the lines written through first-slot stores then read back by
;        companions only

FAILREG	equ	$F100
DONEREG	equ	$F102
BASE	equ	$10000
STRIDE	equ	$800
NL	equ	8

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	62
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$80008000,d0
	movec	d0,cacr

;------------------------------------------------ 1 / 2
	moveq	#1,d5			; distance back: 1, then 4
.case:	moveq	#63,d4			; rounds
	move.l	#$A0000000,d0		; the value stored
.r:	moveq	#NL-1,d1
	lea	(BASE).l,a3
.k:	move.l	d0,(a3)			; store line k (a miss: allocate, push)
	; the companion reads line k - d5 (mod NL), stored d5 stores ago
	move.l	d5,d2
	mulu.w	#STRIDE,d2
	movea.l	a3,a1
	suba.l	d2,a1
	cmpa.l	#BASE,a1
	bge.s	.in
	adda.l	#NL*STRIDE,a1
.in:	addq.l	#1,d6			; first slot: no memory access
	move.l	(a1),d3			; companion load
	; expected: the value d5 stores ago (or last round's, wrapped)
	move.l	d0,d2
	sub.l	d5,d2
	cmpa.l	a3,a1
	bcs.s	.ok1
	; wrapped: written in the previous round (or not yet: 0 / old)
	cmp.l	#$A0000000+NL,d0
	bcs.s	.skip
.ok1:	cmp.l	d2,d3
	beq.s	.skip
	cmp.l	#1,d5
	bne.s	.f2
	failt	1
.f2:	failt	2
.skip:	addq.l	#1,d0
	adda.l	#STRIDE,a3
	dbra	d1,.k
	dbra	d4,.r
	cmp.l	#4,d5
	beq.s	.c3
	moveq	#4,d5
	bra	.case

;------------------------------------------------ 3 read back by companions
.c3:	moveq	#31,d4
.r3:	lea	(BASE).l,a3
	moveq	#NL-1,d1
	moveq	#0,d0
.w3:	move.l	d4,d2
	swap	d2
	or.l	d0,d2
	move.l	d2,(a3)
	addq.l	#1,d0
	adda.l	#STRIDE,a3
	dbra	d1,.w3
	lea	(BASE).l,a1
	moveq	#NL-1,d1
	moveq	#0,d0
.c:	move.l	d4,d2
	swap	d2
	or.l	d0,d2
	addq.l	#1,d6
	move.l	(a1),d3			; companion load
	cmp.l	d2,d3
	beq.s	.n3
	failt	3
.n3:	addq.l	#1,d0
	adda.l	#STRIDE,a1
	dbra	d1,.c
	dbra	d4,.r3

	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99
