; AP68040: programmed I/O reads into cached memory, as the Gayle IDE driver
; copies a sector -- the Minimig's hard-disk boot loaded garbage (every
; command "not enough memory": LoadSeg read nonsense hunk sizes) with dual
; issue on, the floppy (Paula DMA) did not.
; assembled with vasmm68k_mot -Fbin -m68040
;
; $F178 is a data port that counts its reads (the bench), cache-inhibited
; by the bench's map; the buffer at $8000 is cacheable.  Every loop reads
; 128 values, then checks the buffer holds 0..127 (from its start value):
;   1    MOVE.W (A0),D1 / MOVE.W D1,(A1)+ / DBRA
;   2    MOVE.L (A0),D1 / MOVE.L D1,(A1)+ / DBRA
;   3    MOVE.L (A0),(A1)+ x4 / DBRA
;   4    MOVE.W (A3),D1 (the port's low word) with an ADDQ and an EXT
;        between (pairs around the port reads), stored as long words
;   5    MOVE.L (A0),D1 / ADD.L D1,D2 (a checksum) / MOVE.L D1,(A1)+

FAILREG	equ	$F100
DONEREG	equ	$F102
PORT	equ	$F178
BUF	equ	$8000

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
	lea	(PORT).l,a0

;------------------------------------------------ 1 word, through D1
	moveq	#9,d5
.p1:	clr.l	(a0)
	lea	(BUF).l,a1
	move.w	#127,d4
.l1:	move.w	(a0),d1
	move.w	d1,(a1)+
	dbra	d4,.l1
	moveq	#2,d6			; (words: the port's high word, 0)
	bsr	chkw
	tst.l	d7
	beq.s	.k1
	failt	1
.k1:	dbra	d5,.p1

;------------------------------------------------ 2 long, through D1
	moveq	#9,d5
.p2:	clr.l	(a0)
	lea	(BUF).l,a1
	move.w	#127,d4
.l2:	move.l	(a0),d1
	move.l	d1,(a1)+
	dbra	d4,.l2
	bsr	chkl
	tst.l	d7
	beq.s	.k2
	failt	2
.k2:	dbra	d5,.p2

;------------------------------------------------ 3 long, memory to memory, unrolled
	moveq	#9,d5
.p3:	clr.l	(a0)
	lea	(BUF).l,a1
	moveq	#31,d4
.l3:	move.l	(a0),(a1)+
	move.l	(a0),(a1)+
	move.l	(a0),(a1)+
	move.l	(a0),(a1)+
	dbra	d4,.l3
	bsr	chkl
	tst.l	d7
	beq.s	.k3
	failt	3
.k3:	dbra	d5,.p3

;------------------------------------------------ 4 word, pairs around the reads
	moveq	#9,d5
.p4:	clr.l	(a0)
	lea	(BUF).l,a1
	lea	2(a0),a3		; the port's low word
	move.w	#127,d4
	moveq	#0,d3
.l4:	move.w	(a3),d1
	addq.l	#1,d3
	ext.l	d1
	move.l	d1,(a1)+
	dbra	d4,.l4
	bsr	chkl0
	tst.l	d7
	beq.s	.k4
	failt	4
.k4:	dbra	d5,.p4

;------------------------------------------------ 5 long with a checksum
	moveq	#9,d5
.p5:	clr.l	(a0)
	lea	(BUF).l,a1
	move.w	#127,d4
	moveq	#0,d2
.l5:	move.l	(a0),d1
	add.l	d1,d2
	move.l	d1,(a1)+
	dbra	d4,.l5
	cmp.l	#127*128/2,d2
	beq.s	.c5
	failt	6
.c5:	bsr	chkl
	tst.l	d7
	beq.s	.k5
	failt	5
.k5:	dbra	d5,.p5

	move.w	#$600D,(DONEREG).l
	bra.s	*

; the buffer holds 128 long words 0..127: D7 = 0
chkl:	lea	(BUF).l,a2
	moveq	#0,d1
	moveq	#0,d7
.c:	cmp.l	(a2)+,d1
	beq.s	.n
	moveq	#1,d7
.n:	addq.l	#1,d1
	cmp.l	#128,d1
	bne.s	.c
	rts
; long words 0..127 sign-extended from words: the same check
chkl0:	bra	chkl
; 128 words: the port's high word of 0..127 is 0
chkw:	lea	(BUF).l,a2
	moveq	#0,d7
	move.w	#127,d1
.c:	tst.w	(a2)+
	beq.s	.n
	moveq	#1,d7
.n:	dbra	d1,.c
	rts

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99
