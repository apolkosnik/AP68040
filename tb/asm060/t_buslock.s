; AP68060 locked sequences through BUSCR (MC68060UM 7.4, 7.11.2, C.2.2.3)
; assembled with vasmm68k_mot -Fbin -m68040
;
; BUSCR.L asserts LOCK from the next operand access on, LE marks the next
; operand write as the last; LOCK and LOCKE negate after that write.  The
; instructions between the accesses may have to be fetched from the bus:
; those fetches carry LOCK too, and nothing but the LOCKE write, an
; exception, or a MOVEC clearing L ends the sequence.  The bench fails the
; run if LOCK negates inside a sequence any other way, and counts the
; transfers in its watch window ($F2D0/$F2D4): $F2F4 reads without LOCK,
; $F2F8 reads with LOCK, $F2FC writes with LOCK and LOCKE.
;   1-3  caches off: a locked read, NOPs, a taken branch, a DBRA loop and a
;        jump out and back (no stack write: it would be the sequence's last
;        operand write) fetched from the bus inside the sequence, then the
;        LOCKE write: it completes, memory has the write, and the code was
;        fetched with LOCK
;   4    the LOCKE write itself: one write with LOCK and LOCKE
;   5-6  the same with the instruction cache on but cold: the line fills
;        inside the sequence carry LOCK
;   7-8  L alone, then BUSCR cleared without a LOCKE write: the sequence is
;        abandoned, the next read has no LOCK
;   9-11 an exception (TRAP #0) inside a sequence abandons it: L and LE go
;        to SL and SLE, the handler's read and the reads after it have no
;        LOCK
;
;   98   an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
WSTART	equ	$F2D0
WEND	equ	$F2D4
RD_NL	equ	$F2F4
RD_L	equ	$F2F8
WR_LE	equ	$F2FC

buscr_h	equ	$3600		; BUSCR as the TRAP handler read it
rd_h	equ	$3604		; $F2F4 after the handler's read

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro			; register, value, test
	cmp.l	#\2,\1
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

chkm	macro			; memory long, value, test
	cmp.l	#\2,(\1).l
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

window	macro			; start, end: the counters cleared
	move.l	#\1,(WSTART).l
	move.l	#\2,(WEND).l
	clr.l	(RD_NL).l
	clr.l	(RD_L).l
	clr.l	(WR_LE).l
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	30
	dc.l	unexp
	endr
	dc.l	h_trap0		; 32 TRAP #0
	rept	223
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#$2700,sr
	dc.w	$F4D8			; cinva bc
	moveq	#0,d0
	movec	d0,cacr

;------------------------------------------------ 1-4 caches off
	move.l	#$11112222,($9000).l
	window	lk_code,lk_end
	bsr	locked_seq
	moveq	#0,d0
	dc.w	$4E7B,$0008		; movec d0,buscr
	chkl	d1,$11112222,1		; the locked read
	chkm	$9000,$11112223,2	; the LOCKE write reached memory
	tst.l	(RD_L).l		; code inside the sequence fetched
	bne.s	t3ok			; with LOCK
	failt	3
t3ok:
	window	$9000,$9004
	move.l	#$20203030,($9000).l
	bsr	locked_seq
	moveq	#0,d0
	dc.w	$4E7B,$0008
	chkm	WR_LE,1,4		; one LOCKE write
	chkm	$9000,$20203031,1

;------------------------------------------------ 5-6 instruction cache cold
	move.l	#$44445555,($9000).l
	dc.w	$F4D8			; cinva bc
	move.l	#$00008000,d0		; CACR EIC
	movec	d0,cacr
	window	lk_code,lk_end
	bsr	locked_seq
	moveq	#0,d0
	dc.w	$4E7B,$0008
	movec	d0,cacr
	chkm	$9000,$44445556,5
	tst.l	(RD_L).l		; the line fills carried LOCK
	bne.s	t6ok
	failt	6
t6ok:

;------------------------------------------------ 7-8 abandoned by MOVEC
	window	$9000,$9004
	move.l	#$80000000,d0		; L alone
	dc.w	$4E7B,$0008
	move.l	($9000).l,d1		; locked read
	nop
	nop
	moveq	#0,d0
	dc.w	$4E7B,$0008		; L cleared: LOCK negates
	move.l	($9000).l,d1		; an ordinary read
	chkm	RD_L,1,7		; the locked one alone had LOCK
	chkm	RD_NL,1,8

;------------------------------------------------ 9-11 abandoned by an exception
	window	$9000,$9004
	clr.l	(buscr_h).l
	move.l	#$A0000000,d0		; L and LE
	dc.w	$4E7B,$0008
	move.l	($9000).l,d1		; locked read
	trap	#0			; the handler reads $9000 too
	move.l	($9000).l,d1		; an ordinary read
	dc.w	$4E7A,$0008		; movec buscr,d0
	chkl	d0,$50000000,9		; L, LE clear; SL, SLE set
	chkm	rd_h,1,10		; the handler's read had no LOCK
	chkm	RD_L,1,11		; nor the one after it
	moveq	#0,d0
	dc.w	$4E7B,$0008

	move.w	#$600D,(DONEREG).l
	bra.s	*

; L and LE set, a locked read of $9000 into d1, instructions fetched in
; the sequence, then the LOCKE write of d1 + 1
	cnop	0,16
locked_seq:
	move.l	#$A0000000,d0
	dc.w	$4E7B,$0008		; movec d0,buscr
	move.l	($9000).l,d1		; LOCK from here
lk_code:
	nop
	nop
	moveq	#1,d2
	bra.s	lk_skip
	failt	97			; (skipped)
lk_skip:
	moveq	#2,d3
lk_loop:
	add.l	d2,d4
	dbra	d3,lk_loop
	lea	lk_back(pc),a0
	bra.s	lk_sub
lk_back:
	nop
	rept	24
	moveq	#3,d5
	endr
	move.l	d1,d6
	addq.l	#1,d6
	move.l	d6,($9000).l		; the LOCKE write
lk_end:
	rts
lk_sub:
	rept	8
	moveq	#4,d5
	endr
	jmp	(a0)

h_trap0:
	dc.w	$4E7A,$0008		; movec buscr,d0
	move.l	d0,(buscr_h).l
	move.l	($9000).l,d0		; no LOCK
	move.l	(RD_NL).l,(rd_h).l
	rte

fail_all:
	move.w	#$2700,sr
	moveq	#0,d0
	dc.w	$4E7B,$0008
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	98
