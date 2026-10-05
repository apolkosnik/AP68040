; AP68040-60 data cache: loads right after stores to the same line
; assembled with vasmm68k_mot -Fbin -m68040
;
; A load that hits keeps its own copy of the set (DC2); stores to that set
; are merged into the copy as they reach the cache, and a load behind an
; older store to its line waits for it.  Every case here is a store (or a
; few) immediately followed by a load from the same line, in copyback
; mode, with the result checked against what the stores wrote:
;   1-6   long/word/byte stores, read back at every size and offset
;   7-8   a misaligned word and long inside the line
;   9-10  another long word of the line, and another way of the same set
;  11-13  read-modify-write after a store; stores back to back, then loads
;  14     the same, in a loop with the offsets changing every pass
;
; User data is copyback (DTT0, user only); MOVES with SFC = DFC = 1 reach
; it from supervisor mode.  Supervisor data (the bench registers) is
; noncachable (DTT1).

FAILREG	equ	$F100
DONEREG	equ	$F102
BUF	equ	$9000		; a line at $9000, another of the same set at $9400

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

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$0000A040,d0	; DTT1: supervisor data noncachable
	movec	d0,dtt1
	move.l	#$00008020,d0	; DTT0: user data copyback
	movec	d0,dtt0
	moveq	#1,d0
	movec	d0,sfc
	movec	d0,dfc
	move.l	#$80008000,d0
	movec	d0,cacr
	lea	(BUF).l,a0
	lea	(BUF+$400).l,a1
	; warm both lines (allocate) with known contents
	move.l	#$A0A1A2A3,d0
	moves.l	d0,(a0)
	move.l	#$A4A5A6A7,d0
	moves.l	d0,4(a0)
	move.l	#$A8A9AAAB,d0
	moves.l	d0,8(a0)
	move.l	#$ACADAEAF,d0
	moves.l	d0,12(a0)
	move.l	#$B0B1B2B3,d0
	moves.l	d0,(a1)
	moves.l	(a0),d1
	chkl	d1,$A0A1A2A3,90
	moves.l	(a1),d1
	chkl	d1,$B0B1B2B3,91

;--------------------------------------------------- 1-6 sizes and offsets
	move.l	#$11223344,d0
	moves.l	d0,(a0)
	moves.l	(a0),d1
	chkl	d1,$11223344,1
	move.l	#$5566,d0
	moves.w	d0,2(a0)
	moveq	#0,d1
	moves.w	2(a0),d1
	chkl	d1,$5566,2
	moves.l	(a0),d1
	chkl	d1,$11225566,3
	move.l	#$77,d0
	moves.b	d0,1(a0)
	moveq	#0,d1
	moves.b	1(a0),d1
	chkl	d1,$77,4
	moves.l	(a0),d1
	chkl	d1,$11775566,5
	moveq	#0,d1
	moves.w	(a0),d1
	chkl	d1,$1177,6

;-------------------------------------------- 7-8 misaligned in the line
	move.l	#$CAFE,d0
	moves.w	d0,5(a0)		; bytes 5,6
	moves.l	4(a0),d1
	chkl	d1,$A4CAFEA7,7
	move.l	#$DEADBEEF,d0
	moves.l	d0,9(a0)		; bytes 9..12
	moves.l	8(a0),d1
	chkl	d1,$A8DEADBE,8
	moves.l	12(a0),d1
	chkl	d1,$EFADAEAF,80

;------------------------- 9-10 another long word, another way of the set
	move.l	#$12345678,d0
	moves.l	d0,(a0)
	moves.l	4(a0),d1		; not the stored long word
	chkl	d1,$A4CAFEA7,9
	move.l	#$9ABCDEF0,d0
	moves.l	d0,(a0)
	moves.l	(a1),d1			; same set, the other line
	chkl	d1,$B0B1B2B3,10
	moves.l	(a0),d1
	chkl	d1,$9ABCDEF0,81

;-------------------------- 11-13 read-modify-write, stores back to back
	move.l	#100,d0
	moves.l	d0,(a0)
	moves.l	(a0),d1
	addq.l	#1,d1
	moves.l	d1,(a0)
	moves.l	(a0),d2
	chkl	d2,101,11
	; ADDQ to memory needs a supervisor-space operand; use user space
	; through MOVES only: read-add-write-read chains instead
	move.l	#1,d0
	move.l	#2,d3
	move.l	#3,d4
	moves.l	d0,(a0)
	moves.l	d3,4(a0)
	moves.l	d4,8(a0)
	moves.l	(a0),d1
	moves.l	4(a0),d2
	moves.l	8(a0),d5
	add.l	d2,d1
	add.l	d5,d1
	chkl	d1,6,12
	move.l	#$01020304,d0
	moves.b	d0,(a0)
	moves.b	d0,1(a0)
	moves.b	d0,2(a0)
	moves.b	d0,3(a0)
	moves.l	(a0),d1
	chkl	d1,$04040404,13

;--------------------------- 14: a loop, offsets moving every pass
; store the pass number to BUF+4*(n&3), read back all four long words
; and keep a running sum; the expected sum is computed in registers
	moveq	#0,d6			; expected
	moveq	#0,d5			; got
	clr.l	d0
	moves.l	d0,(a0)
	moves.l	d0,4(a0)
	moves.l	d0,8(a0)
	moves.l	d0,12(a0)
	moveq	#0,d3			; shadow copies of the four words
	moveq	#0,d4
	move.l	d4,a2
	move.l	d4,a3
	move.w	#199,d2
	moveq	#0,d0
.l14:	addq.l	#1,d0
	move.l	d0,d1
	and.w	#3,d1
	lsl.w	#2,d1
	moves.l	d0,(a0,d1.w)		; store, then read all four back
	cmp.w	#0,d1
	bne.s	.s1
	move.l	d0,d3
.s1:	cmp.w	#4,d1
	bne.s	.s2
	move.l	d0,d4
.s2:	cmp.w	#8,d1
	bne.s	.s3
	move.l	d0,a2
.s3:	cmp.w	#12,d1
	bne.s	.s4
	move.l	d0,a3
.s4:	moves.l	(a0),d1
	add.l	d1,d5
	moves.l	4(a0),d1
	add.l	d1,d5
	moves.l	8(a0),d1
	add.l	d1,d5
	moves.l	12(a0),d1
	add.l	d1,d5
	add.l	d3,d6
	add.l	d4,d6
	add.l	a2,d6
	add.l	a3,d6
	dbra	d2,.l14
	cmp.l	d6,d5
	beq.s	.ok14
	failt	14
.ok14:

;----------------------------------------------------------------- done
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99
