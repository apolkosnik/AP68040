; AP68040-60: bus snooping under random traffic
; assembled with vasmm68k_mot -Fbin -m68040
;
; While the program runs, the bench's alternate master ($F2C4) issues random
; snooped transfers (tb_ap68040.sv): SC 01 reads of the program's counters
; at $A000 (word and line; the bench fails if one ever goes backwards), SC
; 01 and SC 10 writes of the shared words at $B000 (word and line), and SC
; 01 reads of them (the bench fails on a value nobody wrote).  The program
; meanwhile:
;   - increments its 64 counters at $A000 (copyback, read-modify-write),
;   - reads shared words and checks each names its writer and address,
;   - writes shared words itself (so the other master's writes are sunk
;     into dirty lines, and its reads supplied from them),
;   - streams through $C000-$CFFF, whose lines share the sets of $A000 and
;     $B000, so dirty lines are evicted and pushed while it all runs,
;   - pushes the data cache now and then.
; At the end every counter must hold exactly the number of increments.
;
; User data is copyback (DTT0, user only), reached through MOVES with
; SFC = DFC = 1; supervisor data is noncachable (DTT1).
;   1   a shared word that nobody wrote
;   2   a counter that lost an increment

FAILREG	equ	$F100
DONEREG	equ	$F102
TRAFFIC	equ	$F2C4
CNT	equ	$A000		; 64 counters
SHR	equ	$B000		; 64 shared words
STRM	equ	$C000		; streamed, same sets
N	equ	3000		; iterations

failt	macro
	move.w	#\1,d7
	bra	fail_all
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
	lea	(CNT).l,a0		; clear counters and shared words
	move.w	#63,d1
.c1:	clr.l	(a0)+
	dbra	d1,.c1
	lea	(SHR).l,a0
	move.w	#63,d1
.c2:	clr.l	(a0)+
	dbra	d1,.c2
	move.l	#$80008000,d0
	movec	d0,cacr
	move.w	#$8008,(TRAFFIC).l	; traffic on, a transfer every 8 bus clocks

	lea	(CNT).l,a1
	lea	(SHR).l,a2
	lea	(STRM).l,a3
	moveq	#0,d2			; iteration
.loop:
	; counter d2 & 63 += 1
	move.l	d2,d3
	and.w	#63,d3
	lsl.w	#2,d3
	moves.l	(a1,d3.w),d0
	addq.l	#1,d0
	moves.l	d0,(a1,d3.w)
	; read a shared word and check it: low word = (4j) ^ $5A5A or ^ $A5A5
	move.l	d2,d3
	mulu.w	#5,d3
	and.w	#63,d3
	lsl.w	#2,d3
	moves.l	(a2,d3.w),d0
	tst.l	d0
	beq.s	.okr
	move.w	d3,d4
	eor.w	#$5A5A,d4
	cmp.w	d4,d0
	beq.s	.okr
	move.w	d3,d4
	eor.w	#$A5A5,d4
	cmp.w	d4,d0
	beq.s	.okr
	move.w	#$2700,sr
	failt	1
.okr:
	; every 4th iteration write a shared word: (iteration << 16) | (4k ^ $A5A5)
	move.l	d2,d3
	and.w	#3,d3
	bne.s	.now
	move.l	d2,d3
	mulu.w	#3,d3
	and.w	#63,d3
	lsl.w	#2,d3
	move.l	d2,d0
	swap	d0
	move.w	d3,d0
	eor.w	#$A5A5,d0
	moves.l	d0,(a2,d3.w)
.now:
	; stream: one line of $C000-$CFFF per iteration
	move.l	d2,d3
	and.w	#$FF,d3
	lsl.w	#4,d3
	moves.l	(a3,d3.w),d0
	; push everything now and then
	move.l	d2,d3
	and.w	#$FF,d3
	bne.s	.nopush
	cpusha	dc
.nopush:
	addq.l	#1,d2
	cmp.l	#N,d2
	bne	.loop

	clr.w	(TRAFFIC).l		; traffic off, let the last transfer end
	move.w	#200,d1
.w:	nop
	dbra	d1,.w

	; every counter: N/64 increments, one more below N mod 64
	moveq	#0,d3
.chk:	move.l	#N/64,d1
	cmp.w	#N&63,d3
	bcc.s	.c3
	addq.l	#1,d1
.c3:	move.w	d3,d4
	lsl.w	#2,d4
	moves.l	(a1,d4.w),d0
	cmp.l	d1,d0
	beq.s	.c4
	failt	2
.c4:	addq.w	#1,d3
	cmp.w	#64,d3
	bne.s	.chk

	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	clr.w	(TRAFFIC).l
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99
