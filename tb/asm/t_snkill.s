; AP68040: a discarded DC2 uop whose slow-path job was frozen by a snoop
; assembled with vasmm68k_mot -Fbin -m68040
;
; Found on the NeXT port (OPENSTEP 4.2 under disk DMA, 2026-10-19): a
; load on the wrong side of a mispredicted branch missed the data cache,
; the DMU's engine took it (E_S_START), the branch's redirect killed it in
; the clock a DMA-write snoop froze the engine, and E_S_START's body, run
; after the freeze, cleared the kill mark the kill had set.  The dead job
; then filled its line and raised e_dc2_done, and the next load into DC2
; was answered at once with the dead job's data: a kernel pointer read as
; 0.
;
; The program runs 4000 iterations of a branch on a pseudo-random bit
; whose two arms each load a long word from a line nobody has cached yet
; (two 4 KB regions streamed one line per iteration, the cache invalidated
; at every wrap): a mispredicted branch (half of them) has one arm's load
; killed in DC2 while it misses.  The bench's alternate master meanwhile
; snoops a transfer every 8 bus clocks, holding the bus a random 0-31 bus
; clocks before each ($F2C4 bits 15, 13, 7:0: without the random hold its
; transfer always starts the same clock after the CPU's last transfer
; freed the bus, and the kill never meets the freeze).  Each arm's load must
; return its own line's pattern (address ^ $C0FFEE00); with the bug it
; returns whatever the dead job of the OTHER arm's line accumulated.  The
; bench counts the coincidences the bug needs ($F2F0: a kill while the
; engine sits in E_S_START under a snoop freeze) and the program requires
; at least one, so a pass means the case was exercised, not missed.  The
; DMU also checks itself: a job completing for a discarded uop without
; its kill mark fails the run.
;   1   an arm's load returned another line's data
;   2   the coincidence never occurred (the test did not cover the case)
;
; All data is copyback (DTT0, supervisor and user): plain loads (a MOVES
; decodes a clock slower, and the load then reaches DC2 only in the clock
; of the kill, the engine never having taken it).  The bench's registers
; are reached with DTT0 switched to noncachable, before and after.

FAILREG	equ	$F100
DONEREG	equ	$F102
TRAFFIC	equ	$F2C4
COINC	equ	$F2F0
ARM1	equ	$C000		; 256 lines, arm 1
ARM2	equ	$E000		; 256 lines, arm 2
PAT	equ	$C0FFEE00	; a long word holds its address ^ PAT
N	equ	4000		; iterations

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
	move.l	#$0000C040,d0	; DTT0: all data noncachable (the bench's registers)
	movec	d0,dtt0
	move.l	#$80008000,d0
	movec	d0,cacr
	clr.l	(COINC).l
	move.w	#$A008,(TRAFFIC).l	; snoop traffic, a transfer every 8 bus clocks,
				; each holding the bus a random while before TS
	move.l	#$0000C020,d0	; DTT0: all data copyback
	movec	d0,dtt0

	; both regions ($C000-$EFFF): every long word = its address ^ PAT
	lea	(ARM1).l,a0
	move.w	#3071,d1
.f1:	move.l	a0,d0
	eor.l	#PAT,d0
	move.l	d0,(a0)
	addq.l	#4,a0
	dbra	d1,.f1
	cpusha	dc		; into memory, the cache empty

	move.l	#$2545F491,d3	; the pseudo-random state
	moveq	#0,d2		; iteration
.loop:
	; d3 = d3 * 1103515245 + 12345 (bit 31: the branch's way)
	mulu.l	#1103515245,d3
	add.l	#12345,d3
	; this iteration's lines: a0 in ARM1, a1 in ARM2
	move.l	d2,d4
	and.w	#255,d4
	lsl.w	#4,d4
	lea	(ARM1).l,a0
	adda.w	d4,a0
	lea	(ARM2).l,a1
	adda.w	d4,a1
	tst.l	d3
	bpl.s	.arm2
	move.l	(a0),d6		; arm 1: the fall-through side
	move.l	a0,d4
	bra.s	.join
.arm2:	move.l	(a1),d6		; arm 2: the taken side
	move.l	a1,d4
.join:	eor.l	#PAT,d4
	cmp.l	d4,d6
	bne	.bad
	; a pseudo-random pause (1-8 passes): with the master's random hold
	; it spreads the snoop's start over the clocks around the kill (no
	; store here: a store runs in the DMU's engine and holds DC1, and
	; the load would reach DC2 a clock late, the kill finding the
	; engine idle)
	move.l	d3,d4
	lsr.l	#8,d4
	and.w	#7,d4
.jit:	dbra	d4,.jit
	addq.l	#1,d2
	move.l	d2,d4
	and.w	#255,d4
	bne.s	.nowrap
	cinva	dc		; every line fresh again
.nowrap:
	cmp.l	#N,d2
	bne	.loop

	bsr	noncach
	clr.w	(TRAFFIC).l		; traffic off, let the last transfer end
	move.w	#200,d1
.w:	nop
	dbra	d1,.w
	tst.l	(COINC).l
	beq.s	.nocov
	move.w	#$600D,(DONEREG).l
	bra.s	*
.nocov:	failt	2
.bad:	failt	1

; the bench's registers again: DTT0 noncachable (the data pushed first)
noncach:
	cpusha	dc
	move.l	#$0000C040,d0
	movec	d0,dtt0
	rts

fail_all:
	bsr	noncach
	clr.w	(TRAFFIC).l
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99
