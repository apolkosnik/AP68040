; AP68040: the data cache with CACR.DE clear.  Clearing DE stops
; allocation only; the cache is still searched and a hit is served and
; written.  The MC68040UM 4.3.2 notes a locked access that "hit and the
; data cache was disabled"; cputest (WinUAE's, on real 68040/68060 boards)
; pushes a return address through copyback memory, clears DE with MOVEC in
; the callee and returns with RTS: on Maximig the RTS popped memory's stale
; zero and ran from address 0 (the cputest hang of 2026-10-26).  CDIS
; (t_cache2 19, 26) bypasses the cache and leaves it alone.
; assembled with vasmm68k_mot -Fbin -m68040
;
; Supervisor data is noncachable through DTT1 except where DTT0 (which
; wins) maps $00xxxxxx copyback or write-through.  The bench's watch window
; ($F2D0, $F2D4) counts bus reads ($F2D8) and writes ($F2DC) in a range.
;   1-2   a dirty copyback line, DE cleared: a load hits it (its data, no
;         bus read)
;   3-5   DE clear, a copyback store that hits: the line is written, no bus
;         write; DE set, a load reads it; CPUSHA writes it to memory
;   6-8   DE clear, a store that misses: memory is written, nothing is
;         allocated (DE set, a load fills the line: four beats)
;   9-11  DE clear, a write-through store that hits: the line is written and
;         memory too; DE set, a load hits
;   12    cputest's sequence: JSR with DE set pushes the return address into
;         a copyback line; the callee clears DE; RTS pops it from the cache
;   13    the mirror: with DE clear the JSR writes the valid stack line (no
;         memory write); the callee sets DE; RTS pops it
;   98    an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
WLO	equ	$F2D0
WHI	equ	$F2D4
WRD	equ	$F2D8
WWR	equ	$F2DC

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

watch	macro			; start, end: clear the counters
	move.l	#\1,(WLO).l
	move.l	#\2,(WHI).l
	clr.l	(WRD).l
	clr.l	(WWR).l
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	cinva	bc
	move.l	#$0000A040,d0	; DTT1: supervisor $00xxxxxx noncachable
	movec	d0,dtt1
	moveq	#0,d0
	movec	d0,dtt0		; (DTT0 off: memory is written directly)
	movec	d0,cacr

;------------------------------------------------ 1-2 a dirty line, DE clear
	move.l	#$11112222,($A600).l	; memory
	move.l	#$0000A020,d0	; DTT0: supervisor $00xxxxxx copyback
	movec	d0,dtt0
	move.l	#$80000000,d0	; DE
	movec	d0,cacr
	move.l	#$AAAABBBB,($A600).l	; dirty in the cache
	moveq	#0,d0
	movec	d0,cacr		; DE clear
	watch	$A600,$A610
	move.l	($A600).l,d1
	chkl	d1,$AAAABBBB,1	; the hit's data
	move.l	(WRD).l,d2
	chkl	d2,0,2		; no bus read

;------------------------------------------------ 3-5 DE clear, a copyback store hit
	watch	$A600,$A610
	move.l	#$CCCCDDDD,($A600).l	; hits the line
	move.l	(WWR).l,d2
	chkl	d2,0,3		; no bus write
	move.l	#$80000000,d0
	movec	d0,cacr
	move.l	($A600).l,d1
	chkl	d1,$CCCCDDDD,4
	cpusha	dc
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,dtt0		; noncachable: memory
	move.l	($A600).l,d1
	chkl	d1,$CCCCDDDD,5	; the push wrote the line

;------------------------------------------------ 6-8 DE clear, a store miss
	cinva	dc
	move.l	#$11112222,($A700).l	; memory
	move.l	#$0000A020,d0
	movec	d0,dtt0
	watch	$A700,$A710
	move.l	#$EEEEFFFF,($A700).l	; DE clear: memory, no allocation
	move.l	(WWR).l,d2
	chkl	d2,1,6
	move.l	#$80000000,d0
	movec	d0,cacr
	move.l	($A700).l,d1
	chkl	d1,$EEEEFFFF,7
	move.l	(WRD).l,d2
	chkl	d2,4,8		; a miss: the line filled now
	cpusha	dc
	moveq	#0,d0
	movec	d0,cacr

;------------------------------------------------ 9-11 DE clear, a write-through store hit
	cinva	dc
	move.l	#$0000A000,d0	; DTT0: supervisor $00xxxxxx write-through
	movec	d0,dtt0
	move.l	#$12121212,($A800).l	; (a write-through miss: memory)
	move.l	#$80000000,d0
	movec	d0,cacr
	move.l	($A800).l,d1	; allocate
	moveq	#0,d0
	movec	d0,cacr		; DE clear
	watch	$A800,$A810
	move.l	#$34343434,($A800).l	; the line and memory
	move.l	(WWR).l,d2
	chkl	d2,1,9
	move.l	#$80000000,d0
	movec	d0,cacr
	move.l	($A800).l,d1
	chkl	d1,$34343434,10
	move.l	(WRD).l,d2
	chkl	d2,0,11		; a hit
	moveq	#0,d0
	movec	d0,cacr

;------------------------------------------------ 12 cputest: JSR, DE cleared, RTS
	cinva	dc
	move.l	#$0000A020,d0	; the stack ($3400 down) copyback
	movec	d0,dtt0
	move.l	#$80000000,d0
	movec	d0,cacr
	moveq	#0,d6
	jsr	(sub_clear).l	; the push: a dirty stack line
	addq.l	#1,d6		; the return lands here
	cmpi.l	#$10001,d6
	beq.s	ok12
	failt	12
ok12:
;------------------------------------------------ 13 the mirror: DE clear at the JSR, set in the callee
	moveq	#0,d0
	movec	d0,cacr
	moveq	#0,d6
	jsr	(sub_set).l	; the push writes the valid line
	addq.l	#1,d6
	cmpi.l	#$20001,d6
	beq.s	ok13
	failt	13
ok13:
	cpusha	dc
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,dtt0
	move.w	#$600d,(DONEREG).l
.d:	bra.s	.d

sub_clear:
	add.l	#$10000,d6
	moveq	#0,d0
	movec	d0,cacr		; DE clear: the RTS must still hit the stack
	rts

sub_set:
	add.l	#$20000,d6
	move.l	#$80000000,d0
	movec	d0,cacr		; DE set: the RTS pops the line the JSR wrote
	rts

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$bad0,(DONEREG).l
.f:	bra.s	.f

unexp:
	move.w	#98,(FAILREG).l
	move.w	#$bad0,(DONEREG).l
.u:	bra.s	.u
