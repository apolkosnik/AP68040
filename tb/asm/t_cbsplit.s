; AP68040-60: misaligned and line-crossing accesses are served by the data
; cache, in copyback and in write-through mode
; assembled with vasmm68k_mot -Fbin -m68040
;
; Four lines at BUF hold bytes $40..$7F in the cache while memory holds
; $EE (copyback: the lines are dirty).  The bench counts the CPU's bus
; transfers in BUF..BUF+64 ($F2D0..$F2DC).  Every read below must hit:
;   1    long reads at every offset 0..60 (within a long word, across long
;        words, across lines), no bus transfer in the window
;   2    word reads at every offset 0..62, the same
;   3    byte reads at every offset, the same
;   4-7  a long and a word store across a line, read back across it,
;        still no bus transfer (copyback: no write either, so memory still
;        holds $EE -- a noncachable read could not show it: it pushes a
;        dirty line it hits first, MC68040UM 4.3.2)
;   9    CPUSHA pushes the four dirty lines (16 long-word beats)
;   10   memory then holds what the program wrote, the misaligned stores
;        included
;   11-13 write-through: after a read pass allocates the lines, long, word
;        and byte reads at every offset hit (no bus read in the window)
;   18-19 a store into a line, then at once a load across into it from the
;        line before (and a store into the first line, then a load across
;        out of it), for every store offset in the second line's first long
;        word and every load size: the load must see the store
;   20-27 the same races in user mode with plain MOVE (MOVES serializes,
;        so above the store has completed before the load starts): a store
;        then, after 0..3 other instructions, a load across a line, for
;        stores into either line, of every size, and across the line
;        itself; failures and the end come back through TRAP #0 / #1
;   28-29 a snoop between the two halves of a store across a line: the
;        second line pushed (CPUSHL), a long stored across $D00E (its
;        second line allocated, then written at WB the cycle after the
;        first), while the alternate master writes $D014 (SC 01: a clean
;        line is invalidated) 0..7 bus clocks apart, 250 stores each;
;        every value reads back, and with BCLK at half PCLK the bench saw
;        the DMU's fallback (the second part by the engine)
;   30   CLR (long, word, byte), ST and MOVE from SR to noncachable memory
;        write without reading it (68020 and later): five writes, no read
;   14-17 the MMU on: two logical pages mapped copyback by their page
;        descriptors to $B000 and $C000; 32 bytes around the page boundary
;        written, then read as longs and words at every offset (across the
;        page boundary: two translations), a long stored across it; no bus
;        transfer in the window until CPUSHA, which pushes the two lines
;
; User data goes through MOVES (SFC = DFC = 1) under DTT0; supervisor data
; (the bench registers, and memory checks) is noncachable (DTT1).

FAILREG	equ	$F100
DONEREG	equ	$F102
WLO	equ	$F2D0
WHI	equ	$F2D4
WRD	equ	$F2D8
WWR	equ	$F2DC
BUF	equ	$9800
TRAFFIC	equ	$F2C4
FBCNT	equ	$F2E0
CAPW	equ	$F160
ROOT	equ	$10000		; root table (128 x 4)
PTR	equ	$10200		; pointer table (128 x 4)
PAGE	equ	$10400		; page table (64 x 4, 4 KB pages)
LA	equ	$01050FF0	; root 0, pointer $41, pages $10 and $11
PB	equ	$BFF0		; its physical address (page $10 -> $B000,
				; page $11 -> $C000)

; ucase <fillers>,<store op>,<value>,<store offset>,<load op>,<load
; offset>,<expected>,<test>: bytes 12..19 set to $4C..$53, the store, the
; fillers, the load (user mode)
ucase	macro
	move.l	#$4C4D4E4F,12(a0)
	move.l	#$50515253,16(a0)
	move.l	#\3,d3
	\2	d3,\4(a0)
	rept	\1
	moveq	#0,d6
	endr
	moveq	#0,d1
	\5	\6(a0),d1
	cmp.l	#\7,d1
	beq.s	.ok\@
	move.w	#\8,d7
	trap	#0
.ok\@:
	endm

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
	rept	30
	dc.l	unexp
	endr
	dc.l	h_trap0			; 32: TRAP #0, a user-mode failure (d7)
	dc.l	h_trap1			; 33: TRAP #1, back to supervisor
	rept	222
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$0000A040,d0		; DTT1: supervisor data noncachable
	movec	d0,dtt1
	move.l	#$00008020,d0		; DTT0: user data copyback
	movec	d0,dtt0
	moveq	#1,d0
	movec	d0,sfc
	movec	d0,dfc
	move.l	#$80008000,d0
	movec	d0,cacr
	lea	(BUF).l,a0

	; memory: $EE (supervisor, noncachable)
	moveq	#15,d1
	move.l	a0,a1
.m0:	move.l	#$EEEEEEEE,(a1)+
	dbra	d1,.m0

	; the cache: bytes $40..$7F (user copyback stores allocate the lines)
	moveq	#0,d0
.m1:	bsr	exp_l
	moves.l	d2,(a0,d0.w)
	addq.w	#4,d0
	cmp.w	#64,d0
	bne.s	.m1

	bsr	arm

;------------------------------------------- 1-3 reads at every offset
	moveq	#0,d0
.r1:	moves.l	(a0,d0.w),d1
	bsr	exp_l
	cmp.l	d2,d1
	beq.s	.r1ok
	failt	1
.r1ok:	addq.w	#1,d0
	cmp.w	#61,d0
	bne.s	.r1
	bsr	quiet1

	moveq	#0,d0
.r2:	moveq	#0,d1
	moves.w	(a0,d0.w),d1
	bsr	exp_w
	cmp.l	d2,d1
	beq.s	.r2ok
	failt	2
.r2ok:	addq.w	#1,d0
	cmp.w	#63,d0
	bne.s	.r2
	bsr	quiet2

	moveq	#0,d0
.r3:	moveq	#0,d1
	moves.b	(a0,d0.w),d1
	move.l	d0,d2
	add.l	#$40,d2
	cmp.l	d2,d1
	beq.s	.r3ok
	failt	3
.r3ok:	addq.w	#1,d0
	cmp.w	#64,d0
	bne.s	.r3
	bsr	quiet3

;------------------------------------- 4-7 stores across a line
	move.l	#$A1B2C3D4,d1
	moves.l	d1,14(a0)		; bytes 14..17: lines 0 and 1
	moves.l	12(a0),d1
	chkl	d1,$4C4DA1B2,4
	moves.l	16(a0),d1
	chkl	d1,$C3D45253,4
	moveq	#0,d1
	moves.w	15(a0),d1
	chkl	d1,$B2C3,5
	move.l	#$E5F6,d1
	moves.w	d1,31(a0)		; bytes 31, 32: lines 1 and 2
	moves.l	30(a0),d1
	chkl	d1,$5EE5F661,6
	moves.l	29(a0),d1
	chkl	d1,$5D5EE5F6,6
	move.l	(WRD).l,d1
	chkl	d1,0,7
	move.l	(WWR).l,d1
	chkl	d1,0,7

;------------------------------------ 9-10 memory after the push
	bsr	arm
	cpusha	dc
	nop
	move.l	(WWR).l,d1
	chkl	d1,16,9
	move.l	(WRD).l,d1
	chkl	d1,0,9
	clr.l	(WLO).l			; disarm (the checks read the window)
	clr.l	(WHI).l
	move.l	12(a0),d1		; supervisor: noncachable, memory
	chkl	d1,$4C4DA1B2,10
	move.l	16(a0),d1
	chkl	d1,$C3D45253,10
	move.l	28(a0),d1
	chkl	d1,$5C5D5EE5,10
	move.l	32(a0),d1
	chkl	d1,$F6616263,10
	move.l	60(a0),d1
	chkl	d1,$7C7D7E7F,10

;----------------------------------------- 11-13 write-through
	cinva	dc
	move.l	#$00008000,d0		; DTT0: user data write-through
	movec	d0,dtt0
	moveq	#0,d0			; memory: the pattern again
	move.l	a0,a1
.w0:	bsr	exp_l
	move.l	d2,(a1)+
	addq.w	#4,d0
	cmp.w	#64,d0
	bne.s	.w0
	moveq	#0,d0			; a read pass allocates the lines
.w1:	moves.l	(a0,d0.w),d1
	add.w	#16,d0
	cmp.w	#64,d0
	bne.s	.w1
	bsr	arm
	moveq	#0,d0
.w2:	moves.l	(a0,d0.w),d1
	bsr	exp_l
	cmp.l	d2,d1
	beq.s	.w2ok
	failt	11
.w2ok:	addq.w	#1,d0
	cmp.w	#61,d0
	bne.s	.w2
	move.l	(WRD).l,d1
	chkl	d1,0,11
	moveq	#0,d0
.w3:	moveq	#0,d1
	moves.w	(a0,d0.w),d1
	bsr	exp_w
	cmp.l	d2,d1
	beq.s	.w3ok
	failt	12
.w3ok:	addq.w	#1,d0
	cmp.w	#63,d0
	bne.s	.w3
	move.l	(WRD).l,d1
	chkl	d1,0,12
	moveq	#0,d0
.w4:	moveq	#0,d1
	moves.b	(a0,d0.w),d1
	move.l	d0,d2
	add.l	#$40,d2
	cmp.l	d2,d1
	beq.s	.w4ok
	failt	13
.w4ok:	addq.w	#1,d0
	cmp.w	#64,d0
	bne.s	.w4
	move.l	(WRD).l,d1
	chkl	d1,0,13
	move.l	(WWR).l,d1
	chkl	d1,0,13

;------------------------------- 18-19 a store, then a load across it
; copyback again; d4 counts passes, d5 the stored value
	cinva	dc
	move.l	#$00008020,d0		; DTT0: user data copyback
	movec	d0,dtt0
	moveq	#0,d0
.h0:	bsr	exp_l
	moves.l	d2,(a0,d0.w)
	addq.w	#4,d0
	cmp.w	#64,d0
	bne.s	.h0
	move.l	#$11223344,d5
	moveq	#3,d4
.h1:	; long at 16 (line 1), long load at 13..15 (lines 0 and 1)
	move.l	#$4C4D4E4F,d1		; bytes 12..15 as the pattern
	moves.l	d1,12(a0)
	moves.l	d5,16(a0)
	moves.l	14(a0),d1
	move.l	d5,d2
	swap	d2
	and.l	#$FFFF,d2
	or.l	#$4E4F0000,d2		; bytes 14, 15 never stored
	cmp.l	d2,d1
	beq.s	.h2
	failt	18
.h2:	addq.l	#1,d5
	moves.l	d5,16(a0)
	moves.l	13(a0),d1
	move.l	d5,d2
	rol.l	#8,d2
	and.l	#$FF,d2
	or.l	#$4D4E4F00,d2
	cmp.l	d2,d1
	beq.s	.h3
	failt	18
.h3:	addq.l	#1,d5
	moves.l	d5,16(a0)
	moveq	#0,d1
	moves.w	15(a0),d1
	move.l	d5,d2
	rol.l	#8,d2
	and.l	#$FF,d2
	or.l	#$4F00,d2
	cmp.l	d2,d1
	beq.s	.h4
	failt	18
.h4:	; long at 12 (line 0), long load at 14 (lines 0 and 1)
	addq.l	#1,d5
	moves.l	d5,12(a0)
	moves.l	14(a0),d1
	move.l	d5,d2
	swap	d2
	clr.w	d2			; bytes 14, 15 from the store
	moves.l	16(a0),d3
	swap	d3
	move.w	d3,d2			; bytes 16, 17 as they are
	cmp.l	d2,d1
	beq.s	.h5
	failt	19
.h5:	add.l	#$01010101,d5
	dbra	d4,.h1

;------------------------------- 20-27 user mode, plain MOVE
	lea	(ucont).l,a1
	move.l	a1,(ucont_v).l
	lea	($3000).l,a1
	move	a1,usp
	move.w	#$0000,sr		; user mode, interrupts open (none come)
	moveq	#0,d6
	moveq	#1,d4			; twice: the second pass from the I-cache
uloop:
	ucase	0,move.l,$A1A2A3A4,16,move.l,14,$4E4FA1A2,20
	ucase	1,move.l,$A1A2A3A4,16,move.l,14,$4E4FA1A2,20
	ucase	2,move.l,$A1A2A3A4,16,move.l,14,$4E4FA1A2,20
	ucase	3,move.l,$A1A2A3A4,16,move.l,14,$4E4FA1A2,20
	ucase	0,move.l,$A1A2A3A4,16,move.l,13,$4D4E4FA1,21
	ucase	1,move.l,$A1A2A3A4,16,move.l,13,$4D4E4FA1,21
	ucase	0,move.l,$A1A2A3A4,16,move.w,15,$4FA1,22
	ucase	1,move.l,$A1A2A3A4,16,move.w,15,$4FA1,22
	ucase	0,move.l,$A1A2A3A4,12,move.l,14,$A3A45051,23
	ucase	1,move.l,$A1A2A3A4,12,move.l,14,$A3A45051,23
	ucase	2,move.l,$A1A2A3A4,12,move.l,14,$A3A45051,23
	ucase	0,move.w,$B1B2,16,move.l,14,$4E4FB1B2,24
	ucase	1,move.w,$B1B2,16,move.l,14,$4E4FB1B2,24
	ucase	0,move.b,$C1,17,move.l,15,$4F50C152,25
	ucase	1,move.b,$C1,17,move.l,15,$4F50C152,25
	ucase	0,move.l,$A1A2A3A4,14,move.l,15,$A2A3A452,26
	ucase	1,move.l,$A1A2A3A4,14,move.l,15,$A2A3A452,26
	ucase	2,move.l,$A1A2A3A4,14,move.l,15,$A2A3A452,26
	ucase	0,move.l,$A1A2A3A4,18,move.l,15,$4F5051A1,27
	ucase	1,move.l,$A1A2A3A4,18,move.l,15,$4F5051A1,27
	dbra	d4,uloop
	trap	#1
ucont:

;------------------------- 28-29 a snoop between the halves of a store
	lea	($D00E).l,a3
	lea	($D010).l,a4
	moves.l	(a3),d1			; both lines present
	clr.l	(FBCNT).l
	move.l	#$10000,d5
	move.w	#$C000,d6		; SC 01 writes to $D014, 0..7 clocks apart
.s0:	move.w	d6,(TRAFFIC).l
	move.w	#249,d4
.s1:	cpushl	dc,(a4)			; the second line: pushed, invalidated
	addq.l	#1,d5
	moves.l	d5,(a3)			; across the line
	moves.l	(a3),d1
	cmp.l	d5,d1
	beq.s	.s2
	clr.w	(TRAFFIC).l
	failt	28
.s2:	dbra	d4,.s1
	addq.w	#1,d6
	cmp.w	#$C008,d6
	bne.s	.s0
	clr.w	(TRAFFIC).l
	move.w	#200,d1
.s3:	nop
	dbra	d1,.s3
	moves.l	(a3),d1
	cmp.l	d5,d1
	beq.s	.s4
	failt	28
.s4:	btst	#5,(CAPW+1).l		; BCLK at half PCLK: the other master's
	beq.s	.s5			; snoop can fall between the halves (at
	move.l	(FBCNT).l,d1		; full speed it arbitrates for the bus
	bne.s	.s5			; after the fill, too late)
	failt	29
.s5:

;----------------------------- 30 write-only operations do not read
	bsr	arm			; supervisor data: noncachable (DTT1)
	clr.l	(BUF).l
	clr.w	(BUF+4).l
	clr.b	(BUF+6).l
	st	(BUF+7).l
	move.w	sr,(BUF+8).l
	nop
	move.l	(WRD).l,d1
	chkl	d1,0,30
	move.l	(WWR).l,d1
	chkl	d1,5,30
	clr.l	(WLO).l
	clr.l	(WHI).l

;------------------------------------- 14-17 page-crossing, the MMU on
	cinva	dc
	moveq	#0,d0
	movec	d0,dtt0
	lea	(ROOT).l,a1		; clear the tables
	move.w	#(PAGE+$100-ROOT)/4-1,d1
.t0:	clr.l	(a1)+
	dbra	d1,.t0
	move.l	#PTR+2,(ROOT).l
	move.l	#PAGE+2,(PTR+$41*4).l
	move.l	#$B000+$21,(PAGE+$10*4).l	; resident, copyback (CM 01)
	move.l	#$C000+$21,(PAGE+$11*4).l
	lea	(PB).l,a1		; memory: $EE (supervisor, noncachable)
	moveq	#7,d1
.t1:	move.l	#$EEEEEEEE,(a1)+
	dbra	d1,.t1
	lea	(ROOT).l,a1
	movec	a1,urp
	movec	a1,srp
	move.l	#$0000C000,d0		; ITT0: the program, untranslated
	movec	d0,itt0
	pflusha
	move.l	#$8000,d0		; E, 4 KB pages
	movec	d0,tc
	lea	(LA).l,a2
	moveq	#0,d0			; the cache: the pattern (copyback stores)
.t2:	bsr	exp_l
	moves.l	d2,(a2,d0.w)
	addq.w	#4,d0
	cmp.w	#32,d0
	bne.s	.t2
	move.l	#PB,(WLO).l
	move.l	#PB+32,(WHI).l
	clr.l	(WRD).l
	clr.l	(WWR).l
	moveq	#0,d0
.t3:	moves.l	(a2,d0.w),d1
	bsr	exp_l
	cmp.l	d2,d1
	beq.s	.t3ok
	failt	14
.t3ok:	addq.w	#1,d0
	cmp.w	#29,d0
	bne.s	.t3
	moveq	#0,d0
.t4:	moveq	#0,d1
	moves.w	(a2,d0.w),d1
	bsr	exp_w
	cmp.l	d2,d1
	beq.s	.t4ok
	failt	15
.t4ok:	addq.w	#1,d0
	cmp.w	#31,d0
	bne.s	.t4
	move.l	#$A1B2C3D4,d1
	moves.l	d1,14(a2)		; across the page boundary
	moves.l	12(a2),d1
	chkl	d1,$4C4DA1B2,16
	moves.l	16(a2),d1
	chkl	d1,$C3D45253,16
	move.l	(WRD).l,d1
	chkl	d1,0,16
	move.l	(WWR).l,d1
	chkl	d1,0,16
	cpusha	dc
	nop
	move.l	(WWR).l,d1
	chkl	d1,8,17
	clr.l	(WLO).l
	clr.l	(WHI).l
	move.l	(PB+12).l,d1		; supervisor: noncachable, physical
	chkl	d1,$4C4DA1B2,17
	move.l	(PB+16).l,d1
	chkl	d1,$C3D45253,17
	moveq	#0,d0
	movec	d0,tc
	movec	d0,itt0
	pflusha

;----------------------------------------------------------------- done
	clr.l	(WLO).l
	clr.l	(WHI).l
	move.w	#$600D,(DONEREG).l
	bra.s	*

; the window: BUF..BUF+64, counters cleared
arm:	move.l	#BUF,(WLO).l
	move.l	#BUF+64,(WHI).l
	clr.l	(WRD).l
	clr.l	(WWR).l
	rts

; no bus transfer in the window so far
quiet1:	move.l	(WRD).l,d1
	chkl	d1,0,1
	move.l	(WWR).l,d1
	chkl	d1,0,1
	rts
quiet2:	move.l	(WRD).l,d1
	chkl	d1,0,2
	move.l	(WWR).l,d1
	chkl	d1,0,2
	rts
quiet3:	move.l	(WRD).l,d1
	chkl	d1,0,3
	move.l	(WWR).l,d1
	chkl	d1,0,3
	rts

; d2 = the long word at offset d0 of the pattern (byte i = $40 + i)
exp_l:	move.l	d0,d2
	add.l	#$40,d2
	mulu.l	#$01010101,d2
	add.l	#$00010203,d2
	rts
; d2 = the word at offset d0
exp_w:	move.l	d0,d2
	add.l	#$40,d2
	mulu.l	#$0101,d2
	addq.l	#1,d2
	rts

fail_all:
	clr.l	(WLO).l
	clr.l	(WHI).l
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99

h_trap0:
	bra	fail_all		; d7: the failing test
h_trap1:
	addq.l	#8,sp			; drop the frame (format $0), supervisor
	move.l	(ucont_v).l,-(sp)
	rts
ucont_v	equ	$3610
