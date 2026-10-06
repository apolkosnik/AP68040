; AP68040-60: the data cache in each caching mode, by the bus transfers it
; makes (MC68040UM 4.3, 4.4, table 4-x); the bench counts the CPU's data
; reads and writes in a window ($F2D0-$F2DC, in beats: a line is four)
; assembled with vasmm68k_mot -Fbin -m68040
;
; User data goes through MOVES (SFC = DFC = 1) under DTT0, whose CM field
; selects the mode; supervisor data (the bench registers, the memory
; checks) is noncachable (DTT1).  Lines of one set are $400 apart.
;   write-through (CM 00)
;   1    a read miss fills the line (4 reads); a read hit makes none
;   2    a write hit updates the line and writes through (1 write, and the
;        line reads back the new value without a bus read)
;   3    a write miss writes through without allocating (1 write; the
;        next read of that line misses: 4 reads)
;   copyback (CM 01)
;   4    a read miss fills the line; a write hit makes no transfer
;   5    a write miss allocates (4 reads, no write) and keeps the data
;   6    a fifth line in a set pushes a dirty victim (4 writes); every
;        line then reads back its value (the victim's from memory)
;   7    CPUSHL pushes a dirty line (4 writes, memory updated)
;   8    CINVL drops a dirty line: nothing is written and the next read
;        sees memory's old data
;   noncachable (CM 10 serialized, 11 not)
;   9-10 every read and write is a bus transfer and nothing is cached
;   11   a noncachable read that hits a dirty line (the mode changed under
;        it) pushes it and invalidates it, then reads memory (4 writes,
;        1 read; WinUAE does the same); the next read is a bus read, and
;        back in copyback the line refills with the pushed value

FAILREG	equ	$F100
DONEREG	equ	$F102
WLO	equ	$F2D0
WHI	equ	$F2D4
WRD	equ	$F2D8
WWR	equ	$F2DC
L0	equ	$9C00		; five lines of one set
L1	equ	$A000
L2	equ	$A400
L3	equ	$A800
L4	equ	$AC00

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

; counters: reads \1, writes \2, else test \3
cnt	macro
	move.l	(WRD).l,d1
	chkl	d1,\1,\3
	move.l	(WWR).l,d1
	chkl	d1,\2,\3
	clr.l	(WRD).l
	clr.l	(WWR).l
	endm

; memory (supervisor, noncachable) at \1 must hold \2, else test \3
mem	macro
	clr.l	(WHI).l			; the window empty (start above its end)
	move.l	(\1).l,d1
	chkl	d1,\2,\3
	move.l	#L4+16,(WHI).l
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$0000A040,d0		; DTT1: supervisor data noncachable
	movec	d0,dtt1
	moveq	#1,d0
	movec	d0,sfc
	movec	d0,dfc
	; memory: each line's first long word names the line
	move.l	#$10101010,(L0).l
	move.l	#$11111111,(L1).l
	move.l	#$12121212,(L2).l
	move.l	#$13131313,(L3).l
	move.l	#$14141414,(L4).l
	move.l	#$80008000,d0
	movec	d0,cacr
	move.l	#L0,(WLO).l
	move.l	#L4+16,(WHI).l
	clr.l	(WRD).l
	clr.l	(WWR).l

;------------------------------------------------------- write-through
	cinva	dc
	move.l	#$00008000,d0		; DTT0: user data, write-through
	movec	d0,dtt0
	moves.l	(L0).l,d2		; 1: miss, fill
	chkl	d2,$10101010,1
	cnt	4,0,1
	moves.l	(L0).l,d2		; hit
	moves.l	(L0+4).l,d2
	cnt	0,0,1
	move.l	#$20202020,d2		; 2: write hit
	moves.l	d2,(L0).l
	cnt	0,1,2
	moves.l	(L0).l,d3
	chkl	d3,$20202020,2
	cnt	0,0,2
	mem	L0,$20202020,2
	move.l	#$21212121,d2		; 3: write miss, no allocate
	moves.l	d2,(L1).l
	cnt	0,1,3
	mem	L1,$21212121,3
	moves.l	(L1).l,d3
	chkl	d3,$21212121,3
	cnt	4,0,3

;------------------------------------------------------------ copyback
	cinva	dc
	move.l	#$00008020,d0		; DTT0: user data, copyback
	movec	d0,dtt0
	move.l	#$10101010,(L0).l	; memory as at the start
	move.l	#$11111111,(L1).l
	clr.l	(WRD).l
	clr.l	(WWR).l
	moves.l	(L0).l,d2		; 4: miss, fill
	chkl	d2,$10101010,4
	cnt	4,0,4
	move.l	#$30303030,d2		; write hit
	moves.l	d2,(L0).l
	cnt	0,0,4
	move.l	#$31313131,d2		; 5: write miss, allocate
	moves.l	d2,(L1+4).l
	cnt	4,0,5
	moves.l	(L1).l,d3		; the rest of the line from memory
	chkl	d3,$11111111,5
	moves.l	(L1+4).l,d3
	chkl	d3,$31313131,5
	cnt	0,0,5
	; 6: L0 and L1 dirty; L2, L3 fill the set; L4 needs a victim
	moves.l	(L2).l,d3
	moves.l	(L3).l,d3
	cnt	8,0,6
	move.l	#$32323232,d2		; L2 and L3 dirty too: any victim pushes
	moves.l	d2,(L2).l
	moves.l	d2,(L3).l
	moves.l	(L4).l,d3
	chkl	d3,$14141414,6
	cnt	4,4,6
	; every line still reads its value: the victim refills from memory,
	; where its push left it (which line was the victim does not matter)
	moves.l	(L0).l,d3
	chkl	d3,$30303030,6
	moves.l	(L1+4).l,d3
	chkl	d3,$31313131,6
	moves.l	(L2).l,d3
	chkl	d3,$32323232,6
	moves.l	(L3).l,d3
	chkl	d3,$32323232,6
	moves.l	(L4).l,d3
	chkl	d3,$14141414,6
	cinva	dc			; start the next case clean
	clr.l	(WRD).l
	clr.l	(WWR).l
	; 7: CPUSHL pushes a dirty line
	moves.l	(L1).l,d3		; fill (memory: L1 = $11111111 or the push)
	move.l	#$33333333,d2
	moves.l	d2,(L1).l
	cnt	4,0,7
	lea	(L1).l,a0
	cpushl	dc,(a0)
	nop
	cnt	0,4,7
	mem	L1,$33333333,7
	; 8: CINVL drops a dirty line
	move.l	#$12121212,(L2).l	; memory
	clr.l	(WRD).l
	clr.l	(WWR).l
	moves.l	(L2).l,d3
	move.l	#$34343434,d2
	moves.l	d2,(L2).l
	cnt	4,0,8
	lea	(L2).l,a0
	cinvl	dc,(a0)
	nop
	cnt	0,0,8
	moves.l	(L2).l,d3
	chkl	d3,$12121212,8
	cnt	4,0,8

;---------------------------------------------------------- noncachable
	cinva	dc
	move.l	#$00008040,d0		; 9: DTT0: user data, noncachable serialized
	movec	d0,dtt0
	moveq	#0,d6
	bsr	ncase
	move.l	#$00008060,d0		; 10: noncachable
	movec	d0,dtt0
	addq.w	#1,d6
	bsr	ncase

;------------------------------- 11 a noncachable read hits a dirty line
	cinva	dc
	move.l	#$00008020,d0		; copyback: L0 dirty
	movec	d0,dtt0
	move.l	#$10101010,(L0).l
	moves.l	(L0).l,d3
	move.l	#$50505050,d2
	moves.l	d2,(L0).l
	clr.l	(WRD).l
	clr.l	(WWR).l
	move.l	#$00008060,d0		; noncachable, the line still there
	movec	d0,dtt0
	moves.l	(L0).l,d3
	chkl	d3,$50505050,11
	cnt	1,4,11
	moves.l	(L0).l,d3		; gone: memory again
	chkl	d3,$50505050,11
	cnt	1,0,11
	move.l	#$00008020,d0		; copyback: a fresh fill
	movec	d0,dtt0
	moves.l	(L0).l,d3
	chkl	d3,$50505050,11
	cnt	4,0,11

	clr.l	(WLO).l
	clr.l	(WHI).l
	move.w	#$600D,(DONEREG).l
	bra.s	*

; noncachable: two reads, a write, a read -- four transfers, values from
; memory each time (d6: test number)
ncase:	moveq	#9,d7
	add.w	d6,d7
	move.l	#$40404040,(L3).l
	clr.l	(WRD).l
	clr.l	(WWR).l
	moves.l	(L3).l,d3
	cmp.l	#$40404040,d3
	bne.s	.nf
	moves.l	(L3).l,d3
	move.l	#$41414141,d2
	moves.l	d2,(L3).l
	moves.l	(L3).l,d3
	cmp.l	#$41414141,d3
	bne.s	.nf
	cmp.l	#3,(WRD).l
	bne.s	.nf
	cmp.l	#1,(WWR).l
	bne.s	.nf
	rts
.nf:	bra	fail_all

fail_all:
	clr.l	(WLO).l
	clr.l	(WHI).l
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99
