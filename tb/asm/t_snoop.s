; AP040 the chipset's writes to memory, snooped by the pipelined core's
; caches (caches stage F)
; assembled with vasmm68k_mot -Fbin -m68040 -no-opt
;
; testbench protocol: $F100 fail number, $F102 result magic;
; $F134 w: a poke's address, $F136 w: poke the word there behind the CPU,
; unsnooped; $F15C w: a chipset write's address, $F15E w: write the word
; there as the chipset would -- behind the CPU, and snooped
;
; MC68040UM 4.7 and Table 4-3: another master's write that a cache holds
; invalidates the line (this platform's snoop mode). A poke, unsnooped,
; shows that a line is cached: a read after it is stale. A chipset write
; is then seen by the next read, from the data cache and the instruction
; cache alike. The bench runs with production's windows: chip RAM, where
; everything here is, is cacheable.
;
; Untranslated, write-through: the protocol registers are only ever
; written, so their longwords are never cached. A chipset write happens
; when the register write starting it reaches the bus, which the store
; buffer (caches stage E) may still hold while a read that hits is
; answered: as a program starting DMA would, each waits for it (sync) --
; a read of an alternate space, never cached, goes out only behind every
; write before it.

FAILREG		equ	$F100
DONEREG		equ	$F102
POKEA		equ	$F134
POKED		equ	$F136
DMAA		equ	$F15C
DMAD		equ	$F15E

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

; poke <address>,<word>: memory changed behind the CPU, unsnooped
poke	macro
	move.w	#\1,(POKEA).l
	move.w	#\2,(POKED).l
	endm

; dma <address>,<word>: the chipset writes it, snooped; then the program
; waits for it to have happened
dma	macro
	move.w	#\1,(DMAA).l
	move.w	#\2,(DMAD).l
	moves.w	(a5),d6
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#$2700,sr
	moveq	#0,d0
	movec	d0,tc
	movec	d0,itt0
	movec	d0,itt1
	movec	d0,dtt0
	movec	d0,dtt1
	cinva	bc
	moveq	#3,d0			; the sync's alternate space
	movec	d0,sfc
	lea	(FAILREG).l,a5
	move.l	#$80008000,d0		; DE and IE
	movec	d0,cacr

;------------------------------------------------------------ the data cache
	poke	$5000,$1111
	poke	$5002,$2222
	move.l	($5000).l,d0		; the line read: cached
	chkl	d0,$11112222,1
	poke	$5000,$3333		; unsnooped: the cache still holds the line
	move.l	($5000).l,d0
	chkl	d0,$11112222,2
	dma	$5002,$4444		; the chipset's write: the line goes
	move.l	($5000).l,d0		; read again, from memory
	chkl	d0,$33334444,3
	move.l	($5004).l,d0		; the same line, cached again
	dma	$500C,$5555		; a write elsewhere in the line: it goes too
	move.l	($5008).l,d0
	moveq	#0,d1
	move.w	($500C).l,d1
	chkl	d1,$00005555,4

;----------------------------------------------------- the instruction cache
; sub at $6000 returns the constant its MOVEQ holds
	poke	$6000,$7001		; moveq #1,d0
	poke	$6002,$4E75		; rts
	cinva	ic
	moveq	#0,d0
	jsr	($6000).l		; fetched into the instruction cache
	chkl	d0,1,5
	poke	$6000,$7003		; unsnooped: the cached copy runs
	moveq	#0,d0
	jsr	($6000).l
	chkl	d0,1,6
	dma	$6000,$7002		; the chipset's write: the line goes
	moveq	#0,d0
	jsr	($6000).l		; fetched again, from memory
	chkl	d0,2,7

;----------------------------------------------------------------- done
	moveq	#0,d0
	movec	d0,cacr
	cinva	bc
	move.w	#$600D,(DONEREG).l
	stop	#$2700

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
