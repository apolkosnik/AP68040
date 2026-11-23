; AP68040: the instruction cache follows the processor's own stores
; (self-snoop: not a 68040 behaviour -- software for the 68000-68030 copies
; or patches code and runs it without CINV/CPUSH; a packed program's
; depacker unpacking over itself is the common case)
; assembled with vasmm68k_mot -Fbin -m68040
;
; A routine at CODE (MOVEQ #n,D0 / RTS) is called, so its line is in the
; instruction cache, then rewritten by ordinary stores and called again:
; it must return the new value.  No CINV/CPUSH anywhere.  The bench's
; IC_SELF pin ($F2C0 bit 2) turns the behaviour on; t_cache checks it off.
;   1-3   write-through (translation off), a long store over the MOVEQ
;   4-6   the same with a word store, a byte store
;   7-9   copyback (DTT0 over CODE), the store a cache hit in the data cache
;   10-12 a block copy over a whole line already in the cache (MOVE.L
;         (A0)+,(A1)+), then called
;   13    code that patches the instruction right after the store (the
;         prefetch has fetched it already)
;   20    unchanged: a store to a line not in the instruction cache, then
;         a call there (no effect, the fill reads memory)
;   98    an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
PINS	equ	$F2C0		; bit 2: IC_SELF
CODE	equ	$5000
CODE2	equ	$5400
DATA	equ	$6000

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

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#4,(PINS).l		; IC_SELF
	cinva	bc
	move.l	#$80008000,d0		; IE, DE
	movec	d0,cacr
	moveq	#0,d0
	movec	d0,tc
	movec	d0,dtt0
	movec	d0,dtt1
	movec	d0,itt0
	movec	d0,itt1

;------------------------------------------------ 1-3 write-through, long
	move.l	#$70114E75,(CODE).l	; moveq #$11,d0 / rts
	cpusha	bc			; (only to start clean)
	jsr	(CODE).l
	chkl	d0,$11,1
	jsr	(CODE).l		; now from the instruction cache
	chkl	d0,$11,2
	move.l	#$70224E75,(CODE).l	; moveq #$22,d0 / rts
	jsr	(CODE).l
	chkl	d0,$22,3

;------------------------------------------------ 4-6 word, byte
	move.w	#$7033,(CODE).l		; moveq #$33,d0
	jsr	(CODE).l
	chkl	d0,$33,4
	move.b	#$44,(CODE+1).l		; moveq #$44,d0
	jsr	(CODE).l
	chkl	d0,$44,5
	jsr	(CODE).l
	chkl	d0,$44,6

;------------------------------------------------ 7-9 copyback
	move.l	#$0000C020,d0		; DTT0: $00xxxxxx, S ignored, copyback
	movec	d0,dtt0
	move.l	(CODE).l,d1		; the line into the data cache
	jsr	(CODE).l
	chkl	d0,$44,7
	move.l	#$70554E75,(CODE).l	; a copyback hit: no bus write
	jsr	(CODE).l
	chkl	d0,$55,8
	move.w	#$7066,(CODE).l
	jsr	(CODE).l
	chkl	d0,$66,9
	cpusha	dc
	moveq	#0,d0
	movec	d0,dtt0

;------------------------------------------------ 10-12 a copied line
	lea	(CODE2).l,a1
	move.l	#$70014E75,(a1)		; moveq #1,d0 / rts, then filler
	move.l	#$4E714E71,4(a1)
	move.l	#$4E714E71,8(a1)
	move.l	#$4E714E71,12(a1)
	jsr	(CODE2).l
	chkl	d0,1,10
	lea	newline(pc),a0
	moveq	#3,d1
.cp:	move.l	(a0)+,(a1)+		; the new code over the cached line
	dbra	d1,.cp
	jsr	(CODE2).l
	chkl	d0,$7F,11
	jsr	(CODE2+8).l
	chkl	d0,$7E,12

;------------------------------------------------ 13 patch the next instruction
	moveq	#0,d0
	move.w	#$7077,patch		; moveq #$77,d0 over the moveq #$01
	nop
patch:	moveq	#$01,d0
	chkl	d0,$77,13

;------------------------------------------------ 20 a line not in the cache
	move.l	#$70204E75,(CODE+$100).l
	jsr	(CODE+$100).l
	chkl	d0,$20,20

	clr.w	(PINS).l
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	98

	cnop	0,4
newline:
	dc.l	$707F4E75		; moveq #$7F,d0 / rts
	dc.l	$4E714E71
	dc.l	$707E4E75		; moveq #$7E,d0 / rts
	dc.l	$4E714E71
