; AP040 milestone G cache self test
; assembled with vasmm68k_mot -Fbin -m68040
;
; testbench protocol: $F100 fail number, $F102 result magic,
; word write to $F130 pokes chip memory at $3500 behind the CPU's back
; (simulates DMA writing memory that the data cache has already loaded)

FAILREG	equ	$F100
DONEREG	equ	$F102
POKEREG	equ	$F130

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
;------------------------------------------------- enable both caches
	move.l	#$80008000,d0
	movec	d0,cacr
	movec	cacr,d1
	chkl	d1,$80008000,1

;------------------------ MOVEC source clobbered by the next instruction
; MOVEQ is the one opcode the fast path dispatches straight out of decode
; with its Dn writeback deferred into the following decode.  A MOVEC's
; source read is staged in S_MOVEC1 and sampled in S_MOVEC2; if a MOVEQ
; that FOLLOWS the MOVEC could ever commit ahead of that sample, the
; control register would take the MOVEQ's value.  These cases pin the
; ordering.  They were written while chasing a suspected instance of
; exactly that, which turned out to be a bench running a program image
; that did not exist -- the hazard was never real on this core, and these
; cases passed on first run.  They stay because the class is real and
; cheap to guard.  Two registers: CACR, and DFC, which is harmless if
; wrong so a failure here cannot take the rest of this program down.
	move.l	#$80008000,d0
	movec	d0,cacr
	moveq	#1,d0
	movec	cacr,d1
	chkl	d1,$80008000,20
	move.l	#5,d0
	movec	d0,dfc
	moveq	#1,d0
	movec	dfc,d1
	chkl	d1,5,21
	move.l	#$80008000,d0		; leave both caches on for what follows
	movec	d0,cacr

;--------------------------------------- I-cache and self modifying code
	; stub at $2000: moveq #1,d0 ; rts
	move.w	#$7001,($2000).l
	move.w	#$4E75,($2002).l
	jsr	($2000).l
	chkl	d0,1,2			; also fills the I-cache line

	move.w	#$7002,($2000).l	; memory now says moveq #2
	jsr	($2000).l
	chkl	d0,1,3			; stale I-cache line still serves 1

	cinva	ic
	jsr	($2000).l
	chkl	d0,2,4			; after CINV the new code runs

;----------------------------------------------- D-cache and stale data
	move.l	#$0D0D0001,($3500).l	; write-through (invalidates the set)
	move.l	($3500).l,d0		; fills the D-cache line
	chkl	d0,$0D0D0001,5
	move.w	#$BEEF,(POKEREG).l	; TB pokes $3500 = $BEEF0000 by DMA
	move.l	($3500).l,d0
	chkl	d0,$0D0D0001,6		; stale cached copy, DMA is invisible
	cinva	dc
	move.l	($3500).l,d0
	chkl	d0,$BEEF0000,7		; fresh after invalidation

	; CPU writes stay coherent: write-through invalidates the line
	move.l	($3508).l,d0		; cache the neighbouring longword
	move.l	#$0D0D0002,($3508).l
	move.l	($3508).l,d0
	chkl	d0,$0D0D0002,8

;------------------------------------ transparent cache-inhibited window
	; DTT0 covers all of $00xxxxxx with CM = noncacheable
	move.l	#$0000C040,d0
	movec	d0,dtt0
	move.l	($3500).l,d0		; bypasses the cache
	chkl	d0,$BEEF0000,9
	move.w	#$CAFE,(POKEREG).l	; poke again
	move.l	($3500).l,d0
	chkl	d0,$CAFE0000,10		; visible immediately: no caching
	moveq	#0,d0
	movec	d0,dtt0

	; back to cacheable: the FIRST cache-inhibited access that HIT the
	; resident line invalidated it while it bypassed (WinUAE dcache040:
	; a hit under CACHE_DISABLE_MMU is pushed and invalidated before the
	; uncached access).  An earlier revision of this test expected the
	; stale $BEEF0000 line to survive the CI window and hit again --
	; "exactly like a real 68040", it said.  It is not: retaining the
	; line hands out pre-DMA data the moment the mapping is cacheable
	; again, which is precisely what CM=NC exists to prevent.
	move.l	($3500).l,d0
	chkl	d0,$CAFE0000,11
	cinva	dc
	move.l	($3500).l,d0
	chkl	d0,$CAFE0000,17

;-------------------------------------------------- byte and word hits
	move.l	#$91A2B3C4,($3510).l
	move.l	#$D5E6F708,($3514).l
	move.l	($3510).l,d0		; fill
	move.b	($3511).l,d1
	and.l	#$FF,d1
	chkl	d1,$A2,12		; byte lane from the cached longword
	move.w	($3512).l,d1
	and.l	#$FFFF,d1
	chkl	d1,$B3C4,13
	move.w	($3511).l,d1
	and.l	#$FFFF,d1
	chkl	d1,$A2B3,18		; odd word bypasses the longword cache lane mux
	move.b	($3513).l,d1
	and.l	#$FF,d1
	chkl	d1,$C4,14

	; misaligned long read bypasses but must stay correct
	move.l	($3511).l,d0
	chkl	d0,$A2B3C4D5,15

	; a misaligned long write spans the $3510 and $3520 cache lines;
	; both cached lines must be invalidated by the write-through path
	move.l	#$11223344,($3520).l
	move.l	($3510).l,d0		; cache first line
	move.l	($3520).l,d0		; cache second line
	move.l	#$AABBCCDD,($351F).l
	move.l	($3520).l,d0
	chkl	d0,$BBCCDD44,19

;---------------------------------------------------------- disable
	moveq	#0,d0
	movec	d0,cacr
	move.l	($3500).l,d0
	chkl	d0,$CAFE0000,16

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
