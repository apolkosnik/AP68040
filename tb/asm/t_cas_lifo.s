; AP040 CAS/CAS2 against the data cache: Open Transport's atomic LIFO, instruction for instruction
; assembled with vasmm68k_mot -Fbin -m68040
;
; OTLIFOEnqueue is  move.l (a0),d0 / move.l d0,(a1) / cas.l d0,d1,(a0) / bne
; OTLIFODequeue is  move.l (a0),d0 / movea.l d0,a1 / move.l (a1),d1 /
;                   cas2.l d0:d1,d1:d1,(a0):(a1) / bne
; (address registers as the CAS2 pointers, one register as Dc2, Du1 and Du2).  The Quadra 800's
; first Ethernet bring-up (2026-09-18) died in exactly these two routines: the receive path pops
; a free element with the CAS2 and pushes it with the CAS, and a pop that hands out the same
; element twice makes the work list circular.  The heads and elements below sit in cached lines,
; in one set and in different sets, so every store the two instructions make is a store to a
; line the data cache holds.
;
; testbench protocol: $F100 fail number, $F102 result magic

FAILREG	equ	$F100
DONEREG	equ	$F102

HEAD	equ	$3600			; list head
E1	equ	$3700			; elements: link at +0
E2	equ	$3740
E3	equ	$3780
HEAD2	equ	$4610			; a head in the same cache SET as E1's neighbours
F1	equ	$4700
F2	equ	$5700			; same set index as F1 ($70), different tag

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
	move.l	#$80008000,d0		; both caches on
	movec	d0,cacr

;------------------------------------------------ build E1 -> E2 -> E3 by pushing E3, E2, E1
	clr.l	(HEAD).l
	lea	(HEAD).l,a0
	move.l	#E3,d1
	bsr	push
	move.l	#E2,d1
	bsr	push
	move.l	#E1,d1
	bsr	push
	move.l	(HEAD).l,d0
	chkl	d0,E1,1
	move.l	(E1).l,d0
	chkl	d0,E2,2
	move.l	(E2).l,d0
	chkl	d0,E3,3
	move.l	(E3).l,d0
	chkl	d0,0,4

;------------------------------------------------ pop them: each element exactly once
	bsr	pop
	chkl	d0,E1,10
	move.l	(HEAD).l,d2		; the head the NEXT pop will read
	chkl	d2,E2,11
	bsr	pop
	chkl	d0,E2,12		; E1 again = the stale-head double pop
	bsr	pop
	chkl	d0,E3,13
	bsr	pop
	chkl	d0,0,14

;------------------------------------------------ push/pop churn, the receive path's pattern
	moveq	#63,d6
churn:
	move.l	#E1,d1
	bsr	push
	move.l	#E2,d1
	bsr	push
	bsr	pop
	chkl	d0,E2,20
	bsr	pop
	chkl	d0,E1,21
	bsr	pop
	chkl	d0,0,22
	dbra	d6,churn

;------------------------------------------------ head and elements sharing a cache set
	clr.l	(HEAD2).l
	lea	(HEAD2).l,a0
	move.l	#F2,d1
	bsr	push
	move.l	#F1,d1
	bsr	push
	move.l	(F1).l,d0		; touch both lines again: both cached
	move.l	(F2).l,d0
	bsr	pop
	chkl	d0,F1,30
	bsr	pop
	chkl	d0,F2,31
	bsr	pop
	chkl	d0,0,32
	lea	(HEAD).l,a0

;------------------------------------------------ a CAS2 that must fail leaves memory alone
	move.l	#E1,(HEAD).l
	move.l	#E2,(E1).l
	move.l	(HEAD).l,d0
	movea.l	d0,a1
	move.l	#$12345678,d1		; not E1's link
	cas2.l	d0:d1,d1:d1,(a0):(a1)
	beq	fail_31
	chkl	d1,E2,40		; Dc2 loaded with the memory operand
	move.l	(HEAD).l,d2
	chkl	d2,E1,41
	move.l	(E1).l,d2
	chkl	d2,E2,42

	move.w	#$600D,(DONEREG).l
	stop	#$2700

fail_31:
	failt	39

; OTLIFOEnqueue(a0 = head, d1 = element)
push:
	movea.l	d1,a1
push1:
	move.l	(a0),d0
	move.l	d0,(a1)
	cas.l	d0,d1,(a0)
	bne.s	push1
	rts

; OTLIFODequeue(a0 = head) -> d0
pop:
	move.l	(a0),d0
	beq.s	pop9
	movea.l	d0,a1
	move.l	(a1),d1
	cas2.l	d0:d1,d1:d1,(a0):(a1)
	bne.s	pop
pop9:
	rts

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
