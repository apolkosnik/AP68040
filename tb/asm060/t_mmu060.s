; AP68060 MMU: rules where the 68060 manual is explicit (MC68060UM section 4)
; assembled with vasmm68k_mot -Fbin -m68040
;
; ITT0 and DTT0 map logical $00xxxxxx transparently (the program, its data
; and the tables); logical $01xxxxxx is translated through SRP ($4000):
; root $4000 -> pointer table $4200 (entry 64, at $4300, for $01000000) ->
; page table $4400.
;   1-3  a write meeting a write-protected ROOT descriptor ends the search
;        at once (4.2.6.3): WP, the root's U bit not set, the pointer level
;        never read (its entry is invalid here: a search that went on would
;        report PTB instead)
;   4    PLPA uses an existing ATC entry (4.7.3): after a load fills the
;        ATC, the page descriptor is changed without PFLUSH, and PLPAR with
;        DFC 6 still gives the cached translation -- PLPA works on the data
;        side whatever the DFC (appendix D)
;   5-7  setting U is a locked read-modify-write (4.2.5, table 4-1): for
;        each of the three descriptors (U clear) the search reads it, then
;        reads it again with LOCK and writes it with LOCK and LOCKE (the
;        bench counts the transfers in a watch window over the tables)
;
;   98   an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
cnt_ae	equ	$3600
ae_fslw	equ	$3604
resume	equ	$3608
WLO	equ	$F2D0		; watch window start, end
WHI	equ	$F2D4
LKPLAIN	equ	$F2F4		; reads in it without LOCK
LKREAD	equ	$F2F8		; ... with LOCK
LKWRITE	equ	$F2FC		; writes with LOCK and LOCKE

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

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	h_ae		; 2 access error
	rept	253
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$0000C000,d0	; ITT0/DTT0: $00xxxxxx, both modes, transparent
	movec	d0,itt0
	movec	d0,dtt0
	move.l	#$4000,d0
	movec	d0,srp
	movec	d0,urp
	clr.l	(cnt_ae).l

;------------------------------------------------ 1-3 a write-protected root
	move.l	#$00004207,($4000).l	; root 0: pointer table $4200, W, U clear
	clr.l	($4300).l		; pointer 64 invalid
	pflusha
	move.l	#$8000,d0		; E, 4K pages
	movec	d0,tc
	move.l	#wp_back,(resume).l
	move.l	d0,($01000000).l	; a write: WP at the root
wp_back:
	moveq	#0,d0
	movec	d0,tc
	move.l	(cnt_ae).l,d1
	chkl	d1,1,1
	move.l	(ae_fslw).l,d1
	and.l	#$00001FC0,d1		; PTA PTB IL PF SP WP TWE
	chkl	d1,$00000080,2		; WP (not PTB)
	move.l	($4000).l,d1
	chkl	d1,$00004207,3		; the root's U bit untouched

;------------------------------------------------ 4 PLPA and the ATC
	move.l	#$0000420B,($4000).l	; root 0, U set
	move.l	#$0000440B,($4300).l	; pointer 64 -> page table $4400, U set
	move.l	#$0000800B,($4400).l	; page 0 -> $8000, U set
	move.w	#$4E75,($8000).l	; rts
	pflusha
	move.l	#$8000,d0
	movec	d0,tc
	move.l	($01000000).l,d1	; a data access fills the ATC
	jsr	($01000000).l		; (and an instruction one the I-ATC)
	move.l	#$0000900B,($4400).l	; the descriptor changed, no PFLUSH
	moveq	#6,d0
	movec	d0,dfc
	movea.l	#$01000000,a0
	dc.w	$F5C8			; plpar (a0)
	moveq	#0,d0
	movec	d0,tc
	chkl	a0,$00008000,4		; the cached translation

;------------------------------------------------ 5-7 U set by locked RMW
	move.l	#$00004202,($4000).l	; root 0 -> $4200, U clear
	move.l	#$00004402,($4300).l	; pointer 64 -> $4400, U clear
	move.l	#$00008001,($4400).l	; page 0 -> $8000, U clear
	pflusha
	move.l	#$4000,(WLO).l		; watch the tables
	move.l	#$4500,(WHI).l
	clr.l	(LKPLAIN).l
	clr.l	(LKREAD).l
	clr.l	(LKWRITE).l
	move.l	#$8000,d0
	movec	d0,tc
	move.l	($01000000).l,d1	; the search
	moveq	#0,d0
	movec	d0,tc
	move.l	(LKPLAIN).l,d1
	chkl	d1,3,5			; each descriptor read
	move.l	(LKREAD).l,d1
	chkl	d1,3,6			; ... again with LOCK
	move.l	(LKWRITE).l,d1
	chkl	d1,3,7			; ... and written with LOCK and LOCKE
	clr.l	(WHI).l

	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	moveq	#0,d0
	movec	d0,tc
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

h_ae:
	cmpi.w	#$4008,6(sp)
	bne.s	h_bad
	addq.l	#1,(cnt_ae).l
	move.l	12(sp),(ae_fslw).l
	move.l	(resume).l,2(sp)
	rte
h_bad:
	failt	98

unexp:
	failt	98
