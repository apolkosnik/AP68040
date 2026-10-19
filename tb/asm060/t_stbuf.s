; AP68060 store buffer (MC68060UM 5.9, 5.10, 8.4.6)
; assembled with vasmm68k_mot -Fbin -m68040
;
; With CACR.ESB set, writes to write-through and noncachable imprecise
; pages go through a four-entry FIFO of bus write cycles: the instruction
; completes once its pieces are in, the buffer writes them out later, and
; no other bus cycle passes them (5.10).  A TEA on a buffered write is
; reported later as an access error with FSLW.SBE, at an instruction
; boundary or instead of an exception that comes before the buffer has
; emptied (8.4.6).
;
; User data is write-through (DTT0, user only), reached by MOVES with SFC
; = DFC = 1; supervisor data -- the bench registers, the stack, this
; program's variables -- is noncachable imprecise (DTT1, CM = 11), so it
; goes through the store buffer too.
;   1-2   a write-through write, then a read of it (a miss: the bus read
;         waits for the buffered write) and a noncachable read of memory
;   3-4   a misaligned long through the buffer (three bus cycles: byte,
;         word, byte at $9001): memory holds it, around bytes untouched
;   5-6   bus order: the bench copies a write of $F294 to $F2A8, so the
;         read of $F2A8 right after the write sees the new value only if
;         the buffered write went out first
;   7     self-modified code: an instruction written through a full buffer,
;         the instruction cache invalidated (the 68060 prefetches: code a
;         program writes needs CINV/CPUSH before it runs, 5.6), then
;         called at once -- the refetch's bus read waits for the buffer
;   8-12  a TEA on a buffered write ($F140, bench one-shot): the store
;         completes, the access error comes later with FSLW.SBE, format
;         $4; the instructions before its stacked PC have run, those after
;         it have not (eight ADDQs, then a NOP, which waits for the
;         buffer); after RTE all eight have
;  13-15  the same TEA behind three more buffered writes, then TRAP #0:
;         exception processing waits for the buffer, sees the TEA and
;         takes the access error instead -- the TRAP's exception is
;         discarded, the stacked PC is the one it would have stacked
;  16-17  ESB clear: the same write is precise, faulting at the store
;         itself (stacked PC = the store, FSLW.WE, no SBE)

FAILREG	equ	$F100
DONEREG	equ	$F102
BERRARM	equ	$F142		; arm a one-shot bus error on the next access to $F140
BERRAD	equ	$F140
AMX	equ	$F200		; harmless bench words to fill the buffer
CODE	equ	$3800
BUF	equ	$9000

cnt_aerr	equ	$3600	; word: access errors taken
cnt_trap	equ	$3602	; word: TRAP #0 taken
last_vec	equ	$3604	; word: format/vector of the last access error
last_pc		equ	$3608
last_fslw	equ	$360C
last_d5		equ	$3610

F_SBE	equ	$00002000
F_WE	equ	$00000010

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

chkw	macro			; memory word (absolute), value, test
	cmp.w	#\2,(\1).l
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	h_aerr		; 2 access error
	rept	29
	dc.l	unexp
	endr
	dc.l	h_trap0		; 32 TRAP #0
	rept	223
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$00008000,d0	; DTT0: user data, write-through
	movec	d0,dtt0
	move.l	#$0000A060,d0	; DTT1: supervisor data, noncachable imprecise
	movec	d0,dtt1
	moveq	#1,d0
	movec	d0,sfc
	movec	d0,dfc
	clr.l	(cnt_aerr).l
	move.l	#$A0808000,d0	; EDC, ESB, EBC, EIC
	movec	d0,cacr

;------------------------------------- 1-2 write-through, then read it back
	lea	(BUF).l,a0
	move.l	#$11223344,d0
	moves.l	d0,(a0)			; buffered (a write miss: not allocated)
	moves.l	(a0),d1			; read miss: after the write is out
	chkl	d1,$11223344,1
	move.l	(BUF).l,d2		; memory itself
	chkl	d2,$11223344,2

;---------------------------------- 3-4 a misaligned long: three bus cycles
	move.l	#$EEEEEEEE,(BUF+8).l
	move.l	#$EEEEEEEE,(BUF+12).l
	move.l	#$A1B2C3D4,d0
	lea	(BUF+9).l,a0
	moves.l	d0,(a0)
	move.l	(BUF+8).l,d2
	chkl	d2,$EEA1B2C3,3
	move.l	(BUF+12).l,d2
	chkl	d2,$D4EEEEEE,4

;------------------------------------------ 5-6 no read passes a write
	move.l	#$600DF00D,($F294).l
	move.l	($F2A8).l,d0
	chkl	d0,$600DF00D,5
	move.l	#$0BADCAFE,($F294).l
	move.l	($F2A8).l,d0
	chkl	d0,$0BADCAFE,6

;------------------------------------ 7 code written through a full buffer
	moveq	#0,d0
	move.l	#1,(AMX).l
	move.l	#2,(AMX+4).l
	move.l	#3,(AMX+8).l
	move.l	#$70074E75,(CODE).l	; moveq #7,d0 ; rts
	cinva	ic
	jsr	(CODE).l
	chkl	d0,7,7

;---------------------------------- 8-12 a TEA on a buffered write: SBE
	move.w	#1,(BERRARM).l		; (at reset it is armed already)
	clr.w	(cnt_aerr).l
	moveq	#0,d5
	move.l	#$12345678,(BERRAD).l	; completes; its bus cycle fails later
s8:	addq.l	#1,d5
	addq.l	#1,d5
	addq.l	#1,d5
	addq.l	#1,d5
	addq.l	#1,d5
	addq.l	#1,d5
	addq.l	#1,d5
	addq.l	#1,d5
	nop				; waits for the buffer: the error is in
s8e:	chkw	cnt_aerr,1,8
	move.l	(last_fslw).l,d0
	and.l	#F_SBE,d0
	chkl	d0,F_SBE,9
	chkw	last_vec,$4008,10
	; the stacked PC: after the store, at the latest after the NOP; the
	; ADDQs (two bytes each) before it had run, none after it
	move.l	(last_pc).l,d0
	cmp.l	#s8,d0
	bcs.s	.f11
	cmp.l	#s8e,d0
	bhi.s	.f11
	sub.l	#s8,d0
	lsr.l	#1,d0
	cmp.l	#8,d0
	bls.s	.p11
	moveq	#8,d0			; past the ADDQs: the NOP
.p11:	cmp.l	(last_d5).l,d0
	beq.s	.ok11
.f11:	failt	11
.ok11:	chkl	d5,8,12

;------------------------- 13-15 the TEA found while a TRAP waits: replaced
	move.w	#1,(BERRARM).l
	clr.w	(cnt_aerr).l
	clr.w	(cnt_trap).l
	move.l	#4,(AMX).l
	move.l	#5,(AMX+4).l
	move.l	#6,(AMX+8).l
	move.l	#$87654321,(BERRAD).l
	trap	#0
s13:	chkw	cnt_aerr,1,13
	chkw	cnt_trap,0,14
	move.l	(last_pc).l,d0
	chkl	d0,s13,15

;------------------------------------------- 16-17 ESB clear: precise
	move.l	#$80808000,d0
	movec	d0,cacr
	move.w	#1,(BERRARM).l
	clr.w	(cnt_aerr).l
s16:	move.l	#$0F0F0F0F,(BERRAD).l	; faults here; RTE restarts it (disarmed)
	chkw	cnt_aerr,1,16
	move.l	(last_pc).l,d0
	chkl	d0,s16,16
	move.l	(last_fslw).l,d0
	and.l	#F_SBE|F_WE,d0
	chkl	d0,F_WE,17

;----------------------------------------------------------------- done
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

h_aerr:
	addq.w	#1,(cnt_aerr).l
	move.w	6(sp),(last_vec).l
	move.l	2(sp),(last_pc).l
	move.l	12(sp),(last_fslw).l
	move.l	d5,(last_d5).l
	rte

h_trap0:
	addq.w	#1,(cnt_trap).l
	rte

unexp:
	failt	99
