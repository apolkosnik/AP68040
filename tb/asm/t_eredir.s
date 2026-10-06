; AP68040-60: mispredicted branches redirect the front end from EX
; assembled with vasmm68k_mot -Fbin -m68040
;
; A mispredicted branch that WB redirects for no other reason turns the
; front end round as it leaves EX; the stages behind it are flushed when it
; completes at WB.  Each case puts a mispredicted branch next to something
; that could overtake or undo that early redirect:
;   1-4    a store that takes a bus error just before the branch (format $7,
;          the store restarted, the branch then taken, its fall-through never
;          executed), at four alignments
;   5-6    the same with a load
;   7-9    trace on every instruction across a mispredicted branch: the
;          frames' PCs, in order
;   10-13  DBRA leaving its loop (a mispredict): the counter it wrote, used
;          at once as data and as an index, for several trip counts
;   14     eight mispredicted branches back to back, three passes
;   15     a return behind a mispredicted branch that skipped another
;          return (the return stack must not lose the caller)
;   16-17  a level 2 interrupt arriving at every clock from 1 to 80 across a
;          run of branches that alternate direction: every path taken once,
;          the interrupt taken once
;
; The bench rejects the first access to $F140 after $F142 is written.
; Supervisor data is noncachable (DTT1), so every access reaches it.

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
BERRREG	equ	$F140
BERRCTL	equ	$F142
IPLDLY	equ	$F148

nberr	equ	$3600		; bus errors taken
berr_pc	equ	$3604		; stacked PC of the last one
berr_fa	equ	$3608		; its fault address
ntrace	equ	$360C		; trace frames recorded
nirq	equ	$3610		; level 2 interrupts taken
trbuf	equ	$3700		; trace PCs (16)

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

chkm	macro
	cmp.l	#\2,(\1).l
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	h_berr		; 2 access fault
	rept	6
	dc.l	unexp
	endr
	dc.l	h_trace		; 9 trace
	rept	16
	dc.l	unexp
	endr
	dc.l	h_irq2		; 26 level 2 autovector
	rept	229
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$0000A040,d0		; DTT1: supervisor data noncachable
	movec	d0,dtt1			; (the bench registers)
	move.l	#$80008000,d0
	movec	d0,cacr
	clr.l	(nberr).l
	clr.l	(ntrace).l
	clr.l	(nirq).l
	move.w	#$2700,sr

;------------------------------------------- 1-4 store bus error + branch
; the store sets Z (d0 = 0), the forward BEQ is taken (statically
; predicted not taken): the early redirect races the store's fault
sbe	macro
	move.w	#1,(BERRCTL).l
	moveq	#0,d0
	rept	\1
	nop
	endr
.st\@:	move.l	d0,(BERRREG).l
	beq.s	.tk\@
	failt	\2
.tk\@:	chkm	berr_pc,.st\@,\2
	endm

	sbe	0,1
	chkm	nberr,1,1
	sbe	1,2
	sbe	2,3
	sbe	3,4
	chkm	nberr,4,4
	chkm	berr_fa,BERRREG,4

;-------------------------------------------- 5-6 load bus error + branch
lbe	macro
	move.w	#1,(BERRCTL).l
	moveq	#0,d1
	rept	\1
	nop
	endr
.ld\@:	move.l	(BERRREG).l,d0
	tst.l	d1
	beq.s	.tk\@
	failt	\2
.tk\@:	chkm	berr_pc,.ld\@,\2
	endm

	lbe	0,5
	lbe	1,6
	chkm	nberr,6,6

;------------------------------------------------------ 7-9 trace (T1)
	clr.l	(ntrace).l
	moveq	#0,d0
	move.w	#$A700,sr		; trace from the next instruction
tr0:	tst.l	d0
tr1:	beq.s	tr3			; taken, predicted not taken
tr2:	failt	7
tr3:	moveq	#1,d0
tr4:	bne.s	tr6			; taken again
tr5:	failt	8
tr6:	move.w	#$2700,sr		; traced: the frame after it
tr7:
	chkm	ntrace,5,9
	chkm	trbuf,tr1,9
	chkm	trbuf+4,tr3,9
	chkm	trbuf+8,tr4,9
	chkm	trbuf+12,tr6,9
	chkm	trbuf+16,tr7,9

;-------------------------------------------- 10-13 DBRA leaving its loop
dbx	macro
	moveq	#\1,d0
	moveq	#0,d2
.lp\@:	addq.l	#1,d2
	dbra	d0,.lp\@
	; the falling-through DBRA wrote d0.w = $FFFF; use it at once
	move.l	d0,d1
	cmp.w	#$FFFF,d1
	beq.s	.a\@
	failt	\2
.a\@:	lea	(dbtab+1).l,a0
	move.b	(a0,d0.w),d3		; dbtab[0]
	cmp.b	#$5A,d3
	beq.s	.b\@
	failt	\2
.b\@:	chkl	d2,\1+1,\2
	endm

	dbx	0,10
	dbx	1,11
	dbx	2,12
	dbx	7,13

;------------------------------------- 14 back-to-back mispredicts
	moveq	#2,d4
	moveq	#0,d5
bb0:	moveq	#0,d0
	beq.s	bb1
	failt	14
bb1:	addq.l	#1,d5
	beq.s	bb2			; Z clear: not taken
	nop
bb2:	tst.l	d0
	beq.s	bb3
	failt	14
bb3:	addq.l	#1,d5
	tst.l	d0
	beq.s	bb4
	failt	14
bb4:	addq.l	#1,d5
	tst.l	d0
	beq.s	bb5
	failt	14
bb5:	addq.l	#1,d5
	tst.l	d0
	beq.s	bb6
	failt	14
bb6:	addq.l	#1,d5
	tst.l	d0
	beq.s	bb7
	failt	14
bb7:	addq.l	#1,d5
	tst.l	d0
	beq.s	bb8
	failt	14
bb8:	addq.l	#1,d5
	dbra	d4,bb0
	chkl	d5,21,14

;--------------------------------------- 15 return behind a skipped return
	moveq	#3,d4
	moveq	#0,d5
rs0:	moveq	#0,d0
	bsr	rsf			; returns to rs1
rs1:	addq.l	#1,d5
	moveq	#1,d0
	bsr	rsf
	addq.l	#1,d5
	dbra	d4,rs0
	chkl	d5,8,15

;------------------------------------ 16-17 interrupts across mispredicts
; for each delay: arm a level 2 interrupt, run 16 branches whose direction
; alternates with the pass, count the paths; the handler counts itself
	moveq	#1,d6			; delay
ir0:	clr.l	(nirq).l
	moveq	#0,d5
	moveq	#15,d4
	move.w	#$2000,sr		; interrupts on
	move.w	d6,(IPLDLY).l
ir1:	btst	#0,d4
	beq.s	ir2
	addq.l	#1,d5
	bra.s	ir3
ir2:	addq.l	#2,d5
ir3:	btst	#1,d4
	bne.s	ir4
	addq.l	#4,d5
ir4:	dbra	d4,ir1
	moveq	#99,d0			; let a late interrupt arrive
ir5:	dbra	d0,ir5
	move.w	#$2700,sr
	chkl	d5,8*1+8*2+8*4,16
	chkm	nirq,1,17
	addq.l	#1,d6
	cmp.l	#81,d6
	bne	ir0

;----------------------------------------------------------------- done
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	#$2700,sr
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

; d0 = 0: the BEQ is taken past the first RTS (mispredicted the first
; time); either way one RTS returns to the caller
rsf:	tst.l	d0
	beq.s	rsf1
	rts
rsf1:	nop
	rts

h_berr:
	cmpi.w	#$7008,6(sp)		; format $7, vector 2
	bne.s	hb_bad
	addq.l	#1,(nberr).l
	move.l	2(sp),(berr_pc).l
	move.l	$14(sp),(berr_fa).l
	rte
hb_bad:	failt	97

h_trace:
	move.l	d0,-(sp)
	move.l	(ntrace).l,d0
	cmp.l	#16,d0
	bcc.s	ht1
	lsl.l	#2,d0
	move.l	6(sp),(trbuf,d0.l)
ht1:	addq.l	#1,(ntrace).l
	move.l	(sp)+,d0
	rte

h_irq2:
	clr.w	(IPLREG).l
	addq.l	#1,(nirq).l
	rte

unexp:
	failt	99

	cnop	0,4
dbtab:	dc.b	$5A,$A5,$00,$00
