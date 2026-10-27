; AP68040: the FPU conditionals (FBcc, FScc, FDBcc, FTRAPcc) leave FPIAR
; alone.  MC68040UM 9.2.4: the FPU loads FPIAR for the instructions that
; can raise floating-point exceptions; 9.7.1: a conditional's only one is
; BSUN, and "for MC68881/MC68882 compatibility, the M68040FPSP updates the
; FPIAR by copying the PC value in the pre-instruction stack frame to the
; FPIAR" (x_bsun.sa does exactly that, for all four).  An arithmetic
; instruction does load it.  GitHub issue 19.
; assembled with vasmm68k_mot -Fbin -m68040
;
;   1     FBcc (FBF, not taken)
;   2     FScc to Dn (FSEQ)
;   3     FScc to memory (FSNE)
;   4     FDBcc counting out (FDBF, d2 = 0)
;   5     FDBcc taken (FDBF, d2 = 1, once round)
;   6     FTRAPcc, not taken (FTRAPF)
;   7     FTRAPcc.W #imm, not taken
;   8     FTRAPcc.L #imm, not taken
;   9-10  BSUN: FPCR.BSUN enabled, NAN set, FBGT: vector 48, the stacked PC
;         the FBGT; FPIAR unchanged in the handler (9) and after (10)
;   11    an arithmetic instruction (FADD) loads FPIAR with its address
;   98    an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
MARK	equ	$12345678

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

same	macro			; test: FPIAR still MARK
	fmove.l	fpiar,d0
	cmp.l	#MARK,d0
	beq.s	.ok\@
	failt	\1
.ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	46
	dc.l	unexp
	endr
	dc.l	h_bsun			; 48
	rept	207
	dc.l	unexp
	endr

	org	$400
start:
	moveq	#0,d0
	fmove.l	d0,fpcr
	fmove.l	d0,fpsr
	move.l	#MARK,d0
	fmove.l	d0,fpiar
	fbf	.b1			; 1 FBcc
.b1:	same	1
	fseq	d1			; 2 FScc Dn
	same	2
	fsne	($2000).l		; 3 FScc memory
	same	3
	moveq	#0,d2
	fdbf	d2,.b4			; 4 FDBcc counts out
.b4:	same	4
	moveq	#1,d2
.l5:	fdbf	d2,.l5			; 5 FDBcc taken once, then out
	same	5
	ftrapf				; 6 FTRAPcc
	same	6
	ftrapf.w #$1234			; 7
	same	7
	ftrapf.l #$12345678		; 8
	same	8

	; 9-10 BSUN: enabled, NAN set; FBGT is IEEE nonaware
	moveq	#0,d6
	move.l	#$00008000,d0		; FPCR: BSUN enabled
	fmove.l	d0,fpcr
	move.l	#$01000000,d0		; FPSR: NAN
	fmove.l	d0,fpsr
.bs:	fbgt	.bs2
.bs2:	moveq	#0,d0
	fmove.l	d0,fpcr
	fmove.l	d0,fpsr
	cmpi.l	#1,d6			; the handler ran once
	beq.s	.bs3
	failt	9
.bs3:	lea	.bs(pc),a0
	cmp.l	d5,a0			; the stacked PC: the FBGT
	beq.s	.bs4
	failt	9
.bs4:	cmp.l	#MARK,d4		; FPIAR in the handler
	beq.s	.bs5
	failt	9
.bs5:	same	10

	; 11 an arithmetic instruction loads FPIAR
	fmove.l	#1,fp0
.ar:	fadd.x	fp0,fp0
	fmove.l	fpiar,d0
	lea	.ar(pc),a0
	cmp.l	a0,d0
	beq.s	.ok11
	failt	11
.ok11:
	move.w	#$600d,(DONEREG).l
.d:	bra.s	.d

; vector 48: count, take FPIAR and the stacked PC, disable BSUN (the
; FBGT restarts and is evaluated as the IEEE aware GT)
h_bsun:
	addq.l	#1,d6
	fmove.l	fpiar,d4
	move.l	2(sp),d5		; the stacked PC
	moveq	#0,d0
	fmove.l	d0,fpcr
	rte

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$bad0,(DONEREG).l
.f:	bra.s	.f

unexp:
	move.w	#98,(FAILREG).l
	move.w	#$bad0,(DONEREG).l
.u:	bra.s	.u
