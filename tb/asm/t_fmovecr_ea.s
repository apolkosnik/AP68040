; AP68040: FMOVECR's <ea> field (operation word bits 5-0).  With the field 0
; FMOVECR is an unimplemented floating-point instruction (vector 11, format
; $2, the PC of the next instruction).  Any other field is not an FMOVECR:
; the 68040 takes an F-line exception (vector 11, format $0, the PC of the
; instruction), which the M68040FPSP's x_fline.sa handles ("an fmovecr with
; a non-zero <ea> field": it rebuilds the six-word frame from the four-word
; one and the stacked PC of the F-line instruction).  GitHub issue 18.
; assembled with vasmm68k_mot -Fbin -m68040
;
;   1-2   F200 5C00 (FMOVECR #0,FP0): format/vector $202C, the next PC
;   3-4   F201 5C00 (<ea> field 1): $002C, the instruction's PC
;   5-6   F208 5C00 (<ea> field 8, A0): $002C, the instruction's PC
;   7-8   F23F 5C00 (<ea> field $3F): $002C, the instruction's PC
;   9     FPCR, FPSR untouched by the F-line cases
;   98    an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102

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

; one case: the operation word, its tests
fcase	macro			; operation word, format/vector, at the instruction (1) or past it (0), test
	lea	.r\@(pc),a1		; where the handler resumes
	lea	.i\@(pc),a2		; the instruction
	moveq	#-1,d1
	moveq	#-1,d2
.i\@:	dc.w	\1,$5C00
	nop
.r\@:	chkl	d1,\2,\4
	lea	4(a2),a3
	if \3
	move.l	a2,a3
	endif
	cmp.l	a3,d2
	beq.s	.p\@
	failt	\4+1
.p\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	9
	dc.l	unexp
	endr
	dc.l	h_fline			; 11
	rept	244
	dc.l	unexp
	endr

	org	$400
start:
	moveq	#0,d0
	fmove.l	d0,fpcr
	fmove.l	d0,fpsr
	fcase	$F200,$0000202C,0,1
	fcase	$F201,$0000002C,1,3
	fcase	$F208,$0000002C,1,5
	fcase	$F23F,$0000002C,1,7
	fmove.l	fpcr,d0
	fmove.l	fpsr,d1
	or.l	d1,d0
	chkl	d0,0,9
	move.w	#$600d,(DONEREG).l
.d:	bra.s	.d

; vector 11: the format/vector word into d1, the stacked PC into d2, resume
; at a1 (a format $2 frame's RTE pops its six words, a format $0's four)
h_fline:
	moveq	#0,d1
	move.w	6(sp),d1
	move.l	2(sp),d2
	move.l	a1,2(sp)
	rte

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$bad0,(DONEREG).l
.f:	bra.s	.f

unexp:
	move.w	#98,(FAILREG).l
	move.w	#$bad0,(DONEREG).l
.u:	bra.s	.u
