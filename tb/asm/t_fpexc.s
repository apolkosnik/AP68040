; AP68040 FPU arithmetic exceptions: when they are taken, and the frame
; FSAVE gives the handler (MC68040UM 9.7, table 9-16; WinUAE fpp.cpp
; fpsr_check_arithmetic_exception and fp_exception_pending, the oracle)
; assembled with vasmm68k_mot -Fbin -m68040
;
; The handler (vectors 49-54) FSAVEs into the buffer A5 names, records the
; stacked PC, the format/vector word and the integer marker as it finds
; them, counts, and returns.  A pre-instruction exception returns to the
; FPU instruction that took it, which then runs (FSAVE took the state).
;   1-7   enabled DZ of an FDIV that completes in EX: pending, taken at the
;         next FPU instruction (FNOP), not before the integer store between
;         them: the marker is written, the PC is FNOP's, format $0; the
;         frame is E1 ($41/$30) with the source in ETEMP and the dyadic
;         operation's destination in FPTEMP
;   8-10  enabled SNAN of an FADD, the same way
;   11-15 enabled OPERR of FMOVE.W FP0,<ea> (an integer overflow): taken
;         after the store (format $3), and the handler's FSAVE gives the
;         E1/T frame with the store's command word and the register in
;         ETEMP -- not an IDLE frame
;   16-18 enabled INEX2 of an inexact FMOVE.S FP0,<ea>: the same
;   19-23 enabled INEX2 of FABS at single precision: FSAVE extracts the
;         pending E1 frame, FPTEMP the unrounded intermediate
;   24    the same with INEX2 disabled: nothing pending, an IDLE frame
;   25-33 enabled INEX2 of FMUL by 1 at single and double precision (and
;         single toward zero): the E3 BUSY frame's WBTEMP keeps the bits
;         below the precision (table 9-16; WinUAE
;         getFloatInternalRoundedSome), not the rounded significand
;
;   40-52 enabled SNAN, OPERR, DZ: the condition codes are set from the
;         default result that is not written (WinUAE fp_arithmetic sets
;         them before fpsr_check_arithmetic_exception; the 68060 does the
;         same, Motorola's ftest.sa), the destination is kept, the
;         exception is taken by the next FPU instruction, once
;
;   98   an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
cnt	equ	$3600
rec_pc	equ	$3604
rec_fv	equ	$3608
rec_mk	equ	$360C
marker	equ	$3610
fbuf	equ	$3700

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
	rept	46
	dc.l	unexp
	endr
	rept	7
	dc.l	h_fp		; 48-54 (48, BSUN, is set unexpected below)
	endr
	rept	201
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#unexp,($C0).w	; vector 48 (BSUN)
	lea	(fbuf).l,a5
	clr.l	(cnt).l
	fmove.l	#0,fpcr

;------------------------------------------------ 1-7 DZ pends to the next FPU instruction
	fmove.l	#6,fp0
	fmove.l	#0,fp1
	clr.l	(marker).l
	fmove.l	#$400,fpcr	; enable DZ
	fdiv.x	fp1,fp0
	move.l	#$12345678,(marker).l
dz_sync:
	fnop			; the pending DZ is taken here
	fmove.l	#0,fpcr
	move.l	(cnt).l,d0
	chkl	d0,1,1
	move.l	(rec_mk).l,d0
	chkl	d0,$12345678,2	; the integer store ran first
	move.l	(rec_pc).l,d0
	chkl	d0,dz_sync,3	; pre-instruction: the FNOP's PC
	move.w	(rec_fv).l,d0
	and.l	#$F000,d0
	chkl	d0,0,4		; format $0
	move.l	(fbuf).l,d0
	chkl	d0,$41300000,5	; an E1 frame
	move.l	(fbuf+$1C).l,d0
	chkl	d0,$40010000,6	; FPTEMP: the destination, 6.0
	move.l	(fbuf+$20).l,d0
	chkl	d0,$C0000000,6
	fmove.l	fp0,d0
	chkl	d0,6,7		; the destination not written

;------------------------------------------------ 8-10 SNAN pends to the next FPU instruction
	fmove.l	#6,fp0
	fmovem.x	snan_x,fp1
	clr.l	(marker).l
	fmove.l	#$4000,fpcr	; enable SNAN
	fadd.x	fp1,fp0
	move.l	#$9ABCDEF0,(marker).l
sn_sync:
	fnop
	fmove.l	#0,fpcr
	move.l	(cnt).l,d0
	chkl	d0,2,8
	move.l	(rec_mk).l,d0
	chkl	d0,$9ABCDEF0,9
	move.l	(rec_pc).l,d0
	chkl	d0,sn_sync,10

;------------------------------------------------ 11-15 an OPERR store's frame
	fmove.l	#$7FFFFFFF,fp0
	move.l	#$CAFEBABE,($3100).l
	fmove.l	#$2000,fpcr	; enable OPERR
	fmove.w	fp0,($3100).l	; overflows a word: OPERR, nothing written
	fmove.l	#0,fpcr
	move.l	(cnt).l,d0
	chkl	d0,3,11
	move.w	(rec_fv).l,d0
	and.l	#$F000,d0
	chkl	d0,$3000,12	; post-instruction, format $3
	move.l	(fbuf).l,d0
	chkl	d0,$41300000,13	; not IDLE
	move.l	(fbuf+$18).l,d0
	and.l	#$04100000,d0
	chkl	d0,$04100000,13	; E1 and T
	move.l	(fbuf+$10).l,d0
	and.l	#$FFFF0000,d0
	chkl	d0,$70000000,14	; CMDREG1B: FMOVE.W FP0,<ea>
	move.l	(fbuf+$28).l,d0
	chkl	d0,$401D0000,15	; ETEMP: the register, 2^31-1

;------------------------------------------------ 16-18 an inexact single store's frame
	fmovem.x	inex_x,fp0
	fmove.l	#$200,fpcr	; enable INEX2
	fmove.s	fp0,($3100).l
	fmove.l	#0,fpcr
	move.l	(cnt).l,d0
	chkl	d0,4,16
	move.l	(fbuf).l,d0
	chkl	d0,$41300000,17
	move.l	(fbuf+$18).l,d0
	and.l	#$04100000,d0
	chkl	d0,$04100000,18

;------------------------------------------------ 19-23 FABS's E1 frame: the intermediate
	fmovem.x	inex_x,fp0
	fmove.l	#$240,fpcr	; single precision, INEX2 enabled
	fabs.x	fp0
	fsave	(a5)		; takes the pending state, no trap
	fmove.l	#0,fpcr
	move.l	(cnt).l,d0
	chkl	d0,4,19		; nothing taken
	move.l	(fbuf).l,d0
	chkl	d0,$41300000,20
	move.l	(fbuf+$1C).l,d0
	chkl	d0,$3FFF0000,21	; FPTEMP: the unrounded intermediate
	move.l	(fbuf+$20).l,d0
	chkl	d0,$80000080,22
	move.l	(fbuf+$24).l,d0
	chkl	d0,0,23

;------------------------------------------------ 24 the same, INEX2 disabled
	fmovem.x	inex_x,fp0
	fmove.l	#$40,fpcr	; single precision
	fabs.x	fp0
	fsave	(a5)
	fmove.l	#0,fpcr
	move.l	(fbuf).l,d0
	chkl	d0,$41000000,24	; IDLE

	; 25-33: WBTEMP of an inexact multiply (FSAVE extracts the pend)
	fmovem.x	inex_x,fp0
	fmove.l	#1,fp1
	fmove.l	#$240,fpcr	; single, nearest, INEX2 enabled
	fmul.x	fp1,fp0
	fsave	(a5)
	fmove.l	#0,fpcr
	move.l	(fbuf).l,d0
	chkl	d0,$41600000,25	; BUSY
	move.l	(fbuf+$1C).l,d0
	chkl	d0,$80000080,26	; WBTM high: the bits below single kept
	move.l	(fbuf+$20).l,d0
	chkl	d0,0,27
	fmovem.x	inexd_x,fp0
	fmove.l	#$280,fpcr	; double, nearest, INEX2 enabled
	fmul.x	fp1,fp0
	fsave	(a5)
	fmove.l	#0,fpcr
	move.l	(fbuf).l,d0
	chkl	d0,$41600000,28
	move.l	(fbuf+$1C).l,d0
	chkl	d0,$80000000,29
	move.l	(fbuf+$20).l,d0
	chkl	d0,$00000400,30	; the bits below double kept
	fmovem.x	inex_x,fp0
	fmove.l	#$250,fpcr	; single, toward zero, INEX2 enabled
	fmul.x	fp1,fp0
	fsave	(a5)
	fmove.l	#0,fpcr
	move.l	(fbuf).l,d0
	chkl	d0,$41600000,31
	move.l	(fbuf+$18).l,d0
	chkl	d0,$3FFF0000,32	; WBTS/WBTE
	move.l	(fbuf+$1C).l,d0
	chkl	d0,$80000080,33

;------------------------------------------------ 40-52 enabled SNAN, OPERR, DZ: condition codes
	; SNAN: -SNAN + 2: N and NAN, the quiet NaN keeping the sign
	move.l	(cnt).l,d6
	fmove.l	#$4000,fpcr
	fmovem.x	cc_snan,fp0	; (FMOVEM: loaded as it is)
	fmove.l	#0,fpsr
	fadd.b	#2,fp0
	fnop			; vector 54
	fmove.l	fpsr,d0
	chkl	d0,$09004080,40
	move.l	(cnt).l,d0
	sub.l	d6,d0
	chkl	d0,1,41
	fmove.l	#0,fpcr
	fmovem.x	fp0,(fbuf+$40).l
	move.l	(fbuf+$40).l,d0
	chkl	d0,$FFFF0000,42	; the destination kept
	move.l	(fbuf+$48).l,d0
	chkl	d0,$00000001,43
	; OPERR: -inf + +inf: NAN alone, the default NaN is positive
	move.l	(cnt).l,d6
	fmove.l	#$2000,fpcr
	fmovem.x	cc_minf,fp0
	fmove.l	#0,fpsr
	fadd.s	#$7F800000,fp0
	fnop			; vector 52
	fmove.l	fpsr,d0
	chkl	d0,$01002080,45
	move.l	(cnt).l,d0
	sub.l	d6,d0
	chkl	d0,1,46
	fmove.l	#0,fpcr
	fmovem.x	fp0,(fbuf+$40).l
	move.l	(fbuf+$40).l,d0
	chkl	d0,$FFFF0000,47	; -inf kept
	move.l	(fbuf+$44).l,d0
	chkl	d0,$00000000,48
	; DZ: 2 / 0: I, the default infinity positive
	move.l	(cnt).l,d6
	fmove.l	#$0400,fpcr
	fmovem.x	cc_two,fp0
	fmove.l	#0,fpsr
	fdiv.b	#0,fp0
	fnop			; vector 50
	fmove.l	fpsr,d0
	chkl	d0,$02000410,50
	move.l	(cnt).l,d0
	sub.l	d6,d0
	chkl	d0,1,51
	fmove.l	#0,fpcr
	fmovem.x	fp0,(fbuf+$40).l
	move.l	(fbuf+$40).l,d0
	chkl	d0,$40000000,52	; 2 kept
	fmove.l	#0,fpsr

	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	fmove.l	#0,fpcr
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

h_fp:
	fsave	(a5)
	move.l	2(sp),(rec_pc).l
	move.w	6(sp),(rec_fv).l
	move.l	(marker).l,(rec_mk).l
	addq.l	#1,(cnt).l
	rte

unexp:
	failt	98

	even
snan_x:	dc.l	$7FFF0000,$A0000000,1
cc_snan:	dc.l	$FFFF0000,$00000000,$00000001
cc_minf:	dc.l	$FFFF0000,$00000000,$00000000
cc_two:	dc.l	$40000000,$80000000,$00000000
inex_x:	dc.l	$3FFF0000,$80000080,0
inexd_x:	dc.l	$3FFF0000,$80000000,$00000400
