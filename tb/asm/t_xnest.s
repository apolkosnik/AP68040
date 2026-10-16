; AP68040 nested exceptions: a handler whose fetch fails, and double faults
; assembled with vasmm68k_mot -Fbin -m68040
;
; MC68040UM 8.1: exception processing ends when the first instruction of
; the handler begins to execute.  7.6.3, 8.2.1, 8.2.2, figure 8-1: an
; access error or address error during the processing of an access error,
; an address error or a reset -- the vector fetch, the frame's writes, the
; fetch of the handler's first instruction -- is a double bus fault: the
; processor halts (PST $5) until a reset.  During the processing of any
; other exception a failed fetch of the handler is an ordinary access
; error.  (An interrupt pending meanwhile: t_exceptions 93-105.)
;
; The bench answers a halt with a reset ($F158 says how many times) and
; counts the resets in $F15A, which tells the program its phase.  $F156
; makes every instruction fetch of a long word end in TEA (until the reset
; of a halted processor); addresses past the bench's memory always do.
;
; phase 0
;   1    TRAP #0 to a handler whose fetch fails: an ordinary access error,
;        format $7: FA and the stacked PC the handler's address, the SSW
;        a supervisor code read (RW, TM 6)
;   then an access error whose handler's fetch fails: halt
; phase 1
;   2    the access error handler never ran
;   then an address error whose handler's fetch fails: halt
; phase 2
;   3    the address error handler never ran
;   then an access error whose frame's writes fail (the stack past the
;        memory): halt
; phase 3
;   4    no handler ran
;   then the same for an address error: halt
; phase 4
;   5    no handler ran
;   then a reset whose initial PC's fetch fails: halt
; phase 6 (the timed reset counts too)
;   6    the processor halted in the reset's processing: its reset found
;        the initial PC fetchable and ran the code there
;
;   97   a halt was due
;   98   an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
FTEA	equ	$F156
HALTRST	equ	$F158
HALTCNT	equ	$F15A
TRESET	equ	$F168

BADH	equ	$2000		; the handler whose fetch fails ($F156)
NOMEM	equ	$00200000	; past the bench's memory: TEA

cnt_trap	equ	$3600
cnt_ae		equ	$3604
cnt_adr		equ	$3606
ae_fa		equ	$3608
ae_ssw		equ	$360C
ae_pc		equ	$3610
b3flag		equ	$3614

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkw	macro			; memory word, value, test
	cmp.w	#\2,(\1).l
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

chkl	macro			; memory long, value, test
	cmp.l	#\2,(\1).l
	beq.s	.ok\@
	failt	\3
.ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	h_ae		; 2 access error
	dc.l	h_adr		; 3 address error
	rept	28
	dc.l	unexp
	endr
	dc.l	h_trap		; 32 TRAP #0
	rept	223
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#$2700,sr
	move.l	#$3400,sp
	moveq	#0,d0
	movec	d0,cacr
	move.l	#h_ae,($8).l
	move.l	#h_adr,($C).l
	move.w	(HALTCNT).l,d0
	beq	phase0
	cmp.w	#1,d0
	beq	phase1
	cmp.w	#2,d0
	beq	phase2
	cmp.w	#3,d0
	beq	phase3
	cmp.w	#4,d0
	beq	phase4
	bra	phase6

;------------------------------------------------ phase 0
phase0:
	clr.w	(cnt_trap).l
	clr.w	(cnt_ae).l
	clr.w	(cnt_adr).l
	clr.w	(b3flag).l

; 1 TRAP #0 to a handler whose fetch fails: an access error
	move.w	#BADH,(FTEA).l
	move.l	#BADH,($80).l
	trap	#0
t1_ret:
	chkw	cnt_ae,1,1
	chkw	cnt_trap,0,1
	chkl	ae_fa,BADH,1
	chkl	ae_pc,BADH,1
	move.w	(ae_ssw).l,d0
	and.w	#$0107,d0		; RW, TM
	cmp.w	#$0106,d0
	beq.s	t1a
	failt	1
t1a:

; an access error whose handler's fetch fails: halt
	move.w	#1,(HALTRST).l
	move.w	#BADH,(FTEA).l
	move.l	#BADH,($8).l
	move.l	(NOMEM).l,d0
	failt	97

;------------------------------------------------ phase 1
phase1:
	chkw	cnt_ae,1,2
	move.w	#BADH,(FTEA).l
	move.w	#2,(HALTRST).l
	move.l	#BADH,($C).l
	jmp	($0401).l
	failt	97

;------------------------------------------------ phase 2
phase2:
	chkw	cnt_adr,0,3
	move.w	#3,(HALTRST).l
	move.l	#NOMEM+$100,sp		; the frame's writes fail
	move.l	(NOMEM).l,d0
	failt	97

;------------------------------------------------ phase 3
phase3:
	chkw	cnt_ae,1,4
	chkw	cnt_adr,0,4
	move.w	#4,(HALTRST).l
	move.l	#NOMEM+$100,sp
	jmp	($0401).l
	failt	97

;------------------------------------------------ phase 4
phase4:
	chkw	cnt_ae,1,5
	chkw	cnt_adr,0,5
	move.w	#6,(HALTRST).l		; the timed reset counts as one
	clr.w	(b3flag).l
	move.l	#BADH,($4).l		; the initial PC
	move.w	#BADH,(FTEA).l
	move.w	#61,(TRESET).l
busy:	bra.s	busy

;------------------------------------------------ phase 6
phase6:
	chkw	b3flag,1,6
	move.w	#$600D,(DONEREG).l
	bra.s	*

h_trap:
	addq.w	#1,(cnt_trap).l
	rte

; an ordinary access error (test 1): continue after the TRAP
h_ae:
	addq.w	#1,(cnt_ae).l
	tst.w	(HALTCNT).l		; after phase 0 a halt was due instead
	beq.s	.p0
	failt	97
.p0:
	move.l	2(sp),(ae_pc).l
	move.w	$C(sp),(ae_ssw).l
	move.l	$14(sp),(ae_fa).l
	move.w	#0,(FTEA).l
	move.l	#t1_ret,2(sp)
	rte

h_adr:
	addq.w	#1,(cnt_adr).l
	failt	98

fail_all:
	move.l	#$3400,sp
	move.w	#$2700,sr
	move.w	#0,(IPLREG).l
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	98

; the initial PC of phase 4's reset: fetchable after the reset that
; answered the halt
	org	BADH
	move.w	#1,(b3flag).l
	move.l	#start,($4).l
	jmp	(start).l
