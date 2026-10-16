; AP68060 nested exceptions: the first handler instruction and double faults
; assembled with vasmm68k_mot -Fbin -m68040
;
; MC68060UM 8.1, 8.2.9, 8.3: interrupts are not sampled from the start of an
; exception's processing until the first instruction of its handler has
; executed, and that instruction can raise the mask to keep one out.
; Figures 8-1 and 8-5, 8.2.1, 7.9.3: the fetch of the handler's first
; instruction is part of the processing; an access or address error during
; the processing of an access error, an address error or reset -- a bus
; error on a buffered write of its frame included -- halts the processor.
;
; The bench answers a halt with a reset ($F158 says how many times) and
; counts the resets in $F15A, which tells the program its phase.  $F156
; makes every instruction fetch of a long word end in TEA (until the reset
; of a halted processor); addresses past the bench's 1 Mbyte always do.
;
; phase 0
;   1-2  TRAP #0 whose handler's first instruction raises the mask to 7,
;        IPL 2 raised as the TRAP starts stacking (1) and once its vector
;        is read (2) ($F144): no interrupt
;   3    the same with a handler that leaves the mask: the interrupt comes
;        after its first instruction
;   4    TRAP #0 to a handler whose fetch fails: an ordinary access error
;        (FA the handler's address, FSLW IO)
;   then an access error whose handler's fetch fails: halt
; phase 1
;   5    the access error handler never ran
;   then an address error whose handler's fetch fails: halt
; phase 2
;   6    the address error handler never ran
;   then, CACR.ESB on, an access error whose frame's buffered writes fail
;        (the stack past the memory): halt
; phase 3
;   7    no handler ran
;   then the same for an address error: halt
; phase 4
;   8    no handler ran
;   then a reset whose initial PC's fetch fails: halt
; phase 6 (the timed reset counts too)
;   9    the processor halted in the reset's processing: its reset found
;        the initial PC fetchable and ran the code there
;
;   98   an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
IPLREG	equ	$F110
BERRARM	equ	$F142
IRQEXC	equ	$F144
FTEA	equ	$F156
HALTRST	equ	$F158
HALTCNT	equ	$F15A
TRESET	equ	$F168

BADH	equ	$2000		; the handler whose fetch fails ($F156)
NOMEM	equ	$00200000	; past the bench's memory: TEA

cnt_trap	equ	$3600
cnt_int2	equ	$3602
cnt_ae		equ	$3604
cnt_adr		equ	$3606
ae_fa		equ	$3608
ae_fslw		equ	$360C
b3flag		equ	$3610

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

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	h_ae		; 2 access error
	dc.l	h_adr		; 3 address error
	rept	22
	dc.l	unexp
	endr
	dc.l	h_int2		; 26 level 2 autovector
	rept	5
	dc.l	unexp
	endr
	dc.l	h_mask		; 32 TRAP #0
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
	clr.w	(cnt_int2).l
	clr.w	(cnt_ae).l
	clr.w	(cnt_adr).l
	clr.w	(b3flag).l

; 1-2 the handler's first instruction masks the interrupt
	move.l	#h_mask,($80).l
	move.w	#$2000,sr
	move.w	#1,(IRQEXC).l		; IPL 2 as TRAP #0 starts stacking
	trap	#0
	bsr	settle
	chkw	cnt_int2,0,1
	move.w	#$2000,sr
	move.w	#2,(IRQEXC).l		; ... once its vector is read
	trap	#0
	bsr	settle
	chkw	cnt_int2,0,2
	chkw	cnt_trap,2,2

; 3 the control: the handler leaves the mask
	move.l	#h_nomask,($80).l
	move.w	#$2000,sr
	move.w	#1,(IRQEXC).l
	trap	#0
	bsr	settle
	chkw	cnt_int2,1,3
	move.w	#$2700,sr

; 4 TRAP #0 to a handler whose fetch fails: an access error
	move.w	#BADH,(FTEA).l
	move.l	#BADH,($80).l
	trap	#0
t4_ret:
	chkw	cnt_ae,1,4
	cmp.l	#BADH,(ae_fa).l
	beq.s	t4a
	failt	4
t4a:	move.l	(ae_fslw).l,d0
	btst	#15,d0			; FSLW.IO
	bne.s	t4b
	failt	4
t4b:

; an access error whose handler's fetch fails: halt
	move.w	#1,(HALTRST).l
	move.w	#BADH,(FTEA).l
	move.l	#BADH,($8).l
	move.l	(NOMEM).l,d0
	failt	97

;------------------------------------------------ phase 1
phase1:
	chkw	cnt_ae,1,5
	move.w	#BADH,(FTEA).l
	move.w	#2,(HALTRST).l
	move.l	#BADH,($C).l
	jmp	($0401).l
	failt	97

;------------------------------------------------ phase 2
phase2:
	chkw	cnt_adr,0,6
	move.w	#3,(HALTRST).l
	move.l	#$20000000,d0		; CACR.ESB
	movec	d0,cacr
	nop
	move.l	#NOMEM+$100,sp		; the frame's writes fail
	move.l	(NOMEM).l,d0
	failt	97

;------------------------------------------------ phase 3
phase3:
	chkw	cnt_ae,1,7
	chkw	cnt_adr,0,7
	move.w	#4,(HALTRST).l
	move.l	#$20000000,d0
	movec	d0,cacr
	nop
	move.l	#NOMEM+$100,sp
	jmp	($0401).l
	failt	97

;------------------------------------------------ phase 4
phase4:
	chkw	cnt_ae,1,8
	chkw	cnt_adr,0,8
	move.w	#6,(HALTRST).l		; the timed reset counts as one
	clr.w	(b3flag).l
	move.l	#BADH,($4).l		; the initial PC
	move.w	#BADH,(FTEA).l
	move.w	#61,(TRESET).l
busy:	bra.s	busy

;------------------------------------------------ phase 6
phase6:
	chkw	b3flag,1,9
	move.w	#$600D,(DONEREG).l
	bra.s	*

; IPL released, the time for a late interrupt
settle:
	move.w	#0,(IPLREG).l
	moveq	#40,d1
.s:	dbra	d1,.s
	rts

h_mask:
	move.w	#$2700,sr		; the first instruction: mask 7
	addq.w	#1,(cnt_trap).l
	move.w	#0,(IPLREG).l
	moveq	#40,d1
.w:	dbra	d1,.w
	rte

h_nomask:
	nop
	addq.w	#1,(cnt_trap).l
	rte

h_int2:
	addq.w	#1,(cnt_int2).l
	move.w	#0,(IPLREG).l
	nop
	rte

; an ordinary access error (test 4): continue after the TRAP
h_ae:
	addq.w	#1,(cnt_ae).l
	tst.w	(HALTCNT).l		; after phase 0 a halt was due instead
	beq.s	.p0
	failt	7
.p0:
	move.l	8(sp),(ae_fa).l
	move.l	12(sp),(ae_fslw).l
	move.w	#0,(FTEA).l
	move.l	#t4_ret,2(sp)
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
