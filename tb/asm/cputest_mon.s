; AP68040-60 cputest replay monitor, run by tb_cputest.sv.
; assembled with vasmm68k_mot -Fbin -m68040, loaded at $42110000
;
; The bench writes a round's input state into the mailbox and sets GO; the
; monitor loads it and enters the tested instruction through a format $0
; RTE whose frame the bench placed at the round's ISP - 8 (the native
; cputest runner enters every test through an RTE too).  Every exception
; vector points at a stub that records its vector number and saves the
; registers with absolute addressing only -- the corpus stack holds the
; frame under test and must not be touched -- then tells the bench (a
; bus-visible write of EVT) and waits for its command: resume through RTE
; (a trace stacked on a primary exception) or stop (the round's result).

MBOX	equ	$42120000
GO	equ	MBOX+$000	; word: bench -> monitor, start a round
IN_D	equ	MBOX+$004	; D0-D7/A0-A6, 60 bytes
IN_USP	equ	MBOX+$040
IN_FRM	equ	MBOX+$044	; ISP for the entry RTE (the frame)
IN_MSP	equ	MBOX+$048
IN_VBR	equ	MBOX+$04C
EVT	equ	MBOX+$100	; word: monitor -> bench, an exception entry
EVT_VEC	equ	MBOX+$102	; word: its vector number
CAP_D	equ	MBOX+$104	; D0-D7/A0-A6 at the handler entry
CAP_USP	equ	MBOX+$140
CAP_SP	equ	MBOX+$144	; the active stack pointer: the frame
CAP_MSP	equ	MBOX+$148
CMD	equ	MBOX+$200	; word: bench -> monitor, 1 resume, 2 stop
MSTACK	equ	$42130000

	org	$42110000

; $42110000 + 16 * n: the handler of vector n
n	set	0
	rept	256
	move.w	#n,(EVT_VEC).l
	jmp	(capture).l
	nop
n	set	n+1
	endr

; $42111000
capture:
	movem.l	d0-d7/a0-a6,(CAP_D).l
	move.l	usp,a0
	move.l	a0,(CAP_USP).l
	move.l	a7,(CAP_SP).l
	movec	msp,a0
	move.l	a0,(CAP_MSP).l
	move.w	#1,(EVT).l
cwait:
	move.w	(CMD).l,d0
	beq.s	cwait
	clr.w	(CMD).l
	cmp.w	#1,d0
	bne.s	stop
	movem.l	(CAP_D).l,d0-d7/a0-a6
	rte
stop:
	move.w	#$2700,sr
	lea	(MSTACK).l,sp
	bra.s	iwait

; the reset entry
idle:
	move.w	#$2700,sr
	lea	(MSTACK).l,sp
iwait:
	tst.w	(GO).l
	beq.s	iwait
	clr.w	(GO).l
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,tc
	movec	d0,itt0
	movec	d0,itt1
	movec	d0,dtt0
	movec	d0,dtt1
	movec	d0,sfc
	movec	d0,dfc
	pflusha
	move.l	(IN_USP).l,a0
	move.l	a0,usp
	move.l	(IN_MSP).l,a0
	movec	a0,msp
	move.l	(IN_VBR).l,a0
	movec	a0,vbr
	movem.l	(IN_D).l,d0-d7/a0-a6
	move.l	(IN_FRM).l,a7
	rte
