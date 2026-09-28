; CINV after memory changed behind the CPU, and MOVES straight behind the
; MOVEC that sets its space.
;
; protocol with the testbench (tb_ap040_program.v, tb_ap040_pipe_program.v):
;   word write to $F100 = failing test number
;   word write to $F102 = $BAD0 on failure, $600D when all tests passed
;   word write to $F130 = DMA-style poke: $3500 := data, $3502 := 0, behind
;                         the CPU's back (no store the CPU could snoop)
;   byte write to $F120 must carry FC=1
;
; Test 1-4: code at $3500 is fetched, rewritten by the DMA poke, and run
; again. The poke is not a CPU store to $3500, so nothing but CINVA can make
; the new instruction visible: a prefetch stream, a queue or an I-cache that
; kept the old one runs MOVEQ #1 where MOVEQ #2 is now. The poke, CINVA
; and $3500 are straight together, behind two divides: the fetch reaches
; $3500 while the poke waits for them, so the old word is already in the
; pipeline and the prefetch stream when the poke's write goes out, and
; only CINVA's refetch -- from a stream it has emptied -- runs the new one.
; With the three further apart, the fetch on this 16-bit bus never got to
; $3500 before the poke landed, the new word simply arrived, and both CINV
; mutations survived the first two versions of this test.
;
; Test 5: MOVEC D0,DFC straight ahead of MOVES.B, so the MOVES reaches the
; point where it uses DFC as the MOVEC commits.

FAILREG		equ	$F100
DONEREG		equ	$F102
FCREG		equ	$F120
DMAPOKE		equ	$F130

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

	org	0
	dc.l	$3400		; initial ISP
	dc.l	start		; initial PC
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#$2700,sr
	move.l	#$80008000,d0	; caches on where the core has them
	movec	d0,cacr

;------------------------------------------------------------ CINVA, DMA
	moveq	#0,d0
	jsr	$34E4		; runs MOVEQ #1 at $3500 once: it is fetched
	cmp.l	#1,d0
	beq.s	t1ok
	failt	1
t1ok:
	moveq	#100,d2		; the divides below must run their full length
	move.w	#$7002,d3	; MOVEQ #2,D0
	lea	(DMAPOKE).l,a3
	jsr	$34F4		; the rewrite, CINVA, and the new code, in one run
	cmp.l	#2,d0
	beq.s	t2ok
	failt	2
t2ok:
	; again, with the rewrite undone the same way
	jsr	$34E4
	cmp.l	#2,d0		; MOVEQ #2 stays until poked back
	beq.s	t3ok
	failt	3
t3ok:
	move.w	#$7001,(DMAPOKE).l
	cinva	ic
	jsr	$34E4
	cmp.l	#1,d0
	beq.s	t4ok
	failt	4
t4ok:

;------------------------------------------------------------ MOVES behind MOVEC
	moveq	#0,d0
	movec	d0,sfc
	movec	d0,dfc		; both 0 first: the MOVEC below changes DFC
	moveq	#1,d0
	lea	(FCREG).l,a0
	move.b	#$5A,d1
	movec	d0,dfc
	moves.b	d1,(a0)		; the testbench checks FC=1 on this write
	movec	d0,sfc
	moves.b	(a0),d2
	and.l	#$FF,d2
	cmp.l	#$5A,d2
	beq.s	t5ok
	failt	5
t5ok:

	move.w	#$600D,(DONEREG).l
	stop	#$2700

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1

unexp:
	move.w	#$00FF,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2

; The code the tests call. $34E4: straight into $3500. $34F4: two
; divides, the poke of $3500 to MOVEQ #2,D0 (D3 into (A3) = $F130), CINVA,
; and $3500 at once -- fetched during the divides, before the poke.
	org	$34E4
	bra.s	code3500		; $34E4
	nop				; $34E6
	nop				; $34E8
	nop				; $34EA
	nop				; $34EC
	nop				; $34EE
	nop				; $34F0
	nop				; $34F2
	divu.w	#3,d2			; $34F4
	divu.w	#3,d2			; $34F8
	move.w	d3,(a3)			; $34FC: the poke
	cinva	ic			; $34FE
code3500:
	moveq	#1,d0			; $3500, rewritten by the poke
	dc.w	$0000			; $3502: the poke writes 0 here; with $3504
	dc.w	$0000			;   it is ORI.B #0,D0
	rts				; $3506
