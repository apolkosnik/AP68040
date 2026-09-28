; AP040 a store into prefetched code, with the code mapped elsewhere
; assembled with vasmm68k_mot -Fbin -m68040 -no-opt
;
; testbench protocol: $F100 fail number, $F102 result magic
;
; t_integer.s 192's insurance with translation on: a CPU store into an
; instruction the fetch has already read ahead must be the instruction that
; runs. The 68040 only promises that after CPUSH/CINV; these cores snoop
; their own stores as insurance (see t_integer.s). With translation on and
; the code mapped at a different physical address, the stores and the
; prefetched words must be matched by the address the program uses. In the
; pipelined core the CPU's store snoop catches the queue and the pipeline
; and refetches, which here also empties the bus controller's prefetch
; window; that window's own snoop, which must match logical addresses too
; (the bus controller sees the store only once translated), is
; tb_ap040_pipe_dmuport.v test 7's, where no CPU stands in front of it.
;
; memory map: physical = logical below $40000 (page table $4400); logical
; $40000-$7FFFF is physical $0-$3FFFF again (page table $4600), and the
; tests run there, storing through that alias. Caches off (CACR 0).
; Each test rewrites eight NOPs with MOVEM, 0 to 28 bytes after it, the
; MOVEM's address waiting on a divide so the fetch has run ahead (see the
; macro).

FAILREG		equ	$F100
DONEREG		equ	$F102
ALIAS		equ	$40000

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

; smc <nops between the store and its target>, <test number>
; MOVEM rewrites sixteen bytes of code -- eight NOPs become eight ADDQ.W
; #1,D7 -- while it holds EA-fetch for four writes: decode stalls behind it
; with the fetch queue full and the prefetch window run ahead, so the far
; targets are in the window when the writes land.
smc	macro
	moveq	#0,d7
	lea	t\@(pc),a0
	move.l	#$52475247,d1	; addq.w #1,d7 twice
	move.l	d1,d2
	move.l	d1,d3
	move.l	d1,d4
	move.l	#100,d0
	divu.w	#3,d0		; the fetch runs ahead...
	and.l	#0,d0		; ...while the stores' address waits for this
	movem.l	d1-d4,(a0,d0.l)
	rept	\1
	nop
	endr
t\@:
	rept	8
	nop
	endr
	moveq	#0,d0
	cmp.l	#8,d7		; all eight rewritten instructions ran
	beq.s	ok\@
	move.w	#\2,d6
	bra	fail_here
ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp		; 2-255
	endr

	org	$400
start:
	move.w	#$2700,sr
	moveq	#0,d0
	movec	d0,cacr

;----------------------------------------------------------------- tables
	lea	($4400).l,a0	; physical = logical, pages 0-63
	lea	($4600).l,a1	; logical $40000+ -> physical pages 0-63
	moveq	#0,d0
	moveq	#63,d1
tloop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#4,d2		; i << 12
	addq.l	#3,d2		; resident
	move.l	d2,(a0)+
	move.l	d2,(a1)+
	addq.l	#1,d0
	dbra	d1,tloop
	move.l	#$00004203,($4000).l	; root 0 -> pointer table
	move.l	#$00004403,($4200).l	; pointer 0: $00000-$3FFFF
	move.l	#$00004603,($4204).l	; pointer 1: $40000-$7FFFF

	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	pflusha
	move.l	#$8000,d0	; E=1, 4K pages
	movec	d0,tc

	jmp	(ALIAS+aliased).l

;------------------------------------------------ the tests, run at ALIAS+
aliased:
	smc	0,1
	smc	1,2
	smc	2,3
	smc	3,4
	smc	4,5
	smc	5,6
	smc	6,7
	smc	7,8
	smc	8,9
	smc	9,10
	smc	10,11
	smc	11,12
	smc	12,13
	smc	13,14
	smc	14,15
	jmp	(back).l	; physical = logical again

back:
	moveq	#0,d0
	movec	d0,tc
	pflusha
	move.w	#$600D,(DONEREG).l
	stop	#$2700

fail_here:
	move.w	d6,d7
fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1

unexp:
	move.w	#$0099,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2
