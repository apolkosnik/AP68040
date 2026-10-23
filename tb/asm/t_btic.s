; AP68040: the fetch BTB's target words.  An entry keeps the long word its
; target starts in, as the instruction cache answered; a taken branch with
; them hands them to the decoder without fetching there.  They are a copy
; of the cache: whatever would make the cache (or the translation) answer
; differently must drop them.  Each test calls a routine ten times (the
; entry, then its words, are made and used) and checks D0.
; assembled with vasmm68k_mot -Fbin -m68040
;   1    target at word 0 and word 1 of a long word
;   2    code rewritten, CPUSHA
;   3    ... CPUSHL of the target's line
;   4    ... CPUSHP of its page
;   5    memory changed under a stale cache line, the cache disabled
;        (CACR.IE = 0): the memory's code runs
;   6    the same with the code noncachable by ITT0
;   7    user mode: the words are tagged with the S bit (user program
;        space noncachable by ITT0 over a stale line)
;   8    two branches with the same BTB index, different targets, each
;        taken three times running (the entry, its words, used)
;   9    a target long word that holds a branch (no words kept)
;  10    translation: the supervisor and user roots map the target's
;        page to different code (the words are tagged with the S bit)

FAILREG	equ	$F100
DONEREG	equ	$F102
CODE	equ	$6000		; target at word 0
CODE2	equ	$6100		; target at word 1
USTK	equ	$3000

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

; call \1 ten times, D0 must be \2 after each
runk	macro
	moveq	#9,d5
.l\@:	moveq	#0,d0
	jsr	(\1).l
	cmp.l	#\2,d0
	beq.s	.k\@
	failt	\3
.k\@:	dbra	d5,.l\@
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	30
	dc.l	unexp
	endr
	dc.l	h_trap0		; vector 32
	rept	223
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$80008000,d0
	movec	d0,cacr
	; CODE:   bra.s CODE+8, three ILLEGALs, moveq #$11,d0, addq.l #1,d0, rts
	lea	(CODE).l,a0
	move.w	#$6006,(a0)
	move.w	#$4AFC,2(a0)
	move.w	#$4AFC,4(a0)
	move.w	#$4AFC,6(a0)
	move.w	#$7011,8(a0)
	move.w	#$5280,10(a0)
	move.w	#$4E75,12(a0)
	; CODE2:  bra.s CODE2+6, two ILLEGALs, moveq #$21,d0, addq.l #1,d0, rts
	lea	(CODE2).l,a1
	move.w	#$6004,(a1)
	move.w	#$4AFC,2(a1)
	move.w	#$4AFC,4(a1)
	move.w	#$7021,6(a1)
	move.w	#$5280,8(a1)
	move.w	#$4E75,10(a1)
	cpusha	bc

;------------------------------------------------ 1 target at word 0 / 1
	runk	CODE,$12,1
	runk	CODE2,$22,1

;------------------------------------------------- 2 rewritten, CPUSHA
	move.w	#$7031,8(a0)
	move.w	#$7041,6(a1)
	cpusha	bc
	runk	CODE,$32,2
	runk	CODE2,$42,2

;------------------------------------------------ 3 CPUSHL of the line
	move.w	#$7051,8(a0)
	cpushl	bc,(a0)
	runk	CODE,$52,3

;------------------------------------------------ 4 CPUSHP of the page
	move.w	#$7061,8(a0)
	cpushp	bc,(a0)
	runk	CODE,$62,4

;------------------------------ 5 the cache disabled over a stale line
	move.w	#$7071,8(a0)
	cpushl	dc,(a0)		; memory has it, the instruction cache not
	move.l	#$80000000,d0
	movec	d0,cacr
	runk	CODE,$72,5
	cinva	ic
	move.l	#$80008000,d0
	movec	d0,cacr
	runk	CODE,$72,5

;------------------------------------ 6 noncachable by ITT0, stale line
	move.w	#$7009,8(a0)
	cpushl	dc,(a0)
	move.l	#$0000C040,d0	; everything: noncachable, serialized
	movec	d0,itt0
	runk	CODE,$0A,6
	moveq	#0,d0
	movec	d0,itt0
	cinva	ic
	runk	CODE,$0A,6

;------------------------------------------------------ 7 user mode
; CODE's cache line is left stale (memory holds a newer target): this
; core's supervisor fetch keeps hitting the line, the user's goes to
; memory (ITT0); the supervisor's words must not serve the user
	move.w	#$7019,8(a0)
	cpushl	dc,(a0)
	move.l	#$00008040,d0	; user program space: noncachable, serialized
	movec	d0,itt0
	runk	CODE,$0A,7	; supervisor: the cache's line (and its words)
	lea	(USTK).l,a2
	move.l	a2,usp
	andi.w	#$DFFF,sr
	runk	CODE,$1A,7	; user: the memory's
	runk	CODE2,$42,7
	trap	#0		; back to supervisor mode
	runk	CODE,$0A,7
	moveq	#0,d0
	movec	d0,itt0
	cinva	ic
	runk	CODE,$1A,7

;------------------------------------- 8 the same BTB index, two targets
	moveq	#9,d6
.l8:	moveq	#2,d4
.a8:	moveq	#0,d0
	jsr	(al1).l
	cmp.l	#$12,d0
	bne.s	.f8
	dbra	d4,.a8
	moveq	#2,d4
.b8:	moveq	#0,d0
	jsr	(al2).l
	cmp.l	#$24,d0
	bne.s	.f8
	dbra	d4,.b8
	dbra	d6,.l8
	bra.s	.t9
.f8:	failt	8

;--------------------------------- 9 a branch in the target long word
.t9:	moveq	#0,d0
	moveq	#19,d6
	cnop	0,4
.l9:	bra.s	.a9
	dc.w	$4AFC
.a9:	addq.l	#1,d0		; the target long word: addq | bra.s
	bra.s	.b9
	dc.w	$4AFC
.b9:	addq.l	#2,d0
	dbra	d6,.l9
	cmp.l	#60,d0
	beq.s	t10
	failt	9

;------------------------------ 10 the S bit: one address, two programs
; supervisor and user roots map page 7 ($7000) to different physical
; pages, each with its own target code; the branch to it, at $6FF8 in
; page 6, is the same for both
t10:	lea	($7000).l,a3
	move.w	#$7031,(a3)		; supervisor's $7000: moveq #$31,d0
	move.w	#$5280,2(a3)		;   addq.l #1,d0
	move.w	#$4E75,4(a3)		;   rts
	lea	($E000).l,a3
	move.w	#$7041,(a3)		; user's $7000 (PA $E000): moveq #$41,d0
	move.w	#$5280,2(a3)
	move.w	#$4E75,4(a3)
	lea	($6FF8).l,a3
	move.w	#$6006,(a3)		; $6FF8: bra.s $7000
	move.w	#$4AFC,2(a3)
	move.w	#$4AFC,4(a3)
	move.w	#$4AFC,6(a3)
	; supervisor $4000 -> $4200 -> $4400, user $4800 -> $4A00 -> $4C00,
	; 64 identity pages but the user's page 7
	lea	($4400).l,a3
	lea	($4C00).l,a4
	moveq	#0,d0
	moveq	#63,d1
.pt:	move.l	d0,d2
	moveq	#12,d3
	lsl.l	d3,d2
	addq.l	#3,d2			; resident
	move.l	d2,(a3)+
	move.l	d2,(a4)+
	addq.l	#1,d0
	dbra	d1,.pt
	move.l	#$0000E003,($4C1C).l	; user page 7 -> PA $E000
	move.l	#$00004203,($4000).l
	move.l	#$00004403,($4200).l
	move.l	#$00004A03,($4800).l
	move.l	#$00004C03,($4A00).l
	cpusha	bc
	move.l	#$4000,d0
	movec	d0,srp
	move.l	#$4800,d0
	movec	d0,urp
	pflusha
	move.l	#$8000,d0		; E, 4K pages
	movec	d0,tc
	runk	$6FF8,$32,10
	andi.w	#$DFFF,sr
	runk	$6FF8,$42,10
	trap	#0			; back to supervisor mode
	runk	$6FF8,$32,10
	moveq	#0,d0
	movec	d0,tc
	pflusha

done:
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

; TRAP #0: return in supervisor mode
h_trap0:
	ori.w	#$2000,(sp)
	rte

unexp:
	failt	99

; the same index (PC[9:2]), different tags: 1 KB apart
	cnop	0,1024
al1:	bra.s	.t
	dc.w	$4AFC
.t:	moveq	#$11,d0
	addq.l	#1,d0
	rts
	cnop	0,1024
al2:	bra.s	.t
	dc.w	$4AFC
.t:	moveq	#$22,d0
	addq.l	#2,d0
	rts
