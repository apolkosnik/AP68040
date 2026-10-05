; AP68040-60: the CDIS, MDIS, IPEND and PST pins (MC68040UM 3.6.2, 5.7.1,
; 5.8.2, 5.9.1)
; assembled with vasmm68k_mot -Fbin -m68040
;
; The bench drives CDIS and MDIS from $F2C0 (bit 0, bit 1), and checks PST
; itself: never a reserved encoding, "stopped" during STOP.
;   1-3  CDIS: the data cache holds a line that memory no longer matches (the
;        bench's DMA poke at $F130 writes $3500 behind it); with CDIS the
;        load reads memory, without it the cached copy again (CDIS does not
;        flush)
;   4-7  MDIS: $01050000 is mapped by the page tables to $6000; with MDIS
;        the address is used untranslated -- outside the bench's memory,
;        a bus error -- while the TTRs still map the program; the handler
;        drops MDIS and the instruction, restarted, translates again
;   8    STOP, woken by a level 2 interrupt (the bench checks PST = D)

FAILREG	equ	$F100
DONEREG	equ	$F102
POKEREG	equ	$F130
IRQDLY	equ	$F148
IPLREG	equ	$F110
PINS	equ	$F2C0
ROOT	equ	$10000		; root table (128 x 4)
PTR	equ	$10200		; pointer table (128 x 4)
PAGE	equ	$10400		; page table (64 x 4, 4 KB pages)
LA	equ	$01050000	; root 0, pointer $41, page $10
PA	equ	$6000

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

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	berr		; 2: access fault
	rept	253
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$80008000,d0
	movec	d0,cacr

;------------------------------------------------------------------ CDIS
	move.l	#$11110000,($3500).l	; write-through, no allocate
	move.l	($3500).l,d0		; read miss: the line is cached
	chkl	d0,$11110000,90
	move.w	#$2222,(POKEREG).l	; memory: $22220000, the cache: $11110000
	nop
	move.l	($3500).l,d0
	chkl	d0,$11110000,1		; the stale copy (no snoop of the poke)
	move.w	#1,(PINS).l		; CDIS
	nop
	nop
	nop
	move.l	($3500).l,d0
	chkl	d0,$22220000,2		; disabled: memory
	clr.w	(PINS).l
	nop
	nop
	nop
	move.l	($3500).l,d0
	chkl	d0,$11110000,3		; enabled again, the line kept
	cinva	dc

;------------------------------------------------------------------ MDIS
	move.l	#$0000C000,d0		; TTR: all of $00xxxxxx, both modes
	movec	d0,dtt0
	movec	d0,itt0
	lea	(ROOT).l,a0		; clear the tables
	move.w	#(PAGE+$100-ROOT)/4-1,d1
.clr:	clr.l	(a0)+
	dbra	d1,.clr
	move.l	#PTR+2,(ROOT).l			; root 0: pointer table, resident
	move.l	#PAGE+2,(PTR+$41*4).l		; pointer $41: page table
	move.l	#PA+1,(PAGE+$10*4).l		; page $10: $6000, resident
	move.l	#$CAFEBABE,(PA).l
	lea	(ROOT).l,a0
	movec	a0,urp
	movec	a0,srp
	pflusha
	move.l	#$8000,d0		; E, 4 KB pages
	movec	d0,tc
	moveq	#0,d6
	move.l	(LA).l,d0
	chkl	d0,$CAFEBABE,4		; translated
	move.w	#2,(PINS).l		; MDIS
	nop
	nop
	nop
	move.l	(LA).l,d0		; untranslated: a bus error, the handler
	chkl	d6,1,5			; ... drops MDIS, the load is restarted
	chkl	d0,$CAFEBABE,6
	move.l	(LA).l,d0
	chkl	d0,$CAFEBABE,7
	moveq	#0,d0
	movec	d0,tc
	movec	d0,dtt0
	movec	d0,itt0
	pflusha

;------------------------------------------------------------------ STOP
	move.l	#lev2,(26*4).w
	moveq	#0,d5
	move.w	#200,(IRQDLY).l		; IPL 2 in 200 clocks
	stop	#$2000
	chkl	d5,1,8
	move.w	#$2700,sr

;----------------------------------------------------------------- done
	move.w	#$600D,(DONEREG).l
	bra.s	*

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	99

; access fault: under MDIS (d6 counts), drop MDIS and restart
berr:
	addq.l	#1,d6
	cmp.l	#1,d6
	bne.s	.twice
	clr.w	(PINS).l
	nop
	nop
	nop
	rte
.twice:	failt	98

lev2:
	clr.w	(IPLREG).l
	moveq	#1,d5
	rte
