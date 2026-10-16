; AP68060: Fast RAM on the 68060 bus (rtl/fastram: ap060_fastram_bus in
; front of the AP030 Fast RAM front end and its DDR3 side)
; assembled with vasmm68k_mot -Fbin -m68040
;
; The bench maps $50000000-$500FFFFF to a DDR3 model (random waits and
; latency, a clock of its own) through rtl/fastram/ap060_fastram.sv.  The
; model starts with a pattern: the 64-bit word i holds the 16-bit words
; i, $5EED, i, $FA57 from bit 0 (the Minimig DDR3 layout), so the long
; words at 8i and 8i+4 read $iiii5EED and $iiiiFA57.
;
;   1-2   untouched DDR3 reads as the pattern (long, word and byte reads):
;         the byte order of the DDR3 layout
;   3-6   caches off: long, word and byte writes at every offset, each read
;         back; the bytes around them unchanged; misaligned longs across a
;         long word and a line boundary
;   7-8   copyback (DTT0): 16 Kbytes written through the data cache (twice
;         its size: line fills, pushes of dirty lines), CPUSHA, CINVA,
;         read back with the cache off and on
;   9     write-through: stores straight to Fast RAM, line fills on reads
;   10    cache inhibited (precise): reads and writes of single long words
;   11-12 MOVE16 from low memory into Fast RAM and from Fast RAM to Fast RAM
;   13    code in Fast RAM: copied there, pushed, the instruction cache
;         invalidated, called with the instruction cache on (line fills of
;         program space) and off
;   14    TAS in Fast RAM (a locked read-modify-write, not cached)
;
;   98   an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102
FR	equ	$50000000

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
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#$2700,sr
	dc.w	$F4D8			; cinva bc
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,dtt0
	movec	d0,dtt1
	movec	d0,itt0
	movec	d0,itt1
	movec	d0,tc

;------------------------------------------------ 1-2 the DDR3 pattern
	move.l	(FR+$80008).l,d0	; word $10001
	chkl	d0,$00015EED,1
	move.l	(FR+$8000C).l,d0
	chkl	d0,$0001FA57,1
	moveq	#0,d0
	move.w	(FR+$80012).l,d0	; word $10002: $0002 $5EED $0002 $FA57
	chkl	d0,$5EED,2
	moveq	#0,d0
	move.b	(FR+$80017).l,d0
	chkl	d0,$57,2

;------------------------------------------------ 3-6 caches off
	lea	(FR+$100).l,a0
	move.l	#$01020304,(a0)
	move.l	#$05060708,4(a0)
	move.l	(a0),d0
	chkl	d0,$01020304,3
	move.w	#$A1A2,2(a0)
	move.b	#$B1,(a0)
	move.b	#$B4,7(a0)
	move.l	(a0),d0
	chkl	d0,$B102A1A2,4
	move.l	4(a0),d0
	chkl	d0,$050607B4,4
	; every byte offset of a long word
	lea	(FR+$200).l,a0
	moveq	#0,d1
.b:	move.b	d1,(a0,d1.w)
	addq.w	#1,d1
	cmp.w	#16,d1
	bne.s	.b
	move.l	(a0),d0
	chkl	d0,$00010203,5
	move.l	12(a0),d0
	chkl	d0,$0C0D0E0F,5
	; misaligned: across a long word, across a line
	move.l	#$11223344,(FR+$301).l
	move.l	(FR+$300).l,d0
	chkl	d0,$00112233,6		; (the pattern's byte 0 of word $60 is 0)
	move.l	(FR+$301).l,d0
	chkl	d0,$11223344,6
	move.l	#$55667788,(FR+$30E).l
	move.l	(FR+$30E).l,d0
	chkl	d0,$55667788,6
	moveq	#0,d0
	move.w	(FR+$30C).l,d0
	chkl	d0,$0061,6		; word $61: $0061 $5EED $0061 $FA57

;------------------------------------------------ 7-8 copyback, 16 Kbytes
	move.l	#$5000A020,d0		; DTT0: supervisor $50xxxxxx copyback
	movec	d0,dtt0
	move.l	#$80000000,d0		; CACR.EDC
	movec	d0,cacr
	lea	(FR+$10000).l,a0
	move.l	#$12345678,d1
	move.w	#4095,d2
.w1:	move.l	d1,(a0)+
	add.l	#$01010101,d1
	dbra	d2,.w1
	dc.w	$F478			; cpusha dc
	dc.w	$F458			; cinva dc
	moveq	#0,d0
	movec	d0,cacr
	bsr	check16k
	chkl	d3,0,7
	move.l	#$80000000,d0
	movec	d0,cacr
	bsr	check16k
	chkl	d3,0,8

;------------------------------------------------ 9 write-through
	move.l	#$5000A000,d0		; DTT0: write-through
	movec	d0,dtt0
	lea	(FR+$20000).l,a0
	move.l	#$CAFE0000,d1
	moveq	#63,d2
.w2:	move.l	d1,(a0)+
	addq.l	#1,d1
	dbra	d2,.w2
	dc.w	$F458			; cinva dc: nothing to push
	lea	(FR+$20000).l,a0
	move.l	#$CAFE0000,d1
	moveq	#63,d2
.r2:	cmp.l	(a0)+,d1
	beq.s	.r2ok
	failt	9
.r2ok:	addq.l	#1,d1
	dbra	d2,.r2

;------------------------------------------------ 10 cache inhibited
	move.l	#$5000A040,d0		; DTT0: inhibited, precise
	movec	d0,dtt0
	lea	(FR+$30000).l,a0
	move.l	#$0BADF00D,(a0)
	move.l	#$FEEDFACE,4(a0)
	move.l	(a0),d0
	chkl	d0,$0BADF00D,10
	move.l	4(a0),d0
	chkl	d0,$FEEDFACE,10
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,dtt0

;------------------------------------------------ 11-12 MOVE16
	lea	src16(pc),a0
	lea	(FR+$40000).l,a1
	move16	(a0)+,(a1)+
	move16	(a0)+,(a1)+
	lea	(FR+$40000).l,a0
	move.l	(a0),d0
	chkl	d0,$10111213,11
	move.l	28(a0),d0
	chkl	d0,$2C2D2E2F,11
	lea	(FR+$40000).l,a0
	lea	(FR+$40100).l,a1
	move16	(a0)+,(a1)+
	move16	(a0)+,(a1)+
	move.l	(FR+$40104).l,d0
	chkl	d0,$14151617,12
	move.l	(FR+$4011C).l,d0
	chkl	d0,$2C2D2E2F,12

;------------------------------------------------ 13 code in Fast RAM
	lea	frcode(pc),a0
	lea	(FR+$50000).l,a1
	moveq	#(frcode_end-frcode)/2-1,d2
.c:	move.w	(a0)+,(a1)+
	dbra	d2,.c
	dc.w	$F4F8			; cpusha bc
	move.l	#$00008000,d0		; CACR.EIC
	movec	d0,cacr
	moveq	#0,d0
	jsr	(FR+$50000).l
	chkl	d0,$600D0013,13
	moveq	#0,d0
	movec	d0,cacr
	dc.w	$F4D8			; cinva bc
	moveq	#0,d0
	jsr	(FR+$50000).l
	chkl	d0,$600D0013,13

;------------------------------------------------ 14 TAS
	clr.b	(FR+$60000).l
	tas	(FR+$60000).l
	bne.s	.t1
	moveq	#0,d0
	move.b	(FR+$60000).l,d0
	chkl	d0,$80,14
	tas	(FR+$60000).l
	bmi.s	.t2
.t1:	failt	14
.t2:

	move.w	#$600D,(DONEREG).l
	bra.s	*

; the 16 Kbytes at FR+$10000 as written: d3 = mismatches
check16k:
	lea	(FR+$10000).l,a0
	move.l	#$12345678,d1
	move.w	#4095,d2
	moveq	#0,d3
.r1:	cmp.l	(a0)+,d1
	beq.s	.r1ok
	addq.l	#1,d3
.r1ok:	add.l	#$01010101,d1
	dbra	d2,.r1
	rts

; the routine copied into Fast RAM: a loop, a branch, PC-relative data
frcode:
	moveq	#4,d1
.f:	addq.l	#1,d0
	dbra	d1,.f
	add.l	frdata(pc),d0
	bra.s	.f2
	moveq	#-1,d0
.f2:	rts
	cnop	0,4
frdata:	dc.l	$600D000E
frcode_end:

	cnop	0,16
src16:
	dc.l	$10111213,$14151617,$18191A1B,$1C1D1E1F
	dc.l	$20212223,$24252627,$28292A2B,$2C2D2E2F

fail_all:
	move.w	#$2700,sr
	moveq	#0,d0
	movec	d0,cacr
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	bra.s	*

unexp:
	failt	98
