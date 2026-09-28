; AP040 the pipelined core's data cache in copyback mode (caches stage D)
; assembled with vasmm68k_mot -Fbin -m68040 -no-opt
;
; testbench protocol: $F100 fail number, $F102 result magic;
; $F134 w: a poke's address, $F136 w: poke the word there behind the CPU;
; $F138 w: a peek's address, $F13A r: memory's word there, whatever the
; data cache holds
;
; MC68040UM 4.3.1.2 and 4.6: in a copyback page a write that hits updates
; the line and marks it dirty, without a bus cycle, and one that misses
; reads the line first; memory takes the dirty data only when the line is
; replaced, pushed by CPUSH, or met by a cache-inhibited or locked access
; (4.3.2, 7.4.5); CINV discards it. A write-through write to a dirty line
; updates it and memory, and the line stays dirty (Table 4-4). Table
; searches read descriptors through the data cache and their U/M writes
; update a line holding one (4.3.3).
;
; Peeks show memory; reads show the cache. Run translated, the instruction
; cache off; the program's own pages write-through, the protocol page
; inhibited.
;
; memory map (physical):
;   $0400 code, $3400 ISP top (pages 0-3, write-through)
;   $4000 root, $4200 pointer table, $4400 page table (page 4, WT)
;   $5000 page 5, copyback; $6000 page 6, copyback; $7000 page 7, inhibited
;   $6800 a second page table, in copyback page 6
;   $F000 page 15, the protocol registers, inhibited
; logical: page 8 = physical 5 inhibited; page 9 = physical 5 write-through;
;   $40000-$40FFF through the second page table

FAILREG		equ	$F100
DONEREG		equ	$F102
POKEA		equ	$F134
POKED		equ	$F136
PEEKA		equ	$F138
PEEKD		equ	$F13A
CI5		equ	$8000		; physical page 5, inhibited
WT5		equ	$9000		; physical page 5, write-through

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro
	cmp.l	#\2,\1
	beq.s	ok\@
	failt	\3
ok\@:
	endm

; poke <address>,<longword>: memory changed behind the CPU
poke	macro
	move.w	#\1,(POKEA).l
	move.w	#(\2)>>16,(POKED).l
	move.w	#(\1)+2,(POKEA).l
	move.w	#(\2)&$FFFF,(POKED).l
	endm

; peekl <address>: memory's longword, into d0
peekl	macro
	move.w	#\1,(PEEKA).l
	move.w	(PEEKD).l,d0
	swap	d0
	move.w	#(\1)+2,(PEEKA).l
	move.w	(PEEKD).l,d0
	endm

; mem <address>,<longword>,<test>: memory holds it
mem	macro
	peekl	\1
	chkl	d0,\2,\3
	endm

; want <address>,<longword>,<test>: a read returns it
want	macro
	move.l	(\1).l,d0
	chkl	d0,\2,\3
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
	movec	d0,itt0
	movec	d0,itt1
	movec	d0,dtt0
	movec	d0,dtt1
	cinva	bc

;----------------------------------------------------------------- tables
	lea	($4400).l,a0
	moveq	#0,d0
	moveq	#63,d1
tloop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#4,d2		; i << 12
	addq.l	#3,d2		; resident, CM 00 (write-through)
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,tloop
	move.l	#$00005023,($4414).l	; page 5: CM 01, copyback
	move.l	#$00006023,($4418).l	; page 6: copyback
	move.l	#$00007043,($441C).l	; page 7: CM 10, inhibited
	move.l	#$00005043,($4420).l	; page 8 -> physical 5, inhibited
	move.l	#$00005003,($4424).l	; page 9 -> physical 5, write-through
	move.l	#$0000F043,($443C).l	; page 15: inhibited
	move.l	#$00004203,($4000).l	; root 0
	move.l	#$00004403,($4200).l	; pointer 0: $00000-$3FFFF
	move.l	#$00006803,($4204).l	; pointer 1: $40000-$7FFFF, table in page 6
	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	pflusha
	move.l	#$8000,d0	; E=1, 4K pages
	movec	d0,tc
	move.l	#$80000000,d0	; DE; the instruction cache off
	movec	d0,cacr

;------------------------------------ a copyback write stays in the cache
	poke	$5000,$11111111
	move.l	#$A5A5A5A5,($5000).l	; misses: the line read, then updated
	mem	$5000,$11111111,1	; memory untouched
	want	$5000,$A5A5A5A5,2
	move.w	#$B6B6,($5002).l	; hits
	mem	$5000,$11111111,3
	want	$5000,$A5A5B6B6,4

	poke	$5060,$01234567
	move.b	#$AB,($5062).l		; a byte that misses: the line read, then it
	want	$5060,$0123AB67,40	; the rest of its longword memory's
	mem	$5060,$01234567,41

;------------------------------------------- CPUSH writes it, CINV drops it
	lea	($5000).l,a0
	cpushl	dc,(a0)
	mem	$5000,$A5A5B6B6,5
	poke	$5000,$22222222
	want	$5000,$22222222,6	; and the line went
	poke	$5010,$0C0C0C0C
	move.l	#$C7C7C7C7,($5010).l
	lea	($5010).l,a0
	cinvl	dc,(a0)
	mem	$5010,$0C0C0C0C,7	; the dirty longword never reached memory
	want	$5010,$0C0C0C0C,8

;------------------------------ inhibited: a dirty line is pushed first
	move.l	#$D8D8D8D8,($5020).l
	want	CI5+$20,$D8D8D8D8,9	; the inhibited read sees the pushed line
	mem	$5020,$D8D8D8D8,10
	poke	$5020,$33333333
	want	$5020,$33333333,11	; the line went
	move.l	#$E9E9E9E9,($5030).l
	move.l	#$EAEAEAEA,($5034).l
	move.w	#$1234,(CI5+$32).l	; an inhibited write: merged, pushed, gone
	mem	$5030,$E9E91234,12
	mem	$5034,$EAEAEAEA,13
	poke	$5030,$44444444
	want	$5030,$44444444,14

;-------------------- write-through to a dirty line: the line stays dirty
	move.l	#$F0F0F0F0,($5040).l
	move.l	#$F1F1F1F1,($5044).l
	move.l	#$12121212,(WT5+$40).l
	mem	$5040,$12121212,15	; written through
	mem	$5044,$00000000,16	; the line's other longword still only cached
	want	$5040,$12121212,17
	want	$5044,$F1F1F1F1,18
	lea	($5040).l,a0
	cpushl	dc,(a0)
	mem	$5044,$F1F1F1F1,19

;----------------------------------- a dirty line replaced is pushed
	cinva	dc
	move.l	#$51515151,($5100).l	; set $10, four ways, all dirty
	move.l	#$55555555,($5500).l
	move.l	#$61616161,($6100).l
	move.l	#$65656565,($6500).l
	move.l	#$59595959,($5900).l	; a fifth: one of the four goes out
	want	$5100,$51515151,20
	want	$5500,$55555555,21
	want	$6100,$61616161,22
	want	$6500,$65656565,23
	want	$5900,$59595959,24
	moveq	#0,d5			; how many reached memory
	peekl	$5100
	cmp.l	#$51515151,d0
	bne.s	v1
	addq.l	#1,d5
v1:	peekl	$5500
	cmp.l	#$55555555,d0
	bne.s	v2
	addq.l	#1,d5
v2:	peekl	$6100
	cmp.l	#$61616161,d0
	bne.s	v3
	addq.l	#1,d5
v3:	peekl	$6500
	cmp.l	#$65656565,d0
	bne.s	v4
	addq.l	#1,d5
v4:	tst.l	d5
	bne.s	v_ok
	failt	25			; a dirty line was replaced and lost
v_ok:
	cpusha	dc			; every dirty line pushed
	mem	$5100,$51515151,26
	mem	$5500,$55555555,27
	mem	$6100,$61616161,28
	mem	$6500,$65656565,29
	mem	$5900,$59595959,30

;--------------------------------------------------------- CPUSHP
	move.l	#$62626262,($6200).l
	move.l	#$52525252,($5200).l
	lea	($6ABC).l,a0
	cpushp	dc,(a0)
	mem	$6200,$62626262,31
	mem	$5200,$00000000,32	; page 5's line not pushed
	cpusha	dc
	mem	$5200,$52525252,33

;------------------------------------ locked: a dirty line pushed first
	move.l	#$01020304,($5300).l
	tas	($5300).l		; memory's $01 -- once pushed -- set to $81
	mem	$5300,$81020304,34
	want	$5300,$81020304,35

;------------------------ MOVE16 into a cached line: invalidated, to memory
	move.l	#$0,($5700).l		; the destination line cached (a copyback
	want	$5700,$00000000,42	; write that missed read it)
	move.l	#$71717171,($5600).l
	move.l	#$72727272,($5604).l
	move.l	#$73737373,($5608).l
	move.l	#$74747474,($560C).l
	lea	($5600).l,a0
	lea	($5700).l,a1
	move16	(a0)+,(a1)+
	mem	$5704,$72727272,43	; MOVE16's writes go to memory...
	poke	$5704,$7F7F7F7F
	want	$5704,$7F7F7F7F,44	; ...and the line they hit went

;--------------------------------- the table walker through the cache
; Pointer 1's page table is in copyback page 6. Its first descriptor,
; written now, is dirty in the cache: memory still says invalid. A search
; for $40000 must read it from the cache, and its U write update the line.
	poke	$7000,$7A7A7A7A
	move.l	#$00007003,($6800).l	; $40000 -> physical $7000
	mem	$6800,$00000000,36	; only in the cache
	pflusha
	want	$40000,$7A7A7A7A,37	; searched through the cached descriptor
	want	$6800,$0000700B,38	; the search's U, in the line
	mem	$6800,$0000700B,39	; ...and in memory (the line's longword clean)

;----------------------------------------------------------------- done
	cpusha	dc
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,tc
	pflusha
	cinva	bc
	move.w	#$600D,(DONEREG).l
	stop	#$2700

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
