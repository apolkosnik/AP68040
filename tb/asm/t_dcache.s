; AP040 the pipelined core's data cache, write-through (caches stage C)
; assembled with vasmm68k_mot -Fbin -m68040 -no-opt
;
; testbench protocol: $F100 fail number, $F102 result magic;
; $F134 w: a poke's address, $F136 w: poke the word there behind the CPU
;
; MC68040UM section 4 as it applies to a write-through data cache: 64 sets
; of four 16-byte lines, physically tagged, filled a whole line at a time
; by a read that misses; a write goes to memory and updates a line holding
; it (Table 4-4), and a write that misses allocates nothing; CINV and CPUSH
; act on a line or a 4 KB page at a PHYSICAL address in An, or on
; everything, on the caches they name, whatever CACR says; a disabled cache
; is bypassed and kept; a cache-inhibited access (a TTR's or a page's CM 1x,
; MOVES to an alternate space, a locked access) goes to memory and
; invalidates a line holding it (4.3.2, 7.4.5); the vector fetch and MOVE16
; allocate nothing, and a MOVE16 write that hits invalidates (4.3.3).
;
; What the cache holds is seen through pokes: memory changed behind the
; CPU's back, which a line holding it does not see. Every probe is a
; longword whose poked value differs from what the CPU wrote.
;
; Pipelined core only: rtl/ap040/ap040_core.v widens every CINV/CPUSH to
; all lines, and invalidates rather than updates on a write hit.
;
; memory map:
;   $0400 code, $3400 ISP top
;   $4000 root, $4200 pointer table, $4400 page table (64 x 4K pages)
;   $5000 data (page 5), $6000 data (page 6), $7000 data (page 7)
;   logical page 8 = physical page 5 (translated part)

FAILREG		equ	$F100
DONEREG		equ	$F102
POKEA		equ	$F134
POKED		equ	$F136

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

; want <address>,<longword>,<test>: a longword read
want	macro
	move.l	(\1).l,d0
	chkl	d0,\2,\3
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	30
	dc.l	unexp		; 2-31
	endr
	dc.l	h_trap0		; 32 TRAP #0
	rept	223
	dc.l	unexp		; 33-255
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
	move.l	#$80008000,d0
	movec	d0,cacr

;--------------------------------------------- a read miss allocates
	move.l	#$11111111,($5000).l
	want	$5000,$11111111,1	; fills the line
	poke	$5000,$A1A1A1A1
	want	$5000,$11111111,2	; the line, not memory
	want	$5004,$00000000,3	; the rest of the line came with it
	poke	$5004,$A2A2A2A2
	want	$5004,$00000000,4

;--------------------------------------------- a write hit updates the line
; Each write is followed by a poke of the same longword; the read then
; sees the write only if the line took it (invalidated, it would read the
; poke).
	move.b	#$22,($5001).l
	poke	$5000,$B1B1B1B1
	want	$5000,$11221111,5
	move.w	#$3344,($5002).l
	poke	$5000,$B2B2B2B2
	want	$5000,$11223344,6
	move.w	#$5566,($5001).l	; a word at an odd address, in one longword
	poke	$5000,$B3B3B3B3
	want	$5000,$11556644,7
	move.l	#$778899AA,($5000).l
	poke	$5000,$B4B4B4B4
	want	$5000,$778899AA,8
	move.b	($5003).l,d0		; byte and word reads from the line
	and.l	#$FF,d0
	chkl	d0,$AA,9
	move.w	($5001).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$8899,10
	; a longword spanning two longwords of the line, then two lines
	move.l	#$C0C1C2C3,($5002).l
	poke	$5000,$B5B5B5B5
	poke	$5004,$B6B6B6B6
	want	$5000,$7788C0C1,11
	want	$5004,$C2C30000,12
	move.l	#$0,($5010).l		; the next line, cached too
	want	$5010,$00000000,13
	move.l	#$D0D1D2D3,($500E).l
	poke	$500C,$B7B7B7B7
	poke	$5010,$B8B8B8B8
	want	$500C,$0000D0D1,14
	want	$5010,$D2D30000,15

;------------------------------------ a write miss allocates nothing
	move.l	#$E1E1E1E1,($5100).l	; never read: not cached
	poke	$5100,$F1F1F1F1
	want	$5100,$F1F1F1F1,16

;----------------------------------------------- CINV and CPUSH on DC
	cinva	dc
	move.l	#$10101010,($5200).l
	move.l	#$20202020,($5210).l
	move.l	#$30303030,($6200).l
	want	$5200,$10101010,17	; three lines cached
	want	$5210,$20202020,18
	want	$6200,$30303030,19
	poke	$5200,$11110000
	poke	$5210,$22220000
	poke	$6200,$33330000
	lea	($5204).l,a0
	cinvl	dc,(a0)
	want	$5200,$11110000,20	; the line CINVL named: memory's
	want	$5210,$20202020,21	; the next line: kept
	want	$6200,$30303030,22	; another page: kept
	lea	($5FF0).l,a0
	cinvp	dc,(a0)
	want	$5210,$22220000,23
	want	$6200,$30303030,24
	cinva	ic			; the instruction cache's: the data cache kept
	lea	($6200).l,a0
	cinvl	ic,(a0)
	cpushl	ic,(a0)
	cinvp	ic,(a0)
	cpushp	ic,(a0)
	want	$6200,$30303030,25
	cpusha	dc
	want	$6200,$33330000,26
	want	$5200,$11110000,27	; both cached again
	poke	$5200,$44440000
	poke	$6200,$55550000
	lea	($5200).l,a0
	cpushl	dc,(a0)
	want	$5200,$44440000,28
	want	$6200,$33330000,29	; kept
	lea	($6000).l,a0
	cpushp	dc,(a0)
	want	$6200,$55550000,30
	cinva	bc
	poke	$5200,$66660000
	want	$5200,$66660000,31

;------------------------------------------- disabled: bypassed, and kept
	poke	$5200,$77770000		; the line holds $66660000
	move.l	#$00008000,d0		; DE clear
	movec	d0,cacr
	want	$5200,$77770000,32	; memory's
	move.l	#$80008000,d0
	movec	d0,cacr
	want	$5200,$66660000,33	; the line it held, unchanged
	cinva	dc
	want	$5200,$77770000,34

;------------------------------- cache-inhibited through a DTT: invalidated
	move.l	#$0000C040,d0		; $00xxxxxx, either mode, CM 10
	movec	d0,dtt1
	poke	$5200,$88880000		; the line holds $77770000
	want	$5200,$88880000,35	; memory's -- and the line invalidated
	poke	$5200,$99990000
	want	$5200,$99990000,36
	moveq	#0,d0
	movec	d0,dtt1
	want	$5200,$99990000,37	; no stale line left behind
	poke	$5200,$AAAA0000
	want	$5200,$99990000,38	; cached again

;	a write to an inhibited page that hits invalidates the line too
	move.l	#$0000C040,d0
	movec	d0,dtt1			; the line holds $99990000
	move.l	#$ABAB0000,($5200).l	; inhibited: to memory, and the line goes
	poke	$5200,$ACAC0000
	moveq	#0,d0
	movec	d0,dtt1
	want	$5200,$ACAC0000,39	; memory's -- updated, the line would say $ABAB

;--------------------------------------- MOVES to an alternate space
	moveq	#3,d0			; FC 3: physical, cache-inhibited
	movec	d0,sfc
	poke	$5200,$BBBB0000		; the line holds $99990000
	moves.l	($5200).l,d1
	chkl	d1,$BBBB0000,40
	want	$5200,$BBBB0000,41	; the line went

;------------------------------------------------ locked: TAS and CAS
	move.l	#$01020304,($5300).l
	want	$5300,$01020304,42	; cached
	poke	$5300,$05060708
	tas	($5300).l		; reads memory's $05, writes $85
	want	$5300,$85060708,43	; the line went; memory's, with TAS's write
	move.l	#$11223344,($5310).l
	want	$5310,$11223344,44
	poke	$5310,$55667788
	move.l	#$55667788,d1
	move.l	#$99AABBCC,d2
	cas.l	d1,d2,($5310).l		; compares memory's -- equal -- and stores
	beq.s	cas_eq
	failt	45
cas_eq:
	want	$5310,$99AABBCC,46

;-------------------------------------------- MOVE16 allocates nothing
	cinva	dc
	move.l	#$21212121,($5400).l
	move.l	#$22222222,($5404).l
	move.l	#$23232323,($5408).l
	move.l	#$24242424,($540C).l
	want	$5400,$21212121,47	; the source line cached
	poke	$5404,$2F2F2F2F		; ...its second longword changed behind it
	move.l	#$0,($5500).l
	want	$5500,$00000000,48	; the destination line cached
	lea	($5400).l,a0
	lea	($5500).l,a1
	move16	(a0)+,(a1)+		; reads hit the source line; the write
					; invalidates the destination's
	poke	$5500,$3F3F3F3F
	want	$5500,$3F3F3F3F,49	; not cached after MOVE16's write
	want	$5504,$22222222,50	; the source line's copy, not memory's $2F
	poke	$5600,$40404040
	lea	($5600).l,a0
	lea	($5700).l,a1
	move16	(a0)+,(a1)+		; a source line not cached: not allocated
	poke	$5600,$41414141
	want	$5600,$41414141,51

;------------------------------------------ the vector fetch allocates nothing
	cinva	dc
	trap	#0
	poke	$80,$00000400		; vector 32 changed behind the CPU
	want	$80,$00000400,52	; the vector fetch left no line

;------------------------------------------ translated: the page's CM
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
	move.l	#$00006043,($4418).l	; page 6: CM 10, cache-inhibited
	move.l	#$00005003,($4420).l	; logical page 8 -> physical page 5
	move.l	#$00004203,($4000).l
	move.l	#$00004403,($4200).l
	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	cinva	dc
	pflusha
	move.l	#$8000,d0	; E=1, 4K pages
	movec	d0,tc

	move.l	#$61616161,($6300).l	; page 6, inhibited
	want	$6300,$61616161,53
	poke	$6300,$62626262
	want	$6300,$62626262,54	; memory's every time
	move.l	#$63636363,($5800).l
	want	$5800,$63636363,55	; page 5, cached
	want	$8800,$63636363,56	; the alias: the same physical line
	poke	$5800,$64646464
	want	$8800,$63636363,57
	lea	($8800).l,a0		; a LOGICAL address: physical $8800
	cinvl	dc,(a0)
	want	$5800,$63636363,58
	lea	($5800).l,a0		; the line's physical address
	cinvl	dc,(a0)
	want	$8800,$64646464,59
	move.b	#$65,($8801).l		; a write through the alias updates it
	poke	$5800,$66666666
	want	$5800,$64656464,60

	moveq	#0,d0
	movec	d0,cacr
	movec	d0,tc
	pflusha
	cinva	bc

;----------------------------------------------------------------- done
	move.w	#$600D,(DONEREG).l
	stop	#$2700

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1

h_trap0:
	rte

unexp:
	move.w	#$0099,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2
