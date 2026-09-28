; AP040 MOVES to the alternate address spaces is not translated
; assembled with vasmm68k_mot -Fbin -m68040
;
; testbench protocol: $F100 fail number, $F102 result magic
;
; MC68040UM 3.2 (Table 3-2): MOVES with SFC/DFC $0, $3, $4 or $7 is an
; alternate address space access, "immediately used as a physical address
; without translation" -- no ATC lookup, no table search, no protection, no
; fault from the MMU. $2 and $6 are converted to the data spaces $1 and $5
; and handled as any data access, translated, and on the bus as data. Both
; cores (ap040_pipe_dmu.v and ap040_mmu.v, since caches stage A2). An
; alternate-space access can still take a physical bus error, and its frame
; then reports TT=10 with the raw function code in TM.
;
; memory map (physical = logical except where noted):
;   $0000 vectors, $0400 code, $3400 ISP top
;   $4000 root table, $4200 pointer table, $4400 page table (64 x 4K pages)
;   page  8 ($8000) write protected
;   page  9 ($9000) supervisor only
;   page 10 ($A000) invalid
;   page 11 ($B000) -> physical $C000
;   page 14 ($E000) -> physical $B000, to read physical $B000 back
;   $F120 the bench fails a write here whose function code is not 1
;   $F140 the bench fails the first access here with a bus error
;   $F160 the bench's capability word; bit 2: it can inject bus errors

FAILREG		equ	$F100
DONEREG		equ	$F102
FCREG		equ	$F120	; the bench: a write here must carry FC=1
BERRADR		equ	$F140	; the bench: the first access here is a bus error
CAPREG		equ	$F160
berr_expect	equ	$3600
last_ssw	equ	$3602
last_fa		equ	$3604

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

	org	0
	dc.l	$3400
	dc.l	start
	dc.l	h_aerr		; 2 access error
	rept	253
	dc.l	unexp		; 3-255
	endr

	org	$400
start:
	move.w	#$2700,sr
	moveq	#0,d6		; the test under way, for h_aerr
	clr.w	(berr_expect).l

;----------------------------------------------------------------- tables
	lea	($4400).l,a0
	moveq	#0,d0
	moveq	#63,d1
tloop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#4,d2		; i << 12
	addq.l	#3,d2		; resident
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,tloop

	move.l	#$00008007,($4420).l	; page 8: write protected
	move.l	#$00009083,($4424).l	; page 9: supervisor only
	move.l	#0,($4428).l		; page 10: invalid
	move.l	#$0000C003,($442C).l	; page 11 -> physical $C000
	move.l	#$0000B003,($4438).l	; page 14 -> physical $B000
	move.l	#$00004203,($4000).l	; root 0 -> pointer table
	move.l	#$00004403,($4200).l	; pointer 0 -> page table

;----------------------------------------------- physical data, pre-placed
	move.l	#$B0B0B0B0,($B000).l
	move.l	#$C0C0C0C0,($C000).l
	move.l	#$A0A0A0A0,($A000).l
	move.l	#$90909090,($9000).l
	move.l	#$80808080,($8000).l
	clr.l	($B004).l
	clr.l	($C004).l
	clr.l	($8004).l
	clr.l	($A004).l

;----------------------------------------------------------------- enable
	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	pflusha
	move.l	#$8000,d0		; E=1, 4K pages
	movec	d0,tc

;------------------------------ 1-2: the data spaces are translated (control)
	moveq	#1,d6
	moveq	#5,d0
	movec	d0,sfc
	moves.l	($B000).l,d1
	chkl	d1,$C0C0C0C0,1		; supervisor data: page 11 -> $C000
	moveq	#1,d0
	movec	d0,sfc
	moves.l	($B000).l,d1
	chkl	d1,$C0C0C0C0,2		; user data: the same

;------------------------ 3-4: program spaces become data spaces, translated
	moveq	#3,d6
	moveq	#6,d0
	movec	d0,sfc
	moves.l	($B000).l,d1
	chkl	d1,$C0C0C0C0,3
	moveq	#2,d0
	movec	d0,sfc
	moves.l	($B000).l,d1
	chkl	d1,$C0C0C0C0,4

;------------------------- 5-7: the alternate spaces read physical addresses
	moveq	#5,d6
	moveq	#4,d0
	movec	d0,sfc
	moves.l	($B000).l,d1
	chkl	d1,$B0B0B0B0,5		; not $C0C0C0C0: untranslated
	moveq	#3,d0
	movec	d0,sfc
	moves.l	($B000).l,d1
	chkl	d1,$B0B0B0B0,6
	moveq	#0,d0
	movec	d0,sfc
	moves.l	($B000).l,d1
	chkl	d1,$B0B0B0B0,7

;------------------ 8-9: no protection and no fault in an alternate space
	moveq	#8,d6
	moveq	#4,d0
	movec	d0,sfc
	moves.l	($A000).l,d1		; page 10 is invalid when translated
	chkl	d1,$A0A0A0A0,8
	moveq	#3,d0
	movec	d0,sfc
	moves.l	($9000).l,d1		; supervisor-only page, user-looking space
	chkl	d1,$90909090,9

;-------------------------- 10-13: the alternate spaces write physical addresses
	moveq	#10,d6
	moveq	#4,d0
	movec	d0,dfc
	move.l	#$0B0B0B0B,d1
	moves.l	d1,($B004).l		; lands at physical $B004
	move.l	($E004).l,d0		; page 14 reads physical $B000's page
	chkl	d0,$0B0B0B0B,10
	move.l	($B004).l,d0		; and page 11's physical $C004 is untouched
	chkl	d0,0,11
	moveq	#3,d0
	movec	d0,dfc
	move.l	#$08080808,d1
	moves.l	d1,($8004).l		; write-protected when translated
	move.l	($8004).l,d0
	chkl	d0,$08080808,12
	moveq	#0,d0
	movec	d0,dfc
	move.l	#$0A0A0A0A,d1
	moves.l	d1,($A004).l		; invalid when translated
	moveq	#4,d0
	movec	d0,sfc
	moves.l	($A004).l,d0
	chkl	d0,$0A0A0A0A,13

;-------------------- 14: an alternate-space access crossing a page is whole
; Physical $9FFE-$A001: page 9 is supervisor-only when translated and page
; $A invalid, so a transfer split at the page and checked page by page, as
; a translated one is, would fault on its second page. Untranslated, it is
; one misaligned transfer and lands.
	moveq	#14,d6
	move.l	#$55667788,d1
	moveq	#4,d0
	movec	d0,dfc
	moves.l	d1,($9FFE).l
	movec	d0,sfc
	moves.l	($9FFE).l,d0
	chkl	d0,$55667788,14

;---------------- 15: a program-space MOVES goes on the bus as user data
; The bench fails any write to $F120 whose function code is not 1.
	moveq	#15,d6
	moveq	#2,d0
	movec	d0,dfc
	moveq	#$15,d1
	moves.b	d1,(FCREG).l

;------------------ 16-17: an alternate-space read's physical bus error
; Read, byte, TT=10, TM=0, ATC clear -- no translation was involved -- and
; the fault address as given. Only where the bench can inject the error.
	moveq	#16,d6
	move.w	(CAPREG).l,d0
	btst	#2,d0
	beq.s	no_berr
	move.w	#1,(berr_expect).l
	moveq	#0,d0
	movec	d0,sfc
	moves.b	(BERRADR).l,d1		; faults once; the restart reads it
	tst.w	(berr_expect).l		; the handler clears it
	beq.s	berr_seen
	failt	16
berr_seen:
	move.w	(last_ssw).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$0130,16		; RW read, SIZE byte, TT 10, TM 0
	move.l	(last_fa).l,d0
	chkl	d0,BERRADR,17
no_berr:

;----------------------------------------------------------------- done
	moveq	#0,d0
	movec	d0,tc
	pflusha
	move.w	#$600D,(DONEREG).l
	stop	#$2700

;----------------------------------------------------------------- handlers
; Any access error but the one test 16 asks for is a failure: $E0nn, nn
; the test it happened in.
h_aerr:
	tst.w	(berr_expect).l
	beq.s	aerr_bad
	cmpi.w	#$7008,6(sp)		; format $7, vector 2
	bne.s	aerr_bad
	clr.w	(berr_expect).l
	move.w	$0C(sp),(last_ssw).l
	move.l	$14(sp),(last_fa).l
	rte
aerr_bad:
	move.w	d6,d7
	or.w	#$E000,d7
	bra	fail_all

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
