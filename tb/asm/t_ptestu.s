; AP68040: PTEST's table search updates U (and M) in the descriptors as an
; ordinary search does (MC68040UM 3.2.5, 3.7.3, table 3-1).  GitHub issue
; 17: a PTESTW of a write-protected page with U clear must set U (table
; 3-1, U 0 M 0 WP 1 write: a locked read-modify-write setting U; M stays
; clear).
; assembled with vasmm68k_mot -Fbin -m68040
;
; Translation off (TC 0), so only PTEST searches the tables; SRP $60000,
; DFC 5 (supervisor data), root[8] -> pointer table $60200 -> page table
; $60400: page 0 $80001, page 1 $81001, page 2 $82005 (W).  The issue's
; program, three times: caches off (tests 1x), the data cache on with
; translation off's default write-through (2x), and the data cache on with
; DTT0 mapping the tables copyback, so they are dirty in the cache when
; PTEST searches them (3x).  The descriptors are read from memory after
; CPUSHA with the cache off.
;   x1-x2 root and pointer descriptors: U set
;   x3    page 0 after PTESTR: U
;   x4    page 1 after PTESTW: U and M
;   x5    page 2 (write protected) after PTESTW: U only
;   x6    the MMUSR of that PTESTW: W and R
;   98    an unexpected exception

FAILREG	equ	$F100
DONEREG	equ	$F102

failt	macro			; the pass's base (d7) plus the test
	add.w	#\1,d7
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
	moveq	#0,d0
	movec	d0,dtt1
	movec	d0,itt0
	movec	d0,itt1
	cinva	bc
	moveq	#10,d7			; caches off
	moveq	#0,d0
	moveq	#0,d1
	bsr	pass
	moveq	#20,d7			; data cache, write-through
	moveq	#0,d0
	move.l	#$80000000,d1
	bsr	pass
	moveq	#30,d7			; data cache, DTT0: $00xxxxxx copyback
	move.l	#$0000C020,d0
	move.l	#$80000000,d1
	bsr	pass
	move.w	#$600d,(DONEREG).l
.d:	bra.s	.d

; d0 DTT0, d1 CACR, d7 the pass's base fail code
pass:
	movec	d0,dtt0
	movec	d1,cacr
	move.l	#$00060202,($60000+8*4).l	; root[8]: pointer table, resident
	move.l	#$00060402,($60200).l		; pointer[0]: page table, resident
	move.l	#$00080001,($60400).l		; page 0
	move.l	#$00081001,($60404).l		; page 1
	move.l	#$00082005,($60408).l		; page 2, W
	move.l	#$00060000,d0
	movec	d0,srp
	moveq	#5,d0
	movec	d0,dfc			; supervisor data
	move.l	#$10000000,a0
	ptestr	(a0)
	move.l	#$10001000,a0
	ptestw	(a0)
	move.l	#$10002000,a0
	ptestw	(a0)			; page 2, write protected
	movec	mmusr,d6
	pflusha
	cpusha	dc
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,dtt0
	move.l	($60000+8*4).l,d1
	chkl	d1,$0006020A,1
	move.l	($60200).l,d2
	chkl	d2,$0006040A,2
	move.l	($60400).l,d3
	chkl	d3,$00080009,3		; U
	move.l	($60404).l,d4
	chkl	d4,$00081019,4		; U, M
	move.l	($60408).l,d5
	chkl	d5,$0008200D,5		; U (issue 17)
	and.l	#$FFFFF007,d6
	chkl	d6,$00082005,6		; page 2's frame, W, R
	rts

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$bad0,(DONEREG).l
.f:	bra.s	.f

unexp:
	move.w	#98,(FAILREG).l
	move.w	#$bad0,(DONEREG).l
.u:	bra.s	.u
