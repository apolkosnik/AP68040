; AP040 table searches see the program's stores
; assembled with vasmm68k_mot -Fbin -m68040
;
; testbench protocol: $F100 fail number, $F102 result magic
;
; A table search reads descriptors from memory, over the table walker's own
; port on both cores here. A descriptor the program stored before the access
; that needs the search must be read as stored: the 68040 keeps its bus in
; program order, and so do these cores -- the sequential one holds its
; walker behind the posted-store drain (ap040_tg68k_compat.v), and the
; pipelined one holds its walker while any write its DMU has accepted has
; still to reach memory (ap040_pipe_mmu.v's walk_hold). The pipelined core
; translates beside its bus controller, so without that hold a search starts
; as soon as the access asks for it, while the store is still queued.
;
; Each test stores a descriptor for a region nothing has touched -- so no
; ATC entry, resident or not, stands in the way and no PFLUSH is needed --
; behind another store that keeps the bus busy (a misaligned longword: three
; bus cycles), and then at once makes an access that must search through
; the new descriptor. Searched before the store lands, the descriptor is
; still invalid and the access takes an access error, which fails the test
; it happened in.
;
; memory map (physical = logical below $40000 through root entry 0):
;   $0000 vectors, $0400 code, $3400 ISP top
;   $3000 the longword every alias below reaches
;   $3100 the stores that keep the bus busy
;   $4000 root table, $4200 pointer table, $4400 page table (64 x 4K pages)
;   root entry 0 -> pointer table, pointer entry 0 -> page table
;   made valid by the tests, one each:
;     1  root 1    -> pointer table: LA $02003000 is physical $3000 (read)
;     2  pointer 1 -> page table:    LA $00043000 is physical $3000 (read)
;     3  root 3    -> pointer table: LA $06003000 is physical $3000 (write)

FAILREG		equ	$F100
DONEREG		equ	$F102

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

	move.l	#$00004203,($4000).l	; root 0 -> pointer table
	clr.l	($4004).l		; roots 1-3 invalid
	clr.l	($4008).l
	clr.l	($400C).l
	move.l	#$00004403,($4200).l	; pointer 0 -> page table
	clr.l	($4204).l		; pointer 1 invalid
	move.l	#$5A5A1234,($3000).l

;----------------------------------------------------------------- enable
	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	pflusha
	move.l	#$8000,d0		; E=1, 4K pages
	movec	d0,tc
	; The pages the stores go to, into the ATC: each descriptor store then
	; translates at once and waits only for the bus.
	move.l	($4000).l,d0
	move.l	($3000).l,d0
	chkl	d0,$5A5A1234,10

;------------------------------------------------ 1: a root descriptor, read
	moveq	#1,d6
	move.l	#$11111111,($3101).l	; three bus cycles ahead of the next
	move.l	#$00004203,($4004).l	; root 1 -> pointer table
	move.l	($02003000).l,d0	; searched through root 1
	chkl	d0,$5A5A1234,1
	move.l	($4004).l,d0
	chkl	d0,$0000420B,2		; the search marked it used

;--------------------------------------------- 2: a pointer descriptor, read
	moveq	#2,d6
	move.l	#$22222222,($3105).l
	move.l	#$00004403,($4204).l	; pointer 1 -> page table
	move.l	($00043000).l,d0	; searched through pointer 1
	chkl	d0,$5A5A1234,3
	move.l	($4204).l,d0
	chkl	d0,$0000440B,4

;----------------------------------------------- 3: a root descriptor, write
	moveq	#3,d6
	move.l	#$33333333,($3109).l
	move.l	#$00004203,($400C).l	; root 3 -> pointer table
	move.l	#$C0DEF00D,($06003000).l	; searched through root 3
	move.l	($3000).l,d0
	chkl	d0,$C0DEF00D,5
	move.l	($400C).l,d0
	chkl	d0,$0000420B,6

	; and the stores ahead of them all landed
	move.l	($3101).l,d0
	chkl	d0,$11111111,7
	move.l	($3105).l,d0
	chkl	d0,$22222222,8
	move.l	($3109).l,d0
	chkl	d0,$33333333,9

;----------------------------------------------------------------- done
	moveq	#0,d0
	movec	d0,tc
	pflusha
	move.w	#$600D,(DONEREG).l
	stop	#$2700

;----------------------------------------------------------------- handlers
; Any access error is a failure: $E0nn, nn the test it happened in.
h_aerr:
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
