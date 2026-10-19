; AP68040: the NeXT kernel's page-table shape, reproduced from a hardware dump
; (2026-10-19): 8 KB pages (TC $C000), the kernel's own memory under a
; supervisor-only copyback transparent translation (ITT0/DTT0), and a VA in
; the 0x10000000 window mapped by a three-level table whose root and pointer
; descriptors are type 10 (not 11, as every other test here writes them) and
; whose page descriptor is resident, U, M, copyback, global ($...439).  The
; tables are written with the data cache on (copyback) and never pushed, as
; the kernel does.  On the board a supervisor write through such a mapping
; faulted (ATC fault) on every retry although the tables were valid.
;   1    the mapping valid before the access: a write and a read through it
;        reach the physical page, no fault
;   2    the page descriptor invalid at first: the access error handler
;        makes it valid and PFLUSHANs; the retried write must succeed
;        (exactly one fault)
;   3    the same with PFLUSHA
;   4    the same with a descriptor the handler had READ (a clean cached
;        copy) before writing it
;   5    the same for a pointer descriptor made valid by the handler
FAILREG	equ	$F100
DONEREG	equ	$F102

ROOT	equ	$20000
PTR	equ	$20200
PAGE	equ	$20400		; 32 entries of 8 KB pages
PTR2	equ	$20600		; for test 5
PAGE2	equ	$20800
PHYS	equ	$30000		; physical pages $30000, $32000, $34000, $36000
VA	equ	$10004494	; root 8, pointer 0, page 2
VA3	equ	$10006494	; page 3
VA4	equ	$10008494	; page 4
VA5	equ	$10044494	; root 8, pointer 1, page 2 (PTR2/PAGE2)
VA6	equ	$1000a494	; page 5

cnt_aerr	equ	$3600
fix_addr	equ	$3604
fix_val		equ	$3608
flush_mode	equ	$360C		; 0 pflushan, 1 pflusha
last_fa		equ	$3610
last_ssw	equ	$3614

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
	dc.l	h_aerr		; 2 access error
	rept	253
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#$2700,sr
	clr.l	(cnt_aerr).l
	clr.l	(flush_mode).l
	move.l	#$80008000,d0		; caches on: the tables are written copyback
	movec	d0,cacr

; the kernel's transparent translation for its own memory: ITT0 = DTT0 =
; base 0, mask 0 (this 16 MB), E, S = supervisor only, CM = copyback
	move.l	#$0000A020,d0
	movec	d0,itt0
	movec	d0,dtt0
	moveq	#0,d0
	movec	d0,itt1
	movec	d0,dtt1

; the tables, as the kernel writes them (cached stores, never pushed)
	lea	(ROOT).l,a0
	moveq	#0,d0
	move.w	#127,d1
.cr:	move.l	d0,(a0)+
	dbf	d1,.cr
	lea	(PTR).l,a0
	move.w	#127,d1
.cp:	move.l	d0,(a0)+
	dbf	d1,.cp
	lea	(PTR2).l,a0
	move.w	#127,d1
.cp2:	move.l	d0,(a0)+
	dbf	d1,.cp2
	lea	(PAGE).l,a0
	move.w	#31,d1
.cg:	move.l	d0,(a0)+
	dbf	d1,.cg
	lea	(PAGE2).l,a0
	move.w	#31,d1
.cg2:	move.l	d0,(a0)+
	dbf	d1,.cg2
	move.l	#PTR+$A,(ROOT+8*4).l		; root[8]: type 10, U
	move.l	#PAGE+$A,(PTR+0).l		; ptr[0]:  type 10, U
	move.l	#PHYS+$439,(PAGE+2*4).l		; page[2]: resident U M CM=copyback G
	; page[3], page[4], page[5], ptr[1] stay invalid for tests 2-5

; the physical pages carry a pattern to see what a read returns
	move.l	#$11111111,(PHYS).l
	move.l	#$22222222,(PHYS+$494).l
	move.l	#$33333333,(PHYS+$2494).l
	move.l	#$44444444,(PHYS+$4494).l
	move.l	#$55555555,(PHYS+$6494).l

; enable: SRP (and URP), TC = E + 8K pages, as the kernel does
	move.l	#ROOT,d0
	movec	d0,srp
	movec	d0,urp
	pflusha
	move.l	#$C000,d0
	movec	d0,tc

;------------------------------------------------- 1 the valid mapping
	move.l	(VA).l,d0
	chkl	d0,$22222222,1			; read through the mapping
	move.l	#$600D0001,(VA).l		; the write that looped on the board
	move.l	(PHYS+$494).l,d0
	chkl	d0,$600D0001,2			; reached the physical page
	move.l	(cnt_aerr).l,d0
	chkl	d0,0,3				; and no fault

;------------------------------------------------- 2 fault, fix, PFLUSHAN, retry
	move.l	#PAGE+3*4,(fix_addr).l
	move.l	#PHYS+$2000+$439,(fix_val).l
	move.l	#$600D0002,(VA3).l
	move.l	(cnt_aerr).l,d0
	chkl	d0,1,4				; exactly one fault
	move.l	(PHYS+$2494).l,d0
	chkl	d0,$600D0002,5			; the retried write landed
	move.l	(VA3).l,d0
	chkl	d0,$600D0002,6

;------------------------------------------------- 3 the same with PFLUSHA
	move.l	#1,(flush_mode).l
	move.l	#PAGE+4*4,(fix_addr).l
	move.l	#PHYS+$4000+$439,(fix_val).l
	move.l	#$600D0003,(VA4).l
	move.l	(cnt_aerr).l,d0
	chkl	d0,2,7
	move.l	(PHYS+$4494).l,d0
	chkl	d0,$600D0003,8
	clr.l	(flush_mode).l

;------------------------------------------------- 4 a clean cached copy of the descriptor first
	move.l	(PAGE+5*4).l,d0			; read it (invalid, 0): the line is now cached
	chkl	d0,0,9
	cpusha	dc				; and clean in the cache, current in memory
	move.l	#PAGE+5*4,(fix_addr).l
	move.l	#PHYS+$6000+$439,(fix_val).l
	move.l	#$600D0004,(VA6).l
	move.l	(cnt_aerr).l,d0
	chkl	d0,3,10
	move.l	(PHYS+$6494).l,d0
	chkl	d0,$600D0004,11

;------------------------------------------------- 5 a pointer descriptor made valid by the handler
	move.l	#PHYS+$439,(PAGE2+2*4).l	; page2[2] valid already (maps VA5 to PHYS)
	move.l	#PTR+1*4,(fix_addr).l		; ptr[1] -> PAGE2, written by the handler
	move.l	#PAGE2+$A,(fix_val).l
	move.l	#$600D0005,(VA5).l
	move.l	(cnt_aerr).l,d0
	chkl	d0,4,12
	move.l	(PHYS+$494).l,d0
	chkl	d0,$600D0005,13

	bsr	nocache				; the bench registers: not through the copyback TTR
	move.w	#$600D,(DONEREG).l
	bra.s	*

; DTT0 to supervisor noncachable, so the result writes reach the bench
nocache:
	move.l	#$0000A040,d0
	movec	d0,dtt0
	cpusha	dc
	rts

; access error: format 7 vector 2, an ATC fault; make the descriptor at
; fix_addr valid (a cached store, as the kernel's), flush, restart
h_aerr:
	cmpi.w	#$7008,6(sp)
	bne	hfail
	movem.l	d0/a0,-(sp)
	move.w	$14(sp),(last_ssw).l		; SSW (frame +$C, after movem +8)
	move.l	$1C(sp),(last_fa).l		; FA  (frame +$14)
	move.w	$14(sp),d0
	and.w	#$0400,d0			; must be an ATC fault
	beq	hfail
	movea.l	(fix_addr).l,a0
	move.l	(fix_val).l,(a0)
	tst.l	(flush_mode).l
	bne.s	.pa
	pflushan
	bra.s	.done
.pa:	pflusha
.done:	addq.l	#1,(cnt_aerr).l
	move.l	(cnt_aerr).l,d0
	cmp.l	#8,d0				; a loop: more faults than the tests take
	bhi	hfail
	movem.l	(sp)+,d0/a0
	rte

hfail:	failt	99
unexp:	failt	98

fail_all:
	bsr	nocache
	move.w	d7,(FAILREG).l
	move.w	#$BAD,(DONEREG).l
	bra.s	*
