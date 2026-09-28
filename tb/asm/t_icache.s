; AP040 the pipelined core's instruction cache (caches stage B)
; assembled with vasmm68k_mot -Fbin -m68040 -no-opt
;
; testbench protocol: $F100 fail number, $F102 result magic;
; $F154 w: a one-shot bus error on an instruction fetch at the address
;
; MC68040UM section 4 as it applies to the instruction cache: 64 sets of
; four 16-byte lines, physically tagged (set = PA9-PA4), filled a whole
; line at a time; CPU writes never reach it (4.5), so a store into cached
; code runs the old code until CINV or CPUSH; CINV and CPUSH act on a line
; or a 4 KB page at a PHYSICAL address in An, or on everything, on the
; caches they name, whatever CACR says; a disabled cache is bypassed and
; kept; a cache-inhibited fetch (a TTR's or a page's CM 1x) bypasses it and
; allocates nothing; a line is valid only once all four longwords are in,
; and a bus error on a longword the program does not need faults nothing
; (4.6.1).
;
; Each probe is a two-instruction stub, MOVEQ #n,D0 then RTS, called with
; JSR and rewritten with MOVE.W over its MOVEQ: what D0 comes back as says
; whether the call ran the cached stub or memory's.
;
; The whole test runs translated, identity-mapped, with the pages of the
; program's own code (0-4) cache-inhibited by their descriptors: only the
; stubs are ever cached, so no line of the code can take a way and push a
; stub out.
;
; Pipelined core only: rtl/ap040/ap040_core.v widens every CINV/CPUSH to
; all lines (its header), so the tests that a line or page operation
; leaves the other lines alone do not hold there.
;
; memory map:
;   $0400 code, $3400 ISP top
;   $4000 root, $4200 pointer table, $4400 page table (64 x 4K pages):
;         identity; pages 0-4 and 6 cache-inhibited, 5 and 7 write-through;
;         logical page 8 = physical page 5
;   $5000 stubs (page 5): A $5000, B $5010, G $5020, H $5100, J $5200, and
;         set 0's $5400 $5800 $5C00
;   $6000 stub C (page 6: inhibited), $7000 stub K (page 7, set 0)

FAILREG		equ	$F100
DONEREG		equ	$F102
FBERRCTL	equ	$F154

SA		equ	$5000		; set 0, page 5
SB		equ	$5010		; set 1, page 5
SG		equ	$5020		; set 2, page 5
SH		equ	$5100		; set 16
SJ		equ	$5200		; set 32
S1		equ	$5400		; set 0
S2		equ	$5800		; set 0
S3		equ	$5C00		; set 0
SC		equ	$6000		; set 0, page 6 (inhibited)
SK		equ	$7000		; set 0, page 7
SW		equ	$7010		; set 1, page 7
AL		equ	$8000		; logical page 8: physical page 5, A's

cnt_aerr	equ	$3600		; access errors taken
aerr_fa		equ	$3604		; the last one's fault address
aerr_ok		equ	$3608		; the fault address the test allows

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

; stub <address>,<n>: MOVEQ #n,D0 ; RTS
stub	macro
	move.w	#$7000+\2,(\1).l
	move.w	#$4E75,(\1+2).l
	endm

; patch <address>,<n>: the MOVEQ alone
patch	macro
	move.w	#$7000+\2,(\1).l
	endm

; call <address>,<n expected>,<test>
call	macro
	moveq	#-1,d0
	jsr	(\1).l
	chkl	d0,\2,\3
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
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,itt0
	movec	d0,itt1
	movec	d0,dtt0
	movec	d0,dtt1
	clr.w	(cnt_aerr).l
	clr.l	(aerr_ok).l

;----------------------------------------------------------------- tables
	lea	($4400).l,a0
	moveq	#0,d0
	moveq	#63,d1
tloop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#4,d2		; i << 12
	addq.l	#3,d2		; resident, CM 00 (write-through)
	cmp.l	#5,d0
	bcc.s	tl_wt
	or.w	#$40,d2		; pages 0-4, the code's: CM 10, inhibited
tl_wt:
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,tloop
	move.l	#$00006043,($4418).l	; page 6: inhibited
	move.l	#$00005003,($4420).l	; logical page 8 -> physical page 5
	move.l	#$00004203,($4000).l
	move.l	#$00004403,($4200).l
	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	pflusha
	move.l	#$8000,d0	; E=1, 4K pages
	movec	d0,tc

	cinva	bc		; the caches' contents are undefined until then
	stub	SA,1
	stub	SB,1
	stub	SK,1
	move.l	#$80008000,d0
	movec	d0,cacr

;------------------------------------------------ 4.5: CPU writes miss it
	call	SA,1,1		; each fills its line
	call	SB,1,2
	call	SK,1,3
	patch	SA,2
	patch	SB,2
	patch	SK,2
	call	SA,1,4		; ...which the stores did not reach
	call	SB,1,5
	call	SK,1,6

;------------------------------------------------------- CINV line, page
	lea	(SA+4).l,a0	; any address in the line
	cinvl	ic,(a0)
	call	SA,2,7
	call	SB,1,8		; the next line, same page: kept
	call	SK,1,9		; the same set, another page: kept
	lea	($5FFC).l,a0	; any address in the page
	cinvp	ic,(a0)
	call	SB,2,10
	call	SK,1,11		; another page: kept
	cpusha	ic
	call	SK,2,12

;----------------------------------------------------- CPUSH line, page
	call	SA,2,13		; all three again, after CPUSHA
	call	SB,2,14
	call	SK,2,15
	patch	SA,3
	patch	SB,3
	patch	SK,3
	call	SA,2,16
	call	SB,2,17
	call	SK,2,18
	lea	(SA).l,a0
	cpushl	ic,(a0)
	call	SA,3,19
	call	SB,2,20
	lea	(SK+$800).l,a0
	cpushp	ic,(a0)
	call	SK,3,21
	call	SB,2,22
	cinva	ic
	call	SB,3,23

;------------------------------------ the data cache's operations only
	call	SA,3,24		; A again, after CINVA
	patch	SA,4
	call	SA,3,25
	cinva	dc
	call	SA,3,26		; CINV DC leaves the instruction cache
	lea	(SA).l,a0
	cinvl	dc,(a0)
	cpushl	dc,(a0)
	cpushp	dc,(a0)
	cpusha	dc
	call	SA,3,27
	cinva	bc		; both
	call	SA,4,28

;-------------------------------------- disabled: bypassed, and kept
	patch	SA,5
	move.l	#$80000000,d0	; IE clear
	movec	d0,cacr
	call	SA,5,29		; memory's
	move.l	#$80008000,d0
	movec	d0,cacr
	call	SA,4,30		; the line it held before, unchanged
	cinva	ic
	call	SA,5,31

;-------------------- cache-inhibited through ITT1: bypassed, no fills
; A matching TTR decides the caching mode ahead of the tables.
	stub	SG,6
	patch	SA,6
	move.l	#$0000C040,d0	; $00xxxxxx, either mode, CM 10 (inhibited)
	movec	d0,itt1
	call	SA,6,32		; memory's, not the line's 5
	call	SG,6,33
	moveq	#0,d0
	movec	d0,itt1
	patch	SG,7
	call	SG,7,34		; the inhibited fetch allocated nothing
	move.l	#$0000C000,d0	; CM 00: write-through, cached -- pages 0-4
	movec	d0,itt1		; too, briefly
	cinva	ic
	call	SG,7,35
	patch	SG,8
	call	SG,7,36
	moveq	#0,d0
	movec	d0,itt1
	cinva	ic

;--------------------------------------------------- four ways a set
	stub	SA,10
	stub	S1,10
	stub	S2,10
	stub	S3,10
	stub	SK,12
	call	SA,10,37	; four lines of set 0: the four invalid ways
	call	S1,10,38
	call	S2,10,39
	call	S3,10,40
	patch	SA,11
	patch	S1,11
	patch	S2,11
	patch	S3,11
	call	SA,10,41	; all four held
	call	S1,10,42
	call	S2,10,43
	call	S3,10,44
	call	SK,12,45	; a fifth replaces one of them
	moveq	#0,d6		; how many of the four come back rewritten
	moveq	#-1,d0
	jsr	(SA).l
	cmp.l	#11,d0
	bne.s	w0
	addq.l	#1,d6
w0:	moveq	#-1,d0
	jsr	(S1).l
	cmp.l	#11,d0
	bne.s	w1
	addq.l	#1,d6
w1:	moveq	#-1,d0
	jsr	(S2).l
	cmp.l	#11,d0
	bne.s	w2
	addq.l	#1,d6
w2:	moveq	#-1,d0
	jsr	(S3).l
	cmp.l	#11,d0
	bne.s	w3
	addq.l	#1,d6
w3:	tst.l	d6
	bne.s	w_ok
	failt	46		; four ways, five lines: one must have gone
w_ok:
	cinva	ic

;-------------------------- 4.6.1: an error on a beat nobody waits for
; H is MOVEQ #13,D0 / RTS in its line's third longword ($5108), NOPs in the
; fourth. The fill reads $5108 first, then $510C, $5100, $5104; a bus error
; on $5100 abandons the line and faults nothing -- the stream from $5108
; never asks for the line's first longword -- and the line is not valid:
; rewritten, H runs the new MOVEQ. (An error on a longword past H's RTS
; would not do: the fetch reads on past a return until it redirects, may
; ask for that longword, and fill the line again after the one-shot error.)
	move.l	#$4E714E71,(SH).l
	move.l	#$4E714E71,(SH+4).l
	stub	SH+8,13
	move.l	#$4E714E71,(SH+12).l
	clr.w	(cnt_aerr).l
	move.w	#SH,(FBERRCTL).l
	call	SH+8,13,47
	move.w	#0,(FBERRCTL).l
	tst.w	(cnt_aerr).l
	beq.s	h_noexc
	failt	48		; the abandoned beat surfaced as an exception
h_noexc:
	patch	SH+8,14
	call	SH+8,14,49	; the line was never valid

; J's line: NOPs, then MOVEQ #15,D0 / RTS in its third longword, with a
; bus error on it. Whether the error meets the program -- the stream reads
; ahead, and faults only a longword it is asked for -- is a matter of
; timing; if it does, the access error's fault address is $5208, and the
; restarted fetch succeeds (the error was one-shot). Either way the stub
; runs.
	move.l	#$4E714E71,(SJ).l
	move.l	#$4E714E71,(SJ+4).l
	stub	SJ+8,15
	move.l	#$4E714E71,(SJ+12).l
	clr.w	(cnt_aerr).l
	move.l	#SJ+8,(aerr_ok).l
	move.w	#SJ+8,(FBERRCTL).l
	call	SJ,15,50
	move.w	#0,(FBERRCTL).l
	clr.l	(aerr_ok).l
	cinva	ic

;--------------------------------- the page's caching mode; physical An
	stub	SA,20
	stub	SC,22
	call	SA,20,51
	patch	SA,21
	call	SA,20,52	; a write-through page: cached
	call	SC,22,53
	patch	SC,23
	call	SC,23,54	; an inhibited page: memory's each time
	call	AL,20,55	; the alias reaches the same physical line
	lea	(AL).l,a0	; a LOGICAL address: physical $8000, not A's line
	cinvl	ic,(a0)
	call	AL,20,56
	lea	(SA).l,a0	; A's physical line
	cinvl	ic,(a0)
	call	AL,21,57
	patch	SA,24
	lea	(AL).l,a0
	cinvp	ic,(a0)		; physical page 8: nothing of A's
	call	SA,21,58
	lea	(SA).l,a0
	cinvp	ic,(a0)
	call	AL,24,59

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

;----------------------------------------------------------------- handlers
; An access error is allowed only on the fetch the test names (aerr_ok):
; format $7, fault address that longword, a physical bus error (ATC clear).
; RTE restarts the fetch, which the one-shot error now lets through.
h_aerr:
	move.l	$14(sp),(aerr_fa).l
	cmpi.w	#$7008,6(sp)	; format $7, vector 2
	bne.s	h_bad
	move.l	(aerr_ok).l,d7
	beq.s	h_bad
	cmp.l	$14(sp),d7
	bne.s	h_bad
	move.w	$0C(sp),d7	; SSW
	andi.w	#$0400,d7	; ATC
	bne.s	h_bad
	addq.w	#1,(cnt_aerr).l
	rte
h_bad:
	move.w	#$00EE,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt3:
	bra.s	halt3

unexp:
	move.w	#$0099,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2
