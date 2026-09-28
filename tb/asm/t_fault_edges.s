; Access-fault edges the other programs leave unexercised. Every case runs
; on both cores (tests/ap040/run_verilator.py and run_pipe_verilator.py).
;
;  1-5   An instruction whose EXTENSION word's fetch faults. The frame is the
;        instruction's -- its PC -- and FA is the word that faulted, on the
;        next page; the handler maps the page and RTE runs it again, whole.
;        A pipelined core has already decoded the opcode by then, and must
;        give up the instruction it was gathering rather than finish it with
;        what the faulted fetch left behind.
;  6-10  The same for the THIRD word of a three-word instruction.
; 11-12  Code that ends exactly at a page end, with the next page invalid,
;        runs and returns: fetching ahead into that page must not fault.
; 13-17  A write refused with T1 set. The instruction does not complete, so it
;        is not traced then; the handler's RTE runs it again, and it is traced
;        once, after it has completed. Two traces in all: it, and the MOVE to
;        SR that clears T. A pipelined core that abandons a store after the
;        instruction has left for the stage that writes must also take back
;        the trace it had armed as it left.
; 18-23  A longword write crossing into a write-protected page. The 68040
;        checks both pages before writing any byte, so the handler finds the
;        first page untouched; the frame reports FA = the operand's first
;        byte, MA (the second page refused it), a longword write.
; 24-29  The same write with its data from a register instead of an
;        immediate -- the other store route on a pipelined core.
; 30-37  CLR and Scc on a write-protected page: written without being read,
;        refused, restarted -- once. ST (A0)+ leaves A0 one byte on, not two.
; 46-54  A plain load from an invalid page -- one a pipelined core sends on
;        to EX as its read goes out, so EX sees the fault after the
;        instruction has left the stage that raises it -- then the same
;        under T1: the frame is the load's own, its SSW a longword read,
;        and it is traced once, after the restart has completed it.
; 55-60  Three plain loads in a row under T1, then the MOVE to SR that clears
;        it: each is traced as it completes, the trace owed by one taken
;        before the next begins -- on a pipelined core, before the next load
;        is sent on -- so the handler sees four traces, whose stacked PCs are
;        the next instruction's, in order.
; 61-63  An FPU general operation whose EA is mode 7 with register 5-7, which
;        no EA is: an F-line exception, its PC the instruction's own.
; 64-67  FSAVE and FRESTORE under T0: each is a change of flow for tracing,
;        and traced as it completes -- and so is the MOVE to SR that clears
;        T0, which the reference takes as one too.
; 38-45  MOVEM.L (A0),D1-D4 with D3's longword on an invalid page. The
;        handler finds D1 and D2 loaded and D3 and D4 as they were: the
;        faulted beat writes nothing, and nothing the exception reads after
;        it lands in a register either. The RTE restarts the MOVEM (CM),
;        which then loads all four.
;
; Protocol (tb_ap040_program.v, tb_ap040_pipe_program.v): word write to
; $F100 = failing test number, $F102 = $BAD0 on failure, $600D when done.

FAILREG		equ	$F100
DONEREG		equ	$F102
cnt_aerr	equ	$3600
cnt_trc		equ	$3602
last_pc		equ	$3604
last_fa		equ	$3608
last_ssw	equ	$360C
fix_addr	equ	$3610
fix_val		equ	$3614
pre_wr		equ	$3618		; ($CFFC).l as the access-error handler found it
h_d2		equ	$361C		; D2-D4 as the access-error handler found them
h_d3		equ	$3620
h_d4		equ	$3624
pre_pd		equ	$3628		; page C's descriptor as the access-error handler found it
tr_log		equ	$3640		; the trace handler's stacked PCs, eight
cnt_fl		equ	$3660		; F-line exceptions taken
fl_pc		equ	$3664		; ...and the last one's stacked PC

failt	macro
	move.w	#\1,d7
	bra	fail
	endm

chkl	macro
	cmp.l	#\2,\1
	beq.s	ok\@
	failt	\3
ok\@:
	endm

	org	0
	dc.l	$3400,start
	dc.l	h_aerr			; 2: access fault
	rept	6
	dc.l	unexpected		; 3-8
	endr
	dc.l	h_trace			; 9: trace
	dc.l	unexpected		; 10
	dc.l	h_fline			; 11: F-line
	rept	244
	dc.l	unexpected		; 12-255
	endr

	org	$400
start:
	move.w	#$2700,sr
	clr.w	(cnt_aerr).l
	clr.w	(cnt_trc).l

;------------------------------------------------ 4K identity, $0-$3FFFF
	lea	($4400).l,a0
	moveq	#0,d0
	moveq	#63,d1
tloop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#4,d2			; i << 12
	addq.l	#3,d2			; resident
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,tloop
	move.l	#$00004203,($4000).l
	move.l	#$00004403,($4200).l
	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	move.l	#$8000,d0		; E=1, 4K pages
	movec	d0,tc
	pflusha

;------------------------------------------ 1-5: the extension word faults
	move.l	#$4420,(fix_addr).l	; page 8
	move.l	#$8003,(fix_val).l
	move.l	#0,($4420).l		; page 8 invalid
	pflusha
	moveq	#0,d0
	jsr	($7FFE).l		; MOVE.L #$12345678,D0 with its immediate on page 8
	chkl	d0,$12345678,1
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,1,2
	move.l	(last_pc).l,d0
	chkl	d0,$7FFE,3		; the instruction, not the word
	move.l	(last_fa).l,d0
	chkl	d0,$8000,4		; the word
	moveq	#0,d0
	move.w	(last_ssw).l,d0
	and.l	#$FF9F,d0		; SIZE is how the core fetched: not judged
	chkl	d0,$0506,5		; ATC + read + supervisor program

;------------------------------------------ 6-10: the third word faults
	move.l	#$442C,(fix_addr).l	; page B
	move.l	#$B003,(fix_val).l
	move.l	#0,($442C).l
	pflusha
	moveq	#0,d0
	jsr	($AFFC).l		; MOVE.L #$9ABCDEF0,D0, its low word on page B
	chkl	d0,$9ABCDEF0,6
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,2,7
	move.l	(last_pc).l,d0
	chkl	d0,$AFFC,8
	move.l	(last_fa).l,d0
	chkl	d0,$B000,9
	moveq	#0,d0
	move.w	(last_ssw).l,d0
	and.l	#$FF9F,d0
	chkl	d0,$0506,10

;------------------------------------ 11-12: fetching ahead must not fault
	move.l	#0,($4438).l		; page E invalid
	pflusha
	moveq	#0,d0
	jsr	($DFF8).l		; MOVEQ #7,D0; NOP; NOP; RTS ending at $DFFF
	chkl	d0,7,11
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,2,12			; no fault for the page never reached
	move.l	#$E003,($4438).l
	pflusha

;--------------------------- 13-17: a refused write under T1 is traced once
	move.l	#0,($9000).l
	move.l	#$4424,(fix_addr).l	; page 9
	move.l	#$9003,(fix_val).l
	move.l	#$9007,($4424).l	; page 9 write-protected
	pflusha
	clr.w	(cnt_trc).l
	move.w	#$A700,sr		; T1
	move.l	#$DEADBEEF,($9000).l	; refused, restarted, then traced
	move.w	#$2700,sr		; traced: T1 at its start
	move.l	($9000).l,d0
	chkl	d0,$DEADBEEF,13
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,3,14
	moveq	#0,d0
	move.w	(cnt_trc).l,d0
	chkl	d0,2,15			; not a third, for the refused attempt
	move.l	(last_fa).l,d0
	chkl	d0,$9000,16
	moveq	#0,d0
	move.w	(last_ssw).l,d0
	chkl	d0,$0405,17		; ATC + long write + supervisor data

;------------------------ 18-23: a crossing write refused by its second page
	move.l	#0,($CFFC).l
	move.l	#0,($D000).l
	move.l	#$4434,(fix_addr).l	; page D
	move.l	#$D003,(fix_val).l
	move.l	#$D007,($4434).l	; page D write-protected
	move.l	#$C003,($4430).l	; page C resident, unused, unmodified
	pflusha
	move.l	#$11223344,($CFFE).l	; two bytes on page C, two on page D
	move.l	(pre_wr).l,d0
	chkl	d0,0,18			; nothing written before the fault
	move.l	($CFFC).l,d0
	chkl	d0,$00001122,19		; the restart wrote both pages
	move.l	($D000).l,d0
	chkl	d0,$33440000,20
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,4,21
	move.l	(last_fa).l,d0
	chkl	d0,$CFFE,22		; the operand's first byte
	moveq	#0,d0
	move.w	(last_ssw).l,d0
	chkl	d0,$0C05,23		; MA + ATC + long write + supervisor data
	move.l	(pre_pd).l,d0
	and.l	#$10,d0
	chkl	d0,0,68			; nor marked page C modified before the fault

;------------------------------- 24-29: the same, the data from a register
	move.l	#0,($CFFC).l
	move.l	#0,($D000).l
	move.l	#$D007,($4434).l
	pflusha
	move.l	#$55667788,d1
	move.l	d1,($CFFE).l
	move.l	(pre_wr).l,d0
	chkl	d0,0,24
	move.l	($CFFC).l,d0
	chkl	d0,$00005566,25
	move.l	($D000).l,d0
	chkl	d0,$77880000,26
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,5,27
	move.l	(last_fa).l,d0
	chkl	d0,$CFFE,28
	moveq	#0,d0
	move.w	(last_ssw).l,d0
	chkl	d0,$0C05,29

;------------- 30-37: CLR and Scc, written without a read, refused and restarted
	move.l	#$A5A5A5A5,($9000).l
	move.l	#$4424,(fix_addr).l	; page 9
	move.l	#$9003,(fix_val).l
	move.l	#$9007,($4424).l	; page 9 write-protected
	pflusha
	clr.l	($9000).l
	move.l	($9000).l,d0
	chkl	d0,0,30
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,6,31
	move.l	(last_fa).l,d0
	chkl	d0,$9000,32
	moveq	#0,d0
	move.w	(last_ssw).l,d0
	chkl	d0,$0405,33		; ATC + long write + supervisor data
	lea	($9004).l,a0
	move.b	#0,($9004).l		; the handler left page 9 writable
	move.l	#$9007,($4424).l	; protected again
	pflusha
	st	(a0)+			; refused, restarted
	moveq	#0,d0
	move.b	($9004).l,d0
	chkl	d0,$FF,34
	move.l	a0,d0
	chkl	d0,$9005,35		; stepped once, not once per attempt
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,7,36
	moveq	#0,d0
	move.w	(last_ssw).l,d0
	chkl	d0,$0425,37		; ATC + byte write + supervisor data

;------------ 38-45: a MOVEM load faults on its third register, and restarts
	move.l	#$11111111,($5FF8).l
	move.l	#$22222222,($5FFC).l
	move.l	#$33333333,($6000).l
	move.l	#$44444444,($6004).l
	move.l	#$4418,(fix_addr).l	; page 6
	move.l	#$6003,(fix_val).l
	move.l	#0,($4418).l		; page 6 invalid
	pflusha
	move.l	#$D1D1D1D1,d1
	move.l	#$D2D2D2D2,d2
	move.l	#$D3D3D3D3,d3
	move.l	#$D4D4D4D4,d4
	lea	($5FF8).l,a0
	movem.l	(a0),d1-d4
	chkl	d1,$11111111,38
	chkl	d4,$44444444,39
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,8,40
	move.l	(last_fa).l,d0
	chkl	d0,$6000,41
	move.l	(h_d2).l,d0
	chkl	d0,$22222222,42		; loaded before the fault
	move.l	(h_d3).l,d0
	chkl	d0,$D3D3D3D3,43		; the faulted beat's: untouched
	move.l	(h_d4).l,d0
	chkl	d0,$D4D4D4D4,44
	moveq	#0,d0
	move.w	(last_ssw).l,d0
	and.l	#$FF9F,d0		; SIZE is how the core read: not judged
	chkl	d0,$1505,45		; CM + ATC + read + supervisor data

;----------------- 46-53: a plain load faults, then the same under T1
	move.l	#$0BEEF000,($6008).l	; page 6 is valid again here
	move.l	#0,($4418).l		; and invalid
	pflusha
	moveq	#0,d5
ld46:	move.l	($6008).l,d5		; refused, restarted
	chkl	d5,$0BEEF000,46
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,9,47
	move.l	(last_fa).l,d0
	chkl	d0,$6008,48
	moveq	#0,d0
	move.w	(last_ssw).l,d0
	chkl	d0,$0505,49		; ATC + long read + supervisor data
	move.l	(last_pc).l,d0
	chkl	d0,ld46,54		; the load's own address
	move.l	#0,($4418).l
	pflusha
	clr.w	(cnt_trc).l
	moveq	#0,d5
	move.w	#$A700,sr		; T1
ld50:	move.l	($6008).l,d5		; refused, restarted, then traced
	move.w	#$2700,sr		; traced: T1 at its start
	chkl	d5,$0BEEF000,50
	moveq	#0,d0
	move.w	(cnt_aerr).l,d0
	chkl	d0,10,51
	moveq	#0,d0
	move.w	(cnt_trc).l,d0
	chkl	d0,2,52			; not a third, for the refused attempt
	move.l	(last_pc).l,d0
	chkl	d0,ld50,53

;------------------------- 55-60: loads in a row under T1, each traced
	move.l	#$11110000,($5000).l
	move.l	#$22220000,($5004).l
	move.l	#$33330000,($5008).l
	lea	($5000).l,a1
	clr.w	(cnt_trc).l
	move.w	#$A700,sr		; T1 from the next instruction
ld55a:	move.l	(a1),d1
ld55b:	move.l	4(a1),d2
ld55c:	move.l	8(a1),d3
ld55d:	move.w	#$2700,sr		; traced: T1 at its start
ld55e:	moveq	#0,d0
	move.w	(cnt_trc).l,d0
	chkl	d0,4,55
	move.l	(tr_log).l,d0
	chkl	d0,ld55b,56
	move.l	(tr_log+4).l,d0
	chkl	d0,ld55c,57
	move.l	(tr_log+8).l,d0
	chkl	d0,ld55d,58
	move.l	(tr_log+12).l,d0
	chkl	d0,ld55e,59
	add.l	d2,d1
	add.l	d3,d1
	chkl	d1,$66660000,60

;---------------------------- 61-63: an FPU EA that is no EA is an F-line
	clr.w	(cnt_fl).l
fl61:	dc.w	$F23D,$4000		; FMOVE.L <mode 7, register 5>,FP0
	moveq	#0,d0
	move.w	(cnt_fl).l,d0
	chkl	d0,1,61
	move.l	(fl_pc).l,d0
	chkl	d0,fl61,62		; the instruction's own address
fl63:	dc.w	$F23E,$4000		; register 6: the same
	moveq	#0,d0
	move.w	(cnt_fl).l,d0
	chkl	d0,2,63

;------------------------------- 64-66: FSAVE and FRESTORE traced under T0
	clr.w	(cnt_trc).l
	move.w	#$6700,sr		; T0
fs64:	fsave	-(sp)
fr64:	frestore (sp)+
fe64:	move.w	#$2700,sr
fn64:	moveq	#0,d0
	move.w	(cnt_trc).l,d0
	chkl	d0,3,64
	move.l	(tr_log).l,d0
	chkl	d0,fr64,65		; FSAVE's trace: the next instruction
	move.l	(tr_log+4).l,d0
	chkl	d0,fe64,66
	move.l	(tr_log+8).l,d0
	chkl	d0,fn64,67

	moveq	#0,d0
	movec	d0,tc
	pflusha
	move.w	#$600D,(DONEREG).l
	stop	#$2700

fail:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1

unexpected:
	move.w	#$00FF,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2

; The access error: record the frame, apply the test's fix, run it again.
h_aerr:
	move.l	d2,(h_d2).l
	move.l	d3,(h_d3).l
	move.l	d4,(h_d4).l
	movem.l	d0/a0,-(sp)
	cmpi.w	#$7008,14(sp)		; format $7, vector 2
	bne	unexpected
	addq.w	#1,(cnt_aerr).l
	move.l	10(sp),(last_pc).l
	move.l	28(sp),(last_fa).l
	move.w	20(sp),(last_ssw).l
	move.l	($CFFC).l,(pre_wr).l
	move.l	($4430).l,(pre_pd).l
	move.l	(fix_addr).l,a0
	move.l	(fix_val).l,(a0)
	pflusha
	movem.l	(sp)+,d0/a0
	rte

; F-line: count it, keep its PC, and step over the two-word instruction.
h_fline:
	addq.w	#1,(cnt_fl).l
	move.l	2(sp),(fl_pc).l
	addq.l	#4,2(sp)
	rte

h_trace:
	movem.l	d0/a0,-(sp)
	moveq	#0,d0
	move.w	(cnt_trc).l,d0
	and.w	#7,d0
	lsl.w	#2,d0
	lea	(tr_log).l,a0
	move.l	10(sp),0(a0,d0.w)	; the frame's PC, past the two saved
	movem.l	(sp)+,d0/a0
	addq.w	#1,(cnt_trc).l
	rte

; The code the tests call, placed against page boundaries.
	org	$7FFE
	move.l	#$12345678,d0		; opcode $7FFE, immediate $8000-$8003
	rts				; $8004

	org	$AFFC
	move.l	#$9ABCDEF0,d0		; $AFFC, $AFFE, $B000
	rts				; $B002

	org	$DFF8
	moveq	#7,d0			; $DFF8
	nop				; $DFFA
	nop				; $DFFC
	rts				; $DFFE: the page ends here
