; AP68060: a BSR.L whose target page is invalid -- the NetBSD wedge
; captured on the Minimig (AP060 -005, JTAG ring): BSR.L at $0E010628 to
; $0E00F452, 8K pages, the target page's descriptor zero.  The ring showed
; the table search (root, pointer, page descriptor = 0) as the last bus
; transfer; the processor then retired nothing and took no interrupt.
; MC68060UM 8.4.4.2: the fetch of the target's opword takes an access
; error, format $4, FSLW IO|PF, fault address and stacked PC the target.
;
; 8K pages, identity mapped, copyback, branch cache on:
;   page 0 $0000 vectors, code       page 1 $2000 stack, variables
;   page 2 $4000 tables (CI)         page 5 $A000 the called routine
;   page 6 $C000 the call sites      page 7 $E000 bench MMIO (CI)
;
; A  the first call, nothing learned: the target page invalid
; B  the branch cache has learned the BSR (32 calls with the page valid),
;    then the page is unmapped (descriptor cleared, PFLUSH of the page)
; C  as B with a level-2 interrupt pending under mask 7 at the fault
;    and released inside the access error handler
; E  as A in user mode: the NetBSD case (a shared library's page not yet
;    faulted in -- the captured walk went through the user root)
; F  as B in user mode
; G  as E and H as F, every data cache line dirty first: the BSR's push
;    misses, evicts a dirty line and fills while the fetch of the target
;    searches the tables (the order in the captured ring)
; I  as F with a level-2 interrupt arriving at swept times around the
;    fault, mask 0 (the capture had level 6 pending, unserved): the core
;    bench raises it a swept number of clocks after the arming write, the
;    Minimig bench at random
; J  a five-word instruction from $9FFC whose last three words are in the
;    invalid page: two good words and the two faulted words of $A000 are
;    queued, fewer than the instruction (the same wait for words that
;    never come); FA and stacked PC are the opword's (8.4.4.2), IO|MA

FAILREG		equ	$F100
DONEREG		equ	$F102
IPLREG		equ	$F110
IPLDLY		equ	$F148	; core bench: raise IPL 2 after this many clocks
RNDLVL		equ	$F112	; Minimig bench: random interrupts at this level
RNDMAX		equ	$F114	; ... each after 1..this many clocks

cnt_aerr	equ	$2000
last_fa		equ	$2004
last_pc		equ	$2008
last_fslw	equ	$200C
fix_addr	equ	$2010
fix_val		equ	$2014
cnt_int2	equ	$2018
irq_rel		equ	$201C	; nonzero: the handler releases the level-2 request
uret		equ	$2020
j_var		equ	$2024	; written by J's instruction	; where TRAP #1 returns in supervisor mode

; FASTOFS (assemble with -DFASTOFS=$200000 for the Minimig bench): pages
; 3-6 map to physical FASTOFS + their address -- Zorro II Fast RAM there,
; cacheable, where NetBSD's pages live (chip RAM is TCI in the adapter)
	ifnd	FASTOFS
FASTOFS		equ	0
	endif

PT		equ	$4400	; page table: 32 x 8K
PTE5		equ	PT+5*4
PTE6		equ	PT+6*4
TGT		equ	$A002	; target routine (bit 1 set, as $0E00F452)
CALLS		equ	$C000	; call site loop

F_IO		equ	$00008000
F_R		equ	$01000000
F_TM6		equ	$00060000
F_TM2		equ	$00020000
F_TMM		equ	$00070000
F_PF		equ	$00000200
F_MA		equ	$08000000

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
	dc.l	$3800
	dc.l	start
	dc.l	h_aerr		; 2 access error
	rept	23
	dc.l	unexp		; 3-25
	endr
	dc.l	h_int2		; 26 level-2 autovector
	rept	6
	dc.l	unexp		; 27-32
	endr
	dc.l	h_utrap		; 33 TRAP #1: back to supervisor
	rept	222
	dc.l	unexp
	endr

	org	$400
start:
	move.w	#$2700,sr
	clr.l	(cnt_aerr).l
	clr.l	(cnt_int2).l
	clr.l	(irq_rel).l
; root $4000 -> pointer $4200 -> page table $4400 (8K: 32 entries)
	lea	($4000).l,a0
	move.w	#255,d1
clr_t:	clr.l	(a0)+
	dbra	d1,clr_t
	lea	(PT).l,a0
	moveq	#0,d0
	moveq	#7,d1
pt_l:	move.l	d0,d2
	or.l	#$21,d2			; resident, copyback
	move.l	d2,(a0)+
	add.l	#$2000,d0
	dbra	d1,pt_l
	lea	(PT+3*4).l,a0
	moveq	#3,d1
pt_f:	add.l	#FASTOFS,(a0)+		; pages 3-6 -> Fast RAM
	dbra	d1,pt_f
	if	FASTOFS
	move.b	#$20,($E80048).l	; configure the Zorro II card
	lea	(TGT).l,a0		; the routine and the call site
	lea	(FASTOFS+TGT).l,a1
	moveq	#63,d1
cp_a:	move.l	(a0)+,(a1)+
	dbra	d1,cp_a
	lea	(CALLS).l,a0
	lea	(FASTOFS+CALLS).l,a1
	moveq	#63,d1
cp_c:	move.l	(a0)+,(a1)+
	dbra	d1,cp_c
	endif
	move.l	#$4041,(PT+2*4).l	; tables: cache inhibited, precise
	move.l	#$E041,(PT+7*4).l	; bench MMIO: cache inhibited, precise
	move.l	#$00004203,($4000).l
	move.l	#$00004403,($4200).l

	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	moveq	#5,d0
	movec	d0,dfc		; PFLUSH (An) flushes supervisor entries
	pflusha
	move.l	#$C000,d0		; E=1, P=1: 8K pages
	movec	d0,tc
	move.l	#$A0808000,d0		; EDC, ESB, EBC, EIC
	movec	d0,cacr

;-------------------------------------------- A: first call, page invalid
	clr.l	(PTE5).l
	lea	(TGT).l,a0
	pflush	(a0)
	moveq	#0,d6
	moveq	#1,d0
	move.l	#FASTOFS+$A021,(fix_val).l
	move.l	#PTE5,(fix_addr).l
	jsr	(CALLS).l
	chkl	d6,1,1
	move.l	(cnt_aerr).l,d0
	chkl	d0,1,2
	move.l	(last_fa).l,d0
	chkl	d0,TGT,3
	move.l	(last_pc).l,d0
	chkl	d0,TGT,4
	move.l	(last_fslw).l,d0
	and.l	#F_IO|F_R|F_PF,d0
	chkl	d0,F_IO|F_R|F_PF,5

;----------------------------------- B: learned BSR, then page unmapped
	moveq	#0,d6
	moveq	#32,d0
	jsr	(CALLS).l
	chkl	d6,32,10
	clr.l	(PTE5).l
	lea	(TGT).l,a0
	pflush	(a0)
	moveq	#0,d6
	moveq	#1,d0
	jsr	(CALLS).l
	chkl	d6,1,11
	move.l	(cnt_aerr).l,d0
	chkl	d0,2,12
	move.l	(last_pc).l,d0
	chkl	d0,TGT,13

;------------------------- C: as B, a level-2 request pending at the fault
	moveq	#0,d6
	moveq	#32,d0
	jsr	(CALLS).l
	clr.l	(PTE5).l
	lea	(TGT).l,a0
	pflush	(a0)
	move.l	#1,(irq_rel).l
	move.w	#2,(IPLREG).l		; masked: SR is $2700
	moveq	#0,d6
	moveq	#1,d0
	jsr	(CALLS).l
	chkl	d6,1,20
	move.l	(cnt_aerr).l,d0
	chkl	d0,3,21
	move.w	#$2000,sr		; the request is gone: nothing taken
	nop
	move.w	#$2700,sr
	move.l	(cnt_int2).l,d0
	chkl	d0,0,22
	move.w	#2,(IPLREG).l
	move.w	#$2000,sr
	nop
	nop
	move.w	#$2700,sr
	move.l	(cnt_int2).l,d0
	chkl	d0,1,23
	clr.l	(irq_rel).l

;------------------------------- E: user mode, first call, page invalid
	clr.l	(PTE5).l
	pflusha
	movea.l	#$9C00,a0		; user stack in page 4
	movec	a0,usp
	move.l	#e_back,(uret).l
	moveq	#0,d6
	moveq	#1,d0
	move.w	#$0000,-(sp)		; format $0
	pea	u_entry(pc)
	move.w	#$0000,-(sp)		; user, mask 0
	rte
e_back:
	move.w	#$2700,sr
	chkl	d6,1,40
	move.l	(cnt_aerr).l,d0
	chkl	d0,4,41
	move.l	(last_pc).l,d0
	chkl	d0,TGT,42
	move.l	(last_fa).l,d0
	chkl	d0,TGT,43
	move.l	(last_fslw).l,d0
	and.l	#F_IO|F_R|F_PF|F_TMM,d0
	chkl	d0,F_IO|F_R|F_PF|F_TM2,44

;------------------- F: user mode, learned BSR, then the page unmapped
	move.l	#f_back1,(uret).l
	moveq	#0,d6
	moveq	#32,d0
	move.w	#$0000,-(sp)
	pea	u_entry(pc)
	move.w	#$0000,-(sp)
	rte
f_back1:
	move.w	#$2700,sr
	chkl	d6,32,50
	clr.l	(PTE5).l
	pflusha
	move.l	#f_back2,(uret).l
	moveq	#0,d6
	moveq	#1,d0
	move.w	#$0000,-(sp)
	pea	u_entry(pc)
	move.w	#$0000,-(sp)
	rte
f_back2:
	move.w	#$2700,sr
	chkl	d6,1,51
	move.l	(cnt_aerr).l,d0
	chkl	d0,5,52
	move.l	(last_pc).l,d0
	chkl	d0,TGT,53

;----------------------------- G/H: as E/F with a dirty data cache
	clr.l	(PTE5).l
	pflusha
	bsr	dirty_dc
	move.l	#g_back,(uret).l
	moveq	#0,d6
	moveq	#1,d0
	move.w	#$0000,-(sp)
	pea	u_entry(pc)
	move.w	#$0000,-(sp)
	rte
g_back:
	move.w	#$2700,sr
	chkl	d6,1,60
	move.l	(cnt_aerr).l,d0
	chkl	d0,6,61
	move.l	(last_pc).l,d0
	chkl	d0,TGT,62

	move.l	#h_back1,(uret).l
	moveq	#0,d6
	moveq	#32,d0
	move.w	#$0000,-(sp)
	pea	u_entry(pc)
	move.w	#$0000,-(sp)
	rte
h_back1:
	move.w	#$2700,sr
	chkl	d6,32,70
	clr.l	(PTE5).l
	pflusha
	moveq	#7,d5
h_loop:
	bsr	dirty_dc
	move.l	#h_back2,(uret).l
	moveq	#0,d6
	moveq	#1,d0
	move.w	#$0000,-(sp)
	pea	u_entry(pc)
	move.w	#$0000,-(sp)
	rte
h_back2:
	move.w	#$2700,sr
	chkl	d6,1,71
	clr.l	(PTE5).l
	pflusha
	dbra	d5,h_loop
	move.l	#FASTOFS+$A021,(PTE5).l
	move.l	(cnt_aerr).l,d0
	chkl	d0,14,72

;------------- I: user mode, learned BSR, page unmapped, interrupt swept
	move.l	(cnt_aerr).l,d4
	move.l	#i_back1,(uret).l
	moveq	#0,d6
	moveq	#32,d0
	move.w	#$0000,-(sp)
	pea	u_entry(pc)
	move.w	#$0000,-(sp)
	rte
i_back1:
	move.w	#$2700,sr
	clr.l	(cnt_int2).l
	if	FASTOFS
	move.w	#300,(RNDMAX).l
	move.w	#2,(RNDLVL).l
	endif
	move.w	#199,d5
i_loop:
	clr.l	(PTE5).l
	pflusha
	move.w	d5,(IPLDLY).l		; raised d5 clocks from here
	move.l	#i_back2,(uret).l
	moveq	#0,d6
	moveq	#1,d0
	move.w	#$0000,-(sp)
	pea	u_entry(pc)
	move.w	#$0000,-(sp)
	rte
i_back2:
	move.w	#$2000,sr		; a request still to come is taken here
	moveq	#99,d1
i_w:	dbra	d1,i_w
	move.w	#$2700,sr
	chkl	d6,1,80
	addq.l	#1,d4
	move.l	(cnt_aerr).l,d0
	cmp.l	d4,d0
	beq.s	i_ok
	failt	81
i_ok:	dbra	d5,i_loop
	if	FASTOFS
	clr.w	(RNDLVL).l
	move.w	#$2000,sr
	moveq	#99,d1
i_w2:	dbra	d1,i_w2
	move.w	#$2700,sr
	endif
	move.l	#FASTOFS+$A021,(PTE5).l
	move.l	(cnt_int2).l,d0
	cmp.l	#150,d0
	bhs.s	i_ok2
	failt	82			; the interrupts did arrive
i_ok2:

;------------- J: a five-word instruction running into the invalid page
	move.l	#$23FC1234,($9FFC).l	; move.l #$12345678,(j_var).l
	move.l	#$56780000,($A000).l
	move.l	#$20244E75,($A004).l	; (j_var) ; rts
	cpusha	bc
	clr.l	(j_var).l
	clr.l	(PTE5).l
	pflusha
	move.l	(cnt_aerr).l,d4
	jsr	($9FFC).l
	move.l	(j_var).l,d0
	chkl	d0,$12345678,90
	addq.l	#1,d4
	move.l	(cnt_aerr).l,d0
	cmp.l	d4,d0
	beq.s	j_ok
	failt	91
j_ok:	move.l	(last_fa).l,d0
	chkl	d0,$9FFC,92
	move.l	(last_pc).l,d0
	chkl	d0,$9FFC,93
	move.l	(last_fslw).l,d0
	and.l	#F_IO|F_MA|F_PF,d0
	chkl	d0,F_IO|F_MA|F_PF,94

	moveq	#0,d0
	movec	d0,cacr
	move.w	#$600D,(DONEREG).l
done:	bra.s	done

; access error: format $4 -- SR 0, PC 2, fmt/vec 6, FA 8, FSLW 12
h_aerr:
	cmpi.w	#$4008,6(sp)
	bne	hf
	move.l	2(sp),(last_pc).l
	move.l	8(sp),(last_fa).l
	move.l	12(sp),(last_fslw).l
	addq.l	#1,(cnt_aerr).l
	move.l	a0,-(sp)
	movea.l	(fix_addr).l,a0
	move.l	(fix_val).l,(a0)
	tst.l	(irq_rel).l
	beq.s	h_ae1
	move.w	#0,(IPLREG).l
h_ae1:	movea.l	(sp)+,a0
	rte

; every line of the 8K data cache dirty: write 16K of $6000-$9FFF, then
; a different user stack line each time (the push must miss)
dirty_dc:
	lea	($6000).l,a1
	move.w	#4095,d1
dd_l:	move.l	d1,(a1)+
	dbra	d1,dd_l
	movec	usp,a1
	suba.w	#$40,a1
	cmpa.l	#$8800,a1
	bhi.s	dd_u
	movea.l	#$9C00,a1
dd_u:	movec	a1,usp
	rts

u_entry:
	jsr	(CALLS).l
	trap	#1

h_utrap:
	ori.w	#$2000,(sp)
	move.l	(uret).l,2(sp)
	rte

h_int2:
	addq.l	#1,(cnt_int2).l
	move.w	#0,(IPLREG).l
	rte

hf:	failt	98

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:	bra.s	halt1

unexp:
	move.w	#$0099,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:	bra.s	halt2

;---------------------------------------------------------- the routine
	org	TGT
	addq.l	#1,d6
	rts

;--------------------------------------------- call site: D0 calls of TGT
	org	CALLS
cl:	bsr.l	TGT
	subq.l	#1,d0
	bne.s	cl
	rts

