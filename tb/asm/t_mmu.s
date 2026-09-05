; AP040 milestone E MMU self test
; assembled with vasmm68k_mot -Fbin -m68040
;
; testbench protocol: $F100 fail number, $F102 result magic
;
; memory map (all physical = logical except where noted):
;   $0000 vectors, $0400 code, $3400 ISP top, $3C00 USP top
;   $4000 root table, $4200 pointer table, $4400 page table (64 x 4K pages)
;   page  8 ($8000) write protected
;   page  9 ($9000) supervisor only
;   page 10 ($A000) invalid, fixed by the access error handler
;   page 11 ($B000) remapped to physical $C000
;   page 13 ($D000) invalid, fixed on an instruction fetch fault

FAILREG		equ	$F100
DONEREG		equ	$F102

cnt_aerr	equ	$3600
expect_fa	equ	$3604
fix_addr	equ	$3608
fix_val		equ	$360C
seen_desc	equ	$3610
cnt_stub	equ	$3614
uret		equ	$3618
expect_tm	equ	$361C
expect_ma	equ	$3620
last_ssw	equ	$3624
last_wb3s	equ	$3626
last_wb3d	equ	$3628
last_ea		equ	$362C
wb_complete	equ	$3680	; h_aerr performs valid WB3s like NetBSD trap.c
cnt_int2	equ	$3684	; level-2 interrupts taken (interleave sweeps)
cnt_int3	equ	$3688	; level-3 interrupts taken (spl-storm sweep)
storm_scr	equ	$368C	; scratch the storm decrements, like serintr's count
chkbuf		equ	$3690	; CHK bound operand (stale-record test)
aerr_act	equ	$3694	; nonzero while h_aerr is executing
cnt_trace	equ	$3698	; vector-9 traces taken in the T0 window
last_tpc	equ	$369C	; stacked PC of the first such trace
storm_dly	equ	$368E	; sweep delay handed to h_int3 for its own arming
IPLREG	equ	$F110
IPLDLY	equ	$F148
IPLCAP	equ	$F160	; bench IPL-injection capability (bit 0)
WBERRCTL	equ	$F146

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
	rept	23
	dc.l	unexp		; 3-25
	endr
	dc.l	h_int2		; 26 level-2 autovector (interleave sweeps)
	dc.l	h_int3		; 27 level-3 autovector (spl-storm sweep)
	rept	5
	dc.l	unexp		; 28-32
	endr
	dc.l	h_utrap		; 33 TRAP #1
	rept	222
	dc.l	unexp		; 34-255
	endr

	org	$400
start:
	clr.w	(cnt_aerr).l
	clr.w	(cnt_stub).l
	clr.w	(expect_ma).l
	clr.w	(wb_complete).l
	clr.w	(cnt_int2).l
	clr.w	(cnt_int3).l

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
	move.l	#0,($4434).l		; page 13: invalid

	move.l	#$00004203,($4000).l	; root entry 0 -> pointer table
	move.l	#$00004403,($4200).l	; pointer entry 0 -> page table

;----------------------------------------------- pre-place physical data
	move.l	#$11112222,($3000).l
	move.l	#$0A0A0A0A,($A000).l
	move.l	#$09090909,($9000).l
	move.l	#$AAAA1111,($5000).l
	move.l	#$BBBB2222,($E000).l
	move.w	#$5279,($D000).l	; addq.w #1,(cnt_stub).l
	move.l	#cnt_stub,($D002).l
	move.w	#$4E75,($D006).l	; rts

;----------------------------------------------------------------- enable
	move.l	#$4000,d0
	movec	d0,urp
	movec	d0,srp
	move.l	#$8000,d0	; E=1, 4K pages
	movec	d0,tc

;----------------------------------------------------------- translation
	move.l	($3000).l,d0
	chkl	d0,$11112222,1

	move.l	#$DEAD1234,($B000).l	; lands at physical $C000
	move.l	($C000).l,d0
	chkl	d0,$DEAD1234,2
	move.l	($B000).l,d0
	chkl	d0,$DEAD1234,3

;----------------------------------------------------------- history bits
	move.l	($4000).l,d0
	and.l	#8,d0
	chkl	d0,8,4			; root descriptor U set

	move.l	#$CAFE0505,($5000).l
	move.l	($4414).l,d0
	and.l	#$10,d0
	chkl	d0,$10,5		; page 5 modified

	tst.l	($6000).l
	move.l	($4418).l,d0
	and.l	#$10,d0
	chkl	d0,0,6			; page 6 read only: M clear
	move.l	($4418).l,d0
	and.l	#8,d0
	chkl	d0,8,7			; but U set

;----------------------------------------------------------------- PTEST
	moveq	#5,d0
	movec	d0,dfc
	lea	($5000).l,a0
	ptestr	(a0)
	movec	mmusr,d0
	chkl	d0,$00005011,8		; PA $5000, M, resident

	; PTEST installs its successful search in the selected ATC.  Changing
	; the descriptor alone therefore leaves the probed mapping cached.
	pflusha
	lea	($E000).l,a0
	ptestr	(a0)
	move.l	#$00005003,($4438).l	; page 14 now points at physical $5000
	move.l	($E000).l,d0
	chkl	d0,$BBBB2222,36		; PTEST-installed identity mapping is stale

	; MMU register writes do not flush either ATC.  An unchanged TC write
	; must therefore leave the PTEST-installed mapping stale.
	move.l	#$8000,d0
	movec	d0,tc
	move.l	($E000).l,d0
	chkl	d0,$BBBB2222,37		; TC write did not flush the ATC
	pflusha
	move.l	($E000).l,d0
	chkl	d0,$CAFE0505,60		; explicit PFLUSH exposes the remap
	move.l	#$0000E003,($4438).l
	pflusha

;----------------------------------------------- write protection fault
	move.l	#5,(expect_tm).l	; supervisor data write
	move.l	#$8000,(expect_fa).l
	move.l	#$4420,(fix_addr).l
	move.l	#$8003,(fix_val).l
	move.l	#$FEED0808,($8000).l	; faults, handler unprotects, restarts
	move.l	($8000).l,d0
	chkl	d0,$FEED0808,9
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,1,10
	move.l	(seen_desc).l,d0
	and.l	#8,d0
	chkl	d0,8,45			; denied access still set descriptor U

;----------------------------------------------------- invalid data page
	move.l	#5,(expect_tm).l	; supervisor data read
	move.l	#$A000,(expect_fa).l
	move.l	#$4428,(fix_addr).l
	move.l	#$A003,(fix_val).l
	move.l	($A000).l,d0
	chkl	d0,$0A0A0A0A,11
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,2,12

;------------------------------------------------ instruction fetch fault
	move.l	#6,(expect_tm).l	; supervisor instruction fetch
	move.l	#$D000,(expect_fa).l
	move.l	#$4434,(fix_addr).l
	move.l	#$D003,(fix_val).l
	jsr	($D000).l		; target page invalid: fetch fault
	move.w	(cnt_stub).l,d0
	and.l	#$FFFF,d0
	chkl	d0,1,13
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,3,14

;------------------------------------------- user access to a super page
	movea.l	#$3C00,a0
	movec	a0,usp
	move.l	#ucont,(uret).l
	move.l	#1,(expect_tm).l	; user data read
	move.l	#$9000,(expect_fa).l
	move.l	#$4424,(fix_addr).l
	move.l	#$9003,(fix_val).l
	move.w	#$0000,-(sp)
	pea	user_code(pc)
	move.w	#$0000,-(sp)
	rte

user_code:
	move.l	($9000).l,d0	; faults: S page from user mode
	trap	#1

ucont:
	chkl	d0,$09090909,15
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,4,16

;------------------------------- RTE to user: fetch uses the user root
; a separate user root maps VA $7000 -> PA $F000 (moveq/trap) while the
; supervisor root keeps identity, where $7000 holds ILLEGAL. The first
; fetch after RTE belongs to the restored user context, so it must
; translate through URP; with a supervisor-FC fetch this executes the
; ILLEGAL instead and the unexpected-exception handler fails the test.
	lea	($4C00).l,a0
	moveq	#0,d0
	moveq	#63,d1
utloop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#4,d2
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,utloop
	move.l	#$0000F003,($4C1C).l	; user VA page 7 -> PA $F000
	move.l	#$00004A03,($4800).l
	move.l	#$00004C03,($4A00).l
	move.w	#$702A,($F000).l	; moveq #42,d0
	move.w	#$4E41,($F002).l	; trap #1
	move.w	#$4AFC,($7000).l	; illegal via the supervisor map
	move.l	#$4800,d0
	movec	d0,urp
	pflusha
	move.l	#ucont2,(uret).l
	moveq	#0,d0
	move.w	#$0000,-(sp)
	pea	($7000).l
	move.w	#$0000,-(sp)
	rte

ucont2:
	chkl	d0,42,33		; ran the user-mapped code

;--------------------- MOVE to SR supervisor->user: fetch uses the user root
; The MOVE to SR sits at VA $7000, which the supervisor root identity maps.
; Clearing S means the NEXT fetch (VA $7004) belongs to the user context and
; must translate through URP to PA $F004; the supervisor view of $7004 holds
; ILLEGAL, so a stale supervisor FC on that fetch fails the test.
	move.l	#ucont3,(uret).l
	moveq	#0,d0
	move.w	#$702B,($F004).l	; user view:  moveq #43,d0
	move.w	#$4E41,($F006).l	;             trap #1
	move.w	#$4AFC,($7004).l	; super view: illegal
	move.w	#$46FC,($7000).l	; move.w #$0000,sr
	move.w	#$0000,($7002).l
	pflusha
	jmp	($7000).l

ucont3:
	chkl	d0,43,39		; ran the user-mapped code after MOVE to SR
	move.l	#$4000,d0
	movec	d0,urp			; restore the shared root
	pflusha

;---------------------------- high VA walk: nonzero root/pointer indexes
; VA $1E0C3000: root index 15, pointer index 3, page index 3. The page
; table sits at $4D00 (bit 8 set) to prove the 256-byte table base mask.
	move.l	#$00004E03,($403C).l	; root[15] -> pointer table $4E00
	move.l	#$00004D03,($4E0C).l	; pointer[3] -> page table $4D00
	move.l	#$0000E003,($4D0C).l	; page[3] -> PA $E000
	move.l	($1E0C3000).l,d0
	chkl	d0,$BBBB2222,34
	move.l	#$FEED5678,($1E0C3004).l
	move.l	($E004).l,d0
	chkl	d0,$FEED5678,35

;------------------------- transfers that cross a page boundary
; The MMU translates one address per bus transaction, so a misaligned
; transfer spanning two pages must be split: every byte has to go through
; its own page's mapping.  Page 2 ($2000) keeps its identity mapping while
; page 3 ($3000) is remapped to physical $E000, so a long written at
; $2FFE lands two bytes in $2FFE and two bytes at the top of $E000.
	; First make the second page invalid.  Its ATC fault must report the
	; original longword address/size, not the individually split byte, and
	; set SSW.MA because the fault occurred after crossing the page boundary.
	; Use pages 5/6 rather than 2/3: the active ISP is in page 3, so making
	; page 3 invalid would correctly double-fault while stacking the frame.
	move.w	#$1122,($5FFE).l
	move.w	#$3344,($E000).l
	move.l	#5,(expect_tm).l
	move.l	#$00005FFE,(expect_fa).l
	move.l	#$4418,(fix_addr).l
	move.l	#$0000E003,(fix_val).l
	move.w	#$0800,(expect_ma).l
	move.l	#0,($4418).l
	pflusha
	move.l	($5FFE).l,d0
	chkl	d0,$11223344,46
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,5,47
	clr.w	(expect_ma).l
	move.l	#$00006003,($4418).l
	pflusha

	move.l	#$0000E003,($440C).l	; page 3 -> physical $E000
	pflusha
	move.l	#$00000000,($2FFC).l
	move.l	#$00000000,($E000).l
	move.l	#$11223344,($2FFE).l	; straddles the boundary
	move.l	($2FFC).l,d0
	chkl	d0,$00001122,40		; low half of page 2 got the top bytes
	move.l	($E000).l,d0
	chkl	d0,$33440000,41		; page 3's physical target got the rest
	move.l	($2FFE).l,d0
	chkl	d0,$11223344,42		; and it reads back through both pages

	move.w	#$5566,($2FFF).l	; word crossing at an odd address
	move.b	($2FFF).l,d0
	and.l	#$FF,d0
	chkl	d0,$55,43
	move.b	($E000).l,d0
	and.l	#$FF,d0
	chkl	d0,$66,44

	move.l	#$00003003,($440C).l	; restore the identity mapping
	pflusha
	move.l	#$BBBB2222,($E000).l	; the ATC tests below still need this

;------------------------------------------------------------ TTR bypass
	move.l	#0,($4414).l	; page 5 invalid
	pflusha
	move.l	#$0000C000,d0	; DTT0: base $00, mask $00, both modes
	movec	d0,dtt0
	move.l	($5000).l,d0	; transparent: no fault, no walk
	chkl	d0,$CAFE0505,17
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,5,18
	lea	($5000).l,a0
	ptestr	(a0)
	movec	mmusr,d0
	chkl	d0,$00000003,19		; T and R only: the PA field stays
					; clear on a TTR match (WinUAE PTEST)

	; PTESTW reports a write-protected transparent translation as B
	move.l	#$0000C004,d0
	movec	d0,dtt0
	ptestw	(a0)
	movec	mmusr,d0
	chkl	d0,$00000800,38		; B is bit 11, not the G position

	; DFC program space selects ITT rather than DTT during PTEST
	moveq	#0,d0
	movec	d0,dtt0
	move.l	#$0000C000,d0
	movec	d0,itt0
	moveq	#6,d0
	movec	d0,dfc
	ptestr	(a0)
	movec	mmusr,d0
	chkl	d0,$00000003,39	; ITT match: T and R only
	moveq	#0,d0
	movec	d0,itt0
	moveq	#5,d0
	movec	d0,dfc
	moveq	#0,d0
	movec	d0,dtt0

;--------------------------- chipset IO through a transparent translation
; Everything that covers RTG so far runs with the MMU off, but the MMU is
; the structural difference between this CPU and the TG68K the upstream
; core uses -- and 68040.library always enables it, mapping the low 16MB
; of IO space through a transparent translation rather than page tables.
; Reproduce that idiom exactly: DTT0 base $00, mask $00 (so only
; $00000000-$00FFFFFF matches), E=1, S=both, CM=10 cache-inhibited
; serialized, and read the RTG ID through it.  The page tables here cover
; only $0-$3FFFF, so this access reaches fastchip solely via the TTR.
	move.w	(IPLCAP).l,d0
	btst	#4,d0			; the real fastchip/rtg block is present
	beq	rtg_ttr_done
	move.l	#$0000C040,d0
	movec	d0,dtt0
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,171	; ID/VERSION through the TTR

	; and the registers must still take a write under translation
	move.l	#$02000000,($B80100).l
	move.l	($B80100).l,d1
	chkl	d1,$02000000,172

	; same again with the TTR marked cache-inhibited nonserialized, the
	; other mode 68040.library uses for IO
	move.l	#$0000C060,d0
	movec	d0,dtt0
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,173
	moveq	#0,d0
	movec	d0,dtt0

rtg_ttr_done:

	move.l	#$00005003,($4414).l
	pflusha

; ...and now the case that actually matters.  A TTR bypasses the table walk
; entirely, but SetPatch installs 68040.library, which maps IO space through
; real page tables -- and the RTG ID reads $5001 before SetPatch and $0000
; after.  So walk to it: $00B8010E splits into root index 0, pointer index
; $2E (VA[24:18]), page index 0 (VA[17:12]), and the page frame is
; identity with CM = 10, cache-inhibited serialized, the mode 68040.library
; uses for a register block.
	move.w	(IPLCAP).l,d0
	btst	#4,d0			; the real fastchip/rtg block is present
	beq	rtg_walk_done
	move.l	#$00B80043,($5800).l	; page 0 of the $B8 region, CI serialized
	move.l	#$00005803,($42B8).l	; pointer entry $2E -> that page table
	pflusha
	move.l	#$80008000,d0
	movec	d0,cacr			; SetPatch enables these too
	lea	($B8010E).l,a0
	ptestr	(a0)
	movec	mmusr,d0
	and.l	#$FFFFF001,d0
	chkl	d0,$00B80001,190	; the walk itself resolves
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,174	; ID through a real table walk

	; a write and read-back through the same translation
	move.l	#$02000000,($B80100).l
	move.l	($B80100).l,d1
	chkl	d1,$02000000,175

	; the walk must be repeatable once the ATC entry is resident, and
	; still correct after it is flushed away again
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,176
	pflusha
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,177

	; 68040.library chooses the cache mode in the descriptor, and it does
	; not necessarily know $B80000 is a register block: an unknown region
	; can end up copyback or writethrough rather than cache-inhibited.
	; The L1 must bypass it regardless, because $B8xxxx is outside
	; cache_win -- so every CM encoding has to read the same.
	move.l	#$00B80003,($5800).l	; CM = 00, writethrough
	pflusha
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,191
	move.l	#$00B80023,($5800).l	; CM = 01, copyback
	pflusha
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,192
	move.l	#$00B80063,($5800).l	; CM = 11, cache-inhibited nonserialized
	pflusha
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,193

	; and a write-back-then-read under copyback, which is the mode that
	; would post a dirty line if anything ever cached this page
	move.l	#$00B80023,($5800).l
	pflusha
	move.l	#$02000000,($B80100).l
	move.l	($B80100).l,d1
	chkl	d1,$02000000,194
	cpusha	dc
	move.l	($B80100).l,d1
	chkl	d1,$02000000,195

	; With the region UNMAPPED -- which is what an MMU setup that only
	; covers the boards it knows about leaves behind, and the MiSTer RTG
	; board is not autoconfig -- the access must take a normal access
	; fault, not quietly return data.  A faulted read is what a monitor
	; displays as $0000, which is exactly the post-SetPatch symptom.
	move.w	(cnt_aerr).l,d5		; hand the counter back below
	and.l	#$FFFF,d5
	move.l	#0,($42B8).l
	pflusha
	move.l	#5,(expect_tm).l	; supervisor data read
	move.l	#$00B8010E,(expect_fa).l
	move.l	#$42B8,(fix_addr).l
	move.l	#$00005803,(fix_val).l
	move.w	($B8010E).l,d0		; faults, handler maps it, restarts
	and.l	#$FFFF,d0
	chkl	d0,$00005001,196	; and the restarted access reads the ID
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	sub.l	d5,d0
	chkl	d0,1,197		; exactly one access fault was taken
	move.w	d5,(cnt_aerr).l		; cnt_aerr is cumulative and asserted
					; downstream: hand it back untouched

	moveq	#0,d0
	movec	d0,cacr			; back to the uncached regime
	cinva	bc
	move.l	#0,($42B8).l		; unmap the region again
	pflusha
rtg_walk_done:
	; 8K is checked in the 8K-pages section below, where the tables that
	; map this code have been rebuilt for it -- switching TC alone would
	; reinterpret them and fault on the next instruction fetch

;--------------------------------------- ATC caching and page PFLUSH
	move.l	($5000).l,d0	; walk and cache the mapping
	chkl	d0,$CAFE0505,20
	move.l	#$0000E003,($4414).l	; remap page 5 without a flush
	move.l	($5000).l,d0
	chkl	d0,$CAFE0505,21		; stale ATC entry still used
	lea	($5000).l,a0
	pflush	(a0)
	move.l	($5000).l,d0
	chkl	d0,$BBBB2222,22		; new mapping after the flush
	move.l	#$00005003,($4414).l
	pflusha

;----------------------------------------- MOVEM restart across a fault
	; registers to memory into a page that faults mid-transfer: the
	; 68040 restart model re-executes the whole MOVEM after the fix
	move.l	#5,(expect_tm).l	; supervisor data write (MOVEM)
	move.l	#$00007000,(expect_fa).l
	move.l	#$441C,(fix_addr).l	; page 7 descriptor
	move.l	#$7003,(fix_val).l
	move.l	#0,($441C).l		; make page 7 invalid
	pflusha
	move.l	#$AAAA0001,d1
	move.l	#$BBBB0002,d2
	move.l	#$CCCC0003,d3
	lea	($6FFC).l,a0		; last longword of valid page 6
	movem.l	d1-d3,(a0)		; d1 at $6FFC, d2/d3 fault into page 7
	move.l	($6FFC).l,d0
	chkl	d0,$AAAA0001,24
	move.l	($7000).l,d0
	chkl	d0,$BBBB0002,25
	move.l	($7004).l,d0
	chkl	d0,$CCCC0003,26
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,6,27

	; memory to registers with a fault on the second page
	move.l	#0,($441C).l
	pflusha
	moveq	#0,d1
	moveq	#0,d2
	moveq	#0,d3
	movem.l	(a0),d1-d3
	chkl	d1,$AAAA0001,28
	chkl	d2,$BBBB0002,29
	chkl	d3,$CCCC0003,30
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,7,31

	; the EA base register inside a LOADED list: a fault after the base
	; was loaded must still restart with the ORIGINAL base.  The loaded
	; value may only commit with the last transfer -- a core that writes
	; it mid-loop recomputes the restart EA from the loaded DATA (here a
	; non-address) and reads garbage.
	move.l	#5,(expect_tm).l	; supervisor data read
	move.l	#$00007000,(expect_fa).l
	move.l	#$441C,(fix_addr).l
	move.l	#$7003,(fix_val).l
	move.l	#$11112222,($6FF8).l	; a0's image: not an address
	move.l	#$33334444,($6FFC).l	; a1's image, last valid long
	move.l	#$55556666,($7000).l	; a2's image, first faulting long
	move.l	#0,($441C).l		; page 7 invalid again
	pflusha
	lea	($6FF8).l,a0
	movem.l	(a0),a0-a2		; a0 loads first; a2's read faults
	chkl	a0,$11112222,144
	chkl	a1,$33334444,145
	chkl	a2,$55556666,146
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,8,147
	subq.w	#1,(cnt_aerr).l	; later sections count faults absolutely

;----------------------------------------------------------------- 8K pages
	moveq	#0,d0
	movec	d0,tc		; MMU off while rebuilding tables
	pflusha

	; page table: 32 entries of 8K covering LA 0-$3FFFF, identity
	lea	($4400).l,a0
	moveq	#0,d0
	moveq	#31,d1
t8loop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#5,d2		; i << 13
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,t8loop

	; entry 5 (LA $A000-$BFFF) remapped to PA $C000-$DFFF
	move.l	#$0000C003,($4414).l
	; pre-place physical data through the identity map (MMU off)
	move.l	#$08081111,($C120).l	; seen through LA $A120 (LA12=0)
	move.l	#$08082222,($D120).l	; seen through LA $B120 (LA12=1)

	; the $B8 region again, now under 8K paging: root index 0, pointer
	; index $2E, and the page index is VA[17:13] which is still 0, so the
	; same descriptor serves -- the frame simply carries no bit 12
	move.w	(IPLCAP).l,d0
	btst	#4,d0
	beq	rtg_8k_skip
	move.l	#$00B80043,($5800).l
	move.l	#$00005803,($42B8).l
rtg_8k_skip:

	move.l	#$C000,d0	; E=1, P=1: 8K pages
	movec	d0,tc

	move.l	($3000).l,d0	; identity page, LA12=1 within its 8K page
	chkl	d0,$11112222,24
	move.l	($A120).l,d0
	chkl	d0,$08081111,25
	move.l	($B120).l,d0	; same 8K page, LA bit 12 set
	chkl	d0,$08082222,26

	move.w	(IPLCAP).l,d0
	btst	#4,d0			; the real fastchip/rtg block is present
	beq	rtg_8k_done
	move.w	($B8010E).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$00005001,179	; RTG ID through an 8K page walk
rtg_8k_done:
	move.l	#0,($42B8).l
	pflusha

	lea	($A000).l,a0	; PTEST under 8K paging
	ptestr	(a0)
	movec	mmusr,d0
	and.l	#$FFFFF001,d0
	chkl	d0,$0000C001,27
	lea	($B000).l,a0	; SAME 8K page as $A000 above
	ptestr	(a0)
	movec	mmusr,d0
	and.l	#$FFFFF001,d0
	; MMUSR carries the page FRAME, not the probed LA's translation, so
	; both probes in one 8K page report the same address and bit 12 is
	; clear.  This used to expect $D001 -- the frame with the LA's bit 12
	; folded in -- which is what WinUAE's PTEST does NOT do
	; (mmu_fill_atc: desc & mmu_pagemaski, ~0x1FFF at 8K).
	chkl	d0,$0000C001,28

	; fault and restart under 8K paging
	move.l	#5,(expect_tm).l	; supervisor data write (8K paging)
	move.l	#$0000E000,(expect_fa).l
	move.l	#$441C,(fix_addr).l	; entry 7: LA $E000-$FFFF
	move.l	#$0000E003,(fix_val).l
	move.l	#0,($441C).l
	pflusha
	move.l	#$0E0E0E0E,d1
	move.l	d1,($E000).l	; faults, handler fixes, restart writes
	move.l	($E000).l,d0
	chkl	d0,$0E0E0E0E,29
	move.w	(cnt_aerr).l,d0
	and.l	#$FFFF,d0
	chkl	d0,8,30

;----------------------------------------------------------------- disable
	moveq	#0,d0
	movec	d0,tc
	move.l	($3000).l,d0
	chkl	d0,$11112222,23

;================ WinUAE-oracle MMU/frame audit battery (2026-08-07) ======
; Rebuild the 4K identity tables (the 8K section rewrote the page table).
	lea	($4400).l,a0
	moveq	#0,d1
	move.w	#63,d0
t48loop:
	move.l	d1,d2
	lsl.l	#8,d2
	lsl.l	#4,d2		; page number << 12
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d1
	dbra	d0,t48loop
	pflusha

; PTEST walks the tables even with translation disabled (TC is 0 here):
; a resident probe reports the full translation, not a transparent stub
	lea	($8000).l,a0
	moveq	#5,d0
	movec	d0,dfc
	ptestr	(a0)
	movec	mmusr,d0
	chkl	d0,$00008001,48		; PA + R from a real table search

; a bus error on a descriptor fetch during PTEST reports MMUSR B
	move.w	#1,(WBERRCTL).l
	ptestr	(a0)
	movec	mmusr,d0
	chkl	d0,$00000800,49		; B bit, nothing else

; enable 4K translation for the fault-shape tests
	move.l	#$8000,d0
	movec	d0,tc
	pflusha

; MOVE16 write fault: SSW reports SIZE=line with TT0 and the WB3 slot
; stays clear (the restart model omits the WB2 line writeback)
	move.l	#5,(expect_tm).l
	move.l	#0,(expect_ma).l
	move.l	#$8000,(expect_fa).l
	move.l	#$4420,(fix_addr).l
	move.l	#$8003,(fix_val).l
	move.l	#$16161616,($3100).l
	move.l	#$27272727,($3104).l
	move.l	#$38383838,($3108).l
	move.l	#$49494949,($310C).l
	move.l	#$00008007,($4420).l	; write protect page 8
	pflusha
	lea	($3100).l,a0
	lea	($8000).l,a1
	move16	(a0)+,(a1)+
	move.w	(last_ssw).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$046D,50		; ATC + line size + TT0 + TM=5
	move.w	(last_wb3s).l,d0
	and.l	#$FFFF,d0
	chkl	d0,0,51			; MOVE16 write: WB3 valid stays clear
	move.l	($8000).l,d0
	chkl	d0,$16161616,52		; restart completed the line

; TAS operand fault: a locked RMW reports LK with RW clear
	move.b	#$11,($8005).l
	move.l	#$8005,(expect_fa).l
	move.l	#$00008007,($4420).l
	pflusha
	tas	($8005).l
	move.w	(last_ssw).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$0625,53		; ATC + LK + byte, RW clear
	moveq	#0,d0
	move.b	($8005).l,d0
	chkl	d0,$91,54		; restart completed the set

; MOVES to FC 0 keeps the raw FC in TM and reports TT=10
	move.l	#0,(expect_tm).l
	move.l	#$8006,(expect_fa).l
	move.l	#$00008007,($4420).l
	pflusha
	moveq	#0,d0
	movec	d0,dfc
	move.b	#$5C,d1
	moves.b	d1,($8006).l
	move.w	(last_ssw).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$0430,55		; ATC + byte + TT1, TM=0
	moveq	#0,d0
	move.b	($8006).l,d0
	chkl	d0,$5C,56

; MOVES to FC 2 is remapped onto user data space in TM
	move.l	#1,(expect_tm).l
	move.l	#$8007,(expect_fa).l
	move.l	#$00008007,($4420).l
	pflusha
	moveq	#2,d0
	movec	d0,dfc
	move.b	#$7B,d1
	moves.b	d1,($8007).l
	move.w	(last_ssw).l,d0
	and.l	#$FFFF,d0
	chkl	d0,$0421,57		; ATC + byte, TM remapped to 1
	moveq	#5,d0
	movec	d0,dfc

; a plain write fault carries the write in WB3 and the EA mirrors FA
	move.l	#5,(expect_tm).l
	move.l	#$8000,(expect_fa).l
	move.l	#$00008007,($4420).l
	pflusha
	move.l	#$DEADBEE5,($8000).l
	move.l	(last_wb3d).l,d0
	chkl	d0,$DEADBEE5,58
	move.l	(last_ea).l,d0
	chkl	d0,$8000,59

;----------- walker U/M writes must invalidate a cached descriptor (5.5)
; The table walker updates U/M over its own physical port, behind the data
; cache.  A descriptor the CPU has already read AS DATA would otherwise go
; stale -- the last coherence hole once the internal caches are enabled.
; The descriptor is rewritten and the ATC flushed first, so the test does
; not depend on history bits left by anything above.
	move.l	#$80008000,d0
	movec	d0,cacr			; caches on for this test only
	move.l	#$00007003,($441C).l	; page 7 identity, U and M clear
	pflusha
	move.l	($441C).l,d0		; caches the line holding the descriptor
	and.l	#$18,d0
	chkl	d0,0,148		; U and M start clear
	tst.l	($7000).l		; touch page 7: the walker sets U
	move.l	($441C).l,d0		; must not be served from the stale line
	and.l	#8,d0
	chkl	d0,8,149
	cinva	bc
	moveq	#0,d0
	movec	d0,cacr			; back to the uncached regime

	moveq	#0,d0
	movec	d0,tc

;--------------------- relocated: 8K user-mode demand paging (see below)
; Runs LAST: it rebuilds its own tables and nothing downstream depends on
; the state it leaves.  Re-enable 8K translation with the shared root.
	move.l	#$4000,d0	; shared root: $4000 -> $4200 -> $4400
	movec	d0,urp
	movec	d0,srp
	move.l	#$00004203,($4000).l
	move.l	#$00004403,($4200).l
	lea	($4400).l,a0	; rebuild the 32-entry 8K identity table
	moveq	#0,d0
	moveq	#31,d1
t8loop2:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#5,d2
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,t8loop2
	move.l	#$C000,d0
	movec	d0,tc
	pflusha
	move.w	(cnt_aerr).l,d7	; running last: count faults relative

;--------------------- 8K user-mode demand paging: the NetBSD exec shape
; NetBSD/amiga runs the 040 with 8K pages and execs init by mapping
; nothing, letting the FIRST USER INSTRUCTION FETCH fault, and paging the
; code in from disk at whatever fault address the frame reports.  If the
; stacked FA is wrong under 8K user ifetch, UVM pages in the wrong page
; and the right one never arrives -- the exact live signature captured in
; tests/ap040/hw/netbsd (uvmexp.paging stuck at 1).  This test performs
; that sequence: user table with the code page NOT RESIDENT, RTE to user,
; ifetch faults (TM=2), the handler validates FA and maps the page, the
; restart runs the user code, and a trap returns.  Still under TC=$C000.
	lea	($4C00).l,a0	; user page table: 32 x 8K identity
	moveq	#0,d0
	moveq	#31,d1
u8loop:
	move.l	d0,d2
	lsl.l	#8,d2
	lsl.l	#5,d2		; i << 13
	addq.l	#3,d2
	move.l	d2,(a0)+
	addq.l	#1,d0
	dbra	d1,u8loop
	move.l	#$00004A03,($4800).l	; user root -> pointer -> table
	move.l	#$00004C03,($4A00).l
	; user code page: VA $6000 (8K page 3, covers $6000-$7FFF), NOT
	; resident yet; physical backing prepared at PA $E000
	move.l	#0,($4C0C).l
	move.w	#$702C,($E000).l	; moveq #44,d0
	move.w	#$4E41,($E002).l	; trap #1
	move.l	#$4800,d0
	movec	d0,urp
	pflusha
	move.l	#2,(expect_tm).l	; USER instruction fetch
	move.l	#$6000,(expect_fa).l	; the 8K page base: fetch of VA $6000
	move.l	#$4C0C,(fix_addr).l	; entry 3
	move.l	#$0000E003,(fix_val).l	; map VA $6000-$7FFF -> PA $E000
	move.l	#u8cont,(uret).l
	moveq	#0,d0
	move.w	#$0000,-(sp)
	pea	($6000).l
	move.w	#$0000,-(sp)
	rte			; to user; the ifetch at $6000 faults

u8cont:
	chkl	d0,44,150	; the paged-in user code ran after restart
	move.w	(cnt_aerr).l,d0
	sub.w	d7,d0
	and.l	#$FFFF,d0
	chkl	d0,1,151	; exactly one ifetch fault

	; same shape for a user DATA fault mid-page: fault on a non-resident
	; 8K page at an offset with LA bit 12 SET, so a bit-12 confusion in
	; the fault path (the 8K MMUSR reporting class) would surface here.
	; The user code lives on a FRESH page (VA $B000, upper half of 8K
	; entry 5, identity) written
	; before it ever executes -- and executing from the UPPER 4K half
	; also proves the ifetch side of the 13-bit offset.  The faulting
	; read targets VA $9120 (bit 12 SET: offset $1120) on
	; non-resident entry 4, mapped by the handler to PA $C000, where the
	; datum sits at PA $C000 + $1120 = $D120 (13-bit page offset).
	move.w	#$2039,($B000).l	; move.l ($9120).l,d0
	move.l	#$00009120,($B002).l
	move.w	#$4E41,($B006).l	; trap #1
	cpusha	bc			; the code page must reach memory and
	cinva	ic			; no stale line may shadow it
	move.l	#0,($4C10).l		; entry 4 (VA $8000-$9FFF) not resident
	move.l	#$0BBB1234,($D120).l	; backing for VA $8120: offset $1120
	move.l	#$0000A003,($4C14).l	; entry 5 identity: the code page
					; VA $A000 -> PA $A000; code placed
					; in its SECOND half at PA $B000 so
					; the RTE target VA $B000 needs the
					; 13-bit offset to reach it
	pflusha
	move.l	#1,(expect_tm).l	; USER data read
	move.l	#$9120,(expect_fa).l
	move.l	#$4C10,(fix_addr).l
	move.l	#$0000C003,(fix_val).l
	move.l	#u8dat,(uret).l
	moveq	#0,d0
	move.w	#$0000,-(sp)
	pea	($B000).l		; offset $1000 into the 8K page
	move.w	#$0000,-(sp)
	rte

u8dat:
	chkl	d0,$0BBB1234,152	; read through the paged-in mapping
	move.w	(cnt_aerr).l,d0
	sub.w	d7,d0
	and.l	#$FFFF,d0
	chkl	d0,2,153	; exactly one more, the data fault

	move.l	#$4400,d0	; restore the shared root
	movec	d0,urp

;--------------- write-fault discipline: the NetBSD relocation shape (154/155)
; ld.elf_so relocates libc with add.l %d1,%a0@ on copy-on-write data pages.
; The first store to such a page write-faults; NetBSD repairs the page,
; performs any writeback the frame marks VALID, and returns.  A restart
; model that ALSO advertises a valid WB3 gets the add applied twice --
; captured live on hardware: init's ctor pointer held link VA + 2x load
; base and init looped on an ifetch of the bogus address forever.  Here
; the handler behaves exactly like NetBSD's trap.c (h_aerr completes any
; valid WB3), the faulting instruction is the same RMW shape, and the
; datum must gain the addend EXACTLY ONCE.
	move.w	#1,(wb_complete).l
	move.l	#$11110000,($E000).l	; datum, via the identity map
	move.l	#$0000E007,($441C).l	; 8K entry 7: LA $E000 write-protected
	pflusha
	move.l	#5,(expect_tm).l	; supervisor data write fault
	move.l	#$0000E000,(expect_fa).l
	move.l	#$441C,(fix_addr).l	; handler clears the write protect
	move.l	#$0000E003,(fix_val).l
	move.l	#$00220000,d1
	add.l	d1,($E000).l		; read succeeds, write faults
	move.l	($E000).l,d0
	chkl	d0,$11330000,155	; the addend landed exactly once
	move.w	(cnt_aerr).l,d0
	sub.w	d7,d0
	and.l	#$FFFF,d0
	chkl	d0,3,154		; exactly one write fault
	clr.w	(wb_complete).l

	; The same family, register side effects: a faulting (An)+ / -(An)
	; store must restart with the ORIGINAL address register and leave
	; exactly one increment behind -- NetBSD's copy loops fault like
	; this on every fresh COW page.
	move.l	#$0000E007,($441C).l	; write protect the page again
	pflusha
	lea	($E000).l,a0
	move.l	#$0FEE1234,d1
	move.l	d1,(a0)+		; write faults, handler unprotects
	move.l	($E000).l,d0
	chkl	d0,$0FEE1234,156	; landed at the original address
	move.l	a0,d0
	chkl	d0,$E004,157		; increment applied exactly once

	move.l	#$0000E007,($441C).l
	pflusha
	lea	($E004).l,a0
	move.l	#$0FEE5678,d1
	move.l	d1,-(a0)		; predecrement flavour
	move.l	($E000).l,d0
	chkl	d0,$0FEE5678,158
	move.l	a0,d0
	chkl	d0,$E000,159
	move.w	(cnt_aerr).l,d0
	sub.w	d7,d0
	and.l	#$FFFF,d0
	chkl	d0,5,160		; the two extra write faults, once each

;----- A7 rollback must not corrupt the supervisor stack (168/169/170)
; The killer NetBSD bug, captured live: libc's __cerror ends with
;   MOVE.L (A7)+,(A0)     ; store the error value through the errno pointer
; run in USER mode.  When the (A0) write faults -- routine demand paging --
; the 68040 restart model rolls the A7 post-increment back.  A7 is a
; SHADOWED register: writes land in USP, ISP or MSP according to S/M.  If
; the rollback runs after exception entry has set S, the USER stack value
; is written into the SUPERVISOR pointer; the frame is then stacked in
; user space, that write faults too, and the fault-during-exception halts
; the core.  Hardware beacon at the halt: A7=1dfff9b8 (a user stack) in
; supervisor mode, IR=209f, PC in __cerror.
; Here: user code runs exactly that instruction with a non-resident
; destination.  The handler maps the page and returns.  On a broken core
; the supervisor stack pointer is destroyed and the machine dies inside
; exception processing; on a correct one the ISP is untouched, the store
; completes on restart, and A7 (USP) advances exactly one longword.
	move.l	#$4800,d0		; user root -> pointer -> page table
	movec	d0,urp			; (the 154-160 block left URP elsewhere)
	move.l	#$0000A003,($4C14).l	; user code page identity
	move.l	#$00008003,($4C10).l	; user data page VA $8000 resident
	pflusha
	; user code at PA $B000:  move.l (a7)+,(a0) ; trap #1
	move.w	#$209F,($B000).l
	move.w	#$4E41,($B002).l	; trap #1
	cpusha	bc
	cinva	ic
	; user stack at VA $8100 holding the value to store
	move.l	#$C0DE1234,($8100).l
	move.l	#0,($4C18).l		; USER entry 6 (VA $C000) NOT resident
	pflusha
	move.l	#1,(expect_tm).l	; user data write
	move.l	#$C000,(expect_fa).l
	move.l	#$4C18,(fix_addr).l
	move.l	#$0000C003,(fix_val).l
	move.w	(cnt_aerr).l,d6
	movec	isp,d4			; the supervisor stack, before
	move.l	#u7done,(uret).l
	move.l	#$8100,d0
	movec	d0,usp			; USP = the user stack
	lea	($C000).l,a0		; destination: the non-resident page
	move.w	#$0000,-(sp)
	pea	($B000).l
	move.w	#$0000,-(sp)
	rte				; to user: MOVE.L (A7)+,(A0)

u7done:
	movec	isp,d0
	cmp.l	d4,d0
	beq.s	u7isp
	failt	168			; ISP corrupted by the A7 rollback
u7isp:
	movec	usp,d0
	chkl	d0,$8104,169		; USP advanced exactly one longword
	move.l	($C000).l,d0		; the restarted store landed
	chkl	d0,$C0DE1234,170
	move.w	(cnt_aerr).l,d0
	sub.w	d6,d0
	and.l	#$FFFF,d0
	chkl	d0,1,171		; exactly one fault
	move.l	#$0000C003,($4C18).l
	move.l	#$4400,d0		; hand URP back as the block found it
	movec	d0,urp
	pflusha

;----- stale (An)+ record must die at non-access exception entry (172-174)
; CHK.W (A2)+,D1 with D1 negative: the EA read SUCCEEDS (a2 advances),
; then the CHK exception is taken.  The rollback records exist solely so
; the access-error path can undo the FAULTING instruction's side effects;
; only fetch_next and that consumer cleared them, so an instruction that
; raises a non-access exception after its EA carried a live record
; through exception_prefetch into its handler.  h_chk's FIRST instruction
; stores to a non-resident page; the access error's rollback then
; "restored" a2 from the stale record -- an unrelated instruction's
; register reverted to a stale value.  h_aerr repairs the page and the
; store restarts; afterwards a2 must still hold the post-increment value.
	move.l	#h_chk,($18).l		; CHK, vector 6
	move.l	#0,($C000).l		; scrub the target while still mapped
	move.w	#$7FFF,(chkbuf).l	; CHK bound operand
	move.l	#0,($4418).l		; supervisor VA $C000 not resident
	pflusha
	move.l	#5,(expect_tm).l	; supervisor data write
	move.l	#$C000,(expect_fa).l
	move.l	#$4418,(fix_addr).l
	move.l	#$0000C003,(fix_val).l
	move.w	(cnt_aerr).l,d6
	lea	(chkbuf).l,a2
	moveq	#-1,d1			; negative: CHK always traps
	chk.w	(a2)+,d1
	lea	(chkbuf+2).l,a0
	cmp.l	a0,a2
	beq.s	stale_ok
	failt	172			; a2 reverted by the stale record
stale_ok:
	move.l	($C000).l,d0
	chkl	d0,$FFFFFFFF,173	; h_chk ran and its store landed
	move.w	(cnt_aerr).l,d0
	sub.w	d6,d0
	and.l	#$FFFF,d0
	chkl	d0,1,174		; exactly one access error
	move.l	#unexp,($18).l

;----- MOVES alternate function codes select the ROOT by FC bit 2 (177-180)
; An external audit read MC68040 UM 3.2.5 as requiring FC 0/3/4/7 MOVES to
; BYPASS translation as physical accesses.  WinUAE -- the reference this
; core is validated against, and the source of the cputest corpus -- does
; not do that.  Its 68040 MOVES source read is:
;     bool super = (regs.sfc & 4) != 0;
;     res = mmu_get_user_byte(addr, super, false, sz_byte, false);
; i.e. EVERY function code is translated, and only bit 2 picks the root.
; ap040_mmu.v derives `a_super = c_fc[2]`, which is the same rule, so the
; implementation already agrees with the oracle and the audit's item is a
; manual-versus-reference disagreement.  This core has lost that bet in
; the manual's favour before (the T0 change-of-flow list), so pin the
; behaviour with a test that can actually tell the roots apart: URP and
; SRP address DIFFERENT tables here, so a bypass -- or a super bit taken
; from anything but FC2 -- lands the store in the wrong page.
;   user  VA $C000 -> PA $C000     (URP tables)
;   super VA $C000 -> PA $E000     (SRP tables)
;   super VA $E000 -> PA $C000     (so PA $C000 is readable from here)
	move.l	#$4800,d0
	movec	d0,urp			; user root, distinct from SRP ($4000)
	move.l	#$0000C003,($4C18).l	; user   entry 6: VA $C000 -> PA $C000
	move.l	#$0000E003,($4418).l	; super  entry 6: VA $C000 -> PA $E000
	move.l	#$0000C003,($441C).l	; super  entry 7: VA $E000 -> PA $C000
	pflusha
	move.l	#0,($C000).l		; PA $C000 via super VA $E000 below
	move.l	#0,($E000).l
	pflusha

	moveq	#1,d0			; FC1: user data
	movec	d0,dfc
	move.l	#$11111111,d1
	moves.l	d1,($C000).l
	move.l	($E000).l,d0		; super VA $E000 = PA $C000
	chkl	d0,$11111111,177	; FC1 translated through URP

	moveq	#5,d0			; FC5: supervisor data
	movec	d0,dfc
	move.l	#$55555555,d1
	moves.l	d1,($C000).l
	move.l	($C000).l,d0		; super VA $C000 = PA $E000
	chkl	d0,$55555555,178	; FC5 translated through SRP

	moveq	#0,d0			; FC0: bit 2 clear -> USER, not a bypass
	movec	d0,dfc
	move.l	#$00000000,d1
	moves.l	d1,($C000).l
	move.l	($E000).l,d0
	chkl	d0,0,179		; FC0 landed in the USER page

	moveq	#4,d0			; FC4: bit 2 set -> SUPERVISOR
	movec	d0,dfc
	move.l	#$44444444,d1
	moves.l	d1,($C000).l
	move.l	($C000).l,d0
	chkl	d0,$44444444,180	; FC4 landed in the SUPERVISOR page

	moveq	#5,d0
	movec	d0,dfc			; leave DFC as the block found it
	move.l	#$0000C003,($4418).l
	move.l	#$0000E003,($441C).l
	move.l	#$4400,d0
	movec	d0,urp
	pflusha


;---------------- interrupt-vs-MMU-operation interleave sweeps (161-164)
; The live NetBSD freeze happened inside pmap_enter -- PTE rewrite,
; PFLUSH, fresh table walks -- with a VBL interrupt nested in the same
; window and every interrupt level dead afterwards.  Sweep a delayed
; level-2 request across that whole window.  PFLUSHA empties the ATC
; each round, so the interrupt's own exception entry (vector fetch,
; frame pushes) re-walks the tables while the pipeline is mid-MMU-op:
; exactly the nesting the hardware died in.  Nothing may wedge (the
; bench timeout catches a stall), every request must be taken, and the
; translated accesses must stay correct.
	move.w	(IPLCAP).l,d0	; the interleave sweeps need working IPL
	btst	#0,d0		; injection; benches without it advertise 0
	beq	imix_done	; and the sweeps are bypassed, not faked

	; The bench ports live at PA $F1xx -- inside 8K page 7, which the
	; faulting sweep leaves non-resident.  The interrupt handler must
	; acknowledge through a mapping that never disappears: alias entry 6
	; (LA $C000) onto PA $E000 so VA $D110 reaches the port at PA $F110.
	move.l	#$0000E003,($4418).l
	pflusha
	move.w	#$2000,sr	; open the mask for level 2
	move.l	#$0EE0BEEF,($E000).l
	moveq	#1,d5
imix_a:
	move.w	d5,(IPLDLY).l	; level-2 lands d5 cycles from now
	move.l	#$0000E003,($441C).l
	pflusha			; ATC empty: everything below re-walks
	move.l	($E000).l,d0
	chkl	d0,$0EE0BEEF,162	; translated read correct every round
imix_aw:
	move.w	(cnt_int2).l,d0
	cmp.w	d5,d0		; the request must be delivered before the
	bne.s	imix_aw		; next round arms a new one
	addq.w	#1,d5
	cmp.w	#64,d5
	bls.s	imix_a
	moveq	#0,d0
	move.w	(cnt_int2).l,d0
	chkl	d0,64,161	; one interrupt per round, none lost

	; Same sweep with the access FAULTING: fault entry, h_aerr repair,
	; restart, and the pending interrupt all contend in one window --
	; the trap-plus-uvm_fault-plus-interrupt shape from the live stack.
	move.l	#5,(expect_tm).l
	move.l	#$0000E000,(expect_fa).l
	move.l	#$441C,(fix_addr).l
	move.l	#$0000E003,(fix_val).l
	move.w	(cnt_aerr).l,d6
	moveq	#1,d5
imix_b:
	move.w	d5,(IPLDLY).l	; arm FIRST: the port page ($F1xx) is about
	move.l	#0,($441C).l	; to become non-resident, and a faulting arm
	pflusha			; would deadlock the failure reporting too
	move.l	#$0DDF0000,d1
	add.l	d5,d1
	move.l	d1,($E000).l	; faults, handler repairs, restart stores
	move.l	($E000).l,d0
	cmp.l	d1,d0
	beq.s	imix_bd
	failt	164		; restarted store lost or doubled
imix_bd:
imix_bw:
	move.w	(cnt_int2).l,d0
	sub.w	#64,d0
	cmp.w	d5,d0
	bne.s	imix_bw
	addq.w	#1,d5
	cmp.w	#64,d5
	bls.s	imix_b
	move.w	(cnt_aerr).l,d0
	sub.w	d6,d0
	and.l	#$FFFF,d0
	chkl	d0,64,163	; exactly one repair fault per round
	move.w	#$2700,sr

	; Sweep C: the spl storm.  A level-3 handler does the serintr mask
	; dance while a delayed level-2 request lands at every offset across
	; the handler's life -- entry, mid-storm between mask writes, and the
	; RTE.  The pending 2 must survive every mask transition and be taken
	; exactly once when the mask finally opens.
	move.l	#$0000E003,($441C).l	; page 7 resident again
	pflusha
	move.w	#$2000,sr
	moveq	#1,d5
imix_cc:
	move.w	#96,(storm_scr).l
	move.w	d5,(storm_dly).l
	move.w	#3,(IPLREG).l	; the level-3 device interrupts; h_int3 arms
				; the delayed ports request itself
imix_cw:
	moveq	#0,d0
	move.w	(cnt_int2).l,d0
	sub.w	#128,d0		; sweeps A+B consumed 128
	cmp.w	d5,d0		; one more level-2 per round
	bne.s	imix_cw
	addq.w	#1,d5
	cmp.w	#96,d5
	bls.s	imix_cc
	moveq	#0,d0
	move.w	(cnt_int3).l,d0
	chkl	d0,96,165	; one level-3 service per round
	moveq	#0,d0
	move.w	(cnt_int2).l,d0
	chkl	d0,224,166	; 128 + 96: every pending 2 delivered
	move.l	($3000).l,d0
	chkl	d0,$11112222,167	; the stormed touches stayed coherent
	move.w	#$2700,sr
	move.l	#$0000C003,($4418).l	; entry 6 back to identity
	pflusha
imix_done:

;------------------------------------------------- NeXTSTEP kernel shape
; NeXT Mach 3.3 runs the kernel through transparent translation windows
; and walks tables only for user space and dynamically mapped kernel
; regions (loadable kernel servers around LA $3C000000).  The observed
; panic is a supervisor instruction fetch at $3C014D9C reporting an
; invalid descriptor: root index $1E, pointer index 0, page index 10
; under 8K pages, walked through SRP while URP points at a different
; tree.  Replay exactly that: map the page, call through it, remap it
; the way pmap_enter does, and prove the used/modified writebacks land
; on their own descriptors only.
	moveq	#0,d0
	movec	d0,tc
	pflusha

	; clear root $4000, decoy user root $5000, pointer table $4600,
	; page tables $4800/$4880
	lea	($4000).l,a0
	move.w	#127,d1
nsclr1:	clr.l	(a0)+
	dbra	d1,nsclr1
	lea	($5000).l,a0
	move.w	#127,d1
nsclr2:	clr.l	(a0)+
	dbra	d1,nsclr2
	lea	($4600).l,a0
	move.w	#127,d1
nsclr3:	clr.l	(a0)+
	dbra	d1,nsclr3
	lea	($4800).l,a0
	move.w	#63,d1
nsclr4:	clr.l	(a0)+
	dbra	d1,nsclr4

	; SRP tree: root[$1E] -> pointer $4600, pointer[0] -> page table
	; $4800, page[10] -> PA $6000
	move.l	#$00004603,($4078).l
	move.l	#$00004803,($4600).l
	move.l	#$00006003,($4828).l

	; target routines at the fault's page offset: move.l #tag,d3 / rts
	move.l	#$263CC0DE,($6D9C).l
	move.l	#$00404E75,($6DA0).l
	move.l	#$263CC0DE,($8D9C).l
	move.l	#$00414E75,($8DA0).l

	; SRP the real tree, URP an empty decoy: a supervisor walk that
	; wrongly uses URP dies on an invalid root entry
	move.l	#$4000,d0
	movec	d0,srp
	move.l	#$5000,d0
	movec	d0,urp

	; kernel-style transparent windows for supervisor code and data
	move.l	#$000FA000,d0
	movec	d0,itt0
	movec	d0,dtt0

	move.l	#$C000,d0	; E=1, P=1: 8K pages
	movec	d0,tc
	pflusha

	; the exact kernel gesture: supervisor jsr into the walked region
	moveq	#0,d3
	jsr	($3C014D9C).l
	chkl	d3,$C0DE0040,200

	; the used bit came back on this walk's descriptors and nowhere else
	move.l	($4078).l,d0
	chkl	d0,$0000460B,201	; root: resident + U
	move.l	($4074).l,d0
	chkl	d0,0,202		; root neighbor untouched
	move.l	($4600).l,d0
	chkl	d0,$0000480B,203	; pointer: resident + U
	move.l	($4604).l,d0
	chkl	d0,0,204		; pointer neighbor untouched
	move.l	($4828).l,d0
	chkl	d0,$0000600B,205	; page: resident + U
	move.l	($4824).l,d0
	chkl	d0,0,206
	move.l	($482C).l,d0
	chkl	d0,0,207

	; a walked write sets M on the page descriptor and lands physically
	move.l	#$5EC70001,($3C014000).l
	move.l	($6000).l,d0
	chkl	d0,$5EC70001,208
	move.l	($4828).l,d0
	chkl	d0,$0000601B,209	; page: U + M

	; pmap_enter: retarget the pointer entry at a fresh page table,
	; flush, and the same LA must reach the new physical page
	move.l	#$00008003,($48A8).l	; new table $4880, page[10] -> $8000
	move.l	#$00004883,($4600).l
	pflusha
	moveq	#0,d3
	jsr	($3C014D9C).l
	chkl	d3,$C0DE0041,210

	move.l	($4828).l,d0
	chkl	d0,$0000601B,211	; old page descriptor left alone
	move.l	($4600).l,d0
	chkl	d0,$0000488B,212	; new pointer entry: resident + U

	; translation off first (the code window is a TTR), then the
	; windows themselves
	moveq	#0,d0
	movec	d0,tc
	movec	d0,itt0
	movec	d0,dtt0
	pflusha

	; leave translation off for the harness epilogue
	moveq	#0,d0
	movec	d0,tc
	pflusha

	move.w	#$600D,(DONEREG).l
	stop	#$2700

;----------------------------------------------------------------- handlers
h_chk:
	move.l	d1,($C000).l		; FIRST instruction: faults, restarts
	rte

h_aerr:
	move.w	#1,(aerr_act).l	; a trace taken now is a leaked latch
	cmpi.w	#$7008,6(sp)	; format $7, vector 2
	bne	hfail
	movem.l	d0-d1/a0,-(sp)
	move.w	$18(sp),(last_ssw).l	; capture for the SSW-shape tests
	move.w	$1A(sp),(last_wb3s).l
	move.l	$28(sp),(last_wb3d).l
	move.l	$14(sp),(last_ea).l
	move.l	$14(sp),d0	; EA field: WinUAE stacks the fault address here
	move.w	$18(sp),d1	; MOVE16 stacks the EA aligned to the line
	and.w	#$0060,d1
	cmp.w	#$0060,d1
	bne.s	haerr_eachk
	and.l	#$FFFFFFF0,d0
	cmp.l	(expect_fa).l,d0
	beq.s	haerr_eaok
	bra	hfail
haerr_eachk:
	cmp.l	(expect_fa).l,d0	; (informational: CM/CT are never set)
	bne	hfail
haerr_eaok:
	move.l	$20(sp),d0	; fault address (frame offset $14)
	cmp.l	(expect_fa).l,d0
	bne	hfail
	move.w	$18(sp),d0	; SSW (frame offset $C)
	and.w	#$0400,d0	; ATC fault bit
	beq	hfail
	move.w	$18(sp),d0
	and.w	#$0800,d0	; fault on second page of a split access
	cmp.w	(expect_ma).l,d0
	bne	hfail
	move.w	$18(sp),d0
	and.l	#$0007,d0	; TM: the function code of the faulting access
	cmp.l	(expect_tm).l,d0
	bne	hfail
	; WB3S must be CLEAR on every fault, reads and writes alike: the
	; core restarts the repaired instruction, so a valid WB3 would make
	; an OS that completes writebacks (NetBSD trap.c) apply RMW stores
	; twice.  WB3D still carries the write data for diagnostics; WB3A
	; mirrors the fault address.
	tst.w	$1A(sp)
	bne	hfail
	move.w	$18(sp),d0
	btst	#8,d0		; write fault: WB3A must mirror the FA
	bne.s	haerr_wbok
	move.w	$18(sp),d0	; (MOVE16 line writes keep their aligned EA
	and.w	#$0060,d0	; handling above; WB3A is not checked there)
	cmp.w	#$0060,d0
	beq.s	haerr_wbok
	move.l	$20(sp),d0
	cmp.l	$24(sp),d0	; WB3A (frame offset $18, after movem +12)
	bne	hfail
haerr_wbok:
	; NetBSD's trap.c completes every writeback the frame marks VALID
	; ("the 68040 doesn't re-run instructions that cause write page
	; faults ... we have to write the value out to memory ourselves").
	; Mimic that here when the test asks for it: a restart-model core
	; advertising a valid WB3 gets the store applied TWICE.
	movea.l	(fix_addr).l,a0
	move.l	(a0),(seen_desc).l	; capture descriptor before the handler fixes it
	move.l	(fix_val).l,(a0)
	pflusha
	; NetBSD order: repair the mapping FIRST, then complete writebacks
	tst.w	(wb_complete).l
	beq.s	haerr_nowb
	move.w	$1A(sp),d0
	btst	#7,d0
	beq.s	haerr_nowb
	movea.l	$24(sp),a0	; WB3A
	move.l	$28(sp),(a0)	; WB3D: perform the faulted store
haerr_nowb:
	addq.w	#1,(cnt_aerr).l
	movem.l	(sp)+,d0-d1/a0
	clr.w	(aerr_act).l
	rte

h_int2:
	addq.w	#1,(cnt_int2).l
	move.w	#0,($D110).l	; IPLREG through the always-resident alias:
	rte			; the direct page may be mid-repair (sweep B)

; The NetBSD serintr shape, verbatim from the live freeze: inside a
; level-3 handler, storm the SR mask up and down (splraise/splx pairs at
; $2400/$2500) around translated memory touches, exactly as serintr
; drains its ring -- while a lower-priority request stays pending the
; whole time.  Every mask write resynchronises the pipeline and refills
; the fetch stream through translation.
h_int3:
	movem.l	d0-d1,-(sp)
	move.w	#0,($D110).l	; take the level-3 device down FIRST, then
	move.w	(storm_dly).l,(IPLDLY).l ; arm the ports device: its rise can
	addq.w	#1,(cnt_int3).l	; never race the clear, and the swept delay
	pflusha			; lands it anywhere in the storm below
	moveq	#7,d1
h3storm:
	move.w	sr,d0
	move.w	#$2400,sr	; splraise, serintr-style
	tst.l	($3000).l	; translated data touch
	move.w	#$2500,sr	; deeper raise around the count update
	subq.w	#1,(storm_scr).l
	move.w	d0,sr		; splx back to the entry mask
	dbra	d1,h3storm
	movem.l	(sp)+,d0-d1
	rte

h_utrap:
	ori.w	#$2000,(sp)	; back to supervisor
	move.l	(uret).l,2(sp)
	rte

hfail:
	failt	98

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
