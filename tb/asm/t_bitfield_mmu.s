; Memory bitfields must access only the bytes containing the field.
; Reproduces OPENSTEP 4.2 WindowServer's BFTST at the end of an 8K page.
; Runs under tb_ap040_program.v (zero and varied bus wait states).

FAILREG         equ $F100
DONEREG         equ $F102
cnt_aerr        equ $3600
last_fa         equ $3604
last_ssw        equ $3608
guard_pte       equ $4414       ; page $A000-$BFFF

checkl macro
        cmp.l   #\2,\1
        beq.s   ok\@
        move.w  #\3,d7
        bra     fail
ok\@:
        endm

        org     0
        dc.l    $3400,start,h_aerr
        rept    30
        dc.l    unexpected
        endr
        dc.l    h_user_return   ; TRAP #1
        rept    222
        dc.l    unexpected
        endr

        org     $400
start:
        move.w  #$2700,sr
        lea     ($4400).l,a0
        moveq   #0,d0
        moveq   #31,d1
tables:
        move.l  d0,d2
        lsl.l   #8,d2
        lsl.l   #5,d2
        addq.l  #3,d2
        move.l  d2,(a0)+
        addq.l  #1,d0
        dbra    d1,tables
        move.l  #$4203,($4000).l
        move.l  #$4403,($4200).l
        move.l  #$4000,d0
        movec   d0,srp
        movec   d0,urp
        move.l  #$C000,d0
        movec   d0,tc

        ; Exact crash opcode, displacement and user-data function code.
        ; A repaired speculative over-read still fails the zero-fault check.
        move.b  #$FE,($9FFF).l
        bsr     guard
        lea     ($9FFC).l,a1
        lea     ($3C00).l,a0
        move    a0,usp
        clr.w   -(sp)
        pea     user_bftst(pc)
        move.w  #$0013,-(sp)    ; X/V/C set: BFTST preserves X, clears V/C
        rte
user_bftst:
        bftst   3(a1){6:2}
        move.w  ccr,d0
        trap    #1
user_done:
        and.l   #$1F,d0
        checkl  d0,$18,1        ; X=1 N=1 Z=0 V=0 C=0
        move.w  #2,d7
        bsr     no_fault

        ; One-byte read/modify/write and the register-source BFINS path.
        bfclr   ($9FFF).l{6:2}
        bfset   ($9FFF).l{6:2}
        bfchg   ($9FFF).l{6:2}
        moveq   #1,d3
        bfins   d3,($9FFF).l{6:2}
        moveq   #0,d0
        move.b  ($9FFF).l,d0
        checkl  d0,$FD,3        ; surrounding six bits unchanged
        move.w  #4,d7
        bsr     no_fault

        ; Signed dynamic offset points back from the invalid page to its
        ; predecessor's last byte; offset 30 has the same one-byte span.
        moveq   #-2,d2
        moveq   #2,d3
        bfextu  ($A000).l{d2:d3},d0
        checkl  d0,1,5
        bfextu  ($9FFC).l{30:2},d0
        checkl  d0,1,6
        move.w  #7,d7
        bsr     no_fault

        ; Span 2: right-aligned WORD results must enter the high end of
        ; the work window.  Check signed extraction and first-one offset.
        move.w  #$789A,($9FFE).l
        bfexts  ($9FFE).l{4:12},d0
        checkl  d0,$FFFFF89A,8
        bfffo   ($9FFE).l{4:12},d0
        checkl  d0,4,9
        bfclr   ($9FFE).l{4:12}
        moveq   #0,d0
        move.w  ($9FFE).l,d0
        checkl  d0,$7000,10
        move.w  #11,d7
        bsr     no_fault

        ; Span 3: WORD plus BYTE; never read a fourth byte at $A000.
        move.b  #$56,($9FFD).l
        move.w  #$789A,($9FFE).l
        bfextu  ($9FFD).l{4:20},d0
        checkl  d0,$6789A,12
        bfchg   ($9FFD).l{4:20}
        moveq   #0,d0
        move.b  ($9FFD).l,d0
        checkl  d0,$59,13
        move.w  ($9FFE).l,d0
        checkl  d0,$8765,14
        move.w  #15,d7
        bsr     no_fault

        ; Span 4 retains the LONG path and preserves bits outside the field.
        move.l  #$3456789A,($9FFC).l
        bfextu  ($9FFC).l{4:28},d0
        checkl  d0,$456789A,16
        bfclr   ($9FFC).l{4:28}
        move.l  ($9FFC).l,d0
        checkl  d0,$30000000,17
        move.w  #18,d7
        bsr     no_fault

        ; Span 5 retains LONG plus BYTE.  Both outside nibbles survive BFINS.
        move.b  #$12,($9FFB).l
        move.l  #$3456789A,($9FFC).l
        bfextu  ($9FFB).l{4:32},d0
        checkl  d0,$23456789,19
        move.l  #$ABCDEF01,d3
        bfins   d3,($9FFB).l{4:32}
        moveq   #0,d0
        move.b  ($9FFB).l,d0
        checkl  d0,$1A,20
        move.l  ($9FFC).l,d0
        checkl  d0,$BCDEF01A,21
        move.w  #22,d7
        bsr     no_fault

        ; Genuine crossing, span 2: a split WORD read must still fault.
        ; The handler maps the missing page and retries the instruction.
        move.b  #$FE,($9FFF).l
        bftst   ($9FFF).l{6:3}
        moveq   #0,d0
        move.w  (cnt_aerr).l,d0
        checkl  d0,1,23
        move.l  (last_fa).l,d0
        checkl  d0,$9FFF,24
        moveq   #0,d0
        move.w  (last_ssw).l,d0
        and.l   #$0F67,d0
        checkl  d0,$0D45,25     ; MA + ATC + read + WORD + supervisor data

        ; Genuine crossing on the third byte tests the new WORD+BYTE path.
        move.w  #$1234,($9FFE).l
        move.b  #$56,($A000).l
        bsr     guard
        bfextu  ($9FFE).l{0:24},d0
        checkl  d0,$123456,26
        moveq   #0,d0
        move.w  (cnt_aerr).l,d0
        checkl  d0,1,27
        move.l  (last_fa).l,d0
        checkl  d0,$A000,28

        ; Genuine crossing on the fifth byte retains LONG+BYTE restart.
        move.l  #$12345678,($9FFC).l
        move.b  #$9A,($A000).l
        bsr     guard
        bfextu  ($9FFC).l{4:32},d0
        checkl  d0,$23456789,29
        moveq   #0,d0
        move.w  (cnt_aerr).l,d0
        checkl  d0,1,30
        move.l  (last_fa).l,d0
        checkl  d0,$A000,31

        move.w  #$600D,(DONEREG).l
        stop    #$2700

guard:
        clr.w   (cnt_aerr).l
        clr.l   (guard_pte).l
        pflusha
        rts

no_fault:
        tst.w   (cnt_aerr).l
        bne     fail
        rts

h_aerr:
        cmpi.w  #$7008,6(sp)
        bne     unexpected
        move.l  $14(sp),(last_fa).l
        move.w  $C(sp),(last_ssw).l
        addq.w  #1,(cnt_aerr).l
        move.l  #$A003,(guard_pte).l
        pflusha
        rte

h_user_return:
        ori.w   #$2700,(sp)
        move.l  #user_done,2(sp)
        rte

unexpected:
        move.w  #99,d7
fail:
        move.w  d7,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
halt:
        bra.s   halt
