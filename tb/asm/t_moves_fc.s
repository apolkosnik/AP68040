; MOVES must use the space SFC/DFC selects, even when the transfer is issued
; in the same cycle the instruction asks for it.
;
; The fast issue path starts a transfer from the calling state when the memory
; port is free, which is exactly what a cache hit leaves behind.  MOVES selects
; its space by writing fc_ovr_v/fc_ovr, and those are registers: an issue in
; that same cycle read them one edge early and fell back on the supervisor
; default.  A kernel copying to or from user memory then reads its own space --
; NetBSD's init pathname came back empty because supervisor $2000 is ROM.
;
; Page 9 is mapped supervisor-only, so the space actually used is visible from
; software: a user-space MOVES there must fault, a supervisor-space one must
; not.  The reads are run cache-hot, because that is when the port is free.
;
; Tests 4-6 chase the same defect shape on ORDINARY accesses.  Their function
; code comes from the S bit, and the issue block reads that bit in the same
; clocked block that starts the transfer -- so if the S bit were read one edge
; early, the first access after a mode change would go out in the previous
; mode's space.  The assertion is not just "did it fault": the format $7 frame
; carries the SSW, whose TM field (bits 2:0) is the function code the access
; actually used, so the space is read back directly.

FAILREG         equ $F100
DONEREG         equ $F102
cnt_aerr        equ $3600
ssw_last        equ $3604               ; SSW of the last access fault
fa_last         equ $3608               ; and its fault address

        org     0
        dc.l    $3400,start
        dc.l    h_aerr                  ; vector 2: access fault
        rept    29
        dc.l    unexpected
        endr                            ; vectors 3-31
        dc.l    h_trap0                 ; vector 32: TRAP #0
        rept    223
        dc.l    unexpected
        endr                            ; vectors 33-255

        org     $400
start:
        move.w  #$2700,sr
        clr.w   (cnt_aerr).l

;----------------------------------------------------------------- tables
        lea     ($4400).l,a0
        moveq   #0,d0
        moveq   #63,d1
tloop:
        move.l  d0,d2
        lsl.l   #8,d2
        lsl.l   #4,d2                   ; i << 12
        addq.l  #3,d2                   ; resident
        move.l  d2,(a0)+
        addq.l  #1,d0
        dbra    d1,tloop
        move.l  #$00009083,($4424).l    ; page 9: supervisor only
        move.l  #$00004203,($4000).l
        move.l  #$00004403,($4200).l
        move.l  #$5A5A1234,($9000).l    ; something to read back

        move.l  #$4000,d0
        movec   d0,urp
        movec   d0,srp
        move.l  #$8000,d0               ; E=1, 4K pages
        movec   d0,tc
        pflusha

        ; caches on, and warm a line so the port is free when MOVES issues
        move.l  #$00000808,d0
        movec   d0,cacr
        move.l  #$80008000,d0
        movec   d0,cacr
        move.l  ($3000).l,d0
        move.l  ($3000).l,d0
        move.l  ($9000).l,d0            ; supervisor read: legal, warms it

;--------------------------------- 1: supervisor space must reach page 9
        moveq   #5,d0
        movec   d0,sfc
        clr.w   (cnt_aerr).l
        moves.l ($9000).l,d1
        cmp.l   #$5A5A1234,d1
        bne     f1
        tst.w   (cnt_aerr).l
        bne     f1

;--------------------------------- 2: user space must NOT.  This is the bug:
; with the function code read one edge early the access goes out as
; supervisor data and succeeds, so no fault is counted.
        moveq   #1,d0
        movec   d0,sfc
        clr.w   (cnt_aerr).l
        moveq   #0,d1
        moves.l ($9000).l,d1
        tst.w   (cnt_aerr).l
        beq     f2                      ; no fault -> wrong space was used
        move.l  #$00009083,($4424).l    ; restore supervisor-only
        pflusha

;--------------------------------- 3: the same for the write direction
        moveq   #1,d0
        movec   d0,dfc
        clr.w   (cnt_aerr).l
        move.l  #$DEADBEEF,d1
        moves.l d1,($9000).l
        tst.w   (cnt_aerr).l
        beq     f3
        move.l  #$00009083,($4424).l    ; restore supervisor-only
        pflusha

;--------------------------------- 4: the first USER access after an RTE
; Page 9 is supervisor-only, so a user read of it must fault AND the SSW must
; say the access really went out as user data (TM 1).  If the issue read the
; S bit one edge early it goes out as supervisor data and never faults.
        move.l  #$0000A083,($4428).l    ; page 10 supervisor-only: sup stack
        move.l  #$0000B083,($442C).l    ; page 11 supervisor-only: test 5 data
        move.l  #$00009083,($4424).l    ; page 9 supervisor-only again
        pflusha
        move.l  #$B0B0CAFE,($B000).l    ; supervisor writes: legal
        move.l  #$5A5A1234,($9000).l    ; test 3's retried MOVES left $DEADBEEF
        move.l  ($9000).l,d0            ; warm page 9 again
        clr.w   (cnt_aerr).l
        clr.w   (ssw_last).l

        move.l  #$3800,a0
        move.l  a0,usp
        move.l  #$AF00,a7               ; supervisor stack inside page 10
        move.w  #$0000,-(a7)            ; format 0 / vector
        move.l  #u_start,-(a7)          ; PC
        move.w  #$0000,-(a7)            ; SR: user mode, IPL 0
        rte

u_start:
        move.l  ($9000).l,d1            ; the first access in user mode
        trap    #0                      ; and back, through page 10's stack

;--------------------------------- 5: the first SUPERVISOR access after entry
; Reached only if test 6 held.  Page 11 is supervisor-only, and this is the
; first access after the mode change, so it must NOT fault.
h_trap0:
        move.l  ($B000).l,d2

;--------------------------------- 6: the exception frame push itself
; The TRAP above pushed its frame onto $AF00, inside supervisor-only page 10,
; from user mode.  The mode change precedes the push, so the push must use
; supervisor data.  A user-space push faults inside exception processing,
; which is a double fault -- reaching here at all is the assertion.

        tst.w   (cnt_aerr).l            ; exactly one fault, test 4's
        beq     f4                      ; none: user access used the wrong space
        cmpi.w  #1,(cnt_aerr).l
        bne     f5                      ; more: test 5 faulted when it must not
        move.w  (ssw_last).l,d3
        andi.w  #$0007,d3               ; SSW TM = the space actually used
        cmpi.w  #1,d3
        bne     f6                      ; not user data
        cmp.l   #$00009000,(fa_last).l
        bne     f7
        cmp.l   #$5A5A1234,d1           ; the retried user read got the data
        bne     f8
        cmp.l   #$B0B0CAFE,d2           ; and test 5's read worked
        bne     f5

        moveq   #0,d0
        movec   d0,tc
        movec   d0,cacr
        pflusha
        move.w  #$600D,(DONEREG).l
        stop    #$2700

; Repair and retry, as the other fault tests do: clear page 9's supervisor bit
; so the restarted MOVES completes, and count that the fault happened at all.
; The count is the whole assertion -- a MOVES that never faults used the wrong
; space.
h_aerr:
        movem.l d6/a6,-(sp)             ; frame offsets below include these 8
        cmpi.w  #$7008,14(sp)           ; format $7, vector 2
        bne     unexpected
        addq.w  #1,(cnt_aerr).l
        move.w  20(sp),(ssw_last).l     ; SSW: TM is the space the access used
        move.l  28(sp),(fa_last).l      ; and the fault address
        move.l  28(sp),d6
        lsr.l   #8,d6
        lsr.l   #4,d6                   ; page number
        lsl.l   #2,d6                   ; descriptor offset
        lea     ($4400).l,a6
        adda.l  d6,a6
        bclr    #7,3(a6)                ; drop that page's supervisor bit
        pflusha
        movem.l (sp)+,d6/a6
        rte

f1:     moveq   #1,d7
        bra     fail
f2:     moveq   #2,d7
        bra     fail
f3:     moveq   #3,d7
        bra     fail
f4:     moveq   #4,d7
        bra     fail
f5:     moveq   #5,d7
        bra     fail
f6:     moveq   #6,d7
        bra     fail
f7:     moveq   #7,d7
        bra     fail
f8:     moveq   #8,d7
        bra     fail
unexpected:
        moveq   #15,d7
fail:
        move.w  d7,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
halt:
        bra     halt
