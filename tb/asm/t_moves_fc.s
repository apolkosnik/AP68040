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

FAILREG         equ $F100
DONEREG         equ $F102
cnt_aerr        equ $3600

        org     0
        dc.l    $3400,start
        dc.l    h_aerr                  ; vector 2: access fault
        rept    253
        dc.l    unexpected
        endr

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
        cmpi.w  #$7008,6(sp)            ; format $7, vector 2
        bne     unexpected
        addq.w  #1,(cnt_aerr).l
        move.l  #$00009003,($4424).l    ; page 9: drop the supervisor bit
        pflusha
        rte

f1:     moveq   #1,d7
        bra     fail
f2:     moveq   #2,d7
        bra     fail
f3:     moveq   #3,d7
        bra     fail
unexpected:
        moveq   #15,d7
fail:
        move.w  d7,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
halt:
        bra     halt
