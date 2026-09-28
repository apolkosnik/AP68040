; Failed searches create valid, nonresident 68040 ATC entries. Exercise
; ordinary invalid pages, PTEST invalid/bus-error fills, protection caching,
; separate instruction/data banks, 8K pages, and PTEST replacement.

FAILREG         equ $F100
DONEREG         equ $F102
cnt_aerr        equ $3600
which           equ $3602
expected        equ $3604

        org     0
        dc.l    $3400,start
        dc.l    h_aerr
        rept    253
        dc.l    unexpected
        endr

        org     $400
start:
        move.w  #$2700,sr
        clr.w   (which).l
next_case:
        moveq   #0,d0
        movec   d0,tc
        pflusha
        clr.w   (cnt_aerr).l
        move.w  #2,(expected).l

        lea     ($4400).l,a0
        moveq   #0,d0
        moveq   #63,d1
tloop:
        move.l  d0,d2
        lsl.l   #8,d2
        lsl.l   #4,d2
        cmpi.w  #5,(which).l
        bne.s   page_size_ready
        add.l   d2,d2                  ; 8K page-table layout
page_size_ready:
        addq.l  #3,d2
        move.l  d2,(a0)+
        addq.l  #1,d0
        dbra    d1,tloop
        move.l  #$00004203,($4000).l
        move.l  #$00004403,($4200).l
        move.l  #$4E751234,($9000).l    ; RTS for the instruction-bank control
        lea     ($4424).l,a1
        move.l  #$9003,d4
        cmpi.w  #5,(which).l
        bne.s   table_ready
        lea     ($4410).l,a1
        move.l  #$8003,d4
table_ready:
        clr.l   (a1)                   ; target page INVALID

        move.l  #$4000,d0
        movec   d0,urp
        movec   d0,srp
        ; PTEST is specified to walk even with TC.E=0; arming the bus-error
        ; injection here cannot accidentally fault an instruction/table access.
        lea     ($9000).l,a0
        moveq   #5,d0
        movec   d0,dfc
        cmpi.w  #0,(which).l
        beq.s   enable
        cmpi.w  #5,(which).l
        beq.s   enable
        cmpi.w  #3,(which).l
        beq.s   protect
        move.w  #1,(expected).l
        cmpi.w  #2,(which).l
        bne.s   probe
        move.l  d4,(a1)                ; valid descriptor, but table read fails
        move.w  #1,($F146).l
probe:
        ptestr  (a0)
        movec   mmusr,d0
        cmpi.w  #2,(which).l
        beq.s   check_bus
        tst.l   d0
        bne     f_status
        bra.s   repair_probe
check_bus:
        cmp.l   #$800,d0
        bne     f_status
repair_probe:
        move.l  d4,(a1)                ; no flush: the negative entry must persist
        cmpi.w  #6,(which).l
        bne.s   enable
        ptestr  (a0)                   ; PTEST discards/replaces a negative entry
        movec   mmusr,d0
        cmp.l   #$9001,d0
        bne     f_status
        clr.w   (expected).l
        bra.s   enable
protect:
        move.l  #$9007,(a1)            ; resident but write-protected
enable:
        move.l  #$8000,d0
        cmpi.w  #5,(which).l
        bne.s   tc_ready
        move.l  #$C000,d0
tc_ready:
        movec   d0,tc
        cmpi.w  #4,(which).l
        bne.s   access
        jsr     (a0)                   ; data's negative entry must not poison I-ATC
access:
        cmpi.w  #3,(which).l
        bne.s   read
        move.l  #$4E751234,(a0)        ; protection must persist until flush, too
read:
        moveq   #0,d1
        move.l  ($9000).l,d1            ; faults: page 9 is invalid

        ; Reached only after the handler repaired the descriptor.  Which
        ; repair got us here is what the count says.
        cmp.l   #$4E751234,d1
        bne     f9
        moveq   #0,d0
        movec   d0,tc
        pflusha
        move.w  (expected).l,d0
        cmp.w   (cnt_aerr).l,d0
        bne     f2                      ; repair alone must not bypass the cached fault
        addq.w  #1,(which).l
        cmpi.w  #7,(which).l
        bne     next_case
        move.w  #$600D,(DONEREG).l
        stop    #$2700

h_aerr:
        cmpi.w  #$7008,6(sp)
        bne     unexpected
        cmpi.l  #$9000,20(sp)
        bne     unexpected              ; includes the instruction-bank control
        addq.w  #1,(cnt_aerr).l
        cmpi.w  #2,(cnt_aerr).l
        bhi     f2                      ; fail promptly if invalidation is broken
        move.l  d4,(a1)                 ; make it valid
        movem.l d0,-(sp)
        move.w  (expected).l,d0
        cmp.w   (cnt_aerr).l,d0
        movem.l (sp)+,d0               ; preserve CMP's flags for the branch
        bne     h_noflush
        pflushn (a0)                    ; negative entries are nonglobal
h_noflush:
        rte

f2:     moveq   #2,d7
        bra     f1
f1:     move.w  d7,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
        bra     halt
f9:     moveq   #9,d7
        bra     f1
f_status:
        moveq   #10,d7
        bra     f1
halt:   bra     halt
unexpected:
        moveq   #15,d7
        bra     f1
