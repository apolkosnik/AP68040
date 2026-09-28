; Four-byte PEA trace boundary: exact next PC, trace address, CCR and pushed data.
    org 0
    dc.l $7000,start
    rept 7
    dc.l failed
    endr
    dc.l handler
    rept 246
    dc.l failed
    endr
    org $400
start:
    move.w #$2700,sr
    move.l #$80008000,d0
    movec d0,cacr
    movea.l #$1000,a0
    moveq #7,d1
    clr.w -(a7)
    move.l #$600,-(a7)
    move.w #$a71f,-(a7)
    rte
    org $600
    dc.w $4870,$1801
    bra failed
handler:
    cmpi.w #$a71f,(a7)
    bne failed
    cmpi.l #$604,2(a7)
    bne failed
    cmpi.l #$600,8(a7)
    bne failed
    cmpi.l #$1008,($6ffc).l
    bne failed
    move.w #$600d,($f102).l
    stop #$2700
failed:
    move.w #$4a1f,($f100).l
    move.w #$bad0,($f102).l
    stop #$2700
