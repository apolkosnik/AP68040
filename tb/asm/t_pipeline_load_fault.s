; Forced pipeline load bus error: precise frame, unchanged destination and younger register.
    org 0
    dc.l $7000,start
    dc.l handler
    rept 253
    dc.l failed
    endr
    org $400
start:
    move.w #$2700,sr
    move.l #$80008000,d0
    movec d0,cacr
    movea.l #$f140,a0
    moveq #$55,d1
    moveq #$66,d2
faulting:
    move.l (a0),d1
    moveq #7,d2
    bra failed
handler:
    cmpi.l #faulting,2(a7)
    bne failed
    cmpi.l #$f140,20(a7)
    bne failed
    cmpi.l #$55,d1
    bne failed
    cmpi.l #$66,d2
    bne failed
    move.w 6(a7),d0
    andi.w #$f000,d0
    cmpi.w #$7000,d0
    bne failed
    move.w #$600d,($f102).l
    stop #$2700
failed:
    move.w #$2fad,($f100).l
    move.w #$bad0,($f102).l
    stop #$2700
