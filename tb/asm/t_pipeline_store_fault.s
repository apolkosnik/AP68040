; Forced pipeline predecrement store fault: original An, precise frame and store CCR.
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
    movea.l #$f144,a0
    moveq #-1,d1
    moveq #$66,d2
    move.w #$1f,ccr
    nop                     ; older WB can overlap the store offer
faulting:
    move.l d1,-(a0)
    moveq #7,d2
    bra failed
handler:
    cmpi.w #$2718,(a7)
    bne failed
    cmpa.l #$f144,a0
    bne failed
    cmpi.l #faulting,2(a7)
    bne failed
    cmpi.l #$f140,20(a7)
    bne failed
    cmpi.l #-1,d1
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
    move.w #$3fad,($f100).l
    move.w #$bad0,($f102).l
    stop #$2700
