; PEA user-stack bus fault: preserve USP/CCR, exact PC/FA, discard younger work.
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
    move a0,usp
    movea.l #$1000,a1
    moveq #7,d1
    moveq #$66,d2
    move.w #$001f,sr
    nop
faulting:
    dc.w $4871,$1801
    moveq #1,d2
    bra failed
handler:
    cmpi.w #$001f,(a7)
    bne failed
    move usp,a0
    cmpa.l #$f144,a0
    bne failed
    cmpi.l #faulting,2(a7)
    bne failed
    cmpi.l #$f140,20(a7)
    bne failed
    cmpi.l #$66,d2
    bne failed
    move.w #$600d,($f102).l
    stop #$2700
failed:
    move.w #$4fad,($f100).l
    move.w #$bad0,($f102).l
    stop #$2700
