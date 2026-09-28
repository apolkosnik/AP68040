; Real-core pipeline interruption: handler's stacked PC must match exactly
; the committed ADD count, and replay after RTE must produce all 64 adds.
    org 0
    dc.l $3400,start
    rept 24
    dc.l fail
    endr
    dc.l irq2
    rept 229
    dc.l fail
    endr
    org $400
start:
    move.l #$80008000,d0
    movec d0,cacr
    clr.w ($3600).l
    moveq #0,d0
    moveq #1,d1
    move.w #$2000,sr
    jmp ($600).l
    org $600
    rept 64
    add.l d1,d0
    endr
    cmp.l #64,d0
    bne fail
    cmp.w #1,($3600).l
    bne fail
    move.w #$600d,($f102).l
    stop #$2700
irq2:
    move.l 2(sp),d2
    sub.l #$600,d2
    lsr.l #1,d2
    cmp.l d2,d0
    bne fail
    addq.w #1,($3600).l
    move.w #0,($f110).l
    rte
fail:
    move.w #$face,($f100).l
    move.w #$bad0,($f102).l
    stop #$2700
