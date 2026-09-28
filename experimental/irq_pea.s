    org 0
    dc.l $7000,start
    rept 24
    dc.l failed
    endr
    dc.l handler
    rept 5
    dc.l failed
    endr
    dc.l $600               ; TRAP #0 enters with architectural prefetch
    rept 223
    dc.l failed
    endr
    org $400
start:
    move.w #$2000,sr
    move.l #$80008000,d0
    movec d0,cacr
    clr.l ($6ff4).l
    clr.w ($c100).l
    movea.l #$c000,a0
    moveq #$55,d1
    moveq #0,d2
    trap #0                 ; a resident younger opcode makes cancellation observable
    org $600
pea_at:
    dc.w $4870,$1801
    rept 32
    addq.l #1,d2
    endr
    cmpi.l #32,d2
    bne failed
    cmpi.w #1,($c100).l
    bne failed
    move.w #$600d,($f102).l
    stop #$2700
handler:
    cmpa.l #$c000,a0
    bne failed
    cmpi.l #$c056,($6ff4).l
    bne failed
    cmpi.l #$604,2(a7)
    bne failed
    cmpi.l #$55,d1
    bne failed
    tst.l d2
    bne failed
    move.w #0,($f110).l
    addq.w #1,($c100).l
    rte
failed:
    move.w #$41a1,($f100).l
    move.w #$bad0,($f102).l
    stop #$2700
