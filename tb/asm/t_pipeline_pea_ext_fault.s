; Unavailable PEA extension faults in the sequencer before any push.
    org 0
    dc.l $7000,start
    dc.l handler
    rept 253
    dc.l failed
    endr
    org $400
start:
    move.w #$2700,sr
    moveq #0,d2
    move.l #$12345678,a0
    move.l #$43210000,a1
    move.w #$2000,($f154).l
    jmp ($1ffe).l
handler:
    cmpa.l #$6fc4,a7
    bne failed
    cmpa.l #$12345678,a0
    bne failed
    cmpi.l #$1ffe,2(a7)
    bne failed
    move.w 6(a7),d0
    andi.w #$f000,d0
    cmpi.w #$7000,d0
    bne failed
    move.w #$600d,($f102).l
    stop #$2700
failed:
    move.w #$4eaf,($f100).l
    move.w #$bad0,($f102).l
    stop #$2700
    org $1ffe
    dc.w $4870
    dc.w $1801
    bra failed
