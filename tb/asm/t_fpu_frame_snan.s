; Exception operands must retain their signaling bit and payload. A
; datatype handler's BUSY replay must still deliver an enabled SNAN.
FRAME   equ     $3000
COUNT   equ     $3800
DTCOUNT equ     $3804

check   macro
        cmp.l   #\2,\1
        bne     fail
        endm
sourcecase macro
        addq.w  #1,d7
        fmove.l #6,fp0
        fmovem.x snan,fp1
        \1.\2  \3,fp0
        fsave   (FRAME).l
        check   (FRAME).l,$41300000
        check   (FRAME+40).l,$7fff0000
        check   (FRAME+44).l,\4
        check   (FRAME+48).l,\5
        move.l  (FRAME+12).l,d0
        and.l   #$e0000000,d0
        check   d0,$60000000     ; STAG = NaN, including register sources
        endm

        org     0
        dc.l    $e000,start
        rept    254
        dc.l    fail
        endr
start:
        move.w  #$2700,sr
        moveq   #0,d7
        fmove.l #$4000,fpcr
        sourcecase fadd,x,fp1,$a0000000,1
        sourcecase fsqrt,x,fp1,$a0000000,1
        sourcecase fcmp,x,fp1,$a0000000,1
        sourcecase fadd,x,snan,$a0000000,1
        sourcecase fsqrt,x,snan,$a0000000,1
        sourcecase fadd,s,snan_s,$20000000,0
        sourcecase fadd,d,snan_d,$20000000,0

        ; Destination SNaN is independently retained in FPTEMP.
        addq.w  #1,d7
        fmovem.x snan,fp0
        fmove.l #6,fp1
        fadd.x  fp1,fp0
        fsave   (FRAME).l
        check   (FRAME+28).l,$7fff0000
        check   (FRAME+32).l,$a0000000
        check   (FRAME+36).l,1

        move.l  #datatype_handler,($dc).w
        move.l  #snan_handler,($d8).w
        clr.l   (COUNT).l
        clr.l   (DTCOUNT).l
        addq.w  #1,d7
        fmovem.x denormal,fp0
        fmovem.x snan,fp1
        move.l  #after_snan,a3
        move.l  #$a0000000,d3
        fadd.x  fp1,fp0
        bra     fail
after_snan:
        fnop
        check   (COUNT).l,1
        check   (DTCOUNT).l,1
        ; A QNaN control takes the datatype exception, with no SNAN.
        addq.w  #1,d7
        fmovem.x denormal,fp0
        fmovem.x qnan,fp1
        move.l  #after_qnan,a3
        move.l  #$e0000000,d3
        fadd.x  fp1,fp0
        bra     fail
after_qnan:
        fnop
        check   (COUNT).l,1
        check   (DTCOUNT).l,2
        move.w  #$600d,($f102).l
        stop    #$2700

datatype_handler:
        fsave   (FRAME).l
        check   (FRAME).l,$41600000
        check   (FRAME+88).l,$7fff0000
        cmp.l   (FRAME+92).l,d3
        bne     fail
        check   (FRAME+96).l,1
        fmove.l fpsr,d0
        and.l   #$ff00,d0
        check   d0,0             ; datatype precedes arithmetic status
        addq.l  #1,(DTCOUNT).l
        move.l  #$fe000000,(FRAME+8).l
        frestore (FRAME).l
        move.l  a3,2(sp)
        rte
snan_handler:
        fsave   ($3400).l
        check   ($3400).l,$41300000
        check   ($342c).l,$a0000000
        addq.l  #1,(COUNT).l
        rte
fail:
        move.w  d7,($f100).l
        move.w  #$bad0,($f102).l
        stop    #$2700
snan:   dc.l    $7fff0000,$a0000000,1
qnan:   dc.l    $7fff0000,$e0000000,1
snan_s: dc.l    $7fa00000
snan_d: dc.l    $7ff40000,0
denormal: dc.l  0,$40000000,0
