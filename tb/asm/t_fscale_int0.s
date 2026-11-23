; FSCALE (unimplemented on the 68040) with an integer source of zero: the
; UNIMP frame's ETEMP must be a true zero ($0000 0000 0000), not an
; unnormal zero with the integer's exponent ($401E/$400E/$4006, mantissa
; 0) -- the FPSP's FSCALE then returned +-Inf, and NeXTSTEP's scalb() and
; pow() with it. Likewise a nonzero integer arrives normalized.
; Assemble with REV40=1 for $40, otherwise the existing $41 format.
        ifd     REV40
UNIMP   equ     $40280000
BIAS    equ     -8
        else
UNIMP   equ     $41300000
BIAS    equ     0
        endif
FRAME   equ     $3000
COUNT   equ     $3800
FPT     equ     FRAME+28+BIAS           ; FPTEMP: the destination
ET      equ     FRAME+40+BIAS           ; ETEMP: the source

check   macro
        cmp.l   #\2,\1
        beq.s   ok\@
        move.w  #\3,($f100).l
        jmp     fail
ok\@:
        endm

; \1 = test number base; checks the frame against UNIMP, FPTEMP 7.0 and
; ETEMP \2 (exponent word) \3 (mantissa high)
frame   macro
        move.l  (FRAME).l,d0
        check   d0,UNIMP,\1
        move.l  (FPT).l,d0
        check   d0,$40010000,\1+1
        move.l  (FPT+4).l,d0
        check   d0,$e0000000,\1+2
        move.l  (ET).l,d0
        check   d0,\2,\1+3
        move.l  (ET+4).l,d0
        check   d0,\3,\1+4
        move.l  (ET+8).l,d0
        check   d0,0,\1+5
        endm

        org     0
        dc.l    $e000,start
        rept    254
        dc.l    fail
        endr
start:
        move.w  #$2700,sr
        move.l  #unimp_handler,($2c).w
        clr.l   (COUNT).l

        fmove.l #7,fp0
        moveq   #0,d0
        fscale.l d0,fp0
        frame   10,$00000000,$00000000

        fmove.l #7,fp0
        moveq   #0,d0
        fscale.w d0,fp0
        frame   20,$00000000,$00000000

        fmove.l #7,fp0
        moveq   #0,d0
        fscale.b d0,fp0
        frame   30,$00000000,$00000000

        ; a nonzero integer: 2 = $4000 8000 0000 0000
        fmove.l #7,fp0
        moveq   #2,d0
        fscale.l d0,fp0
        frame   40,$40000000,$80000000

        ; a negative one: -1 = $BFFF 8000 0000 0000
        fmove.l #7,fp0
        moveq   #-1,d0
        fscale.w d0,fp0
        frame   50,$bfff0000,$80000000

        move.l  (COUNT).l,d0
        check   d0,5,60
        check   sp,$e000,61
        move.w  #$600d,($f102).l
        bra.s   *
unimp_handler:
        addq.l  #1,(COUNT).l
        fsave   (FRAME).l
        frestore (null_frame).l
        rte
null_frame:
        dc.l    0
fail:
        move.w  #$bad0,($f102).l
        bra.s   *
