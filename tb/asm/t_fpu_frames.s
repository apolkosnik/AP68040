; Revision-specific FSAVE/FRESTORE wire format, pointers, and lifecycle.
; Assemble with REV40=1 for $40, otherwise the existing $41 format.
        ifd     REV40
IDLE    equ     $40000000
UNIMP   equ     $40280000
BUSY    equ     $40600000
OTHER   equ     $41000000
OTHERU  equ     $41300000
ULEN    equ     44
BIAS    equ     -8
        else
IDLE    equ     $41000000
UNIMP   equ     $41300000
BUSY    equ     $41600000
OTHER   equ     $40000000
OTHERU  equ     $40280000
ULEN    equ     52
BIAS    equ     0
        endif
FRAME   equ     $3000
COPY    equ     $3100
BUFEND  equ     FRAME+ULEN
COUNT   equ     $3800

check   macro
        cmp.l   #\2,\1
        beq.s   ok\@
        move.w  #\3,($f100).l
        jmp     fail
ok\@:
        endm

        org     0
        dc.l    $e000,start
        rept    254
        dc.l    fail
        endr
start:
        move.w  #$2700,sr
        move.l  #unimp_handler,($2c).w
        move.l  #format_handler,($38).w
        move.l  #snan_handler,($d8).w
        clr.l   (COUNT).l
        clr.l   (COUNT+4).l
        lea     (FRAME).l,a1
        fsave   (a1)
        move.l  (a1),d0
        check   d0,0,1
        frestore (a1)+
        check   a1,FRAME+4,2
        fmove.l #7,fp0
        fsave   -(a1)
        check   a1,FRAME,3
        move.l  (a1),d0
        check   d0,IDLE,4
        frestore (a1)+
        check   a1,FRAME+4,5

        ; F-line handler saves an actual pending UNIMP, without emulation.
        move.l  #$a55a1234,(FRAME-4).l
        move.l  #$5aa54321,(BUFEND).l
        fsin.x  fp0
        move.l  (COUNT).l,d0
        check   d0,1,6
        move.l  (FRAME-4).l,d0
        check   d0,$a55a1234,7
        move.l  (BUFEND).l,d0
        check   d0,$5aa54321,8
        move.l  (FRAME).l,d0
        check   d0,UNIMP,9
        ifnd    REV40
        move.l  (FRAME+4).l,d0
        check   d0,$00260000,10
        move.l  (FRAME+8).l,d0
        check   d0,0,11
        endif
        move.l  (FRAME+12+BIAS).l,d0
        check   d0,0,12
        move.l  (FRAME+16+BIAS).l,d0
        check   d0,$000e0000,13
        move.l  (FRAME+20+BIAS).l,d0
        check   d0,0,14
        move.l  (FRAME+24+BIAS).l,d0
        check   d0,0,15
        move.l  (FRAME+28+BIAS).l,d0
        check   d0,$40010000,16
        move.l  (FRAME+32+BIAS).l,d0
        check   d0,$e0000000,17
        move.l  (FRAME+36+BIAS).l,d0
        check   d0,0,18
        move.l  (FRAME+40+BIAS).l,d0
        check   d0,$40010000,19
        move.l  (FRAME+44+BIAS).l,d0
        check   d0,$e0000000,20
        move.l  (FRAME+48+BIAS).l,d0
        check   d0,0,21
        lea     (FRAME).l,a1
        frestore (a1)+
        check   a1,BUFEND,22
        lea     (COPY+ULEN).l,a2
        fsave   -(a2)
        check   a2,COPY,23
        lea     (FRAME).l,a1
        moveq   #ULEN/4-1,d1
compare_unimp:
        move.l  (a1)+,d0
        cmp.l   (a2)+,d0
        bne     fail
        dbra    d1,compare_unimp

        ; Synthetic legal BUSY fields round-trip independently of revision.
        ; E1/E3 clear: restoring must not arm an arithmetic exception.
        lea     (FRAME).l,a1
        moveq   #24,d1
zero_busy:
        clr.l   (a1)+
        dbra    d1,zero_busy
        move.l  #BUSY,(FRAME).l
        move.l  #$40010000,(FRAME+24).l
        move.l  #$e0000000,(FRAME+28).l
        move.l  #$12345678,(FRAME+32).l
        move.l  #$1234abcd,(FRAME+40).l
        move.l  #$00260000,(FRAME+52).l
        move.l  #$c3800000,(FRAME+60).l ; STAG=6, GRS=7
        move.l  #$000e0000,(FRAME+64).l
        move.l  #$a0100000,(FRAME+68).l ; DTAG=5, WBTE15=1
        move.l  #$00100000,(FRAME+72).l ; T (no E1/E3)
        move.l  #$3fff0000,(FRAME+76).l
        move.l  #$80000000,(FRAME+80).l
        move.l  #$11223344,(FRAME+84).l
        move.l  #$40000000,(FRAME+88).l
        move.l  #$98765432,(FRAME+92).l
        move.l  #$55667788,(FRAME+96).l
        move.l  #$feedbeef,(FRAME+100).l
        lea     (FRAME).l,a1
        frestore (a1)+
        check   a1,FRAME+100,24
        lea     (COPY+100).l,a2
        move.l  #$facefeed,(a2)
        fsave   -(a2)
        check   a2,COPY,25
        move.l  (COPY+100).l,d0
        check   d0,$facefeed,26
        lea     (FRAME).l,a1
        moveq   #24,d1
compare_busy:
        move.l  (a1)+,d0
        cmp.l   (a2)+,d0
        bne     fail
        dbra    d1,compare_busy

        ; After BUSY, a short UNIMP cannot inherit omitted CMDREG3B.
        lea     (FRAME).l,a1
        moveq   #ULEN/4-1,d1
zero_unimp:
        clr.l   (a1)+
        dbra    d1,zero_unimp
        move.l  #UNIMP,(FRAME).l
        move.l  #$000e0000,(FRAME+16+BIAS).l
        frestore (FRAME).l
        fsave   (COPY).l
        lea     (FRAME).l,a1
        lea     (COPY).l,a2
        moveq   #ULEN/4-1,d1
compare_second_unimp:
        move.l  (a1)+,d0
        cmp.l   (a2)+,d0
        bne     fail
        dbra    d1,compare_second_unimp

        ; A mismatched revision or frame size must raise format error.
        move.l  #OTHER,(FRAME).l
        frestore (FRAME).l
        move.l  #OTHERU,(FRAME).l
        frestore (FRAME).l
        move.l  #OTHER+$600000,(FRAME).l
        frestore (FRAME).l
        move.l  #IDLE+$10000,(FRAME).l
        frestore (FRAME).l
        move.l  (COUNT+4).l,d0
        check   d0,4,27
        clr.l   (FRAME).l
        frestore (FRAME).l
        fsave   (COPY).l
        move.l  (COPY).l,d0
        check   d0,0,28

        ; Deferred E1 uses the same short/long frame, including GRS and
        ; WBTE15. Restoring it must re-arm an enabled SNAN, not lose it.
        clr.l   (COUNT+8).l
        fmove.l #5,fp2
        fmove.l #$4000,fpcr
        move.l  #$7f800001,($3300).l
        fmove.s ($3300).l,fp2
        fsave   (FRAME).l
        move.l  (COUNT+8).l,d0
        check   d0,0,30
        move.l  (FRAME).l,d0
        check   d0,UNIMP,31
        move.l  (FRAME+24+BIAS).l,d0
        and.l   #$06100000,d0
        check   d0,$04000000,32
        move.l  (FRAME+12+BIAS).l,d0
        and.l   #$03800000,d0
        check   d0,$03800000,33
        move.l  (FRAME+20+BIAS).l,d0
        and.l   #$00100000,d0
        check   d0,$00100000,34
        frestore (FRAME).l
        fnop
        move.l  (COUNT+8).l,d0
        check   d0,1,35
        fmove.l fp2,d0
        check   d0,5,36
        fmove.l #0,fpcr
        frestore (FRAME).l
        fnop
        move.l  (COUNT+8).l,d0
        check   d0,1,37
        check   sp,$e000,29
        move.w  #$600d,($f102).l
        bra.s   *
unimp_handler:
        addq.l  #1,(COUNT).l
        fsave   (FRAME).l
        rte
format_handler:
        addq.l  #1,(COUNT+4).l
        addq.l  #6,2(sp) ; FRESTORE abs.l is six bytes
        rte
snan_handler:
        addq.l  #1,(COUNT+8).l
        rte
fail:
        move.w  #$bad0,($f102).l
        bra.s   *
