; Cached vs bypassing memory bitfield reads must agree.
;
; The size of a bitfield's memory read decides whether ap040_cache serves it:
; only an access inside one aligned longword is served, so a longword at an
; arbitrary address bypasses three times in four while a byte never does.
; Sizing the read by the field's span therefore moves most bitfield reads onto
; the cached path, and nothing else here notices -- the cputest corpus forces
; CACR and TC to zero, and t_cache runs untranslated.
;
; This runs the same sweep twice, once with both caches off and once with them
; on, and compares the two result buffers.  No oracle is needed: the uncached
; pass IS the oracle, and any disagreement is the cache serving something the
; bus would not have.

FAILREG         equ $F100
DONEREG         equ $F102
buf             equ $5000       ; 16 bytes of pattern
resA            equ $5100       ; uncached results
resB            equ $5500       ; cached results
resC            equ $5900       ; cached + translated results

        org     0
        dc.l    $3400,start
        rept    254
        dc.l    unexpected
        endr

        org     $400
start:
        move.w  #$2700,sr
        lea     (buf).l,a0
        move.l  #$12345678,(a0)
        move.l  #$9ABCDEF0,4(a0)
        move.l  #$0F1E2D3C,8(a0)
        move.l  #$4B5A6978,12(a0)

        ; pass 1: caches OFF
        moveq   #0,d0
        movec   d0,cacr
        lea     (resA).l,a1
        bsr     sweep

        ; pass 2: caches ON, cold
        move.l  #$00000808,d0          ; CINV both, all
        movec   d0,cacr
        move.l  #$80008000,d0
        movec   d0,cacr
        lea     (resB).l,a1
        bsr     sweep

        ; pass 3: caches ON and TRANSLATION ON.  NetBSD runs here and nothing
        ; else does -- t_cache is untranslated, and the cputest corpus forces
        ; both CACR and TC to zero.  The mapping is identity, so the answers
        ; must match the uncached untranslated pass exactly.
        lea     ($4400).l,a0
        moveq   #0,d0
        moveq   #63,d1
tloop:
        move.l  d0,d2
        lsl.l   #8,d2
        lsl.l   #4,d2                  ; i << 12
        addq.l  #3,d2                  ; resident
        move.l  d2,(a0)+
        addq.l  #1,d0
        dbra    d1,tloop
        move.l  #$00004203,($4000).l   ; root entry 0 -> pointer table
        move.l  #$00004403,($4200).l   ; pointer entry 0 -> page table
        move.l  #$4000,d0
        movec   d0,urp
        movec   d0,srp
        move.l  #$8000,d0              ; E=1, 4K pages
        movec   d0,tc
        pflusha
        move.l  #$00000808,d0          ; CINV both, all
        movec   d0,cacr
        move.l  #$80008000,d0
        movec   d0,cacr
        lea     (resC).l,a1
        bsr     sweep

        ; back to a quiet regime before comparing
        moveq   #0,d0
        movec   d0,cacr
        movec   d0,tc
        pflusha

        ; compare
        lea     (resA).l,a1
        lea     (resB).l,a2
        lea     (resC).l,a3
        move.w  #191,d3
cmp_lp:
        move.l  (a1)+,d0
        move.l  (a2)+,d1
        move.l  (a3)+,d4
        cmp.l   d0,d1
        bne     mismatch
        cmp.l   d0,d4
        bne     mismatch_x
        dbra    d3,cmp_lp

        move.w  #$600D,(DONEREG).l
        stop    #$2700

; sweep: for bib 0..7 and width 1..24, extract at (buf){bib:w} and store
sweep:
        lea     (buf).l,a0
        moveq   #0,d1                  ; bib
sw_off:
        moveq   #1,d2                  ; width
sw_w:
        bfextu  (a0){d1:d2},d0
        move.l  d0,(a1)+
        addq.l  #1,d2
        cmp.l   #25,d2
        bne     sw_w
        addq.l  #1,d1
        cmp.l   #8,d1
        bne     sw_off
        rts

mismatch_x:
        ; cached+translated disagreed with the bus: add 1000 to the index
        move.w  #191,d7
        sub.w   d3,d7
        add.w   #1000,d7
        move.w  d7,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
        bra     halt

mismatch:
        ; d3 counts down from 191; report the index that disagreed
        move.w  #191,d7
        sub.w   d3,d7
        move.w  d7,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
        bra     halt

unexpected:
        move.w  #$FFFF,(FAILREG).l
        move.w  #$BAD0,(DONEREG).l
halt:
        bra     halt
