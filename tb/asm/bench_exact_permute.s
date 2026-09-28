; Original CODE 3 Swap/InitPermute/Permute bytes at original link addresses.
; One n=7 invocation of the recursive routine; the Speedometer wrapper calls
; it 25 times. T(1)=1, T(n)=1+n*T(n-1), hence T(7)=8660. Every swap is undone,
; so the initialized array must be restored, including both adjacent guards.
; Host extraction verifies resource and 200-byte kernel SHA-256 identities.
 org 0
 dc.l $e000
 dc.l start
 rept 254
 dc.l unexpected
 endr
 org $400
start:
 move.w #$2700,sr
 move.l #$80008000,d0
 movec d0,cacr
 lea ($8000).l,a5
 lea ($4000).l,a2
 move.w #$a55a,(a2)
 move.w #$5aa5,16(a2)
 clr.l -$2058(a5)
 move.l a2,-(sp)
 jsr ($65ba).l
 addq.l #4,sp
 move.l a2,-(sp)
 move.w #7,-(sp)
 jsr ($65ea).l
 addq.l #6,sp
 cmpi.l #8660,-$2058(a5)
 bne unexpected
 cmpi.w #$a55a,(a2)
 bne unexpected
 cmpi.w #$5aa5,16(a2)
 bne unexpected
 moveq #0,d0
 lea 2(a2),a0
check:
 cmp.w (a0)+,d0
 bne unexpected
 addq.w #1,d0
 cmpi.w #7,d0
 bne check
 move.w #$600d,($f102).l
 stop #$2700
unexpected:
 move.w #$99,($f100).l
 move.w #$bad0,($f102).l
 bra unexpected
 org $659c
 incbin "build/exact_permute.bin"
