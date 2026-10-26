; Audit: a load after a store to the same line, with the dirty line pushed
; and invalidated in between (CPUSHA DC / BC), reads the value stored --
; as a program that pushes a return address, calls CacheClearU and
; returns (exec's ramlib; cputest's flushcache then RTS).  Copyback
; memory, many addresses and alignments, the RTS form and the data form;
; interleaved with a line that was never cached and one dirty in another
; set.  On Maximig (Z3 RAM) such an RTS returned the slot's old contents.
;   1  data form: wrong value, 2  RTS form: returned to the wrong place,
;   3  BC form: wrong value, 99 an unexpected exception
 org 0
 dc.l $3400,start
 rept 254
 dc.l unexpected
 endr
 org $400
start:
 move.w #$2700,sr
 ; copyback everywhere but the bench's page: DTT0 all (CM=01 copyback),
 ; DTT1 $00000000-$0000FFFF... (a TTR covers 16 MB blocks: the whole low
 ; space is copyback, the registers at $F000 are reached through DTT0 too:
 ; the bench ignores caching of them -- its mailbox writes go to the bus
 ; (write-through would be needed); so keep them cache-inhibited by a
 ; second TTR that matches first: DTT0 = $F0xx..? the bench registers are
 ; at $F000-$FFFF within the first 16 MB.  Use the MMU instead: identity
 ; pages, page $F000 cache-inhibited, the rest copyback.
 lea ($4400).l,a0
 moveq #0,d0
 moveq #63,d2
.tl:
 move.l d0,d3
 lsl.l #8,d3
 lsl.l #4,d3
 addq.l #3,d3
 cmpi.l #15,d0
 bne.s .cb
 ori.w #$40,d3            ; page $F000: cache-inhibited
 bra.s .st
.cb:
 ori.w #$20,d3            ; copyback
.st:
 move.l d3,(a0)+
 addq.l #1,d0
 dbra d2,.tl
 move.l #$00004203,($4000).l
 move.l #$00004403,($4200).l
 cpusha bc
 move.l #$4000,d0
 movec d0,urp
 movec d0,srp
 pflusha
 move.l #$8000,d0
 movec d0,tc
 move.l #$80008000,d0
 movec d0,cacr
 ; leave the vector table and the tables alone: data at $5000-$7FFF,
 ; the stack at $8000-$8FFF, a second dirty region at $9000-$9FFF
 lea $9000,sp
 moveq #0,d7              ; the pass counter
 move.l #$12345678,d6     ; the value, changes every pass
pass:
 ; (a) data form, every long of a 256-byte window, stride 20 bytes to
 ; walk the lines and the alignments
 lea $5000,a0
 move.w #200,d2
.d:
 addq.l #1,d6
 move.l d6,(a0)
 move.l d7,($9000).l      ; another dirty line in another set
 cpusha dc
 move.l (a0),d0
 cmp.l d6,d0
 beq.s .dok
 move.w #1,($f100).l
 bra failed
.dok:
 lea 20(a0),a0
 dbra d2,.d
 ; (b) the RTS form: push the return, flush, return
 move.w #40,d2
.r:
 pea .rok(pc)
 move.l d6,-(sp)
 addq.l #4,sp
 cpusha dc
 rts
 move.w #2,($f100).l
 bra failed
.rok:
 lea -8(sp),sp            ; move the stack down: new lines each time
 dbra d2,.r
 lea $9000,sp
 ; (c) both caches: store, CPUSHA BC, load
 lea $6000,a0
 move.w #60,d2
.b:
 addq.l #1,d6
 move.l d6,(a0)
 cpusha bc
 move.l (a0),d0
 cmp.l d6,d0
 beq.s .bok
 move.w #3,($f100).l
 bra failed
.bok:
 lea 36(a0),a0
 dbra d2,.b
 addq.l #1,d7
 cmpi.l #8,d7
 bcs pass
 move.w #$600d,($f102).l
.done:
 bra.s .done
unexpected:
 move.w #99,($f100).l
failed:
 move.w #$bad0,($f102).l
.f:
 bra.s .f
