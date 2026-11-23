; Audit: an interrupt
; taken while the front end runs down a mispredicted path in slow memory.
; Run with +slowlo=N (transfers below $1000 slow and fetches there
; cache-inhibited, as the A4000's chip RAM is): the vector table at 0, and
; each call site with the return its RAS predicts, are in that region; the
; subroutines, the handler and the stacks are fast.  Each subroutine
; rewrites its return address (the RTS goes to ok_*, not the predicted
; back_*) and raises a level-3 interrupt 0-24 instructions before its RTS.
; Each call site runs four times (the fetch's branch target buffer learns
; the RTS, predicting it before decode does).  Four sweeps: user mode caches off; caches on; the MMU on (identity, 4K
; pages, page 0 cache-inhibited); supervisor mode with the ATC flushed.
; On the Amiga (Maximig, cputest under MMU.lib) the CPU went from such an
; interrupt's entry to the predicted path ($14) instead of the handler.
;   1  the predicted path ran, 2  the handler ran a wrong number of times,
;   99 an unexpected exception
HITS equ $3600
 org 0
 dc.l $3400,start
 rept 25
 dc.l unexpected
 endr
 dc.l h_irq
 rept 4
 dc.l unexpected
 endr
 dc.l h_trap0
 rept 223
 dc.l unexpected
 endr

; the call sites: slow memory
 org $400
ccall_0:
 bsr.w csub_0
cback_0:
 bra.w fail1
cok_0:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_1:
 bsr.w csub_1
cback_1:
 bra.w fail1
cok_1:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_2:
 bsr.w csub_2
cback_2:
 bra.w fail1
cok_2:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_3:
 bsr.w csub_3
cback_3:
 bra.w fail1
cok_3:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_4:
 bsr.w csub_4
cback_4:
 bra.w fail1
cok_4:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_5:
 bsr.w csub_5
cback_5:
 bra.w fail1
cok_5:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_6:
 bsr.w csub_6
cback_6:
 bra.w fail1
cok_6:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_7:
 bsr.w csub_7
cback_7:
 bra.w fail1
cok_7:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_8:
 bsr.w csub_8
cback_8:
 bra.w fail1
cok_8:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_9:
 bsr.w csub_9
cback_9:
 bra.w fail1
cok_9:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_10:
 bsr.w csub_10
cback_10:
 bra.w fail1
cok_10:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_11:
 bsr.w csub_11
cback_11:
 bra.w fail1
cok_11:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_12:
 bsr.w csub_12
cback_12:
 bra.w fail1
cok_12:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_13:
 bsr.w csub_13
cback_13:
 bra.w fail1
cok_13:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_14:
 bsr.w csub_14
cback_14:
 bra.w fail1
cok_14:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_15:
 bsr.w csub_15
cback_15:
 bra.w fail1
cok_15:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_16:
 bsr.w csub_16
cback_16:
 bra.w fail1
cok_16:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_17:
 bsr.w csub_17
cback_17:
 bra.w fail1
cok_17:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_18:
 bsr.w csub_18
cback_18:
 bra.w fail1
cok_18:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_19:
 bsr.w csub_19
cback_19:
 bra.w fail1
cok_19:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_20:
 bsr.w csub_20
cback_20:
 bra.w fail1
cok_20:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_21:
 bsr.w csub_21
cback_21:
 bra.w fail1
cok_21:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_22:
 bsr.w csub_22
cback_22:
 bra.w fail1
cok_22:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_23:
 bsr.w csub_23
cback_23:
 bra.w fail1
cok_23:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
ccall_24:
 bsr.w csub_24
cback_24:
 bra.w fail1
cok_24:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_0:
 bsr.w fsub_0
fback_0:
 bra.w fail1
fok_0:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_1:
 bsr.w fsub_1
fback_1:
 bra.w fail1
fok_1:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_2:
 bsr.w fsub_2
fback_2:
 bra.w fail1
fok_2:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_3:
 bsr.w fsub_3
fback_3:
 bra.w fail1
fok_3:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_4:
 bsr.w fsub_4
fback_4:
 bra.w fail1
fok_4:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_5:
 bsr.w fsub_5
fback_5:
 bra.w fail1
fok_5:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_6:
 bsr.w fsub_6
fback_6:
 bra.w fail1
fok_6:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_7:
 bsr.w fsub_7
fback_7:
 bra.w fail1
fok_7:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_8:
 bsr.w fsub_8
fback_8:
 bra.w fail1
fok_8:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_9:
 bsr.w fsub_9
fback_9:
 bra.w fail1
fok_9:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_10:
 bsr.w fsub_10
fback_10:
 bra.w fail1
fok_10:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_11:
 bsr.w fsub_11
fback_11:
 bra.w fail1
fok_11:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_12:
 bsr.w fsub_12
fback_12:
 bra.w fail1
fok_12:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_13:
 bsr.w fsub_13
fback_13:
 bra.w fail1
fok_13:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_14:
 bsr.w fsub_14
fback_14:
 bra.w fail1
fok_14:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_15:
 bsr.w fsub_15
fback_15:
 bra.w fail1
fok_15:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_16:
 bsr.w fsub_16
fback_16:
 bra.w fail1
fok_16:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_17:
 bsr.w fsub_17
fback_17:
 bra.w fail1
fok_17:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_18:
 bsr.w fsub_18
fback_18:
 bra.w fail1
fok_18:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_19:
 bsr.w fsub_19
fback_19:
 bra.w fail1
fok_19:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_20:
 bsr.w fsub_20
fback_20:
 bra.w fail1
fok_20:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_21:
 bsr.w fsub_21
fback_21:
 bra.w fail1
fok_21:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_22:
 bsr.w fsub_22
fback_22:
 bra.w fail1
fok_22:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_23:
 bsr.w fsub_23
fback_23:
 bra.w fail1
fok_23:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts
fcall_24:
 bsr.w fsub_24
fback_24:
 bra.w fail1
fok_24:
 cmpi.l #1,(HITS).w
 bne.w fail2
 rts

; fast memory
 org $1000
start:
 move.w #$2700,sr
 moveq #0,d0
 movec d0,cacr
 movec d0,tc
 clr.w ($f110).l
 lea $3000,a0
 move.l a0,usp
 move.w #$0000,sr
 moveq #3,d7
rp1:
 clr.l (HITS).w
 jsr ccall_0
 dbra d7,rp1
 moveq #3,d7
rp2:
 clr.l (HITS).w
 jsr ccall_1
 dbra d7,rp2
 moveq #3,d7
rp3:
 clr.l (HITS).w
 jsr ccall_2
 dbra d7,rp3
 moveq #3,d7
rp4:
 clr.l (HITS).w
 jsr ccall_3
 dbra d7,rp4
 moveq #3,d7
rp5:
 clr.l (HITS).w
 jsr ccall_4
 dbra d7,rp5
 moveq #3,d7
rp6:
 clr.l (HITS).w
 jsr ccall_5
 dbra d7,rp6
 moveq #3,d7
rp7:
 clr.l (HITS).w
 jsr ccall_6
 dbra d7,rp7
 moveq #3,d7
rp8:
 clr.l (HITS).w
 jsr ccall_7
 dbra d7,rp8
 moveq #3,d7
rp9:
 clr.l (HITS).w
 jsr ccall_8
 dbra d7,rp9
 moveq #3,d7
rp10:
 clr.l (HITS).w
 jsr ccall_9
 dbra d7,rp10
 moveq #3,d7
rp11:
 clr.l (HITS).w
 jsr ccall_10
 dbra d7,rp11
 moveq #3,d7
rp12:
 clr.l (HITS).w
 jsr ccall_11
 dbra d7,rp12
 moveq #3,d7
rp13:
 clr.l (HITS).w
 jsr ccall_12
 dbra d7,rp13
 moveq #3,d7
rp14:
 clr.l (HITS).w
 jsr ccall_13
 dbra d7,rp14
 moveq #3,d7
rp15:
 clr.l (HITS).w
 jsr ccall_14
 dbra d7,rp15
 moveq #3,d7
rp16:
 clr.l (HITS).w
 jsr ccall_15
 dbra d7,rp16
 moveq #3,d7
rp17:
 clr.l (HITS).w
 jsr ccall_16
 dbra d7,rp17
 moveq #3,d7
rp18:
 clr.l (HITS).w
 jsr ccall_17
 dbra d7,rp18
 moveq #3,d7
rp19:
 clr.l (HITS).w
 jsr ccall_18
 dbra d7,rp19
 moveq #3,d7
rp20:
 clr.l (HITS).w
 jsr ccall_19
 dbra d7,rp20
 moveq #3,d7
rp21:
 clr.l (HITS).w
 jsr ccall_20
 dbra d7,rp21
 moveq #3,d7
rp22:
 clr.l (HITS).w
 jsr ccall_21
 dbra d7,rp22
 moveq #3,d7
rp23:
 clr.l (HITS).w
 jsr ccall_22
 dbra d7,rp23
 moveq #3,d7
rp24:
 clr.l (HITS).w
 jsr ccall_23
 dbra d7,rp24
 moveq #3,d7
rp25:
 clr.l (HITS).w
 jsr ccall_24
 dbra d7,rp25
 moveq #0,d1
 trap #0
 moveq #3,d7
rp26:
 clr.l (HITS).w
 jsr ccall_0
 dbra d7,rp26
 moveq #3,d7
rp27:
 clr.l (HITS).w
 jsr ccall_1
 dbra d7,rp27
 moveq #3,d7
rp28:
 clr.l (HITS).w
 jsr ccall_2
 dbra d7,rp28
 moveq #3,d7
rp29:
 clr.l (HITS).w
 jsr ccall_3
 dbra d7,rp29
 moveq #3,d7
rp30:
 clr.l (HITS).w
 jsr ccall_4
 dbra d7,rp30
 moveq #3,d7
rp31:
 clr.l (HITS).w
 jsr ccall_5
 dbra d7,rp31
 moveq #3,d7
rp32:
 clr.l (HITS).w
 jsr ccall_6
 dbra d7,rp32
 moveq #3,d7
rp33:
 clr.l (HITS).w
 jsr ccall_7
 dbra d7,rp33
 moveq #3,d7
rp34:
 clr.l (HITS).w
 jsr ccall_8
 dbra d7,rp34
 moveq #3,d7
rp35:
 clr.l (HITS).w
 jsr ccall_9
 dbra d7,rp35
 moveq #3,d7
rp36:
 clr.l (HITS).w
 jsr ccall_10
 dbra d7,rp36
 moveq #3,d7
rp37:
 clr.l (HITS).w
 jsr ccall_11
 dbra d7,rp37
 moveq #3,d7
rp38:
 clr.l (HITS).w
 jsr ccall_12
 dbra d7,rp38
 moveq #3,d7
rp39:
 clr.l (HITS).w
 jsr ccall_13
 dbra d7,rp39
 moveq #3,d7
rp40:
 clr.l (HITS).w
 jsr ccall_14
 dbra d7,rp40
 moveq #3,d7
rp41:
 clr.l (HITS).w
 jsr ccall_15
 dbra d7,rp41
 moveq #3,d7
rp42:
 clr.l (HITS).w
 jsr ccall_16
 dbra d7,rp42
 moveq #3,d7
rp43:
 clr.l (HITS).w
 jsr ccall_17
 dbra d7,rp43
 moveq #3,d7
rp44:
 clr.l (HITS).w
 jsr ccall_18
 dbra d7,rp44
 moveq #3,d7
rp45:
 clr.l (HITS).w
 jsr ccall_19
 dbra d7,rp45
 moveq #3,d7
rp46:
 clr.l (HITS).w
 jsr ccall_20
 dbra d7,rp46
 moveq #3,d7
rp47:
 clr.l (HITS).w
 jsr ccall_21
 dbra d7,rp47
 moveq #3,d7
rp48:
 clr.l (HITS).w
 jsr ccall_22
 dbra d7,rp48
 moveq #3,d7
rp49:
 clr.l (HITS).w
 jsr ccall_23
 dbra d7,rp49
 moveq #3,d7
rp50:
 clr.l (HITS).w
 jsr ccall_24
 dbra d7,rp50
 moveq #1,d1
 trap #0
 moveq #3,d7
rp51:
 clr.l (HITS).w
 jsr ccall_0
 dbra d7,rp51
 moveq #3,d7
rp52:
 clr.l (HITS).w
 jsr ccall_1
 dbra d7,rp52
 moveq #3,d7
rp53:
 clr.l (HITS).w
 jsr ccall_2
 dbra d7,rp53
 moveq #3,d7
rp54:
 clr.l (HITS).w
 jsr ccall_3
 dbra d7,rp54
 moveq #3,d7
rp55:
 clr.l (HITS).w
 jsr ccall_4
 dbra d7,rp55
 moveq #3,d7
rp56:
 clr.l (HITS).w
 jsr ccall_5
 dbra d7,rp56
 moveq #3,d7
rp57:
 clr.l (HITS).w
 jsr ccall_6
 dbra d7,rp57
 moveq #3,d7
rp58:
 clr.l (HITS).w
 jsr ccall_7
 dbra d7,rp58
 moveq #3,d7
rp59:
 clr.l (HITS).w
 jsr ccall_8
 dbra d7,rp59
 moveq #3,d7
rp60:
 clr.l (HITS).w
 jsr ccall_9
 dbra d7,rp60
 moveq #3,d7
rp61:
 clr.l (HITS).w
 jsr ccall_10
 dbra d7,rp61
 moveq #3,d7
rp62:
 clr.l (HITS).w
 jsr ccall_11
 dbra d7,rp62
 moveq #3,d7
rp63:
 clr.l (HITS).w
 jsr ccall_12
 dbra d7,rp63
 moveq #3,d7
rp64:
 clr.l (HITS).w
 jsr ccall_13
 dbra d7,rp64
 moveq #3,d7
rp65:
 clr.l (HITS).w
 jsr ccall_14
 dbra d7,rp65
 moveq #3,d7
rp66:
 clr.l (HITS).w
 jsr ccall_15
 dbra d7,rp66
 moveq #3,d7
rp67:
 clr.l (HITS).w
 jsr ccall_16
 dbra d7,rp67
 moveq #3,d7
rp68:
 clr.l (HITS).w
 jsr ccall_17
 dbra d7,rp68
 moveq #3,d7
rp69:
 clr.l (HITS).w
 jsr ccall_18
 dbra d7,rp69
 moveq #3,d7
rp70:
 clr.l (HITS).w
 jsr ccall_19
 dbra d7,rp70
 moveq #3,d7
rp71:
 clr.l (HITS).w
 jsr ccall_20
 dbra d7,rp71
 moveq #3,d7
rp72:
 clr.l (HITS).w
 jsr ccall_21
 dbra d7,rp72
 moveq #3,d7
rp73:
 clr.l (HITS).w
 jsr ccall_22
 dbra d7,rp73
 moveq #3,d7
rp74:
 clr.l (HITS).w
 jsr ccall_23
 dbra d7,rp74
 moveq #3,d7
rp75:
 clr.l (HITS).w
 jsr ccall_24
 dbra d7,rp75
 moveq #2,d1
 trap #0
 moveq #3,d7
rp76:
 clr.l (HITS).w
 jsr fcall_0
 dbra d7,rp76
 moveq #3,d7
rp77:
 clr.l (HITS).w
 jsr fcall_1
 dbra d7,rp77
 moveq #3,d7
rp78:
 clr.l (HITS).w
 jsr fcall_2
 dbra d7,rp78
 moveq #3,d7
rp79:
 clr.l (HITS).w
 jsr fcall_3
 dbra d7,rp79
 moveq #3,d7
rp80:
 clr.l (HITS).w
 jsr fcall_4
 dbra d7,rp80
 moveq #3,d7
rp81:
 clr.l (HITS).w
 jsr fcall_5
 dbra d7,rp81
 moveq #3,d7
rp82:
 clr.l (HITS).w
 jsr fcall_6
 dbra d7,rp82
 moveq #3,d7
rp83:
 clr.l (HITS).w
 jsr fcall_7
 dbra d7,rp83
 moveq #3,d7
rp84:
 clr.l (HITS).w
 jsr fcall_8
 dbra d7,rp84
 moveq #3,d7
rp85:
 clr.l (HITS).w
 jsr fcall_9
 dbra d7,rp85
 moveq #3,d7
rp86:
 clr.l (HITS).w
 jsr fcall_10
 dbra d7,rp86
 moveq #3,d7
rp87:
 clr.l (HITS).w
 jsr fcall_11
 dbra d7,rp87
 moveq #3,d7
rp88:
 clr.l (HITS).w
 jsr fcall_12
 dbra d7,rp88
 moveq #3,d7
rp89:
 clr.l (HITS).w
 jsr fcall_13
 dbra d7,rp89
 moveq #3,d7
rp90:
 clr.l (HITS).w
 jsr fcall_14
 dbra d7,rp90
 moveq #3,d7
rp91:
 clr.l (HITS).w
 jsr fcall_15
 dbra d7,rp91
 moveq #3,d7
rp92:
 clr.l (HITS).w
 jsr fcall_16
 dbra d7,rp92
 moveq #3,d7
rp93:
 clr.l (HITS).w
 jsr fcall_17
 dbra d7,rp93
 moveq #3,d7
rp94:
 clr.l (HITS).w
 jsr fcall_18
 dbra d7,rp94
 moveq #3,d7
rp95:
 clr.l (HITS).w
 jsr fcall_19
 dbra d7,rp95
 moveq #3,d7
rp96:
 clr.l (HITS).w
 jsr fcall_20
 dbra d7,rp96
 moveq #3,d7
rp97:
 clr.l (HITS).w
 jsr fcall_21
 dbra d7,rp97
 moveq #3,d7
rp98:
 clr.l (HITS).w
 jsr fcall_22
 dbra d7,rp98
 moveq #3,d7
rp99:
 clr.l (HITS).w
 jsr fcall_23
 dbra d7,rp99
 moveq #3,d7
rp100:
 clr.l (HITS).w
 jsr fcall_24
 dbra d7,rp100
 move.w #$600d,($f102).l
.done:
 bra.s .done

csub_0:
 move.l #cok_0,(sp)
 move.w #3,($f110).l
 tst.l d0
 rts
csub_1:
 move.l #cok_1,(sp)
 move.w #3,($f110).l
 nop
 tst.l d0
 rts
csub_2:
 move.l #cok_2,(sp)
 move.w #3,($f110).l
 nop
 nop
 tst.l d0
 rts
csub_3:
 move.l #cok_3,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 tst.l d0
 rts
csub_4:
 move.l #cok_4,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_5:
 move.l #cok_5,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_6:
 move.l #cok_6,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_7:
 move.l #cok_7,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_8:
 move.l #cok_8,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_9:
 move.l #cok_9,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_10:
 move.l #cok_10,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_11:
 move.l #cok_11,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_12:
 move.l #cok_12,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_13:
 move.l #cok_13,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_14:
 move.l #cok_14,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_15:
 move.l #cok_15,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_16:
 move.l #cok_16,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_17:
 move.l #cok_17,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_18:
 move.l #cok_18,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_19:
 move.l #cok_19,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_20:
 move.l #cok_20,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_21:
 move.l #cok_21,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_22:
 move.l #cok_22,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_23:
 move.l #cok_23,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
csub_24:
 move.l #cok_24,(sp)
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_0:
 move.l #fok_0,(sp)
 pflusha
 move.w #3,($f110).l
 tst.l d0
 rts
fsub_1:
 move.l #fok_1,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 tst.l d0
 rts
fsub_2:
 move.l #fok_2,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 tst.l d0
 rts
fsub_3:
 move.l #fok_3,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 tst.l d0
 rts
fsub_4:
 move.l #fok_4,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_5:
 move.l #fok_5,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_6:
 move.l #fok_6,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_7:
 move.l #fok_7,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_8:
 move.l #fok_8,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_9:
 move.l #fok_9,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_10:
 move.l #fok_10,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_11:
 move.l #fok_11,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_12:
 move.l #fok_12,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_13:
 move.l #fok_13,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_14:
 move.l #fok_14,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_15:
 move.l #fok_15,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_16:
 move.l #fok_16,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_17:
 move.l #fok_17,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_18:
 move.l #fok_18,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_19:
 move.l #fok_19,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_20:
 move.l #fok_20,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_21:
 move.l #fok_21,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_22:
 move.l #fok_22,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_23:
 move.l #fok_23,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts
fsub_24:
 move.l #fok_24,(sp)
 pflusha
 move.w #3,($f110).l
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 nop
 tst.l d0
 rts

; trap #0: d1 = 0 the caches on, 1 and the MMU (identity, 4K pages, page 0
; and the bench registers' page cache-inhibited), 2 and supervisor mode
h_trap0:
 move.l #$80008000,d0
 movec d0,cacr
 cpusha bc
 tst.l d1
 beq.s .t_rte
 lea ($4400).l,a0
 moveq #0,d0
 moveq #63,d2
.tl:
 move.l d0,d3
 lsl.l #8,d3
 lsl.l #4,d3
 addq.l #3,d3
 tst.l d0
 beq.s .ci
 cmpi.l #15,d0
 bne.s .tc
.ci:
 ori.w #$40,d3
.tc:
 move.l d3,(a0)+
 addq.l #1,d0
 dbra d2,.tl
 move.l #$00004203,($4000).l
 move.l #$00004403,($4200).l
 cpusha dc
 move.l #$4000,d0
 movec d0,urp
 movec d0,srp
 pflusha
 move.l #$8000,d0
 movec d0,tc
 cmpi.l #2,d1
 bne.s .t_rte
 ori.w #$2000,(sp)
.t_rte:
 rte

h_irq:
 addq.l #1,(HITS).w
 clr.w ($f110).l
 rte

fail1:
 move.w #1,($f100).l
 bra.s failed
fail2:
 move.w #2,($f100).l
 bra.s failed
unexpected:
 move.w #99,($f100).l
failed:
 move.w #$bad0,($f102).l
.f:
 bra.s .f
