// Dump WinUAE's per-opcode 68040 decode table (readcpu.cpp over table68k).
#include "sysconfig.h"
#include "sysdeps.h"
#include "readcpu.h"
#include <stdio.h>
int main() {
	init_table68k();
	// cpu_level 4 = 68040, as gencpu applies it
	for (int op = 0; op < 65536; op++) {
		struct instr *t = &table68k[op];
		int valid = t->mnemo != i_ILLG && t->clev <= 4 &&
		            !(t->unimpclev > 0 && 4 >= t->unimpclev);
		const char *name = "ILLG";
		for (int i = 0; lookuptab[i].name; i++)
			if (lookuptab[i].mnemo == t->mnemo) { name = (const char*)lookuptab[i].name; break; }
		printf("%04x %d %s %d %d %d %d %d %d %d %d %d\n", op, valid, name, t->size,
		       t->smode, t->sreg, t->dmode, t->dreg, t->plev, t->cc, t->clev, t->unimpclev);
	}
	return 0;
}
