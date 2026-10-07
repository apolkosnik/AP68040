/*
 * AP68040-60 bench: a mix of small C kernels, each bracketed by cycle
 * stamps (bench register $F108), each checked against a value computed
 * independently in the program.  Built by tb/build_c.sh kern (vbcc);
 * ASFLAGS=-DCOPYBACK=1 runs it in user mode with copyback caches.
 * main returns 0 on success, else the number of the failing kernel.
 *   1 CRC-32, table driven, 4 KB
 *   2 insertion sort of 256 longs
 *   3 integer matrix multiply 16x16
 *   4 double matrix multiply 8x8 (FPU)
 *   5 string copy, length and reverse, 64 strings
 *   6 linked list walk, 512 nodes, 8 passes
 *   7 byte histogram of 4 KB
 */
extern void stamp(void);

static unsigned long crc_tab[256];
static unsigned char buf[4096];
static long arr[256];
static long ma[16][16], mb[16][16], mc[16][16];
static double da[8][8], db[8][8], dc[8][8];
static char s1[64][32], s2[64][32];
struct node { struct node *next; long v; };
static struct node nodes[512];
static unsigned long hist[256];

static unsigned long lcg = 12345;
static unsigned long rnd(void)
{
	lcg = lcg * 1103515245UL + 12345UL;
	return lcg >> 8;
}

int main(void)
{
	unsigned long crc, i, j, k, sum;
	long t;
	double ds;
	struct node *p;

	for (i = 0; i < 256; i++) {
		unsigned long c = i;
		for (k = 0; k < 8; k++)
			c = (c & 1) ? (0xEDB88320UL ^ (c >> 1)) : (c >> 1);
		crc_tab[i] = c;
	}
	for (i = 0; i < 4096; i++)
		buf[i] = (unsigned char)rnd();
	for (i = 0; i < 256; i++)
		arr[i] = (long)(rnd() & 0xFFFF) - 0x8000;
	for (i = 0; i < 16; i++)
		for (j = 0; j < 16; j++) {
			ma[i][j] = (long)(i + j) - 7;
			mb[i][j] = (long)(i * 3) - (long)j;
		}
	for (i = 0; i < 8; i++)
		for (j = 0; j < 8; j++) {
			da[i][j] = (double)(i + 1) / (double)(j + 2);
			db[i][j] = (double)(j + 1) * 0.25 - (double)i;
		}
	for (i = 0; i < 64; i++) {
		for (j = 0; j < 31; j++)
			s1[i][j] = (char)('a' + (i + j) % 26);
		s1[i][(i % 30) + 1] = 0;
	}
	for (i = 0; i < 512; i++) {
		nodes[i].v = (long)i;
		nodes[i].next = &nodes[(i * 37 + 11) % 512];
	}

	stamp();

	/* 1: CRC-32 */
	crc = 0xFFFFFFFFUL;
	for (i = 0; i < 4096; i++)
		crc = crc_tab[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
	crc ^= 0xFFFFFFFFUL;
	stamp();

	/* 2: insertion sort */
	for (i = 1; i < 256; i++) {
		t = arr[i];
		for (j = i; j > 0 && arr[j - 1] > t; j--)
			arr[j] = arr[j - 1];
		arr[j] = t;
	}
	stamp();

	/* 3: integer matrix multiply */
	for (i = 0; i < 16; i++)
		for (j = 0; j < 16; j++) {
			long a = 0;
			for (k = 0; k < 16; k++)
				a += ma[i][k] * mb[k][j];
			mc[i][j] = a;
		}
	stamp();

	/* 4: double matrix multiply */
	for (i = 0; i < 8; i++)
		for (j = 0; j < 8; j++) {
			double a = 0.0;
			for (k = 0; k < 8; k++)
				a += da[i][k] * db[k][j];
			dc[i][j] = a;
		}
	stamp();

	/* 5: strings: copy, length, reverse */
	sum = 0;
	for (i = 0; i < 64; i++) {
		char *d = s2[i], *s = s1[i];
		unsigned long n;
		while ((*d++ = *s++) != 0)
			;
		for (n = 0; s2[i][n]; n++)
			;
		for (j = 0; j < n / 2; j++) {
			char c = s2[i][j];
			s2[i][j] = s2[i][n - 1 - j];
			s2[i][n - 1 - j] = c;
		}
		sum += n + (unsigned char)s2[i][0];
	}
	stamp();

	/* 6: linked list walk */
	t = 0;
	for (k = 0; k < 8; k++) {
		p = &nodes[0];
		for (i = 0; i < 512; i++) {
			t += p->v;
			p = p->next;
		}
	}
	stamp();

	/* 7: byte histogram */
	for (i = 0; i < 4096; i++)
		hist[buf[i]]++;
	stamp();

	/* checks, recomputed differently */
	{
		/* 1: CRC bit by bit */
		unsigned long c = 0xFFFFFFFFUL;
		for (i = 0; i < 4096; i++) {
			c ^= buf[i];
			for (k = 0; k < 8; k++)
				c = (c >> 1) ^ (0xEDB88320UL & (0UL - (c & 1)));
		}
		if ((c ^ 0xFFFFFFFFUL) != crc)
			return 1;
	}
	for (i = 1; i < 256; i++)
		if (arr[i - 1] > arr[i])
			return 2;
	for (i = 0; i < 16; i += 5)
		for (j = 0; j < 16; j += 3) {
			long a = 0;
			for (k = 0; k < 16; k++)
				a += ((long)(i + k) - 7) * ((long)(k * 3) - (long)j);
			if (a != mc[i][j])
				return 3;
		}
	ds = 0.0;
	for (i = 0; i < 8; i++)
		for (j = 0; j < 8; j++)
			ds += dc[i][j];
	{
		/* sum of dc = sum_k (sum_i da[i][k]) * (sum_j db[k][j]) */
		double e = 0.0;
		for (k = 0; k < 8; k++) {
			double ra = 0.0, rb = 0.0;
			for (i = 0; i < 8; i++)
				ra += da[i][k];
			for (j = 0; j < 8; j++)
				rb += db[k][j];
			e += ra * rb;
		}
		if (ds - e > 1e-9 || e - ds > 1e-9)
			return 4;
	}
	{
		unsigned long s = 0;
		for (i = 0; i < 64; i++) {
			unsigned long n = (i % 30) + 1;
			s += n + (unsigned char)s1[i][n - 1];
		}
		if (s != sum)
			return 5;
	}
	{
		long e = 0;
		unsigned long x = 0;
		for (i = 0; i < 512; i++) {
			e += (long)x;
			x = (x * 37 + 11) % 512;
		}
		if (t != e * 8)
			return 6;
	}
	sum = 0;
	for (i = 0; i < 256; i++)
		sum += hist[i];
	if (sum != 4096)
		return 7;
	return 0;
}
