/* Whetstone (Curnow and Wichmann, double precision, the module structure and
   loop counts of the classic C version: one LOOP is 100,000 Whetstone
   instructions, so MWIPS = LOOP * 0.1 / seconds) for the AP68060 bench.

   No libc.  The 68060 has no transcendental instructions (an Amiga runs them
   in the 68060.library's software package), so sin, cos, atan, exp and log
   are computed here in C -- range reduction and polynomials, about double
   precision; sqrt is the FPU's FSQRT.  The same source compiled on the host
   (tb/run_bench.sh, -DHOST) produces the reference results (whet_ref.h) that
   main checks at the end: every value within 1e-9, relative.

   LOOPS comes from the build (two counts); two stamps bracket the timed
   modules (tb/c/crt0.s). */

#ifdef HOST
#include <stdio.h>
#include <math.h>
#define SQRT(x) sqrt(x)
#else
extern void stamp(void);
double __fsqrt(__reg("fp0") double x) = "\tfsqrt.x\tfp0,fp0";
#define SQRT(x) __fsqrt(x)
#endif

#ifndef LOOPS
#define LOOPS 1
#endif

#define PI_2_HI 1.57079632673412561417e+00
#define PI_2_LO 6.07710050650619224932e-11
#define LN2_HI  6.93147180369123816490e-01
#define LN2_LO  1.90821492927058770002e-10

typedef union { double d; unsigned long w[2]; } dbits;   /* big-endian on the 68k */

static double my_floor_near(double x)      /* round to nearest integer */
{
	long k = (long)(x >= 0.0 ? x + 0.5 : x - 0.5);
	return (double)k;
}

/* sin and cos of |r| <= pi/4 */
static double sin_k(double r)
{
	double z = r * r;
	return r + r * z * (-1.66666666666666666e-1 + z * (8.33333333333333333e-3 +
	       z * (-1.98412698412698413e-4 + z * (2.75573192239858907e-6 +
	       z * (-2.50521083854417188e-8 + z * (1.60590438368216146e-10 +
	       z * -7.64716373181981648e-13))))));
}
static double cos_k(double r)
{
	double z = r * r;
	return 1.0 + z * (-0.5 + z * (4.16666666666666667e-2 + z * (-1.38888888888888889e-3 +
	       z * (2.48015873015873016e-5 + z * (-2.75573192239858907e-7 +
	       z * (2.08767569878680990e-9 + z * -1.14707455977297247e-11))))));
}
static double my_sin(double x)
{
	double k = my_floor_near(x / (PI_2_HI + PI_2_LO));
	double r = (x - k * PI_2_HI) - k * PI_2_LO;
	long q = (long)k & 3;
	if (q == 0) return sin_k(r);
	if (q == 1) return cos_k(r);
	if (q == 2) return -sin_k(r);
	return -cos_k(r);
}
static double my_cos(double x)
{
	double k = my_floor_near(x / (PI_2_HI + PI_2_LO));
	double r = (x - k * PI_2_HI) - k * PI_2_LO;
	long q = (long)k & 3;
	if (q == 0) return cos_k(r);
	if (q == 1) return -sin_k(r);
	if (q == 2) return -cos_k(r);
	return sin_k(r);
}
/* atan: |x| > 1 -> pi/2 - atan(1/x); > tan(pi/12) -> pi/6 + atan of the
   reduced argument; then the series to x^25 */
static double my_atan(double x)
{
	double s = 1.0, a = 0.0, z, p;
	int inv = 0, six = 0;
	if (x < 0.0) { s = -1.0; x = -x; }
	if (x > 1.0) { x = 1.0 / x; inv = 1; }
	if (x > 0.26794919243112270) {
		x = (x * 1.73205080756887729 - 1.0) / (x + 1.73205080756887729);
		six = 1;
	}
	z = x * x;
	p = x * (1.0 + z * (-1.0/3 + z * (1.0/5 + z * (-1.0/7 + z * (1.0/9 + z * (-1.0/11 +
	    z * (1.0/13 + z * (-1.0/15 + z * (1.0/17 + z * (-1.0/19 + z * (1.0/21 +
	    z * (-1.0/23 + z * (1.0/25)))))))))))));
	if (six) p += 0.52359877559829887;
	if (inv) p = (PI_2_HI + PI_2_LO) - p;
	return s * p;
}
/* exp: x = k ln2 + r, |r| <= ln2/2; e^r by its series; 2^k into the exponent */
static double my_exp(double x)
{
	double k = my_floor_near(x / (LN2_HI + LN2_LO));
	double r = (x - k * LN2_HI) - k * LN2_LO;
	double p = 1.0 + r * (1.0 + r * (1.0/2 + r * (1.0/6 + r * (1.0/24 + r * (1.0/120 +
	           r * (1.0/720 + r * (1.0/5040 + r * (1.0/40320 + r * (1.0/362880 +
	           r * (1.0/3628800 + r * (1.0/39916800 + r * (1.0/479001600))))))))))));
	dbits b;
	b.d = p;
#ifdef HOST
	{ unsigned long long u; __builtin_memcpy(&u, &p, 8); u += (unsigned long long)(long long)k << 52;
	  __builtin_memcpy(&p, &u, 8); return p; }
#else
	b.w[0] += (unsigned long)((long)k) << 20;
	return b.d;
#endif
}
/* log: x = m 2^e, m in [sqrt(1/2), sqrt(2)); log m = 2 atanh((m-1)/(m+1)) */
static double my_log(double x)
{
	double m, s, z, p;
	long e;
#ifdef HOST
	{ unsigned long long u; __builtin_memcpy(&u, &x, 8);
	  e = (long)((u >> 52) & 0x7FF) - 1023; u = (u & 0x000FFFFFFFFFFFFFULL) | 0x3FF0000000000000ULL;
	  __builtin_memcpy(&m, &u, 8); }
#else
	{ dbits b; b.d = x;
	  e = (long)((b.w[0] >> 20) & 0x7FF) - 1023; b.w[0] = (b.w[0] & 0x000FFFFF) | 0x3FF00000;
	  m = b.d; }
#endif
	if (m > 1.41421356237309505) { m = m * 0.5; e++; }
	s = (m - 1.0) / (m + 1.0);
	z = s * s;
	p = 2.0 * s * (1.0 + z * (1.0/3 + z * (1.0/5 + z * (1.0/7 + z * (1.0/9 + z * (1.0/11 +
	    z * (1.0/13 + z * (1.0/15 + z * (1.0/17 + z * (1.0/19 + z * 1.0/21))))))))));
	return (double)e * LN2_HI + ((double)e * LN2_LO + p);
}

static double T, T1, T2, E1[5];
static int J, K, L;

static void PA(double E[])
{
	J = 0;
	do {
		E[1] = (E[1] + E[2] + E[3] - E[4]) * T;
		E[2] = (E[1] + E[2] - E[3] + E[4]) * T;
		E[3] = (E[1] - E[2] + E[3] + E[4]) * T;
		E[4] = (-E[1] + E[2] + E[3] + E[4]) / T2;
		J += 1;
	} while (J < 6);
}
static void P0(void) { E1[J] = E1[K]; E1[K] = E1[L]; E1[L] = E1[J]; }
static void P3(double X, double Y, double *Z)
{
	double X1 = X, Y1 = Y;
	X1 = T * (X1 + Y1);
	Y1 = T * (X1 + Y1);
	*Z = (X1 + Y1) / T2;
}

/* the results checked at the end */
static double R[16];

int main(void)
{
	long I, N2, N3, N4, N6, N7, N8, N9, N11;
	double X1, X2, X3, X4, X, Y, Z;
	int ix;

	T = .499975; T1 = 0.50025; T2 = 2.0;
	N2 = 12 * LOOPS; N3 = 14 * LOOPS; N4 = 345 * LOOPS; N6 = 210 * LOOPS;
	N7 = 32 * LOOPS; N8 = 899 * LOOPS; N9 = 616 * LOOPS; N11 = 93 * LOOPS;
	X1 = 1.0; X2 = X3 = X4 = -1.0;
#ifndef HOST
	stamp();
#endif
	/* module 1 (N1 = 0) is not run; module 2: array elements */
	E1[1] = 1.0; E1[2] = -1.0; E1[3] = -1.0; E1[4] = -1.0;
	for (I = 1; I <= N2; I++) {
		E1[1] = ( E1[1] + E1[2] + E1[3] - E1[4]) * T;
		E1[2] = ( E1[1] + E1[2] - E1[3] + E1[4]) * T;
		E1[3] = ( E1[1] - E1[2] + E1[3] + E1[4]) * T;
		E1[4] = (-E1[1] + E1[2] + E1[3] + E1[4]) * T;
	}
	R[0] = E1[1]; R[1] = E1[4];
	/* module 3: array as parameter */
	for (I = 1; I <= N3; I++) PA(E1);
	R[2] = E1[1]; R[3] = E1[4];
	/* module 4: conditional jumps */
	J = 1;
	for (I = 1; I <= N4; I++) {
		if (J == 1) J = 2; else J = 3;
		if (J > 2) J = 0; else J = 1;
		if (J < 1) J = 1; else J = 0;
	}
	R[4] = J;
	/* module 6: integer arithmetic (module 5 is omitted) */
	J = 1; K = 2; L = 3;
	for (I = 1; I <= N6; I++) {
		J = J * (K - J) * (L - K);
		K = L * K - (L - J) * K;
		L = (L - K) * (K + J);
		E1[L - 1] = J + K + L;
		E1[K - 1] = J * K * L;
	}
	R[5] = E1[1]; R[6] = E1[2];
	/* module 7: trigonometric functions */
	X = Y = 0.5;
	for (I = 1; I <= N7; I++) {
		X = T * my_atan(T2 * my_sin(X) * my_cos(X) / (my_cos(X + Y) + my_cos(X - Y) - 1.0));
		Y = T * my_atan(T2 * my_sin(Y) * my_cos(Y) / (my_cos(X + Y) + my_cos(X - Y) - 1.0));
	}
	R[7] = X; R[8] = Y;
	/* module 8: procedure calls */
	X = Y = Z = 1.0;
	for (I = 1; I <= N8; I++) P3(X, Y, &Z);
	R[9] = Z;
	/* module 9: array references */
	J = 1; K = 2; L = 3;
	E1[1] = 1.0; E1[2] = 2.0; E1[3] = 3.0;
	for (I = 1; I <= N9; I++) P0();
	R[10] = E1[1]; R[11] = E1[3];
	/* module 10: integer arithmetic (N10 = 0) is not run; module 11:
	   standard functions */
	X = 0.75;
	for (I = 1; I <= N11; I++) X = SQRT(my_exp(my_log(X) / T1));
	R[12] = X;
#ifdef HOST
	(void)X1; (void)X2; (void)X3; (void)X4;
	printf("/* generated by tb/run_bench.sh from whet.c -DHOST, LOOPS=%d */\n", LOOPS);
	printf("static const double REF[13] = {\n");
	for (ix = 0; ix < 13; ix++) printf("\t%.17g,\n", R[ix]);
	printf("};\n");
	return 0;
#else
	stamp();
	(void)X1; (void)X2; (void)X3; (void)X4;
	{
#include "whet_ref.h"
		for (ix = 0; ix < 13; ix++) {
			double d = R[ix] - REF[ix], m = REF[ix] < 0.0 ? -REF[ix] : REF[ix];
			if (d < 0.0) d = -d;
			if (d > 1e-9 * (m > 1.0 ? m : 1.0)) return 1 + ix;
		}
	}
	return 0;
#endif
}
