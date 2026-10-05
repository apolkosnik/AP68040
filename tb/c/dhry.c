/*
 * AP68040-60 bench: Dhrystone 2.1 (R. P. Weicker), bare metal.
 *
 * The algorithm is the standard one; the I/O is replaced: the loop is
 * bracketed by cycle stamps (bench register $F108), and the final values
 * the benchmark specifies are checked instead of printed (main returns
 * the number of the first wrong one, 0 when all are right).
 *
 * DMIPS = (NUMBER_OF_RUNS / seconds) / 1757, seconds = cycles / f.
 */

#define NUMBER_OF_RUNS 2000

typedef enum { Ident_1, Ident_2, Ident_3, Ident_4, Ident_5 } Enumeration;
typedef int One_Thirty;
typedef int One_Fifty;
typedef char Capital_Letter;
typedef int Boolean;
typedef char Str_30[31];
typedef int Arr_1_Dim[50];
typedef int Arr_2_Dim[50][50];

typedef struct record {
	struct record *Ptr_Comp;
	Enumeration    Discr;
	union {
		struct {
			Enumeration Enum_Comp;
			int         Int_Comp;
			char        Str_Comp[31];
		} var_1;
		struct {
			Enumeration E_Comp_2;
			char        Str_2_Comp[31];
		} var_2;
		struct {
			char Ch_1_Comp;
			char Ch_2_Comp;
		} var_3;
	} variant;
} Rec_Type, *Rec_Pointer;

#define true  1
#define false 0

extern void stamp(void);

Rec_Pointer    Ptr_Glob, Next_Ptr_Glob;
int            Int_Glob;
Boolean        Bool_Glob;
char           Ch_1_Glob, Ch_2_Glob;
int            Arr_1_Glob[50];
int            Arr_2_Glob[50][50];
Rec_Type       Rec_1, Rec_2;

static void str_copy(char *d, const char *s)
{
	while ((*d++ = *s++) != 0)
		;
}

static int str_cmp(const char *a, const char *b)
{
	while (*a && *a == *b) {
		a++;
		b++;
	}
	return (unsigned char)*a - (unsigned char)*b;
}

Boolean Func_3(Enumeration Enum_Par_Val);

Enumeration Func_1(Capital_Letter Ch_1_Par_Val, Capital_Letter Ch_2_Par_Val)
{
	Capital_Letter Ch_1_Loc, Ch_2_Loc;
	Ch_1_Loc = Ch_1_Par_Val;
	Ch_2_Loc = Ch_1_Loc;
	if (Ch_2_Loc != Ch_2_Par_Val)
		return Ident_1;
	else {
		Ch_1_Glob = Ch_1_Loc;
		return Ident_2;
	}
}

Boolean Func_2(Str_30 Str_1_Par_Ref, Str_30 Str_2_Par_Ref)
{
	One_Thirty     Int_Loc;
	Capital_Letter Ch_Loc;
	Int_Loc = 2;
	Ch_Loc = 0;
	while (Int_Loc <= 2)
		if (Func_1(Str_1_Par_Ref[Int_Loc], Str_2_Par_Ref[Int_Loc + 1]) == Ident_1) {
			Ch_Loc = 'A';
			Int_Loc += 1;
		}
	if (Ch_Loc >= 'W' && Ch_Loc < 'Z')
		Int_Loc = 7;
	if (Ch_Loc == 'R')
		return true;
	else {
		if (str_cmp(Str_1_Par_Ref, Str_2_Par_Ref) > 0) {
			Int_Loc += 7;
			Int_Glob = Int_Loc;
			return true;
		}
		else
			return false;
	}
}

Boolean Func_3(Enumeration Enum_Par_Val)
{
	Enumeration Enum_Loc;
	Enum_Loc = Enum_Par_Val;
	if (Enum_Loc == Ident_3)
		return true;
	else
		return false;
}

void Proc_7(One_Fifty Int_1_Par_Val, One_Fifty Int_2_Par_Val, One_Fifty *Int_Par_Ref)
{
	One_Fifty Int_Loc;
	Int_Loc = Int_1_Par_Val + 2;
	*Int_Par_Ref = Int_2_Par_Val + Int_Loc;
}

void Proc_6(Enumeration Enum_Val_Par, Enumeration *Enum_Ref_Par)
{
	*Enum_Ref_Par = Enum_Val_Par;
	if (!Func_3(Enum_Val_Par))
		*Enum_Ref_Par = Ident_4;
	switch (Enum_Val_Par) {
	case Ident_1:
		*Enum_Ref_Par = Ident_1;
		break;
	case Ident_2:
		if (Int_Glob > 100)
			*Enum_Ref_Par = Ident_1;
		else
			*Enum_Ref_Par = Ident_4;
		break;
	case Ident_3:
		*Enum_Ref_Par = Ident_2;
		break;
	case Ident_4:
		break;
	case Ident_5:
		*Enum_Ref_Par = Ident_3;
		break;
	}
}

void Proc_3(Rec_Pointer *Ptr_Ref_Par)
{
	if (Ptr_Glob != 0)
		*Ptr_Ref_Par = Ptr_Glob->Ptr_Comp;
	Proc_7(10, Int_Glob, &Ptr_Glob->variant.var_1.Int_Comp);
}

void Proc_1(Rec_Pointer Ptr_Val_Par)
{
	Rec_Pointer Next_Record = Ptr_Val_Par->Ptr_Comp;
	*Ptr_Val_Par->Ptr_Comp = *Ptr_Glob;
	Ptr_Val_Par->variant.var_1.Int_Comp = 5;
	Next_Record->variant.var_1.Int_Comp = Ptr_Val_Par->variant.var_1.Int_Comp;
	Next_Record->Ptr_Comp = Ptr_Val_Par->Ptr_Comp;
	Proc_3(&Next_Record->Ptr_Comp);
	if (Next_Record->Discr == Ident_1) {
		Next_Record->variant.var_1.Int_Comp = 6;
		Proc_6(Ptr_Val_Par->variant.var_1.Enum_Comp, &Next_Record->variant.var_1.Enum_Comp);
		Next_Record->Ptr_Comp = Ptr_Glob->Ptr_Comp;
		Proc_7(Next_Record->variant.var_1.Int_Comp, 10, &Next_Record->variant.var_1.Int_Comp);
	}
	else
		*Ptr_Val_Par = *Ptr_Val_Par->Ptr_Comp;
}

void Proc_2(One_Fifty *Int_Par_Ref)
{
	One_Fifty   Int_Loc;
	Enumeration Enum_Loc;
	Int_Loc = *Int_Par_Ref + 10;
	Enum_Loc = Ident_2;
	do
		if (Ch_1_Glob == 'A') {
			Int_Loc -= 1;
			*Int_Par_Ref = Int_Loc - Int_Glob;
			Enum_Loc = Ident_1;
		}
	while (Enum_Loc != Ident_1);
}

void Proc_4(void)
{
	Boolean Bool_Loc;
	Bool_Loc = Ch_1_Glob == 'A';
	Bool_Glob = Bool_Loc | Bool_Glob;
	Ch_2_Glob = 'B';
}

void Proc_5(void)
{
	Ch_1_Glob = 'A';
	Bool_Glob = false;
}

void Proc_8(Arr_1_Dim Arr_1_Par_Ref, Arr_2_Dim Arr_2_Par_Ref, int Int_1_Par_Val, int Int_2_Par_Val)
{
	One_Fifty Int_Index, Int_Loc;
	Int_Loc = Int_1_Par_Val + 5;
	Arr_1_Par_Ref[Int_Loc] = Int_2_Par_Val;
	Arr_1_Par_Ref[Int_Loc + 1] = Arr_1_Par_Ref[Int_Loc];
	Arr_1_Par_Ref[Int_Loc + 30] = Int_Loc;
	for (Int_Index = Int_Loc; Int_Index <= Int_Loc + 1; ++Int_Index)
		Arr_2_Par_Ref[Int_Loc][Int_Index] = Int_Loc;
	Arr_2_Par_Ref[Int_Loc][Int_Loc - 1] += 1;
	Arr_2_Par_Ref[Int_Loc + 20][Int_Loc] = Arr_1_Par_Ref[Int_Loc];
	Int_Glob = 5;
}

int main(void)
{
	One_Fifty   Int_1_Loc;
	One_Fifty   Int_2_Loc;
	One_Fifty   Int_3_Loc;
	char        Ch_Index;
	Enumeration Enum_Loc;
	Str_30      Str_1_Loc;
	Str_30      Str_2_Loc;
	int         Run_Index;

	Next_Ptr_Glob = &Rec_1;
	Ptr_Glob = &Rec_2;
	Ptr_Glob->Ptr_Comp = Next_Ptr_Glob;
	Ptr_Glob->Discr = Ident_1;
	Ptr_Glob->variant.var_1.Enum_Comp = Ident_3;
	Ptr_Glob->variant.var_1.Int_Comp = 40;
	str_copy(Ptr_Glob->variant.var_1.Str_Comp, "DHRYSTONE PROGRAM, SOME STRING");
	str_copy(Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING");
	Arr_2_Glob[8][7] = 10;

	stamp();
	for (Run_Index = 1; Run_Index <= NUMBER_OF_RUNS; ++Run_Index) {
		Proc_5();
		Proc_4();
		Int_1_Loc = 2;
		Int_2_Loc = 3;
		str_copy(Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING");
		Enum_Loc = Ident_2;
		Bool_Glob = !Func_2(Str_1_Loc, Str_2_Loc);
		while (Int_1_Loc < Int_2_Loc) {
			Int_3_Loc = 5 * Int_1_Loc - Int_2_Loc;
			Proc_7(Int_1_Loc, Int_2_Loc, &Int_3_Loc);
			Int_1_Loc += 1;
		}
		Proc_8(Arr_1_Glob, Arr_2_Glob, Int_1_Loc, Int_3_Loc);
		Proc_1(Ptr_Glob);
		for (Ch_Index = 'A'; Ch_Index <= Ch_2_Glob; ++Ch_Index) {
			if (Enum_Loc == Func_1(Ch_Index, 'C')) {
				Proc_6(Ident_1, &Enum_Loc);
				str_copy(Str_2_Loc, "DHRYSTONE PROGRAM, 3'RD STRING");
				Int_2_Loc = Run_Index;
				Int_Glob = Run_Index;
			}
		}
		Int_2_Loc = Int_2_Loc * Int_1_Loc;
		Int_1_Loc = Int_2_Loc / Int_3_Loc;
		Int_2_Loc = 7 * (Int_2_Loc - Int_3_Loc) - Int_1_Loc;
		Proc_2(&Int_1_Loc);
	}
	stamp();

	/* the values Dhrystone 2.1 says must come out */
	if (Int_Glob != 5) return 1;
	if (Bool_Glob != 1) return 2;
	if (Ch_1_Glob != 'A') return 3;
	if (Ch_2_Glob != 'B') return 4;
	if (Arr_1_Glob[8] != 7) return 5;
	if (Arr_2_Glob[8][7] != NUMBER_OF_RUNS + 10) return 6;
	if (Ptr_Glob->Discr != 0) return 7;
	if (Ptr_Glob->variant.var_1.Enum_Comp != 2) return 8;
	if (Ptr_Glob->variant.var_1.Int_Comp != 17) return 9;
	if (str_cmp(Ptr_Glob->variant.var_1.Str_Comp, "DHRYSTONE PROGRAM, SOME STRING")) return 10;
	if (Next_Ptr_Glob->Ptr_Comp != Ptr_Glob->Ptr_Comp) return 11;
	if (Next_Ptr_Glob->Discr != 0) return 12;
	if (Next_Ptr_Glob->variant.var_1.Enum_Comp != 1) return 13;
	if (Next_Ptr_Glob->variant.var_1.Int_Comp != 18) return 14;
	if (str_cmp(Next_Ptr_Glob->variant.var_1.Str_Comp, "DHRYSTONE PROGRAM, SOME STRING")) return 15;
	if (Int_1_Loc != 5) return 16;
	if (Int_2_Loc != 13) return 17;
	if (Int_3_Loc != 7) return 18;
	if (Enum_Loc != 1) return 19;
	if (str_cmp(Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING")) return 20;
	if (str_cmp(Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING")) return 21;
	return 0;
}
