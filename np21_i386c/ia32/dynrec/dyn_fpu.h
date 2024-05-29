/*
 *  Copyright (C) 2002-2021  The DOSBox Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/* NTS: When generating code, do NOT use &TOP (address of TOP) because TOP is
 *      a 3-bit bitfield within the FPU status word. &TOP in reality resolves
 *      to the address of the FPU status word! Instead, use &FPUSW (address of
 *      the status word), shift right 11 bits, add whatever offset you need,
 *      and AND by 7 to produce the correct index. If you use &TOP directly
 *      you are in reality calling src/fpu.cpp code with the entire FPU status
 *      word as the FPU register index which can cause memory corruption and
 *      unexpected things. */

#include "dosbox.h"
#if C_FPU

#include <math.h>
#include <float.h>
#include "cross.h"
#include "mem.h"
#include "fpu.h"
#include "cpu.h"


static void FPU_FDECSTP(){
	TOP = (TOP - 1) & 7;
}

static void FPU_FINCSTP(){
	TOP = (TOP + 1) & 7;
}

static void FPU_FNSTCW(PhysPt addr){
	mem_writew(addr, CPU_STATSAVE.fpu_regs.control);
}

static void FPU_FFREE(Bitu st) {
	CPU_STATSAVE.fpu_regs.tag |= ((TAG_Empty) << (st * 2));
}


#if C_FPU_X86
#include "../../fpu/fpu_instructions_x86.h"
#elif defined(HAS_LONG_DOUBLE)
#include "../../fpu/fpu_instructions_longdouble.h"
#else
//#include "../../fpu/fpu_instructions.h"
//#include "../instructions/fpu/fpemul_softfloat.cpp"
#include "../instructions/fpu/fpemul_softfloat.h"

#include <float.h>
#include <math.h>
#include "../ia32.mcr"

#include "../instructions/fpu/fp.h"
#include "../instructions/fpu/fpumem.h"
#ifdef USE_SSE
#include "../instructions/sse/sse.h"
#endif

#if 1
#undef	TRACEOUT
#define	TRACEOUT(s)	(void)(s)
#endif	/* 0 */

#define FPU_WORKCLOCK	6

#define PI		3.14159265358979323846
#define L2E		1.4426950408889634
#define L2T		3.3219280948873623
#define LN2		0.69314718055994531
#define LG2		0.3010299956639812

static void FPU_FINIT(void);

static void
fpu_check_NM_EXCEPTION() {
	// タスクスイッチまたはエミュレーション時にNM(デバイス使用不可例外)を発生させる
	if ((CPU_CR0 & (CPU_CR0_TS)) || (CPU_CR0 & CPU_CR0_EM)) {
		EXCEPTION(NM_EXCEPTION, 0);
	}
}
static void
fpu_check_NM_EXCEPTION2() {
	// タスクスイッチまたはエミュレーション時にNM(デバイス使用不可例外)を発生させる
	if ((CPU_CR0 & (CPU_CR0_TS)) || (CPU_CR0 & CPU_CR0_EM)) {
		EXCEPTION(NM_EXCEPTION, 0);
	}
}

static const FPU_PTR zero_ptr = { 0, 0, 0 };

/*
 * FPU interface
 */

static INLINE UINT FPU_GET_TOP(void) {
	return (FPU_STATUSWORD & 0x3800) >> 11;
}

static INLINE void FPU_SET_TOP(UINT val) {
	FPU_STATUSWORD &= ~0x3800;
	FPU_STATUSWORD |= (val & 7) << 11;
}


static INLINE void FPU_SET_C0(UINT C) {
	FPU_STATUSWORD &= ~0x0100;
	if (C) FPU_STATUSWORD |= 0x0100;
}

static INLINE void FPU_SET_C1(UINT C) {
	FPU_STATUSWORD &= ~0x0200;
	if (C) FPU_STATUSWORD |= 0x0200;
}

static INLINE void FPU_SET_C2(UINT C) {
	FPU_STATUSWORD &= ~0x0400;
	if (C) FPU_STATUSWORD |= 0x0400;
}

static INLINE void FPU_SET_C3(UINT C) {
	FPU_STATUSWORD &= ~0x4000;
	if (C) FPU_STATUSWORD |= 0x4000;
}

static INLINE void FPU_SET_D(UINT C) {
	FPU_STATUSWORD &= ~0x0002;
	if (C) FPU_STATUSWORD |= 0x0002;
}

static INLINE void FPU_SetCW(UINT16 cword)
{
	// HACK: Bits 13-15 are not defined. Apparently, one program likes to test for
	// Cyrix EMC87 by trying to set bit 15. We want the test program to see
	// us as an Intel 287 when cputype == 286.
	cword &= 0x7FFF;
	FPU_CTRLWORD = cword;
	FPU_STAT.round = (FP_RND)((cword >> 10) & 3);
	switch (FPU_STAT.round) {
	case ROUND_Nearest:
		float_rounding_mode = float_round_nearest_even;
		break;
	case ROUND_Down:
		float_rounding_mode = float_round_down;
		break;
	case ROUND_Up:
		float_rounding_mode = float_round_up;
		break;
	case ROUND_Chop:
		float_rounding_mode = float_round_to_zero;
		break;
	default:
		break;
	}
}

static void FPU_FLDCW(UINT32 addr)
{
	UINT16 temp = cpu_vmemoryread_w(CPU_INST_SEGREG_INDEX, addr);
	FPU_SetCW(temp);
}

static UINT16 FPU_GetTag(void)
{
	UINT i;

	UINT16 tag = 0;
	for (i = 0; i < 8; i++)
		tag |= ((FPU_STAT.tag[i] & 3) << (2 * i));
	return tag;
}
static UINT8 FPU_GetTag8(void)
{
	UINT i;

	UINT8 tag = 0;
	for (i = 0; i < 8; i++)
		tag |= ((FPU_STAT.tag[i] == TAG_Empty ? 0 : 1) << (i));
	return tag;
}

static INLINE void FPU_SetTag(UINT16 tag)
{
	UINT i;

	for (i = 0; i < 8; i++) {
		FPU_STAT.tag[i] = (FP_TAG)((tag >> (2 * i)) & 3);

	}
}
static INLINE void FPU_SetTag8(UINT8 tag)
{
	UINT i;

	for (i = 0; i < 8; i++) {
		FPU_STAT.tag[i] = (((tag >> i) & 1) == 0 ? TAG_Empty : TAG_Valid);
	}
}

static void FPU_FCLEX(void) {
	//FPU_STATUSWORD &= 0xff00;			//should clear exceptions
	FPU_STATUSWORD &= 0x7f00;			//should clear exceptions?
}

static void FPU_FNOP(void) {
	return;
}

static void FPU_PUSH(floatx80 in) {
	FPU_STAT_TOP = (FPU_STAT_TOP - 1) & 7;
	//actually check if empty
	FPU_STAT.tag[FPU_STAT_TOP] = TAG_Valid;
	FPU_STAT.reg[FPU_STAT_TOP].d = in;
	//	LOG(LOG_FPU,LOG_ERROR)("Pushed at %d  %g to the stack",newtop,in);
	return;
}

static void FPU_PREP_PUSH(void) {
	FPU_SET_C1(FPU_STAT_TOP == 0 ? 1 : 0);
	FPU_STAT_TOP = (FPU_STAT_TOP - 1) & 7;
	FPU_STAT.tag[FPU_STAT_TOP] = TAG_Valid;
}

static void FPU_FPOP(void) {
	FPU_STAT.tag[FPU_STAT_TOP] = TAG_Empty;
	FPU_STAT.mmxenable = 0;
	//maybe set zero in it as well
	FPU_STAT_TOP = ((FPU_STAT_TOP + 1) & 7);
	//	LOG(LOG_FPU,LOG_ERROR)("popped from %d  %g off the stack",top,fpu.regs[top].d);
	return;
}

static floatx80 FROUND(floatx80 in) {
	return floatx80_round_to_int(in);
}

#define BIAS80 16383
#define BIAS64 1023

static void FPU_FLD80(UINT32 addr, UINT reg)
{
	FPU_STAT.reg[reg].ul.lower = fpu_memoryread_d(addr);
	FPU_STAT.reg[reg].ul.upper = fpu_memoryread_d(addr + 4);
	FPU_STAT.reg[reg].ul.ext = fpu_memoryread_w(addr + 8);
}

static void FPU_ST80(UINT32 addr, UINT reg)
{
	fpu_memorywrite_d(addr, FPU_STAT.reg[reg].ul.lower);
	fpu_memorywrite_d(addr + 4, FPU_STAT.reg[reg].ul.upper);
	fpu_memorywrite_w(addr + 8, FPU_STAT.reg[reg].ul.ext);
}


static void FPU_FLD_F32(UINT32 addr, UINT store_to) {
	union {
		float f;
		UINT32 l;
	}	blah;

	blah.l = fpu_memoryread_d(addr);
	FPU_STAT.reg[store_to].d = c_float_to_floatx80(blah.f);
}

static void FPU_FLD_F64(UINT32 addr, UINT store_to) {
	union {
		double d;
		UINT64 l;
	}	blah;
	blah.l = fpu_memoryread_q(addr);
	FPU_STAT.reg[store_to].d = c_double_to_floatx80(blah.d);
}

static void FPU_FLD_F80(UINT32 addr) {
	FPU_FLD80(addr, FPU_STAT_TOP);
}

static void FPU_FLD_I16(UINT32 addr, UINT store_to) {
	SINT16 blah;

	blah = fpu_memoryread_w(addr);
	FPU_STAT.reg[store_to].d = int32_to_floatx80((SINT32)blah);
}

static void FPU_FLD_I32(UINT32 addr, UINT store_to) {
	SINT32 blah;

	blah = fpu_memoryread_d(addr);
	FPU_STAT.reg[store_to].d = int32_to_floatx80(blah);
}

static void FPU_FLD_I64(UINT32 addr, UINT store_to) {
	SINT64 blah;

	blah = fpu_memoryread_q(addr);
	FPU_STAT.reg[store_to].d = int64_to_floatx80(blah);
}

static void FPU_FBLD(UINT32 addr, UINT store_to)
{
	UINT i;
	floatx80 temp;

	UINT64 val = 0;
	UINT in = 0;
	UINT64 base = 1;
	for (i = 0; i < 9; i++) {
		in = fpu_memoryread_b(addr + i);
		val += ((in & 0xf) * base); //in&0xf shouldn't be higher then 9
		base *= 10;
		val += (((in >> 4) & 0xf) * base);
		base *= 10;
	}

	//last number, only now convert to float in order to get
	//the best signification
	temp = int64_to_floatx80(val);
	in = fpu_memoryread_b(addr + 9);
	temp = floatx80_add(temp, int64_to_floatx80((in & 0xf) * base));
	if (in & 0x80) temp = floatx80_mul(temp, int32_to_floatx80(-1));
	FPU_STAT.reg[store_to].d = temp;
}


static INLINE void FPU_FLD_F32_EA(UINT32 addr) {
	FPU_FLD_F32(addr, 8);
}
static INLINE void FPU_FLD_F64_EA(UINT32 addr) {
	FPU_FLD_F64(addr, 8);
}
static INLINE void FPU_FLD_I32_EA(UINT32 addr) {
	FPU_FLD_I32(addr, 8);
}
static INLINE void FPU_FLD_I16_EA(UINT32 addr) {
	FPU_FLD_I16(addr, 8);
}

static void FPU_FST_F32(UINT32 addr) {
	union {
		float f;
		UINT32 l;
	}	blah;

	blah.f = floatx80_to_c_float(FPU_STAT.reg[FPU_STAT_TOP].d);
	fpu_memorywrite_d(addr, blah.l);
}

static void FPU_FST_F64(UINT32 addr) {
	union {
		double d;
		UINT64 l;
	}	blah;

	blah.d = floatx80_to_c_double(FPU_STAT.reg[FPU_STAT_TOP].d);
	fpu_memorywrite_q(addr, blah.l);
}

static void FPU_FST_F80(UINT32 addr) {
	FPU_ST80(addr, FPU_STAT_TOP);
}

static void FPU_FST_I16(UINT32 addr) {
	INT16 blah;

	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	blah = (SINT16)floatx80_to_int32(FPU_STAT.reg[FPU_STAT_TOP].d);
	fpu_memorywrite_w(addr, (UINT16)blah);
	FPU_STATUSWORD |= float_exception_flags;
}

static void FPU_FST_I32(UINT32 addr) {
	INT32 blah;

	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	blah = floatx80_to_int32(FPU_STAT.reg[FPU_STAT_TOP].d);
	fpu_memorywrite_d(addr, blah);
	FPU_STATUSWORD |= float_exception_flags;
}

static void FPU_FST_I64(UINT32 addr) {
	INT64 blah;

	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	blah = floatx80_to_int64(FPU_STAT.reg[FPU_STAT_TOP].d);
	fpu_memorywrite_q(addr, blah);
	FPU_STATUSWORD |= float_exception_flags;
}

static void FPU_FBST(UINT32 addr)
{
	FP_REG val;
	UINT32 p;
	UINT i;
	BOOL sign;
	floatx80 temp;
	floatx80 m1 = int32_to_floatx80(-1);
	floatx80 p10 = int32_to_floatx80(10);
	signed char oldrnd = float_rounding_mode;
	float_rounding_mode = float_round_down;

	val = FPU_STAT.reg[FPU_STAT_TOP];
	sign = FALSE;
	if (FPU_STAT.reg[FPU_STAT_TOP].l.ext & 0x8000) { //sign
		sign = TRUE;
		val.d = floatx80_mul(val.d, m1);
	}
	//numbers from back to front
	temp = val.d;
	for (i = 0; i < 9; i++) {
		val.d = temp;
		temp = floatx80_round_to_int(floatx80_div(val.d, p10));
		p = floatx80_to_int32_round_to_zero(floatx80_sub(val.d, floatx80_mul(temp, p10)));
		val.d = temp;
		temp = floatx80_round_to_int(floatx80_div(val.d, p10));
		p |= (floatx80_to_int32_round_to_zero(floatx80_sub(val.d, floatx80_mul(temp, p10))) << 4);

		fpu_memorywrite_b(addr + i, (UINT8)p);
	}
	val.d = temp;
	temp = floatx80_round_to_int(floatx80_div(val.d, p10));
	p = floatx80_to_int32_round_to_zero(floatx80_sub(val.d, floatx80_mul(temp, p10)));
	if (sign)
		p |= 0x80;
	fpu_memorywrite_b(addr + 9, (UINT8)p);

	float_rounding_mode = oldrnd;
	FPU_STATUSWORD |= float_exception_flags;
}

#define isinf(x) (!(_finite(x) || _isnan(x)))
#define isdenormal(x) (_fpclass(x) == _FPCLASS_ND || _fpclass(x) == _FPCLASS_PD)

static void FPU_FADD(UINT op1, UINT op2) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[op1].d = floatx80_add(FPU_STAT.reg[op1].d, FPU_STAT.reg[op2].d);
	FPU_STATUSWORD |= float_exception_flags;
	return;
}

static void FPU_FSIN(void) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(sin(floatx80_to_c_double(FPU_STAT.reg[FPU_STAT_TOP].d)));
	FPU_SET_C2(0);
	//flags and such :)
	FPU_STATUSWORD |= float_exception_flags;
	return;
}

static void FPU_FSINCOS(void) {
	double temp;

	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	temp = floatx80_to_c_double(FPU_STAT.reg[FPU_STAT_TOP].d);
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(sin(temp));
	FPU_PUSH(c_double_to_floatx80(cos(temp)));
	FPU_SET_C2(0);
	//flags and such :)
	FPU_STATUSWORD |= float_exception_flags;
	return;
}

static void FPU_FCOS(void) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(cos(floatx80_to_c_double(FPU_STAT.reg[FPU_STAT_TOP].d)));
	FPU_SET_C2(0);
	//flags and such :)
	FPU_STATUSWORD |= float_exception_flags;
	return;
}

static void FPU_FSQRT(void) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[FPU_STAT_TOP].d = floatx80_sqrt(FPU_STAT.reg[FPU_STAT_TOP].d);
	//flags and such :)
	FPU_STATUSWORD |= float_exception_flags;
	return;
}
static void FPU_FPATAN(void) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[FPU_ST(1)].d = c_double_to_floatx80(atan2(floatx80_to_c_double(FPU_STAT.reg[FPU_ST(1)].d), floatx80_to_c_double(FPU_STAT.reg[FPU_STAT_TOP].d)));
	FPU_FPOP();
	//flags and such :)
	FPU_STATUSWORD |= float_exception_flags;
	return;
}
static void FPU_FPTAN(void) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(tan(floatx80_to_c_double(FPU_STAT.reg[FPU_STAT_TOP].d)));
	FPU_PUSH(c_double_to_floatx80(1.0));
	FPU_SET_C2(0);
	//flags and such :)
	FPU_STATUSWORD |= float_exception_flags;
	return;
}
static void FPU_FDIV(UINT st, UINT other) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	//if(floatx80_eq(FPU_STAT.reg[other].d, c_double_to_floatx80(0.0))){
	//	FPU_STATUSWORD |= FP_ZE_FLAG;
	//	if(!(FPU_CTRLWORD & FP_ZE_FLAG))
	//		return;
	//}
	FPU_STAT.reg[st].d = floatx80_div(FPU_STAT.reg[st].d, FPU_STAT.reg[other].d);
	//flags and such :)
	FPU_STATUSWORD |= float_exception_flags;
	return;
}

static void FPU_FDIVR(UINT st, UINT other) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	//if(floatx80_eq(FPU_STAT.reg[st].d, c_double_to_floatx80(0.0))){
	//	FPU_STATUSWORD |= FP_ZE_FLAG;
	//	if(!(FPU_CTRLWORD & FP_ZE_FLAG))
	//		return;
	//}
	FPU_STAT.reg[st].d = floatx80_div(FPU_STAT.reg[other].d, FPU_STAT.reg[st].d);
	// flags and such :)
	FPU_STATUSWORD |= float_exception_flags;
	return;
}

static void FPU_FMUL(UINT st, UINT other) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[st].d = floatx80_mul(FPU_STAT.reg[st].d, FPU_STAT.reg[other].d);
	//flags and such :)
	FPU_STATUSWORD |= float_exception_flags;
	return;
}

static void FPU_FSUB(UINT st, UINT other) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[st].d = floatx80_sub(FPU_STAT.reg[st].d, FPU_STAT.reg[other].d);
	//flags and such :)
	return;
}

static void FPU_FSUBR(UINT st, UINT other) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[st].d = floatx80_sub(FPU_STAT.reg[other].d, FPU_STAT.reg[st].d);
	//flags and such :)
	FPU_STATUSWORD |= float_exception_flags;
	return;
}

static void FPU_FXCH(UINT st, UINT other) {
	FP_TAG tag;
	FP_REG reg;

	tag = FPU_STAT.tag[other];
	reg = FPU_STAT.reg[other];
	FPU_STAT.tag[other] = FPU_STAT.tag[st];
	FPU_STAT.reg[other] = FPU_STAT.reg[st];
	FPU_STAT.tag[st] = tag;
	FPU_STAT.reg[st] = reg;
}

static void FPU_FST(UINT st, UINT other) {
	FPU_STAT.tag[other] = FPU_STAT.tag[st];
	FPU_STAT.reg[other] = FPU_STAT.reg[st];
}

static void FPU_FCOM(UINT st, UINT other) {
	if (((FPU_STAT.tag[st] != TAG_Valid) && (FPU_STAT.tag[st] != TAG_Zero)) ||
		((FPU_STAT.tag[other] != TAG_Valid) && (FPU_STAT.tag[other] != TAG_Zero)) ||
		(floatx80_is_nan(FPU_STAT.reg[st].d) || floatx80_is_nan(FPU_STAT.reg[other].d))) {
		FPU_SET_C3(1);
		FPU_SET_C2(1);
		FPU_SET_C0(1);
		return;
	}

	if (floatx80_eq(FPU_STAT.reg[st].d, FPU_STAT.reg[other].d)) {
		FPU_SET_C3(1);
		FPU_SET_C2(0);
		FPU_SET_C0(0);
		return;
	}
	if (floatx80_lt(FPU_STAT.reg[st].d, FPU_STAT.reg[other].d)) {
		FPU_SET_C3(0);
		FPU_SET_C2(0);
		FPU_SET_C0(1);
		return;
	}
	// st > other
	FPU_SET_C3(0);
	FPU_SET_C2(0);
	FPU_SET_C0(0);
	return;
}
static void FPU_FCOMI(UINT st, UINT other) {
	if (((FPU_STAT.tag[st] != TAG_Valid) && (FPU_STAT.tag[st] != TAG_Zero)) ||
		((FPU_STAT.tag[other] != TAG_Valid) && (FPU_STAT.tag[other] != TAG_Zero)) ||
		(floatx80_is_nan(FPU_STAT.reg[st].d) || floatx80_is_nan(FPU_STAT.reg[other].d))) {
		CPU_FLAGL = (CPU_FLAGL & ~Z_FLAG) | Z_FLAG;
		CPU_FLAGL = (CPU_FLAGL & ~P_FLAG) | P_FLAG;
		CPU_FLAGL = (CPU_FLAGL & ~C_FLAG) | C_FLAG;
		return;
	}

	if (floatx80_eq(FPU_STAT.reg[st].d, FPU_STAT.reg[other].d)) {
		CPU_FLAGL = (CPU_FLAGL & ~Z_FLAG) | Z_FLAG;
		CPU_FLAGL = (CPU_FLAGL & ~P_FLAG) | 0;
		CPU_FLAGL = (CPU_FLAGL & ~C_FLAG) | 0;
		return;
	}
	if (floatx80_lt(FPU_STAT.reg[st].d, FPU_STAT.reg[other].d)) {
		CPU_FLAGL = (CPU_FLAGL & ~Z_FLAG) | 0;
		CPU_FLAGL = (CPU_FLAGL & ~P_FLAG) | 0;
		CPU_FLAGL = (CPU_FLAGL & ~C_FLAG) | C_FLAG;
		return;
	}
	// st > other
	CPU_FLAGL = (CPU_FLAGL & ~Z_FLAG) | 0;
	CPU_FLAGL = (CPU_FLAGL & ~P_FLAG) | 0;
	CPU_FLAGL = (CPU_FLAGL & ~C_FLAG) | 0;
	return;
}

static void FPU_FUCOM(UINT st, UINT other) {
	//does atm the same as fcom 
	FPU_FCOM(st, other);
}
static void FPU_FUCOMI(UINT st, UINT other) {
	//does atm the same as fcomi 
	FPU_FCOMI(st, other);
}

static void FPU_FCMOVB(UINT st, UINT other) {
	if (CPU_FLAGL & C_FLAG) {
		FPU_STAT.tag[st] = FPU_STAT.tag[other];
		FPU_STAT.reg[st] = FPU_STAT.reg[other];
	}
}
static void FPU_FCMOVE(UINT st, UINT other) {
	if (CPU_FLAGL & Z_FLAG) {
		FPU_STAT.tag[st] = FPU_STAT.tag[other];
		FPU_STAT.reg[st] = FPU_STAT.reg[other];
	}
}
static void FPU_FCMOVBE(UINT st, UINT other) {
	if (CPU_FLAGL & (C_FLAG | Z_FLAG)) {
		FPU_STAT.tag[st] = FPU_STAT.tag[other];
		FPU_STAT.reg[st] = FPU_STAT.reg[other];
	}
}
static void FPU_FCMOVU(UINT st, UINT other) {
	if (CPU_FLAGL & P_FLAG) {
		FPU_STAT.tag[st] = FPU_STAT.tag[other];
		FPU_STAT.reg[st] = FPU_STAT.reg[other];
	}
}

static void FPU_FCMOVNB(UINT st, UINT other) {
	if (!(CPU_FLAGL & C_FLAG)) {
		FPU_STAT.tag[st] = FPU_STAT.tag[other];
		FPU_STAT.reg[st] = FPU_STAT.reg[other];
	}
}
static void FPU_FCMOVNE(UINT st, UINT other) {
	if (!(CPU_FLAGL & Z_FLAG)) {
		FPU_STAT.tag[st] = FPU_STAT.tag[other];
		FPU_STAT.reg[st] = FPU_STAT.reg[other];
	}
}
static void FPU_FCMOVNBE(UINT st, UINT other) {
	if (!(CPU_FLAGL & (C_FLAG | Z_FLAG))) {
		FPU_STAT.tag[st] = FPU_STAT.tag[other];
		FPU_STAT.reg[st] = FPU_STAT.reg[other];
	}
}
static void FPU_FCMOVNU(UINT st, UINT other) {
	if (!(CPU_FLAGL & P_FLAG)) {
		FPU_STAT.tag[st] = FPU_STAT.tag[other];
		FPU_STAT.reg[st] = FPU_STAT.reg[other];
	}
}

static void FPU_FRNDINT(void) {

	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[FPU_STAT_TOP].d = floatx80_round_to_int(FPU_STAT.reg[FPU_STAT_TOP].d);
	FPU_STATUSWORD |= float_exception_flags;
}

static void FPU_FPREM(void) {
	floatx80 valtop;
	floatx80 valdiv;
	SINT64 ressaved;

	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	valtop = FPU_STAT.reg[FPU_STAT_TOP].d;
	valdiv = FPU_STAT.reg[FPU_ST(1)].d;
	ressaved = floatx80_to_int64_round_to_zero(floatx80_div(valtop, valdiv));
	// Some backups
	//	Real64 res=valtop - ressaved*valdiv; 
	//      res= fmod(valtop,valdiv);
	FPU_STAT.reg[FPU_STAT_TOP].d = floatx80_sub(valtop, floatx80_mul(int64_to_floatx80(ressaved), valdiv));
	FPU_SET_C0((UINT)(ressaved & 4));
	FPU_SET_C3((UINT)(ressaved & 2));
	FPU_SET_C1((UINT)(ressaved & 1));
	FPU_SET_C2(0);
	FPU_STATUSWORD |= float_exception_flags;
}

static void FPU_FPREM1(void) {
	floatx80 valtop;
	floatx80 valdiv, quot, quotf, quot_sub_quotf;
	SINT64 ressaved;
	floatx80 v05 = c_double_to_floatx80(0.5);
	signed char oldrnd = float_rounding_mode;

	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	valtop = FPU_STAT.reg[FPU_STAT_TOP].d;
	valdiv = FPU_STAT.reg[FPU_ST(1)].d;
	quot = floatx80_div(valtop, valdiv);
	float_rounding_mode = float_round_down;
	quotf = floatx80_round_to_int(quot);

	quot_sub_quotf = floatx80_sub(quot, quotf);
	if (floatx80_lt(v05, quot_sub_quotf)) ressaved = floatx80_to_int64_round_to_zero(quotf) + 1;
	else if (floatx80_lt(quot_sub_quotf, v05)) ressaved = floatx80_to_int64_round_to_zero(quotf);
	else ressaved = ((((floatx80_to_int64_round_to_zero(quotf)) & 1) != 0) ? floatx80_to_int64_round_to_zero(quotf) + 1 : floatx80_to_int64_round_to_zero(quotf));

	FPU_STAT.reg[FPU_STAT_TOP].d = floatx80_sub(valtop, floatx80_mul(int64_to_floatx80(ressaved), valdiv));
	FPU_SET_C0((UINT)(ressaved & 4));
	FPU_SET_C3((UINT)(ressaved & 2));
	FPU_SET_C1((UINT)(ressaved & 1));
	FPU_SET_C2(0);

	float_rounding_mode = oldrnd;
	FPU_STATUSWORD |= float_exception_flags;
}

static void FPU_FXAM(void) {
	if (FPU_STAT.reg[FPU_STAT_TOP].d.high & 0x8000)	//sign
	{
		FPU_SET_C1(1);
	}
	else
	{
		FPU_SET_C1(0);
	}
	if (FPU_STAT.tag[FPU_STAT_TOP] == TAG_Empty)
	{
		FPU_SET_C3(1); FPU_SET_C2(0); FPU_SET_C0(1);
		return;
	}
	if (floatx80_is_nan(FPU_STAT.reg[FPU_STAT_TOP].d))
	{
		FPU_SET_C3(0); FPU_SET_C2(0); FPU_SET_C0(1);
		return;
	}
	if (floatx80_is_inf(FPU_STAT.reg[FPU_STAT_TOP].d))
	{
		FPU_SET_C3(0); FPU_SET_C2(1); FPU_SET_C0(1);
		return;
	}
	if (floatx80_eq(FPU_STAT.reg[FPU_STAT_TOP].d, c_double_to_floatx80(0.0)))		//zero or normalized number.
	{
		FPU_SET_C3(1); FPU_SET_C2(0); FPU_SET_C0(0);
	}
	else
	{
		FPU_SET_C3(0); FPU_SET_C2(1); FPU_SET_C0(0);
	}
}

static void FPU_F2XM1(void) {
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(pow(2.0, floatx80_to_c_double(FPU_STAT.reg[FPU_STAT_TOP].d)) - 1);
	return;
}

static void FPU_FYL2X(void) {
	FPU_STAT.reg[FPU_ST(1)].d = floatx80_mul(FPU_STAT.reg[FPU_ST(1)].d, c_double_to_floatx80(log(floatx80_to_c_double(FPU_STAT.reg[FPU_STAT_TOP].d)) / log((double)(2.0))));
	FPU_FPOP();
	return;
}

static void FPU_FYL2XP1(void) {
	FPU_STAT.reg[FPU_ST(1)].d = floatx80_mul(FPU_STAT.reg[FPU_ST(1)].d, c_double_to_floatx80(log(floatx80_to_c_double(FPU_STAT.reg[FPU_STAT_TOP].d) + 1.0) / log((double)(2.0))));
	FPU_FPOP();
	return;
}

static void FPU_FSCALE(void) {
	FPU_STAT.reg[FPU_STAT_TOP].d = floatx80_mul(FPU_STAT.reg[FPU_STAT_TOP].d, c_double_to_floatx80(pow(2.0, floatx80_to_c_double(FPU_STAT.reg[FPU_ST(1)].d))));
	return; //2^x where x is chopped.
}

static void FPU_FSTENV(UINT32 addr)
{
	//	descriptor_t *sdp = &CPU_CS_DESC;	
	FPU_SET_TOP(FPU_STAT_TOP);

	//	switch ((CPU_CR0 & 1) | (SEG_IS_32BIT(sdp) ? 0x100 : 0x000))
	switch ((CPU_CR0 & 1) | (CPU_INST_OP32 ? 0x100 : 0x000))
	{
	case 0x000: case 0x001:
		fpu_memorywrite_w(addr + 0, FPU_CTRLWORD);
		fpu_memorywrite_w(addr + 2, FPU_STATUSWORD);
		fpu_memorywrite_w(addr + 4, FPU_GetTag());
		fpu_memorywrite_w(addr + 10, FPU_LASTINSTOP);
		break;

	case 0x100: case 0x101:
		fpu_memorywrite_d(addr + 0, (UINT32)(FPU_CTRLWORD));
		fpu_memorywrite_d(addr + 4, (UINT32)(FPU_STATUSWORD));
		fpu_memorywrite_d(addr + 8, (UINT32)(FPU_GetTag()));
		fpu_memorywrite_d(addr + 20, FPU_LASTINSTOP);
		break;
	}
}

static void FPU_FLDENV(UINT32 addr)
{
	//	descriptor_t *sdp = &CPU_CS_DESC;	

	//	switch ((CPU_CR0 & 1) | (SEG_IS_32BIT(sdp) ? 0x100 : 0x000)) {
	switch ((CPU_CR0 & 1) | (CPU_INST_OP32 ? 0x100 : 0x000)) {
	case 0x000: case 0x001:
		FPU_SetCW(fpu_memoryread_w(addr + 0));
		FPU_STATUSWORD = fpu_memoryread_w(addr + 2);
		FPU_SetTag(fpu_memoryread_w(addr + 4));
		FPU_LASTINSTOP = fpu_memoryread_w(addr + 10);
		break;

	case 0x100: case 0x101:
		FPU_SetCW((UINT16)fpu_memoryread_d(addr + 0));
		FPU_STATUSWORD = (UINT16)fpu_memoryread_d(addr + 4);
		FPU_SetTag((UINT16)fpu_memoryread_d(addr + 8));
		FPU_LASTINSTOP = (UINT16)fpu_memoryread_d(addr + 20);
		break;
	}
	FPU_STAT_TOP = FPU_GET_TOP();
}

static void FPU_FSAVE(UINT32 addr)
{
	UINT start;
	UINT i;

	//	descriptor_t *sdp = &CPU_CS_DESC;

	FPU_FSTENV(addr);
	//	start = ((SEG_IS_32BIT(sdp))?28:14);
	start = ((CPU_INST_OP32) ? 28 : 14);
	for (i = 0; i < 8; i++) {
		FPU_ST80(addr + start, FPU_ST(i));
		start += 10;
	}
	FPU_FINIT();
}

static void FPU_FRSTOR(UINT32 addr)
{
	UINT start;
	UINT i;

	//	descriptor_t *sdp = &CPU_CS_DESC;

	FPU_FLDENV(addr);
	//	start = ((SEG_IS_32BIT(sdp))?28:14);
	start = ((CPU_INST_OP32) ? 28 : 14);
	for (i = 0; i < 8; i++) {
		FPU_FLD80(addr + start, FPU_ST(i));
		start += 10;
	}
}

static void FPU_FXSAVE(UINT32 addr) {
	UINT start;
	UINT i;

	//	descriptor_t *sdp = &CPU_CS_DESC;

		//FPU_FSTENV(addr);
	FPU_SET_TOP(FPU_STAT_TOP);
	fpu_memorywrite_w(addr + 0, FPU_CTRLWORD);
	fpu_memorywrite_w(addr + 2, FPU_STATUSWORD);
	fpu_memorywrite_b(addr + 4, FPU_GetTag8());
#ifdef USE_SSE
	fpu_memorywrite_d(addr + 24, SSE_MXCSR);
#endif
	start = 32;
	for (i = 0; i < 8; i++) {
		FPU_ST80(addr + start, FPU_ST(i));
		start += 16;
	}
#ifdef USE_SSE
	start = 160;
	for (i = 0; i < 8; i++) {
		fpu_memorywrite_q(addr + start + 0, SSE_XMMREG(i).ul64[0]);
		fpu_memorywrite_q(addr + start + 8, SSE_XMMREG(i).ul64[1]);
		start += 16;
	}
#endif
}
static void FPU_FXRSTOR(UINT32 addr) {
	UINT start;
	UINT i;

	//	descriptor_t *sdp = &CPU_CS_DESC;

		//FPU_FLDENV(addr);
	FPU_SetCW(fpu_memoryread_w(addr + 0));
	FPU_STATUSWORD = fpu_memoryread_w(addr + 2);
	FPU_SetTag8(fpu_memoryread_b(addr + 4));
	FPU_STAT_TOP = FPU_GET_TOP();
#ifdef USE_SSE
	SSE_MXCSR = fpu_memoryread_d(addr + 24);
#endif
	start = 32;
	for (i = 0; i < 8; i++) {
		FPU_FLD80(addr + start, FPU_ST(i));
		start += 16;
	}
#ifdef USE_SSE
	start = 160;
	for (i = 0; i < 8; i++) {
		SSE_XMMREG(i).ul64[0] = fpu_memoryread_q(addr + start + 0);
		SSE_XMMREG(i).ul64[1] = fpu_memoryread_q(addr + start + 8);
		start += 16;
	}
#endif
}

static void SF_FPU_FXSAVERSTOR(void) {
	UINT32 op;
	UINT idx, sub;
	UINT32 maddr;

	CPU_WORKCLOCK(FPU_WORKCLOCK);
	GET_PCBYTE((op));
	idx = (op >> 3) & 7;
	sub = (op & 7);

	fpu_check_NM_EXCEPTION2(); // XXX: 根拠無し
	switch (idx) {
	case 0: // FXSAVE
		maddr = calc_ea_dst(op);
		FPU_FXSAVE(maddr);
		break;
	case 1: // FXRSTOR
		maddr = calc_ea_dst(op);
		FPU_FXRSTOR(maddr);
		break;
#ifdef USE_SSE
	case 2: // LDMXCSR
		maddr = calc_ea_dst(op);
		SSE_LDMXCSR(maddr);
		break;
	case 3: // STMXCSR
		maddr = calc_ea_dst(op);
		SSE_STMXCSR(maddr);
		break;
	case 4: // SFENCE
		SSE_SFENCE();
		break;
	case 5: // LFENCE
		SSE_LFENCE();
		break;
	case 6: // MFENCE
		SSE_MFENCE();
		break;
	case 7: // CLFLUSH;
		SSE_CLFLUSH(op);
		break;
#endif
	default:
		ia32_panic("invalid opcode = %02x\n", op);
		break;
	}
}

static void FPU_FXTRACT(void) {
	// function stores real bias in st and 
	// pushes the significant number onto the stack
	// if double ever uses a different base please correct this function
	FP_REG test;
	SINT64 exp80, exp80final;
	floatx80 mant;

	test = FPU_STAT.reg[FPU_STAT_TOP];
	exp80 = test.ll & QWORD_CONST(0x7ff0000000000000);
	exp80final = (exp80 >> 52) - BIAS64;
	mant = floatx80_div(test.d, c_double_to_floatx80(pow(2.0, (double)(exp80final))));
	FPU_STAT.reg[FPU_STAT_TOP].d = int64_to_floatx80(exp80final);
	FPU_PUSH(mant);
}

static void FPU_FCHS(void) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	FPU_STAT.reg[FPU_STAT_TOP].d = floatx80_mul(c_double_to_floatx80(-1.0), FPU_STAT.reg[FPU_STAT_TOP].d);
	FPU_STATUSWORD |= float_exception_flags;
}

static void FPU_FABS(void) {
	float_exception_flags = (FPU_STATUSWORD & 0x3f);
	if (floatx80_le(FPU_STAT.reg[FPU_STAT_TOP].d, c_double_to_floatx80(0.0))) {
		FPU_STAT.reg[FPU_STAT_TOP].d = floatx80_mul(c_double_to_floatx80(-1.0), FPU_STAT.reg[FPU_STAT_TOP].d);
	}
	FPU_STATUSWORD |= float_exception_flags;
}

static void FPU_FTST(void) {
	FPU_STAT.reg[8].d = c_double_to_floatx80(0.0);
	FPU_FCOM(FPU_STAT_TOP, 8);
}

static void FPU_FLD1(void) {
	FPU_PREP_PUSH();
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(1.0);
}

static void FPU_FLDL2T(void) {
	FPU_PREP_PUSH();
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(L2T);
}

static void FPU_FLDL2E(void) {
	FPU_PREP_PUSH();
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(L2E);
}

static void FPU_FLDPI(void) {
	FPU_PREP_PUSH();
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(PI);
}

static void FPU_FLDLG2(void) {
	FPU_PREP_PUSH();
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(LG2);
}

static void FPU_FLDLN2(void) {
	FPU_PREP_PUSH();
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(LN2);
}

static void FPU_FLDZ(void) {
	FPU_PREP_PUSH();
	FPU_STAT.reg[FPU_STAT_TOP].d = c_double_to_floatx80(0.0);
	FPU_STAT.tag[FPU_STAT_TOP] = TAG_Zero;
	FPU_STAT.mmxenable = 0;
}


static INLINE void FPU_FADD_EA(UINT op1) {
	FPU_FADD(op1, 8);
}
static INLINE void FPU_FMUL_EA(UINT op1) {
	FPU_FMUL(op1, 8);
}
static INLINE void FPU_FSUB_EA(UINT op1) {
	FPU_FSUB(op1, 8);
}
static INLINE void FPU_FSUBR_EA(UINT op1) {
	FPU_FSUBR(op1, 8);
}
static INLINE void FPU_FDIV_EA(UINT op1) {
	FPU_FDIV(op1, 8);
}
static INLINE void FPU_FDIVR_EA(UINT op1) {
	FPU_FDIVR(op1, 8);
}
static INLINE void FPU_FCOM_EA(UINT op1) {
	FPU_FCOM(op1, 8);
}

/*
 * FPU interface
 */
 //int fpu_updateEmuEnv(void);
static void
FPU_FINIT(void)
{
	int i;
	FPU_SetCW(0x37F);
	FPU_STATUSWORD = 0;
	FPU_STAT_TOP = FPU_GET_TOP();
	for (i = 0; i < 8; i++) {
		FPU_STAT.tag[i] = TAG_Empty;
		// レジスタの内容は消してはいけない ver0.86 rev40
		//FPU_STAT.reg[i].d = 0;
		//FPU_STAT.reg[i].l.lower = 0;
		//FPU_STAT.reg[i].l.upper = 0;
		//FPU_STAT.reg[i].ll = 0;
	}
	FPU_STAT.tag[8] = TAG_Valid; // is only used by us
	FPU_STAT.mmxenable = 0;
}

#endif


static INLINE void dyn_fpu_top() {
	gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
	gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
	gen_add_imm(FC_OP2,decode.modrm.rm);
	gen_and_imm(FC_OP2,7);
	gen_mov_word_to_reg(FC_OP1,(void*)(&FPUSW),true);
	gen_shr_imm(FC_OP1,11); /* stack top is 3-bit value starting at bit 11 */
	gen_and_imm(FC_OP1,7);
}

static INLINE void dyn_fpu_top_swapped() {
	gen_mov_word_to_reg(FC_OP1,(void*)(&FPUSW),true);
	gen_shr_imm(FC_OP1,11); /* stack top is 3-bit value starting at bit 11 */
	gen_add_imm(FC_OP1,decode.modrm.rm);
	gen_and_imm(FC_OP1,7);
	gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
	gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
	gen_and_imm(FC_OP2,7);
}

static void dyn_eatree() {
//	Bitu group = (decode.modrm.val >> 3) & 7;
	Bitu group = decode.modrm.reg&7; //It is already that, but compilers.
	switch (group){
	case 0x00:		// FADD ST,STi
		gen_call_function_R(FPU_FADD_EA,FC_OP1);
		break;
	case 0x01:		// FMUL  ST,STi
		gen_call_function_R(FPU_FMUL_EA,FC_OP1);
		break;
	case 0x02:		// FCOM  STi
		gen_call_function_R(FPU_FCOM_EA,FC_OP1);
		break;
	case 0x03:		// FCOMP STi
		gen_call_function_R(FPU_FCOM_EA,FC_OP1);
		gen_call_function_raw(FPU_FPOP);
		break;
	case 0x04:		// FSUB  ST,STi
		gen_call_function_R(FPU_FSUB_EA,FC_OP1);
		break;	
	case 0x05:		// FSUBR ST,STi
		gen_call_function_R(FPU_FSUBR_EA,FC_OP1);
		break;
	case 0x06:		// FDIV  ST,STi
		gen_call_function_R(FPU_FDIV_EA,FC_OP1);
		break;
	case 0x07:		// FDIVR ST,STi
		gen_call_function_R(FPU_FDIVR_EA,FC_OP1);
		break;
	default:
		break;
	}
}

static void dyn_fpu_esc0(){
	dyn_get_modrm(); 
//	if (decode.modrm.val >= 0xc0) {
	if (decode.modrm.mod == 3) { 
		dyn_fpu_top();
		switch (decode.modrm.reg){
		case 0x00:		//FADD ST,STi
			gen_call_function_RR(FPU_FADD,FC_OP1,FC_OP2);
			break;
		case 0x01:		// FMUL  ST,STi
			gen_call_function_RR(FPU_FMUL,FC_OP1,FC_OP2);
			break;
		case 0x02:		// FCOM  STi
			gen_call_function_RR(FPU_FCOM,FC_OP1,FC_OP2);
			break;
		case 0x03:		// FCOMP STi
			gen_call_function_RR(FPU_FCOM,FC_OP1,FC_OP2);
			gen_call_function_raw(FPU_FPOP);
			break;
		case 0x04:		// FSUB  ST,STi
			gen_call_function_RR(FPU_FSUB,FC_OP1,FC_OP2);
			break;	
		case 0x05:		// FSUBR ST,STi
			gen_call_function_RR(FPU_FSUBR,FC_OP1,FC_OP2);
			break;
		case 0x06:		// FDIV  ST,STi
			gen_call_function_RR(FPU_FDIV,FC_OP1,FC_OP2);
			break;
		case 0x07:		// FDIVR ST,STi
			gen_call_function_RR(FPU_FDIVR,FC_OP1,FC_OP2);
			break;
		default:
			break;
		}
	} else { 
		dyn_fill_ea(FC_ADDR);
		gen_call_function_R(FPU_FLD_F32_EA,FC_ADDR); 
		gen_mov_word_to_reg(FC_OP1,(void*)(&FPUSW),true);
		gen_shr_imm(FC_OP1,11); /* stack top is 3-bit value starting at bit 11 */
		gen_and_imm(FC_OP1,7);
		dyn_eatree();
	}
}


static void dyn_fpu_esc1(){
	dyn_get_modrm();  
//	if (decode.modrm.val >= 0xc0) { 
	if (decode.modrm.mod == 3) {
		switch (decode.modrm.reg){
		case 0x00: /* FLD STi */
			gen_mov_word_to_reg(FC_OP1,(void*)(&FPUSW),true);
			gen_shr_imm(FC_OP1,11); /* stack top is 3-bit value starting at bit 11 */
			gen_add_imm(FC_OP1,decode.modrm.rm);
			gen_and_imm(FC_OP1,7);
			gen_protect_reg(FC_OP1);
			gen_call_function_raw(FPU_PREP_PUSH); 
			gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
			gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
			gen_and_imm(FC_OP2,7);
			gen_restore_reg(FC_OP1);
			gen_call_function_RR(FPU_FST,FC_OP1,FC_OP2);
			break;
		case 0x01: /* FXCH STi */
			dyn_fpu_top();
			gen_call_function_RR(FPU_FXCH,FC_OP1,FC_OP2);
			break;
		case 0x02: /* FNOP */
			gen_call_function_raw(FPU_FNOP);
			break;
		case 0x03: /* FSTP STi */
			dyn_fpu_top();
			gen_call_function_RR(FPU_FST,FC_OP1,FC_OP2);
			gen_call_function_raw(FPU_FPOP);
			break;   
		case 0x04:
			switch(decode.modrm.rm){
			case 0x00:       /* FCHS */
				gen_call_function_raw(FPU_FCHS);
				break;
			case 0x01:       /* FABS */
				gen_call_function_raw(FPU_FABS);
				break;
			case 0x02:       /* UNKNOWN */
			case 0x03:       /* ILLEGAL */
				LOG(LOG_FPU,LOG_WARN)("ESC 1:Unhandled group %X subfunction %X",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
				break;
			case 0x04:       /* FTST */
				gen_call_function_raw(FPU_FTST);
				break;
			case 0x05:       /* FXAM */
				gen_call_function_raw(FPU_FXAM);
				break;
			case 0x06:       /* FTSTP (cyrix)*/
			case 0x07:       /* UNKNOWN */
				LOG(LOG_FPU,LOG_WARN)("ESC 1:Unhandled group %X subfunction %X",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
				break;
			}
			break;
		case 0x05:
			switch(decode.modrm.rm){	
			case 0x00:       /* FLD1 */
				gen_call_function_raw(FPU_FLD1);
				break;
			case 0x01:       /* FLDL2T */
				gen_call_function_raw(FPU_FLDL2T);
				break;
			case 0x02:       /* FLDL2E */
				gen_call_function_raw(FPU_FLDL2E);
				break;
			case 0x03:       /* FLDPI */
				gen_call_function_raw(FPU_FLDPI);
				break;
			case 0x04:       /* FLDLG2 */
				gen_call_function_raw(FPU_FLDLG2);
				break;
			case 0x05:       /* FLDLN2 */
				gen_call_function_raw(FPU_FLDLN2);
				break;
			case 0x06:       /* FLDZ*/
				gen_call_function_raw(FPU_FLDZ);
				break;
			case 0x07:       /* ILLEGAL */
				LOG(LOG_FPU,LOG_WARN)("ESC 1:Unhandled group %X subfunction %X",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
				break;
			}
			break;
		case 0x06:
			switch(decode.modrm.rm){
			case 0x00:	/* F2XM1 */
				gen_call_function_raw(FPU_F2XM1);
				break;
			case 0x01:	/* FYL2X */
				gen_call_function_raw(FPU_FYL2X);
				break;
			case 0x02:	/* FPTAN  */
				gen_call_function_raw(FPU_FPTAN);
				break;
			case 0x03:	/* FPATAN */
				gen_call_function_raw(FPU_FPATAN);
				break;
			case 0x04:	/* FXTRACT */
				gen_call_function_raw(FPU_FXTRACT);
				break;
			case 0x05:	/* FPREM1 */
				gen_call_function_raw(FPU_FPREM1);
				break;
			case 0x06:	/* FDECSTP */
				gen_call_function_raw(FPU_FDECSTP);
				break;
			case 0x07:	/* FINCSTP */
				gen_call_function_raw(FPU_FINCSTP);
				break;
			default:
				LOG(LOG_FPU,LOG_WARN)("ESC 1:Unhandled group %X subfunction %X",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
				break;
			}
			break;
		case 0x07:
			switch(decode.modrm.rm){
			case 0x00:		/* FPREM */
				gen_call_function_raw(FPU_FPREM);
				break;
			case 0x01:		/* FYL2XP1 */
				gen_call_function_raw(FPU_FYL2XP1);
				break;
			case 0x02:		/* FSQRT */
				gen_call_function_raw(FPU_FSQRT);
				break;
			case 0x03:		/* FSINCOS */
				gen_call_function_raw(FPU_FSINCOS);
				break;
			case 0x04:		/* FRNDINT */
				gen_call_function_raw(FPU_FRNDINT);
				break;
			case 0x05:		/* FSCALE */
				gen_call_function_raw(FPU_FSCALE);
				break;
			case 0x06:		/* FSIN */
				gen_call_function_raw(FPU_FSIN);
				break;
			case 0x07:		/* FCOS */
				gen_call_function_raw(FPU_FCOS);
				break;
			default:
				LOG(LOG_FPU,LOG_WARN)("ESC 1:Unhandled group %X subfunction %X",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
				break;
			}
			break;
		default:
			LOG(LOG_FPU,LOG_WARN)("ESC 1:Unhandled group %X subfunction %X",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
		}
	} else {
		switch(decode.modrm.reg){
		case 0x00: /* FLD float*/
			gen_call_function_raw(FPU_PREP_PUSH);
			dyn_fill_ea(FC_OP1);
			gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
			gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
			gen_and_imm(FC_OP2,7);
			gen_call_function_RR(FPU_FLD_F32,FC_OP1,FC_OP2);
			break;
		case 0x01: /* UNKNOWN */
			LOG(LOG_FPU,LOG_WARN)("ESC EA 1:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
		case 0x02: /* FST float*/
			dyn_fill_ea(FC_ADDR);
			gen_call_function_R(FPU_FST_F32,FC_ADDR);
			break;
		case 0x03: /* FSTP float*/
			dyn_fill_ea(FC_ADDR);
			gen_call_function_R(FPU_FST_F32,FC_ADDR);
			gen_call_function_raw(FPU_FPOP);
			break;
		case 0x04: /* FLDENV */
			dyn_fill_ea(FC_ADDR);
			gen_call_function_RI(FPU_FLDENV, FC_ADDR, !decode.big_op);
			break;
		case 0x05: /* FLDCW */
			dyn_fill_ea(FC_ADDR);
			gen_call_function_R(FPU_FLDCW,FC_ADDR);
			break;
		case 0x06: /* FSTENV */
			dyn_fill_ea(FC_ADDR);
			gen_call_function_RI(FPU_FSTENV, FC_ADDR, !decode.big_op);
			break;
		case 0x07:  /* FNSTCW*/
			dyn_fill_ea(FC_ADDR);
			gen_call_function_R(FPU_FNSTCW,FC_ADDR);
			break;
		default:
			LOG(LOG_FPU,LOG_WARN)("ESC EA 1:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
		}
	}
}

static void dyn_fpu_esc2(){
	dyn_get_modrm();  
//	if (decode.modrm.val >= 0xc0) { 
	if (decode.modrm.mod == 3) {
		switch(decode.modrm.reg){
		case 0x05:
			switch(decode.modrm.rm){
			case 0x01:		/* FUCOMPP */
				gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
				gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
				gen_add_imm(FC_OP2,1);
				gen_and_imm(FC_OP2,7);
				gen_mov_word_to_reg(FC_OP1,(void*)(&FPUSW),true);
				gen_shr_imm(FC_OP1,11); /* stack top is 3-bit value starting at bit 11 */
				gen_and_imm(FC_OP1,7);
				gen_call_function_RR(FPU_FUCOM,FC_OP1,FC_OP2);
				gen_call_function_raw(FPU_FPOP);
				gen_call_function_raw(FPU_FPOP);
				break;
			default:
				LOG(LOG_FPU,LOG_WARN)("ESC 2:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm); 
				break;
			}
			break;
		default:
	   		LOG(LOG_FPU,LOG_WARN)("ESC 2:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
		}
	} else {
		dyn_fill_ea(FC_ADDR);
		gen_call_function_R(FPU_FLD_I32_EA,FC_ADDR); 
		gen_mov_word_to_reg(FC_OP1,(void*)(&FPUSW),true);
		gen_shr_imm(FC_OP1,11); /* stack top is 3-bit value starting at bit 11 */
		gen_and_imm(FC_OP1,7);
		dyn_eatree();
	}
}

static void dyn_fpu_esc3(){
	dyn_get_modrm();  
//	if (decode.modrm.val >= 0xc0) { 
	if (decode.modrm.mod == 3) {
		switch (decode.modrm.reg) {
		case 0x04:
			switch (decode.modrm.rm) {
			case 0x00:				//FNENI
			case 0x01:				//FNDIS
				LOG(LOG_FPU,LOG_ERROR)("8087 only fpu code used esc 3: group 4: subfunction: %d",(int)decode.modrm.rm);
				break;
			case 0x02:				//FNCLEX FCLEX
				gen_call_function_raw(FPU_FCLEX);
				break;
			case 0x03:				//FNINIT FINIT
				gen_call_function_raw(FPU_FINIT);
				break;
			case 0x04:				//FNSETPM
			case 0x05:				//FRSTPM
//				LOG(LOG_FPU,LOG_ERROR)("80267 protected mode (un)set. Nothing done");
				break;
			default:
				E_Exit("ESC 3:ILLEGAL OPCODE group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			}
			break;
		default:
			LOG(LOG_FPU,LOG_WARN)("ESC 3:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
		}
	} else {
		switch(decode.modrm.reg){
		case 0x00:	/* FILD */
			gen_call_function_raw(FPU_PREP_PUSH);
			dyn_fill_ea(FC_OP1); 
			gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
			gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
			gen_and_imm(FC_OP2,7);
			gen_call_function_RR(FPU_FLD_I32,FC_OP1,FC_OP2);
			break;
		case 0x01:	/* FISTTP */
			LOG(LOG_FPU,LOG_WARN)("ESC 3 EA:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
		case 0x02:	/* FIST */
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_R(FPU_FST_I32,FC_ADDR);
			break;
		case 0x03:	/* FISTP */
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_R(FPU_FST_I32,FC_ADDR);
			gen_call_function_raw(FPU_FPOP);
			break;
		case 0x05:	/* FLD 80 Bits Real */
			gen_call_function_raw(FPU_PREP_PUSH);
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_R(FPU_FLD_F80,FC_ADDR);
			break;
		case 0x07:	/* FSTP 80 Bits Real */
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_R(FPU_FST_F80,FC_ADDR);
			gen_call_function_raw(FPU_FPOP);
			break;
		default:
			LOG(LOG_FPU,LOG_WARN)("ESC 3 EA:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
		}
	}
}

static void dyn_fpu_esc4(){
	dyn_get_modrm();  
//	if (decode.modrm.val >= 0xc0) { 
	if (decode.modrm.mod == 3) {
		switch(decode.modrm.reg){
		case 0x00:	/* FADD STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FADD,FC_OP1,FC_OP2);
			break;
		case 0x01:	/* FMUL STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FMUL,FC_OP1,FC_OP2);
			break;
		case 0x02:  /* FCOM*/
			dyn_fpu_top();
			gen_call_function_RR(FPU_FCOM,FC_OP1,FC_OP2);
			break;
		case 0x03:  /* FCOMP*/
			dyn_fpu_top();
			gen_call_function_RR(FPU_FCOM,FC_OP1,FC_OP2);
			gen_call_function_raw(FPU_FPOP);
			break;
		case 0x04:  /* FSUBR STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FSUBR,FC_OP1,FC_OP2);
			break;
		case 0x05:  /* FSUB  STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FSUB,FC_OP1,FC_OP2);
			break;
		case 0x06:  /* FDIVR STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FDIVR,FC_OP1,FC_OP2);
			break;
		case 0x07:  /* FDIV STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FDIV,FC_OP1,FC_OP2);
			break;
		default:
			break;
		}
	} else { 
		dyn_fill_ea(FC_ADDR);
		gen_call_function_R(FPU_FLD_F64_EA,FC_ADDR); 
		gen_mov_word_to_reg(FC_OP1,(void*)(&FPUSW),true);
		gen_shr_imm(FC_OP1,11); /* stack top is 3-bit value starting at bit 11 */
		gen_and_imm(FC_OP1,7);
		dyn_eatree();
	}
}

static void dyn_fpu_esc5(){
	dyn_get_modrm();  
//	if (decode.modrm.val >= 0xc0) { 
	if (decode.modrm.mod == 3) {
		dyn_fpu_top();
		switch(decode.modrm.reg){
		case 0x00: /* FFREE STi */
			gen_call_function_R(FPU_FFREE,FC_OP2);
			break;
		case 0x01: /* FXCH STi*/
			gen_call_function_RR(FPU_FXCH,FC_OP1,FC_OP2);
			break;
		case 0x02: /* FST STi */
			gen_call_function_RR(FPU_FST,FC_OP1,FC_OP2);
			break;
		case 0x03:  /* FSTP STi*/
			gen_call_function_RR(FPU_FST,FC_OP1,FC_OP2);
			gen_call_function_raw(FPU_FPOP);
			break;
		case 0x04:	/* FUCOM STi */
			gen_call_function_RR(FPU_FUCOM,FC_OP1,FC_OP2);
			break;
		case 0x05:	/*FUCOMP STi */
			gen_call_function_RR(FPU_FUCOM,FC_OP1,FC_OP2);
			gen_call_function_raw(FPU_FPOP);
			break;
		default:
			LOG(LOG_FPU,LOG_WARN)("ESC 5:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
        }
	} else {
		switch(decode.modrm.reg){
		case 0x00:  /* FLD double real*/
			gen_call_function_raw(FPU_PREP_PUSH);
			dyn_fill_ea(FC_OP1); 
			gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
			gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
			gen_and_imm(FC_OP2,7);
			gen_call_function_RR(FPU_FLD_F64,FC_OP1,FC_OP2);
			break;
		case 0x01:  /* FISTTP longint*/
			LOG(LOG_FPU,LOG_WARN)("ESC 5 EA:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
		case 0x02:   /* FST double real*/
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_R(FPU_FST_F64,FC_ADDR);
			break;
		case 0x03:	/* FSTP double real*/
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_R(FPU_FST_F64,FC_ADDR);
			gen_call_function_raw(FPU_FPOP);
			break;
		case 0x04:	/* FRSTOR */
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_RI(FPU_FRSTOR, FC_ADDR, !decode.big_op);
			break;
		case 0x06:	/* FSAVE */
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_RI(FPU_FSAVE, FC_ADDR, !decode.big_op);
			break;
		case 0x07:   /*FNSTSW */
			dyn_fill_ea(FC_OP1); 
			gen_mov_word_to_reg(FC_OP2,(void*)(&CPU_STATSAVE.fpu_regs.status),false);
			gen_call_function_RR(mem_writew,FC_OP1,FC_OP2);
			break;
		default:
			LOG(LOG_FPU,LOG_WARN)("ESC 5 EA:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
		}
	}
}

static void dyn_fpu_esc6(){
	dyn_get_modrm();  
//	if (decode.modrm.val >= 0xc0) { 
	if (decode.modrm.mod == 3) {
		switch(decode.modrm.reg){
		case 0x00:	/*FADDP STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FADD,FC_OP1,FC_OP2);
			break;
		case 0x01:	/* FMULP STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FMUL,FC_OP1,FC_OP2);
			break;
		case 0x02:  /* FCOMP5*/
			dyn_fpu_top();
			gen_call_function_RR(FPU_FCOM,FC_OP1,FC_OP2);
			break;	/* TODO IS THIS ALRIGHT ????????? */
		case 0x03:  /*FCOMPP*/
			if(decode.modrm.rm != 1) {
				LOG(LOG_FPU,LOG_WARN)("ESC 6:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
				return;
			}
			gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
			gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
			gen_add_imm(FC_OP2,1);
			gen_and_imm(FC_OP2,7);
			gen_mov_word_to_reg(FC_OP1,(void*)(&FPUSW),true);
			gen_shr_imm(FC_OP1,11); /* stack top is 3-bit value starting at bit 11 */
			gen_and_imm(FC_OP1,7);
			gen_call_function_RR(FPU_FCOM,FC_OP1,FC_OP2);
			gen_call_function_raw(FPU_FPOP); /* extra pop at the bottom*/
			break;
		case 0x04:  /* FSUBRP STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FSUBR,FC_OP1,FC_OP2);
			break;
		case 0x05:  /* FSUBP  STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FSUB,FC_OP1,FC_OP2);
			break;
		case 0x06:	/* FDIVRP STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FDIVR,FC_OP1,FC_OP2);
			break;
		case 0x07:  /* FDIVP STi,ST*/
			dyn_fpu_top_swapped();
			gen_call_function_RR(FPU_FDIV,FC_OP1,FC_OP2);
			break;
		default:
			break;
		}
		gen_call_function_raw(FPU_FPOP);		
	} else {
		dyn_fill_ea(FC_ADDR);
		gen_call_function_R(FPU_FLD_I16_EA,FC_ADDR); 
		gen_mov_word_to_reg(FC_OP1,(void*)(&FPUSW),true);
		gen_shr_imm(FC_OP1,11); /* stack top is 3-bit value starting at bit 11 */
		gen_and_imm(FC_OP1,7);
		dyn_eatree();
	}
}

static void dyn_fpu_esc7(){
	dyn_get_modrm();  
//	if (decode.modrm.val >= 0xc0) { 
	if (decode.modrm.mod == 3) {
		switch (decode.modrm.reg){
		case 0x00: /* FFREEP STi */
			dyn_fpu_top();
			gen_call_function_R(FPU_FFREE,FC_OP2);
			gen_call_function_raw(FPU_FPOP);
			break;
		case 0x01: /* FXCH STi*/
			dyn_fpu_top();
			gen_call_function_RR(FPU_FXCH,FC_OP1,FC_OP2);
			break;
		case 0x02:  /* FSTP STi*/
		case 0x03:  /* FSTP STi*/
			dyn_fpu_top();
			gen_call_function_RR(FPU_FST,FC_OP1,FC_OP2);
			gen_call_function_raw(FPU_FPOP);
			break;
		case 0x04:
			switch(decode.modrm.rm){
				case 0x00:     /* FNSTSW AX*/
					gen_mov_word_to_reg(FC_OP1,(void*)(&CPU_STATSAVE.fpu_regs.status),false);
					MOV_REG_WORD16_FROM_HOST_REG(FC_OP1,DRC_REG_EAX);
					break;
				default:
					LOG(LOG_FPU,LOG_WARN)("ESC 7:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
					break;
			}
			break;
		default:
			LOG(LOG_FPU,LOG_WARN)("ESC 7:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
		}
	} else {
		switch(decode.modrm.reg){
		case 0x00:  /* FILD int16_t */
			gen_call_function_raw(FPU_PREP_PUSH);
			dyn_fill_ea(FC_OP1); 
			gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
			gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
			gen_and_imm(FC_OP2,7);
			gen_call_function_RR(FPU_FLD_I16,FC_OP1,FC_OP2);
			break;
		case 0x01:
			LOG(LOG_FPU,LOG_WARN)("ESC 7 EA:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
		case 0x02:   /* FIST int16_t */
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_R(FPU_FST_I16,FC_ADDR);
			break;
		case 0x03:	/* FISTP int16_t */
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_R(FPU_FST_I16,FC_ADDR);
			gen_call_function_raw(FPU_FPOP);
			break;
		case 0x04:   /* FBLD packed BCD */
			gen_call_function_raw(FPU_PREP_PUSH);
			dyn_fill_ea(FC_OP1);
			gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
			gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
			gen_and_imm(FC_OP2,7);
			gen_call_function_RR(FPU_FBLD,FC_OP1,FC_OP2);
			break;
		case 0x05:  /* FILD int64_t */
			gen_call_function_raw(FPU_PREP_PUSH);
			dyn_fill_ea(FC_OP1);
			gen_mov_word_to_reg(FC_OP2,(void*)(&FPUSW),true);
			gen_shr_imm(FC_OP2,11); /* stack top is 3-bit value starting at bit 11 */
			gen_and_imm(FC_OP2,7);
			gen_call_function_RR(FPU_FLD_I64,FC_OP1,FC_OP2);
			break;
		case 0x06:	/* FBSTP packed BCD */
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_R(FPU_FBST,FC_ADDR);
			gen_call_function_raw(FPU_FPOP);
			break;
		case 0x07:  /* FISTP int64_t */
			dyn_fill_ea(FC_ADDR); 
			gen_call_function_R(FPU_FST_I64,FC_ADDR);
			gen_call_function_raw(FPU_FPOP);
			break;
		default:
			LOG(LOG_FPU,LOG_WARN)("ESC 7 EA:Unhandled group %d subfunction %d",(unsigned int)decode.modrm.reg,(unsigned int)decode.modrm.rm);
			break;
		}
	}
}

#endif
