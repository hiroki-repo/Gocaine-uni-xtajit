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

#ifndef DOSBOX_REGS_H
#define DOSBOX_REGS_H

#include "../cpu.h"

#include "mem.h"

#define FLAG_CF		0x00000001U
#define FLAG_PF		0x00000004U
#define FLAG_AF		0x00000010U
#define FLAG_ZF		0x00000040U
#define FLAG_SF		0x00000080U
#define FLAG_OF		0x00000800U

#define FLAG_TF		0x00000100U
#define FLAG_IF		0x00000200U
#define FLAG_DF		0x00000400U

#define FLAG_IOPL	0x00003000U
#define FLAG_NT		0x00004000U
#define FLAG_VM		0x00020000U
#define FLAG_AC		0x00040000U
#define FLAG_ID		0x00200000U

#define FMASK_TEST		(FLAG_CF | FLAG_PF | FLAG_AF | FLAG_ZF | FLAG_SF | FLAG_OF)
#define FMASK_NORMAL	(FMASK_TEST | FLAG_DF | FLAG_TF | FLAG_IF )	
#define FMASK_ALL		(FMASK_NORMAL | FLAG_IOPL | FLAG_NT)

#define SETFLAGBIT(TYPE,TEST) if (TEST) reg_flags|=FLAG_ ## TYPE; else reg_flags&=~FLAG_ ## TYPE

#define GETFLAG(TYPE) (reg_flags & FLAG_ ## TYPE)
#define GETFLAGBOOL(TYPE) ((reg_flags & FLAG_ ## TYPE) ? true : false )

#define GETFLAG_IOPL ((reg_flags & FLAG_IOPL) >> 12)

struct Segment {
	uint16_t val;
	PhysPt phys;							/* The physical address start in emulated machine */
	PhysPt limit;
};

enum SegNames { es=0,cs,ss,ds,fs,gs};

struct Segments {
	Bitu val[8];
	PhysPt phys[8];
	PhysPt limit[8];
	bool expanddown[8];
};

union GenReg32 {
	uint32_t dword[1];
	uint16_t word[2];
	uint8_t byte[4];
};

#ifdef WORDS_BIGENDIAN

#define DW_INDEX 0
#define W_INDEX 1
#define BH_INDEX 2
#define BL_INDEX 3

#else

#define DW_INDEX 0
#define W_INDEX 0
#define BH_INDEX 1
#define BL_INDEX 0

#endif

struct CPU_Regs {
	GenReg32 regs[8],ip;
	Bitu flags;
};

//extern Segments Segs;
//extern CPU_Regs cpu_regs;

#define Segs CPU_STATSAVE.cpu_regs.sreg
//#define cpu_regs CPU_STATSAVE.cpu_regs.reg

static INLINE PhysPt SegLimit(SegNames index) {
	return CPU_STATSAVE.cpu_stat.sreg[index].u.seg.limit;
}

static INLINE PhysPt SegPhys(SegNames index) {
	return CPU_STATSAVE.cpu_stat.sreg[index].u.seg.segbase;
}

static INLINE uint16_t SegValue(SegNames index) {
	return (uint16_t)CPU_STATSAVE.cpu_regs.sreg[index];
}
	
static INLINE RealPt RealMakeSeg(SegNames index,uint16_t off) {
	return RealMake(SegValue(index),off);	
}


static INLINE void SegSet16(Bitu index,uint16_t val) {
	CPU_STATSAVE.cpu_regs.sreg[index]=(Bitu)val;
	CPU_STATSAVE.cpu_stat.sreg[index].u.seg.segbase=(PhysPt)((unsigned int)val << 4U);
	/* real mode does not update limit */
}

enum {
	REGI_AX, REGI_CX, REGI_DX, REGI_BX,
	REGI_SP, REGI_BP, REGI_SI, REGI_DI
};

enum {
	REGI_AL, REGI_CL, REGI_DL, REGI_BL,
	REGI_AH, REGI_CH, REGI_DH, REGI_BH
};


//macros to convert a 3-bit register index to the correct register
#define reg_8l(reg) (CPU_REGS_BYTEL(reg))
#define reg_8h(reg) (CPU_REGS_BYTEH(reg))
#define reg_8(reg) ((reg & 4) ? reg_8h((reg) & 3) : reg_8l((reg) & 3))
#define reg_16(reg) (CPU_REGS_WORD(reg))
#define reg_32(reg) (CPU_REGS_DWORD(reg))

#define reg_al CPU_REGS_BYTEL(REGI_AX)
#define reg_ah CPU_REGS_BYTEH(REGI_AX)
#define reg_ax CPU_REGS_WORD(REGI_AX)
#define reg_eax CPU_REGS_DWORD(REGI_AX)

#define reg_bl CPU_REGS_BYTEL(REGI_BX)
#define reg_bh CPU_REGS_BYTEH(REGI_BX)
#define reg_bx CPU_REGS_WORD(REGI_BX)
#define reg_ebx CPU_REGS_DWORD(REGI_BX)

#define reg_cl CPU_REGS_BYTEL(REGI_CX)
#define reg_ch CPU_REGS_BYTEH(REGI_CX)
#define reg_cx CPU_REGS_WORD(REGI_CX)
#define reg_ecx CPU_REGS_DWORD(REGI_CX)

#define reg_dl CPU_REGS_BYTEL(REGI_DX)
#define reg_dh CPU_REGS_BYTEH(REGI_DX)
#define reg_dx CPU_REGS_WORD(REGI_DX)
#define reg_edx CPU_REGS_DWORD(REGI_DX)

#define reg_si CPU_REGS_WORD(REGI_SI)
#define reg_esi CPU_REGS_DWORD(REGI_SI)

#define reg_di CPU_REGS_WORD(REGI_DI)
#define reg_edi CPU_REGS_DWORD(REGI_DI)

#define reg_sp CPU_REGS_WORD(REGI_SP)
#define reg_esp CPU_REGS_DWORD(REGI_SP)

#define reg_bp CPU_REGS_WORD(REGI_BP)
#define reg_ebp CPU_REGS_DWORD(REGI_BP)

#define reg_ip CPU_STATSAVE.cpu_regs.eip.w
#define reg_eip CPU_STATSAVE.cpu_regs.eip.d

#define reg_flags CPU_STATSAVE.cpu_regs.eflags.d

#endif
