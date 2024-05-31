/*
 * Copyright (c) 2024 Gocaine Project
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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

  //#include "compiler.h"
 //#include "dosio.h"

#include "cpu.h"
#include "ia32.mcr"

#include "inst_table.h"

#if defined(ENABLE_TRAP)
#include "trap/steptrap.h"
#endif

#if defined(SUPPORT_ASYNC_CPU)
#include "timing.h"
#include "nevent.h"
#include "pccore.h"
#include	"iocore.h"
#include	"sound/sound.h"
#include	"sound/beep.h"
#include	"sound/fmboard.h"
#include	"sound/soundrom.h"
#include	"cbus/mpu98ii.h"
#if defined(SUPPORT_SMPU98)
#include	"cbus/smpu98.h"
#endif
#endif

#include "jit.h"

#include "ctrlxfer.h"

#include "dynrec/config.h"

#if (C_DYNREC)
#include <string.h>

#if defined (WIN32)
#include <windows.h>
#include <winbase.h>
#endif

#if (C_HAVE_MPROTECT)
#include <sys/mman.h>

#include <limits.h>
#ifndef PAGESIZE
#define PAGESIZE 4096
#endif
#endif /* C_HAVE_MPROTECT */

#include "dynrec/callback.h"
#include "dynrec/regs.h"
#include "dynrec/mem.h"
#include "dynrec/cpu.h"
#include "dynrec/debug.h"
#include "dynrec/paging.h"
#include "dynrec/inout.h"
#include "dynrec/lazyflags.h"
#include "dynrec/pic.h"

#define CACHE_MAXSIZE	(4096*2)
#define CACHE_TOTAL		(1024*1024*8)
#define CACHE_PAGES		(512)
#define CACHE_BLOCKS	(128*1024)
#define CACHE_ALIGN		(16)
#define DYN_HASH_SHIFT	(4)
#define DYN_PAGE_HASH	(4096>>DYN_HASH_SHIFT)
#define DYN_LINKS		(16)

// the emulated x86 registers
#define DRC_REG_EAX 0
#define DRC_REG_ECX 1
#define DRC_REG_EDX 2
#define DRC_REG_EBX 3
#define DRC_REG_ESP 4
#define DRC_REG_EBP 5
#define DRC_REG_ESI 6
#define DRC_REG_EDI 7

// the emulated x86 segment registers
#define DRC_SEG_ES 0
#define DRC_SEG_CS 1
#define DRC_SEG_SS 2
#define DRC_SEG_DS 3
#define DRC_SEG_FS 4
#define DRC_SEG_GS 5


// access to a general register
#define DRCD_REG_VAL(_reg) (&CPU_STATSAVE.cpu_regs.reg[_reg].d)
// access to the flags register
#define DRCD_REG_FLAGS (&CPU_STATSAVE.cpu_regs.eflags.d)
// access to a segment register
#define DRCD_SEG_VAL(_seg) (&CPU_STATSAVE.cpu_regs.sreg[_seg])
// access to the physical value of a segment register/selector
#define DRCD_SEG_PHYS(_seg) (&CPU_STATSAVE.cpu_stat.sreg[_seg].u.seg.segbase)

// access to an 8bit general register
#define DRCD_REG_BYTE(_reg,idx) (idx?((void*)(&CPU_STATSAVE.cpu_regs.reg[_reg].b.h)):((void*)(&CPU_STATSAVE.cpu_regs.reg[_reg].b.l)))
// access to  16/32bit general registers
#define DRCD_REG_WORD(_reg,dwrd) ((dwrd)?((void*)(&CPU_STATSAVE.cpu_regs.reg[_reg].d)):((void*)(&CPU_STATSAVE.cpu_regs.reg[_reg].w)))


enum BlockReturnDynRec {
	BR_Normal = 0,
	BR_Cycles,
	BR_Link1, BR_Link2,
	BR_Opcode,
#if (C_DEBUG)
	BR_OpcodeFull,
#endif
	BR_Iret,
	BR_CallBack,
	BR_SMCBlock,
	BR_Trap
};

// identificator to signal self-modification of the currently executed block
#define SMC_CURRENT_BLOCK	0xffff


static void IllegalOptionDynrec(const char* msg) {
	//E_Exit("DynrecCore: illegal option in %s", msg);
}

static struct {
	BlockReturnDynRec(*runcode)(uint8_t*);		// points to code that can start a block
	Bitu callback;				// the occurred callback
	Bitu readdata;				// spare space used when reading from memory
	uint32_t protected_regs[8];	// space to save/restore register values
} core_dynrec;


#include "dynrec/cache.h"

#define X86			0x01
#define X86_64		0x02
#define MIPSEL		0x03
#define ARMV4LE		0x04
#define ARMV7LE		0x05
#define ARMV8LE		0x07

#if !defined(C_TARGETCPU)
# if defined(_MSC_VER) && defined(_M_AMD64)
#  define C_TARGETCPU X86_64
# elif defined(_MSC_VER) && defined(_M_ARM)
#  define C_TARGETCPU ARMV7LE
# elif defined(_MSC_VER) && defined(_M_ARM64)
#  define C_TARGETCPU ARMV8LE
# endif
#endif

#if C_TARGETCPU == X86_64
#include "dynrec/risc_x64.h"
#elif C_TARGETCPU == X86
#include "dynrec/risc_x86.h"
#elif C_TARGETCPU == MIPSEL
#include "dynrec/risc_mipsel32.h"
#elif (C_TARGETCPU == ARMV4LE) || (C_TARGETCPU == ARMV7LE)
#ifdef _MSC_VER
#pragma message("warning: Using ARMV4 or ARMV7")
#else
#warning Using ARMV4 or ARMV7
#endif
#include "dynrec/risc_armv4le.h"
#elif C_TARGETCPU == ARMV8LE
#include "dynrec/risc_armv8le.h"
#endif

#include "dynrec/decoder.h"

/* Dynarec on Windows ARM32 works, but only on Windows 10.
 * Windows RT 8.x only runs ARMv7 Thumb-2 code, which we don't have a backend for
 * Attempting to use dynarec on RT will result in an instant crash.
 * So we need to detect the Windows version before enabling dynarec. */
#if defined(WIN32) && defined(_M_ARM)
bool IsWin10OrGreater() {
	HMODULE handle = GetModuleHandleW(L"ntdll.dll");
	if (handle) {
		typedef LONG(WINAPI* RtlGetVersionPtr)(RTL_OSVERSIONINFOW*);
		RtlGetVersionPtr getVersionPtr = (RtlGetVersionPtr)GetProcAddress(handle, "RtlGetVersion");
		if (getVersionPtr != NULL) {
			RTL_OSVERSIONINFOW info = { 0 };
			info.dwOSVersionInfoSize = sizeof(info);
			if (getVersionPtr(&info) == 0) { /* STATUS_SUCCESS == 0 */
				if (info.dwMajorVersion >= 10)
					return true;
			}
		}
	}
	return false;
}

static bool is_win10 = false;
static bool winrt_warning = true;
#endif


CacheBlockDynRec* LinkBlocks(BlockReturnDynRec ret) {
	CacheBlockDynRec* block = NULL;
	// the last instruction was a control flow modifying instruction
	uint32_t temp_ip = SegPhys(cs) + reg_eip;
	CodePageHandlerDynRec* temp_handler = (CodePageHandlerDynRec*)get_tlb_readhandler(temp_ip);
	if (temp_handler->flags & PFLAG_HASCODE) {
		// see if the target is an already translated block
		block = temp_handler->FindCacheBlock(temp_ip & 4095);
		if (!block) return NULL;

		// found it, link the current block to
		cache.block.running->LinkTo(ret == BR_Link2, block);
		return block;
	}
	return NULL;
}

/*
	The core tries to find the block that should be executed next.
	If such a block is found, it is run, otherwise the instruction
	stream starting at ip_point is translated (see decoder.h) and
	makes up a new code block that will be run.
	When control is returned to CPU_Core_Dynrec_Run (which might
	be right after the block is run, or somewhen long after that
	due to the direct cacheblock linking) the returncode decides
	the next action. This might be continuing the translation and
	execution process, or returning from the core etc.
*/

extern bool use_dynamic_core_with_paging;
extern int dynamic_core_cache_block_size;

static bool paging_warning = true;

//syntax : 2byte|prefix|callgate|opcodetogetimm|invalidop|priviledged|imm16/32bytest|imm8bytest|7|6|5|4|3|2|1|0|unused7|unused6|unused5|unused4|unused3|unused2|unused1|unused0|sse+(NO AS MMX)|imm16|offset16/32|offset8|imm16/32|imm8|modrm16/32|modrm8
UINT32 opcodeoperantsizedesc[4][256] = {
	{ // prim op
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000004,
		0x00000008,
		0x00000000,
		0x00000000,// 0x07
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000004,
		0x00000008,
		0x00000000,
		0x80000000,// 0x0f
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000004,
		0x00000008,
		0x00000000,
		0x00000000,// 0x17
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000004,
		0x00000008,
		0x00000000,
		0x00000000,// 0x1f
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000004,
		0x00000008,
		0x40000000,
		0x00000000,// 0x27
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000004,
		0x00000008,
		0x40000000,
		0x00000000,// 0x2f
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000004,
		0x00000008,
		0x40000000,
		0x00000000,// 0x37
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000004,
		0x00000008,
		0x40000000,
		0x00000000,// 0x3f
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,// 0x47
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,// 0x4f
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,// 0x57
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,// 0x5f
		0x00000000,
		0x00000000,
		0x00000002,
		0x00000002,
		0x40000000,
		0x40000000,
		0x40000000,
		0x40000000,// 0x67
		0x00000008,
		0x0000000a,
		0x00000004,
		0x00000006,
		0x04000000,
		0x04000000,
		0x04000000,
		0x04000000,// 0x6f
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,// 0x77
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,// 0x7f
		0x00000005,
		0x0000000a,
		0x00000005,
		0x00000006,
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,// 0x87
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,// 0x8f
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,// 0x97
		0x00000000,
		0x00000000,
		0x20000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,// 0x9f
		0x00000010,
		0x00000020,
		0x00000010,
		0x00000020,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,// 0xa7
		0x00000004,
		0x00000008,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,// 0xaf
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,// 0xb7
		0x00000008,
		0x00000008,
		0x00000008,
		0x00000008,
		0x00000008,
		0x00000008,
		0x00000008,
		0x00000008,// 0xbf
		0x00000005,
		0x00000006,
		0x00000040,
		0x00000000,
		0x00000002,
		0x00000002,
		0x00000005,
		0x0000000a,// 0xc7
		0x00000044,
		0x00000000,
		0x00000040,
		0x00000000,
		0x00000000,
		0x00000004,
		0x00000000,
		0x00000000,// 0xcf
		0x00000001,
		0x00000002,
		0x00000001,
		0x00000002,
		0x00000004,
		0x00000004,
		0x00000000,
		0x00000000,// 0xd7
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0xdf
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,
		0x04000004,
		0x04000004,
		0x04000004,
		0x04000004,// 0xe7
		0x00000008,
		0x00000008,
		0x20000000,
		0x00000004,
		0x04000000,
		0x04000000,
		0x04000000,
		0x04000000,// 0xef
		0x40000000,
		0x00000000,
		0x40000000,
		0x40000000,
		0x04000000,
		0x00000000,
		0x11030001,
		0x12030002,// 0xf7
		0x00000000,
		0x00000000,
		0x04000000,
		0x04000000,
		0x00000000,
		0x00000000,
		0x00000001,
		0x00000002 // 0xff
	},
	{ // 0x0f
		0x04000002,
		0x04000002,
		0x00000002,
		0x00000002,
		0x08000000,
		0x08000000,
		0x00000000,
		0x08000000,// 0x0f 0x07
		0x00000000,
		0x00000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x00000002,
		0x08000000,
		0x08000000,// 0x0f 0x0f
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x17
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x1f
		0x04000002,
		0x04000002,
		0x04000002,
		0x04000002,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,// 0x0f 0x27
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x2f
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x08000000,
		0x08000000,// 0x0f 0x37
		0x80000000,
		0x08000000,
		0x80000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,// 0x0f 0x3f
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x47
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x4f
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x57
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x5f
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x67
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x6f
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x77
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0x7f
		0x00000020,
		0x00000020,
		0x00000020,
		0x00000020,
		0x00000020,
		0x00000020,
		0x00000020,
		0x00000020,// 0x0f 0x87
		0x00000020,
		0x00000020,
		0x00000020,
		0x00000020,
		0x00000020,
		0x00000020,
		0x00000020,
		0x00000020,// 0x0f 0x8f
		0x00000001,
		0x00000001,
		0x00000001,
		0x00000001,
		0x00000001,
		0x00000001,
		0x00000001,
		0x00000001,// 0x0f 0x97
		0x00000001,
		0x00000001,
		0x00000001,
		0x00000001,
		0x00000001,
		0x00000001,
		0x00000001,
		0x00000001,// 0x0f 0x9f
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000002,
		0x00000006,
		0x00000002,
		0x08000000,
		0x08000000,// 0x0f 0xa7
		0x00000000,
		0x00000000,
		0x04000000,
		0x00000002,
		0x00000006,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0xaf
		0x00000001,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0xb7
		0x00000002,
		0x08000000,
		0x00000006,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0xbf
		0x00000001,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,// 0x0f 0xc7
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,//0x0f 0xcf
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,//0x0f 0xd7
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,//0x0f 0xdf
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,//0x0f 0xe7
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,//0x0f 0xef
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,//0x0f 0xf7
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002 //0x0f 0xff
	},
	{ // 0x0f 0x38
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,//0x0f 0x38 0x07
		0x00000002,
		0x00000002,
		0x00000002,
		0x00000002,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x0f
		0x00000082,
		0x08000000,
		0x08000000,
		0x08000000,
		0x00000082,
		0x00000082,
		0x08000000,
		0x00000082,//0x0f 0x38 0x17
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x00000002,
		0x00000002,
		0x00000002,
		0x08000000,//0x0f 0x38 0x1f
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x08000000,
		0x08000000,//0x0f 0x38 0x27
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x2f
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x08000000,
		0x00000082,//0x0f 0x38 0x37
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,
		0x00000082,//0x0f 0x38 0x3f
		0x00000082,
		0x00000082,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x47
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x4f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x57
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x5f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x67
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x6f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x77
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x7f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x87
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x8f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x97
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0x9f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xa7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xaf
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xb7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xbf
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xc7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xcf
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xd7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xdf
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xe7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xef
		0x00000002,
		0x00000002,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x38 0xf7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000 //0x0f 0x38 0xff
	},
	{ // 0x0f 0x3a
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x07
		0x00000086,
		0x00000086,
		0x00000086,
		0x00000086,
		0x00000086,
		0x00000086,
		0x00000086,
		0x00000082,//0x0f 0x3a 0x0f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x00000086,
		0x00000086,
		0x00000086,
		0x00000086,//0x0f 0x3a 0x17
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x1f
		0x00000086,
		0x00000086,
		0x00000086,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x27
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x2f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x37
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x3f
		0x00000082,
		0x00000082,
		0x00000086,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x47
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x4f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x57
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x5f
		0x00000086,
		0x00000086,
		0x00000086,
		0x00000086,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x67
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x6f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x77
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x7f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x87
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x8f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x97
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0x9f
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xa7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xaf
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xb7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xbf
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xc7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xcf
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xd7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xdf
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xe7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xef
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,//0x0f 0x3a 0xf7
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000,
		0x08000000 //0x0f 0x3a 0xff
	}
};

#ifdef _M_X64
void cpuidhost(UINT32 prm_0, UINT32 prm_1, void* prm_2) {
	__cpuidex((int*)prm_2, prm_0, prm_1);
	return;
}
#else
#ifdef _M_IX86
__declspec(naked) void cpuidhost(UINT32, UINT32, void*) {
	__asm {
		push esi
		mov esi, esp + 8
		push eax
		push ebx
		push ecx
		push edx
		push edi
		pushfd
		pop eax
		or eax, 0x100000
		push eax
		popfd
		pushfd
		pop eax
		test eax, 0x100000
		jz nocpuid
		mov eax, [esi + 0]
		mov ecx, [esi + 4]
		cpuid
		mov edi, [esi + 8]
		mov[edi + 0], eax
		mov[edi + 4], ebx
		mov[edi + 8], ecx
		mov[edi + 12], edx
		pop edi
		pop edx
		pop ecx
		pop ebx
		pop eax
		pop esi
		ret
nocpuid:
		xor eax, eax
		mov edi, [esi + 8]
		mov[edi + 0], eax
		mov[edi + 4], eax
		mov[edi + 8], eax
		mov[edi + 12], eax
		pop edi
		pop edx
		pop ecx
		pop ebx
		pop eax
		pop esi
		ret
	};
}
#else
void cpuidhost(UINT32 prm_0, UINT32 prm_1, void* prm_2) {
	((int*)prm_2)[0] = 0;
	((int*)prm_2)[1] = 0;
	((int*)prm_2)[2] = 0;
	((int*)prm_2)[3] = 0;
	return;
}
#endif
#endif

#ifdef _M_X64
#else
#ifdef _M_IX86
#else
#ifdef _M_ARM64
#define GEN_LOADINT_FUNCTION(__reg,__val) *(UINT32*)((*(UINT64*)(pos))) = 0xd2800000|(__reg&0x1f)|((((__val)>>(16*0))&0xFFFF)<<5); ((*(UINT64*)(pos))) += 4; if ((((__val)>>(16*1))&0xFFFF) != 0){*(UINT32*)((*(UINT64*)(pos))) = 0xF2A00000|(__reg&0x1f)|((((__val)>>(16*1))&0xFFFF)<<5); ((*(UINT64*)(pos))) += 4;} if ((((__val)>>(16*2))&0xFFFF) != 0){*(UINT32*)((*(UINT64*)(pos))) = 0xF2C00000|(__reg&0x1f)|((((__val)>>(16*2))&0xFFFF)<<5); ((*(UINT64*)(pos))) += 4;} if ((((__val)>>(16*3))&0xFFFF) != 0){*(UINT32*)((*(UINT64*)(pos))) = 0xF2E00000|(__reg&0x1f)|((((__val)>>(16*3))&0xFFFF)<<5); ((*(UINT64*)(pos))) += 4;}
#define GEN_PUSHNATIVE_FUNCTION(__reg1,__reg2) *(UINT32*)((*(UINT64*)(pos))) = 0xA9BF03E0|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<11); ((*(UINT64*)(pos))) += 4;
#define GEN_POPNATIVE_FUNCTION(__reg1,__reg2) *(UINT32*)((*(UINT64*)(pos))) = 0xA8C103E0|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<11); ((*(UINT64*)(pos))) += 4;
#define GEN_LOAD_FUNCTION(__bitsz,__reg1,__reg2,__reg3) *(UINT32*)((*(UINT64*)(pos))) = 0x38606800|((__bitsz&3)<<30)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); ((*(UINT64*)(pos))) += 4;
#define GEN_STOR_FUNCTION(__bitsz,__reg1,__reg2,__reg3) *(UINT32*)((*(UINT64*)(pos))) = 0x38206800|((__bitsz&3)<<30)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); ((*(UINT64*)(pos))) += 4;
#define GEN_ADD_FUNCTION(__setflg,__bitsz,__reg1,__reg2,__reg3) *(UINT32*)((*(UINT64*)(pos))) = 0x0b000000|((__bitsz&1)<<31)|((__setflg&1)<<29)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); ((*(UINT64*)(pos))) += 4;
#define GEN_SUB_FUNCTION(__setflg,__bitsz,__reg1,__reg2,__reg3) *(UINT32*)((*(UINT64*)(pos))) = 0x4b000000|((__bitsz&1)<<31)|((__setflg&1)<<29)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); ((*(UINT64*)(pos))) += 4;
#define GEN_AND_FUNCTION(__setflg,__bitsz,__reg1,__reg2,__reg3) *(UINT32*)((*(UINT64*)(pos))) = 0x0a000000|((__bitsz&1)<<31)|((__setflg&1)<<29)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); ((*(UINT64*)(pos))) += 4;
#define GEN_XOR_FUNCTION(__setflg,__bitsz,__reg1,__reg2,__reg3) *(UINT32*)((*(UINT64*)(pos))) = 0x4a000000|((__bitsz&1)<<31)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); ((*(UINT64*)(pos))) += 4;if ((__reg1&0x1f) != 31 || (__setflg&1) != 0){GEN_AND_FUNCTION(__setflg,__bitsz,31,__reg1,__reg1);}
#define GEN_OR_FUNCTION(__setflg,__bitsz,__reg1,__reg2,__reg3) *(UINT32*)((*(UINT64*)(pos))) = 0x2a000000|((__bitsz&1)<<31)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); ((*(UINT64*)(pos))) += 4;if ((__reg1&0x1f) != 31 || (__setflg&1) != 0){GEN_AND_FUNCTION(__setflg,__bitsz,31,__reg1,__reg1);}
#define GEN_CMP_FUNCTION(__bitsz,__reg1,__reg2) GEN_SUB_FUNCTION(1,__bitsz,31,__reg1,__reg2);
#define GEN_TST_FUNCTION(__bitsz,__reg1,__reg2) GEN_AND_FUNCTION(1,__bitsz,31,__reg1,__reg2);
#define GEN_MOV_FUNCTION(__bitsz,__reg1,__reg2) if (__reg2==31 || __reg1==31){*(UINT32*)((*(UINT64*)(pos))) = 0x11000000|((__bitsz&1)<<31)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5); ((*(UINT64*)(pos))) += 4;}else{GEN_OR_FUNCTION(0,__bitsz,__reg1,31,__reg2);}
#define GEN_BLR_FUNCTION(__reg) *(UINT32*)((*(UINT64*)(pos))) = 0xD63F0000|((__reg&0x1f)<<5); ((*(UINT64*)(pos))) += 4
#define GEN_BR_FUNCTION(__reg) *(UINT32*)((*(UINT64*)(pos))) = 0xD61F0000|((__reg&0x1f)<<5); ((*(UINT64*)(pos))) += 4
#define GEN_CALL_FUNCTION(__addr) GEN_PUSHNATIVE_FUNCTION(9,30); GEN_LOADINT_FUNCTION(9,__addr); GEN_BLR_FUNCTION(9); GEN_POPNATIVE_FUNCTION(9,30);
#define GEN_JP_FUNCTION(__addr) GEN_LOADINT_FUNCTION(9,__addr); GEN_BR_FUNCTION(9);
#define GEN_RET_FUNCTION() *(UINT32*)((*(UINT64*)(pos))) = 0xD65F03C0; ((*(UINT64*)(pos))) += 4;
#else
#endif
#endif
#endif

UINT64 _get_modrm8() {
	UINT32 op;
	UINT32 madr;
	SINT32 res;
	SINT8 src, dst;
	UINT64 modrmpos;
	UINT64 eipnow = CPU_EIP;
	GET_PCBYTE(op);

	if (op >= 0xc0) {
		src = *(reg8_b20[op]);
	}
	else {
		madr = calc_ea_dst(op);
		src = cpu_vmemoryread(CPU_INST_SEGREG_INDEX, madr);
	}
	modrmpos = (CPU_EIP - eipnow);
	CPU_EIP = eipnow;
	return modrmpos;
}

UINT64 _get_modrm16() {
	UINT32 op;
	UINT32 madr;
	SINT32 res;
	SINT8 src, dst;
	UINT64 modrmpos;
	UINT64 eipnow = CPU_EIP;
	GET_PCBYTE(op);

	if (op >= 0xc0) {
		src = *(reg16_b20[op]);
	}
	else {
		madr = calc_ea_dst(op);
		src = cpu_vmemoryread_w(CPU_INST_SEGREG_INDEX, madr);
	}
	modrmpos = (CPU_EIP - eipnow);
	CPU_EIP = eipnow;
	return modrmpos;
}

UINT64 _get_modrm32() {
	UINT32 op;
	UINT32 madr;
	SINT32 res;
	SINT8 src, dst;
	UINT64 modrmpos;
	UINT64 eipnow = CPU_EIP;
	GET_PCBYTE(op);

	if (op >= 0xc0) {
		src = *(reg32_b20[op]);
	}
	else {
		madr = calc_ea_dst(op);
		src = cpu_vmemoryread_d(CPU_INST_SEGREG_INDEX, madr);
	}
	modrmpos = (CPU_EIP - eipnow);
	CPU_EIP = eipnow;
	return modrmpos;
}

bool isretreqired(UINT64 prm_1, UINT64 prm_2) {
	if (opcodeoperantsizedesc[prm_2][prm_1]&0x0C000000) { return 1; }
	return 0;
}

UINT64 _getmodrmsize(UINT64 prm_1,UINT64 prm_2) {
	UINT64 sizetmp = 0;
	UINT64 op_byte = 0;
	if (opcodeoperantsizedesc[prm_2][prm_1] & 0x00000001) { sizetmp += _get_modrm8(); }
	if (opcodeoperantsizedesc[prm_2][prm_1] & 0x00000002) { sizetmp += ((CPU_INST_OP32) ? _get_modrm32() : _get_modrm16()); }
	if (opcodeoperantsizedesc[prm_2][prm_1] & 0x00000004) { sizetmp += 1; }
	if (opcodeoperantsizedesc[prm_2][prm_1] & 0x00000008) { sizetmp += ((CPU_INST_OP32) ? 4 : 2); }
	if (opcodeoperantsizedesc[prm_2][prm_1] & 0x00000010) { sizetmp += 1; }
	if (opcodeoperantsizedesc[prm_2][prm_1] & 0x00000020) { sizetmp += ((CPU_INST_OP32) ? 4 : 2); }
	if (opcodeoperantsizedesc[prm_2][prm_1] & 0x00000040) { sizetmp += 2; }
	return sizetmp;
}

void _aot_rep16(UINT8 op32_byte, UINT8 op_byte) {
	CPU_WORKCLOCK(5);
	for (;;) {
		(*insttable_1byte[op32_byte][op_byte])();
		if (--CPU_CX == 0) {
#if defined(DEBUG)
			cpu_debug_rep_cont = 0;
#endif
			break;
		}
		/*if (CPU_REMCLOCK <= 0) {
			CPU_EIP = CPU_PREV_EIP;
			break;
		}*/
	}
}

void _aot_repe16(UINT8 op32_byte,UINT8 op_byte) {
	CPU_WORKCLOCK(5);
	for (;;) {
		(*insttable_1byte[op32_byte][op_byte])();
		if (--CPU_CX == 0 || CC_NZ) {
#if defined(DEBUG)
			cpu_debug_rep_cont = 0;
#endif
			break;
		}
		/*if (CPU_REMCLOCK <= 0) {
			CPU_EIP = CPU_PREV_EIP;
			break;
		}*/
	}
}

void _aot_repne16(UINT8 op32_byte, UINT8 op_byte) {
	CPU_WORKCLOCK(5);
	for (;;) {
		(*insttable_1byte[op32_byte][op_byte])();
		if (--CPU_CX == 0 || CC_Z) {
#if defined(DEBUG)
			cpu_debug_rep_cont = 0;
#endif
			break;
		}
		/*if (CPU_REMCLOCK <= 0) {
			CPU_EIP = CPU_PREV_EIP;
			break;
		}*/
	}
}

void _aot_rep32(UINT8 op32_byte, UINT8 op_byte) {
	CPU_WORKCLOCK(5);
	for (;;) {
		(*insttable_1byte[op32_byte][op_byte])();
		if (--CPU_ECX == 0) {
#if defined(DEBUG)
			cpu_debug_rep_cont = 0;
#endif
			break;
		}
		/*if (CPU_REMCLOCK <= 0) {
			CPU_EIP = CPU_PREV_EIP;
			break;
		}*/
	}
}

void _aot_repe32(UINT8 op32_byte, UINT8 op_byte) {
	CPU_WORKCLOCK(5);
	for (;;) {
		(*insttable_1byte[op32_byte][op_byte])();
		if (--CPU_ECX == 0 || CC_NZ) {
#if defined(DEBUG)
			cpu_debug_rep_cont = 0;
#endif
			break;
		}
		/*if (CPU_REMCLOCK <= 0) {
			CPU_EIP = CPU_PREV_EIP;
			break;
		}*/
	}
}

void _aot_repne32(UINT8 op32_byte, UINT8 op_byte) {
	CPU_WORKCLOCK(5);
	for (;;) {
		(*insttable_1byte[op32_byte][op_byte])();
		if (--CPU_ECX == 0 || CC_Z) {
#if defined(DEBUG)
			cpu_debug_rep_cont = 0;
#endif
			break;
		}
		/*if (CPU_REMCLOCK <= 0) {
			CPU_EIP = CPU_PREV_EIP;
			break;
		}*/
	}
}

#if 0
inline bool GenNativecode(UINT64 pos) {
	int prefix;
	UINT32 op;

	if ((*(UINT64*)(&CPU_STATSAVE.cpu_inst)) != (*(UINT64*)(&CPU_STATSAVE.cpu_inst_default))) {
		GEN_LOADINT_FUNCTION(0, (*(UINT64*)(&CPU_STATSAVE.cpu_inst_default)))
		GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_STATSAVE.cpu_inst)
		GEN_LOADINT_FUNCTION(2, 0)
		GEN_STOR_FUNCTION(3, 0, 1, 2)
	}
#if defined(USE_DEBUGGER)
	CPU_PREV_CS = CPU_CS;
#endif
	CPU_PREV_EIP = CPU_EIP;
	CPU_STATSAVE.cpu_inst = CPU_STATSAVE.cpu_inst_default;

#if defined(ENABLE_TRAP)
	steptrap(CPU_CS, CPU_EIP);
#endif

#if defined(IA32_INSTRUCTION_TRACE)
	ctx[ctx_index].regs = CPU_STATSAVE.cpu_regs;
	if (cpu_inst_trace) {
		disasm_context_t* d = &ctx[ctx_index].disasm;
		UINT32 eip = CPU_EIP;
		int rv;

		rv = disasm(&eip, d);
		if (rv == 0) {
			char buf[256];
			char tmp[32];
			int len = d->nopbytes > 8 ? 8 : d->nopbytes;
			int i;

			buf[0] = '\0';
			for (i = 0; i < len; i++) {
				snprintf(tmp, sizeof(tmp), "%02x ", d->opbyte[i]);
				milstr_ncat(buf, tmp, sizeof(buf));
			}
			for (; i < 8; i++) {
				milstr_ncat(buf, "   ", sizeof(buf));
			}
			VERBOSE(("%04x:%08x: %s%s", CPU_CS, CPU_EIP, buf, d->str));

			buf[0] = '\0';
			for (; i < d->nopbytes; i++) {
				snprintf(tmp, sizeof(tmp), "%02x ", d->opbyte[i]);
				milstr_ncat(buf, tmp, sizeof(buf));
				if ((i % 8) == 7) {
					VERBOSE(("             : %s", buf));
					buf[0] = '\0';
				}
			}
			if ((i % 8) != 0) {
				VERBOSE(("             : %s", buf));
			}
		}
	}
	ctx[ctx_index].opbytes = 0;
#endif

	for (prefix = 0; prefix < MAX_PREFIX; prefix++) {
		GET_PCBYTE(op);
		if (prefix == 0) {
#if defined(USE_DEBUGGER)
			add_cpu_trace(CPU_PREV_PC, CPU_PREV_CS, CPU_PREV_EIP);
#endif
		}
#if defined(IA32_INSTRUCTION_TRACE)
		ctx[ctx_index].op[prefix] = op;
		ctx[ctx_index].opbytes++;
#endif

		/* prefix */
		if (insttable_info[op] & INST_PREFIX) {
			(*insttable_1byte[0][op])();
			continue;
		}
		break;
	}
	if (prefix == MAX_PREFIX) {
		//EXCEPTION(UD_EXCEPTION, 0);
		GEN_LOADINT_FUNCTION(0, UD_EXCEPTION)
		GEN_LOADINT_FUNCTION(1, 0)
		GEN_CALL_FUNCTION((UINT64)&exception)
		GEN_RET_FUNCTION()
		return 1;
	}

	if (prefix != 0) {
		GEN_LOADINT_FUNCTION(0, (*(UINT64*)(&CPU_STATSAVE.cpu_inst)))
		GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_STATSAVE.cpu_inst)
		GEN_LOADINT_FUNCTION(2, 0)
		GEN_STOR_FUNCTION(3, 0, 1, 2)
	}

#if defined(IA32_INSTRUCTION_TRACE)
	if (op == 0x0f) {
		BYTE op2;
		op2 = cpu_codefetch(CPU_EIP);
		ctx[ctx_index].op[prefix + 1] = op2;
		ctx[ctx_index].opbytes++;
	}
	ctx_index = (ctx_index + 1) % NELEMENTS(ctx);
#endif

	/* normal / rep, but not use */
	if (!(insttable_info[op] & INST_STRING) || !CPU_INST_REPUSE) {
#if defined(DEBUG)
		cpu_debug_rep_cont = 0;
#endif
		//(*insttable_1byte[CPU_INST_OP32][op])();
#if 1
		if (op == 0x0f) {
			GET_PCBYTE(op);
			if (op == 0x38) {
				if (CPU_INST_OP32) {
#ifdef USE_SSSE3
					if (!(i386cpuid.cpu_feature_ecx & CPU_FEATURE_ECX_SSSE3)) {
						GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
						GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
						GEN_LOADINT_FUNCTION(2, 0)
						GEN_STOR_FUNCTION(3, 0, 1, 2)
						GEN_LOADINT_FUNCTION(0, UD_EXCEPTION)
						GEN_LOADINT_FUNCTION(1, 0)
						GEN_CALL_FUNCTION((UINT64)&exception)
						GEN_RET_FUNCTION()
						return 1;
					}
					else {
						GET_PCBYTE(op);
						GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
						GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
						GEN_LOADINT_FUNCTION(2, 0)
						GEN_STOR_FUNCTION(3, 0, 1, 2)
						if (insttable_3byte660F38_32[op] && CPU_INST_OP32 == !CPU_STATSAVE.cpu_inst_default.op_32) {
							//(*insttable_3byte660F38_32[op])();
							GEN_CALL_FUNCTION((UINT64)*insttable_3byte660F38_32[op])
						}
						else if (insttable_3byteF20F38_32[op] && CPU_INST_REPUSE == 0xf2) {
							//(*insttable_3byteF20F38_32[op])();
							GEN_CALL_FUNCTION((UINT64)*insttable_3byteF20F38_32[op])
						}
						else if (insttable_2byte0F38_32[op]) {
							//(*insttable_2byte0F38_32[op])();
							GEN_CALL_FUNCTION((UINT64)*insttable_2byte0F38_32[op])
						}
						else {
							GEN_LOADINT_FUNCTION(0, UD_EXCEPTION)
							GEN_LOADINT_FUNCTION(1, 0)
							GEN_CALL_FUNCTION((UINT64)&exception)
							GEN_RET_FUNCTION()
							return 1;
						}
						CPU_EIP += _getmodrmsize(op, 2);
						return 0;
					}
#else
					GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
					GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
					GEN_LOADINT_FUNCTION(2, 0)
					GEN_STOR_FUNCTION(3, 0, 1, 2)
					GEN_LOADINT_FUNCTION(0, UD_EXCEPTION)
					GEN_LOADINT_FUNCTION(1, 0)
					GEN_CALL_FUNCTION((UINT64)&exception)
					GEN_RET_FUNCTION()
					return 1;
#endif
				}
				else {
#ifdef USE_SSSE3
					if (!(i386cpuid.cpu_feature_ecx & CPU_FEATURE_ECX_SSSE3)) {
						GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
						GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
						GEN_LOADINT_FUNCTION(2, 0)
						GEN_STOR_FUNCTION(3, 0, 1, 2)
						GEN_LOADINT_FUNCTION(0, UD_EXCEPTION)
						GEN_LOADINT_FUNCTION(1, 0)
						GEN_CALL_FUNCTION((UINT64)&exception)
						GEN_RET_FUNCTION()
						return 1;
					}
					else {
						GET_PCBYTE(op);
						GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
						GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
						GEN_LOADINT_FUNCTION(2, 0)
						GEN_STOR_FUNCTION(3, 0, 1, 2)
						if (insttable_3byte660F38_32[op] && CPU_INST_OP32 == !CPU_STATSAVE.cpu_inst_default.op_32) {
							//(*insttable_3byte660F38_32[op])();
							GEN_CALL_FUNCTION((UINT64)*insttable_3byte660F38_32[op])
						}
						else if (insttable_3byteF20F38_16[op] && CPU_INST_REPUSE == 0xf2) {
							//(*insttable_3byteF20F38_16[op])();
							GEN_CALL_FUNCTION((UINT64)*insttable_3byteF20F38_16[op])
						}
						else if (insttable_2byte0F38_32[op]) {
							//(*insttable_2byte0F38_32[op])();
							GEN_CALL_FUNCTION((UINT64)*insttable_2byte0F38_32[op])
						}
						else {
							GEN_LOADINT_FUNCTION(0, UD_EXCEPTION)
							GEN_LOADINT_FUNCTION(1, 0)
							GEN_CALL_FUNCTION((UINT64)&exception)
							GEN_RET_FUNCTION()
							return 1;
						}
						CPU_EIP += _getmodrmsize(op, 2);
						return 0;
					}
#else
					GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
					GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
					GEN_LOADINT_FUNCTION(2, 0)
					GEN_STOR_FUNCTION(3, 0, 1, 2)
					GEN_LOADINT_FUNCTION(0, UD_EXCEPTION)
					GEN_LOADINT_FUNCTION(1, 0)
					GEN_CALL_FUNCTION((UINT64)&exception)
					GEN_RET_FUNCTION()
					return 1;
#endif
				}
				GEN_RET_FUNCTION()
				return 1;
			}
			else if (op == 0x3a) {
#ifdef USE_SSSE3
				if (!(i386cpuid.cpu_feature_ecx & CPU_FEATURE_ECX_SSSE3)) {
					GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
					GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
					GEN_LOADINT_FUNCTION(2, 0)
					GEN_STOR_FUNCTION(3, 0, 1, 2)
					GEN_LOADINT_FUNCTION(0, UD_EXCEPTION)
					GEN_LOADINT_FUNCTION(1, 0)
					GEN_CALL_FUNCTION((UINT64)&exception)
					GEN_RET_FUNCTION()
					return 1;
				}
				else {
					GET_PCBYTE(op);
					GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
					GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
					GEN_LOADINT_FUNCTION(2, 0)
					GEN_STOR_FUNCTION(3, 0, 1, 2)
					if (insttable_3byte660F3A_32[op] && CPU_INST_OP32 == !CPU_STATSAVE.cpu_inst_default.op_32) {
						//(*insttable_3byte660F3A_32[op])();
						GEN_CALL_FUNCTION((UINT64)*insttable_3byte660F3A_32[op])
					}
					else if (insttable_2byte0F3A_32[op]) {
						//(*insttable_2byte0F3A_32[op])();
						GEN_CALL_FUNCTION((UINT64)*insttable_2byte0F3A_32[op])
					}
					else {
						GEN_LOADINT_FUNCTION(0, UD_EXCEPTION)
						GEN_LOADINT_FUNCTION(1, 0)
						GEN_CALL_FUNCTION((UINT64)&exception)
						GEN_RET_FUNCTION()
						return 1;
					}
					CPU_EIP += _getmodrmsize(op, 3);
					return 0;
				}
#else
				GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
				GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
				GEN_LOADINT_FUNCTION(2, 0)
				GEN_STOR_FUNCTION(3, 0, 1, 2)
				GEN_LOADINT_FUNCTION(0, UD_EXCEPTION)
				GEN_LOADINT_FUNCTION(1, 0)
				GEN_CALL_FUNCTION((UINT64)&exception)
				GEN_RET_FUNCTION()
				return 1;
#endif
				GEN_RET_FUNCTION()
				return 1;
			}
			else {
				if ((op >= 0x80 && op <= 0x8f) || op == 0xaa || isretreqired(op, 1)) { 
					GEN_RET_FUNCTION()
					return 1;
				}
				GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
				GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
				GEN_LOADINT_FUNCTION(2, 0)
				GEN_STOR_FUNCTION(3, 0, 1, 2)
				if (CPU_INST_OP32) {
#ifdef USE_SSE
					if (insttable_2byte660F_32[op] && CPU_INST_OP32 == !CPU_STATSAVE.cpu_inst_default.op_32) {
						//(*insttable_2byte660F_32[op])();
						GEN_CALL_FUNCTION((UINT64)*insttable_2byte660F_32[op])
					}
					else if (insttable_2byteF20F_32[op] && CPU_INST_REPUSE == 0xf2) {
						//(*insttable_2byteF20F_32[op])();
						GEN_CALL_FUNCTION((UINT64)*insttable_2byteF20F_32[op])
					}
					else if (insttable_2byteF30F_32[op] && CPU_INST_REPUSE == 0xf3) {
						//(*insttable_2byteF30F_32[op])();
						GEN_CALL_FUNCTION((UINT64)*insttable_2byteF30F_32[op])
					}
					else {
						//(*insttable_2byte[1][op])();
						GEN_CALL_FUNCTION((UINT64)*insttable_2byte[1][op])
					}
#else
					//(*insttable_2byte[1][op])();
					GEN_CALL_FUNCTION((UINT64)*insttable_2byte[1][op])
#endif
				}
				else {
#ifdef USE_SSE
					if (insttable_2byte660F_32[op] && CPU_INST_OP32 == !CPU_STATSAVE.cpu_inst_default.op_32) {
						//(*insttable_2byte660F_32[op])();
						GEN_CALL_FUNCTION((UINT64)*insttable_2byte660F_32[op])
					}
					else if (insttable_2byteF20F_32[op] && CPU_INST_REPUSE == 0xf2) {
						//(*insttable_2byteF20F_32[op])();
						GEN_CALL_FUNCTION((UINT64)*insttable_2byteF20F_32[op])
					}
					else if (insttable_2byteF30F_32[op] && CPU_INST_REPUSE == 0xf3) {
						//(*insttable_2byteF30F_32[op])();
						GEN_CALL_FUNCTION((UINT64)*insttable_2byteF30F_32[op])
					}
					else {
						//(*insttable_2byte[0][op])();
						GEN_CALL_FUNCTION((UINT64)*insttable_2byte[0][op])
					}
#else
					//(*insttable_2byte[0][op])();
					GEN_CALL_FUNCTION((UINT64)*insttable_2byte[0][op])
#endif
				}
				CPU_EIP += _getmodrmsize(op, 1);
				if ((op >= 0x80 && op <= 0x8f) || op == 0xaa || isretreqired(op, 1)) { 
					GEN_RET_FUNCTION()
					return 1;
				}
			}
		}
		else 
#endif
		{
			if (op == 0x0f || (op >= 0x70 && op <= 0x7f) || op == 0x9a || op == 0xc2 || op == 0xc3 || (op >= 0xca && op <= 0xcf) || (op >= 0xd8 && op <= 0xdf) || (op >= 0xe0 && op <= 0xef) || op == 0xf4 || op == 0xff || isretreqired(op, 0)) { 
				GEN_RET_FUNCTION()
				return 1;
			}
			GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
			GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
			GEN_LOADINT_FUNCTION(2, 0)
			GEN_STOR_FUNCTION(3, 0, 1, 2)
			GEN_CALL_FUNCTION((UINT64)*insttable_1byte[CPU_INST_OP32][op])
			CPU_EIP += _getmodrmsize(op, 0);
			if (op == 0x0f || (op >= 0x70 && op <= 0x7f) || op == 0x9a || op == 0xc2 || op == 0xc3 || (op >= 0xca && op <= 0xcf) || (op >= 0xd8 && op <= 0xdf) || (op >= 0xe0 && op <= 0xe3) || (op >= 0xe8 && op <= 0xeb) || op == 0xf4 || op == 0xff || isretreqired(op, 0)) { 
				GEN_RET_FUNCTION()
				return 1;
			}
		}
		return 0;
	}

	GEN_RET_FUNCTION()
	return 1;

	/* rep */
	//CPU_WORKCLOCK(5);
#if defined(DEBUG)
	if (!cpu_debug_rep_cont) {
		cpu_debug_rep_cont = 1;
		cpu_debug_rep_regs = CPU_STATSAVE.cpu_regs;
	}
#endif
	GEN_LOADINT_FUNCTION(0, (((UINT64)CPU_PREV_EIP) << 32) | CPU_EIP)
	GEN_LOADINT_FUNCTION(1, (UINT64)&CPU_EIP)
	GEN_LOADINT_FUNCTION(2, 0)
	GEN_STOR_FUNCTION(3, 0, 1, 2)
	if (!CPU_INST_AS32) {
		if (CPU_CX != 0) {
			if (!(insttable_info[op] & REP_CHECKZF)) {
				/* rep */
				GEN_LOADINT_FUNCTION(0, CPU_INST_OP32)
				GEN_LOADINT_FUNCTION(1, op)
				GEN_CALL_FUNCTION((UINT64)&_aot_rep16)
			}
			else if (CPU_INST_REPUSE != 0xf2) {
				/* repe */
				GEN_LOADINT_FUNCTION(0, CPU_INST_OP32)
				GEN_LOADINT_FUNCTION(1, op)
				GEN_CALL_FUNCTION((UINT64)&_aot_repe16)
			}
			else {
				/* repne */
				GEN_LOADINT_FUNCTION(0, CPU_INST_OP32)
				GEN_LOADINT_FUNCTION(1, op)
				GEN_CALL_FUNCTION((UINT64)&_aot_repne16)
			}
		}
	}
	else {
		if (CPU_ECX != 0) {
			if (!(insttable_info[op] & REP_CHECKZF)) {
				/* rep */
				GEN_LOADINT_FUNCTION(0, CPU_INST_OP32)
				GEN_LOADINT_FUNCTION(1, op)
				GEN_CALL_FUNCTION((UINT64)&_aot_rep32)
			}
			else if (CPU_INST_REPUSE != 0xf2) {
				/* repe */
				GEN_LOADINT_FUNCTION(0, CPU_INST_OP32)
				GEN_LOADINT_FUNCTION(1, op)
				GEN_CALL_FUNCTION((UINT64)&_aot_repe32)
			}
			else {
				/* repne */
				GEN_LOADINT_FUNCTION(0, CPU_INST_OP32)
				GEN_LOADINT_FUNCTION(1, op)
				GEN_CALL_FUNCTION((UINT64)&_aot_repne32)
			}
		}
	}
	return 0;
}

inline void InsertRettoJITC(UINT64 pos) {
	GEN_RET_FUNCTION()
}
inline void InsertexecallsteptoJITC(UINT64 pos) {
	GEN_CALL_FUNCTION((UINT64)&exec_allstep)
}
inline void Insertexec1steptoJITC(UINT64 pos) {
	GEN_CALL_FUNCTION((UINT64)&exec_1step)
}
#endif

typedef void function4xecutejited();

HMODULE ntdll4virtualmem = 0;
typedef NTSYSAPI NTSTATUS  WINAPI typeof_NtAllocateVirtualMemory(HANDLE, PVOID*, ULONG_PTR, SIZE_T*, ULONG, ULONG);
typedef NTSYSAPI NTSTATUS  WINAPI typeof_NtFreeVirtualMemory(HANDLE, PVOID*, SIZE_T*, ULONG);
typeof_NtAllocateVirtualMemory* NtAllocateVirtualMemory;
typeof_NtFreeVirtualMemory* NtFreeVirtualMemory;


#define DOFLAG_PF	reg_flags=(reg_flags & ~FLAG_PF) | parity_lookup[lf_resb];

#define DOFLAG_AF	reg_flags=(reg_flags & ~FLAG_AF) | (((lf_var1b ^ lf_var2b) ^ lf_resb) & 0x10U);

#define DOFLAG_ZFb	SETFLAGBIT(ZF,lf_resb==0);
#define DOFLAG_ZFw	SETFLAGBIT(ZF,lf_resw==0);
#define DOFLAG_ZFd	SETFLAGBIT(ZF,lf_resd==0);

#define DOFLAG_SFb	reg_flags=(reg_flags & ~FLAG_SF) | ((lf_resb & 0x80U) >> 0U);
#define DOFLAG_SFw	reg_flags=(reg_flags & ~FLAG_SF) | ((lf_resw & 0x8000U) >> 8U);
#define DOFLAG_SFd	reg_flags=(reg_flags & ~FLAG_SF) | ((lf_resd & 0x80000000U) >> 24U);

#define SETCF(NEWBIT) reg_flags=(reg_flags & ~FLAG_CF)|(NEWBIT);

#define SET_FLAG SETFLAGBIT

#if 0

Bitu FillFlags(void) {
	switch (lflags.type) {
	case t_UNKNOWN:
		break;
	case t_ADDb:
		SET_FLAG(CF, (lf_resb < lf_var1b));
		DOFLAG_AF;
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, ((lf_var1b ^ lf_var2b ^ 0x80) & (lf_resb ^ lf_var1b)) & 0x80);
		DOFLAG_PF;
		break;
	case t_ADDw:
		SET_FLAG(CF, (lf_resw < lf_var1w));
		DOFLAG_AF;
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, ((lf_var1w ^ lf_var2w ^ 0x8000) & (lf_resw ^ lf_var1w)) & 0x8000);
		DOFLAG_PF;
		break;
	case t_ADDd:
		SET_FLAG(CF, (lf_resd < lf_var1d));
		DOFLAG_AF;
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, ((lf_var1d ^ lf_var2d ^ 0x80000000) & (lf_resd ^ lf_var1d)) & 0x80000000);
		DOFLAG_PF;
		break;
	case t_ADCb:
		SET_FLAG(CF, (lf_resb < lf_var1b) || (lflags.oldcf && (lf_resb == lf_var1b)));
		DOFLAG_AF;
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, ((lf_var1b ^ lf_var2b ^ 0x80) & (lf_resb ^ lf_var1b)) & 0x80);
		DOFLAG_PF;
		break;
	case t_ADCw:
		SET_FLAG(CF, (lf_resw < lf_var1w) || (lflags.oldcf && (lf_resw == lf_var1w)));
		DOFLAG_AF;
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, ((lf_var1w ^ lf_var2w ^ 0x8000) & (lf_resw ^ lf_var1w)) & 0x8000);
		DOFLAG_PF;
		break;
	case t_ADCd:
		SET_FLAG(CF, (lf_resd < lf_var1d) || (lflags.oldcf && (lf_resd == lf_var1d)));
		DOFLAG_AF;
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, ((lf_var1d ^ lf_var2d ^ 0x80000000) & (lf_resd ^ lf_var1d)) & 0x80000000);
		DOFLAG_PF;
		break;


	case t_SBBb:
		SET_FLAG(CF, (lf_var1b < lf_resb) || (lflags.oldcf && (lf_var2b == 0xff)));
		DOFLAG_AF;
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, (lf_var1b ^ lf_var2b) & (lf_var1b ^ lf_resb) & 0x80);
		DOFLAG_PF;
		break;
	case t_SBBw:
		SET_FLAG(CF, (lf_var1w < lf_resw) || (lflags.oldcf && (lf_var2w == 0xffff)));
		DOFLAG_AF;
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, (lf_var1w ^ lf_var2w) & (lf_var1w ^ lf_resw) & 0x8000);
		DOFLAG_PF;
		break;
	case t_SBBd:
		SET_FLAG(CF, (lf_var1d < lf_resd) || (lflags.oldcf && (lf_var2d == 0xffffffff)));
		DOFLAG_AF;
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, (lf_var1d ^ lf_var2d) & (lf_var1d ^ lf_resd) & 0x80000000);
		DOFLAG_PF;
		break;


	case t_SUBb:
	case t_CMPb:
		SET_FLAG(CF, (lf_var1b < lf_var2b));
		DOFLAG_AF;
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, (lf_var1b ^ lf_var2b) & (lf_var1b ^ lf_resb) & 0x80);
		DOFLAG_PF;
		break;
	case t_SUBw:
	case t_CMPw:
		SET_FLAG(CF, (lf_var1w < lf_var2w));
		DOFLAG_AF;
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, (lf_var1w ^ lf_var2w) & (lf_var1w ^ lf_resw) & 0x8000);
		DOFLAG_PF;
		break;
	case t_SUBd:
	case t_CMPd:
		SET_FLAG(CF, (lf_var1d < lf_var2d));
		DOFLAG_AF;
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, (lf_var1d ^ lf_var2d) & (lf_var1d ^ lf_resd) & 0x80000000);
		DOFLAG_PF;
		break;


	case t_ORb:
		SET_FLAG(CF, false);
		SET_FLAG(AF, false);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		break;
	case t_ORw:
		SET_FLAG(CF, false);
		SET_FLAG(AF, false);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		break;
	case t_ORd:
		SET_FLAG(CF, false);
		SET_FLAG(AF, false);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		break;


	case t_TESTb:
	case t_ANDb:
		SET_FLAG(CF, false);
		SET_FLAG(AF, false);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		break;
	case t_TESTw:
	case t_ANDw:
		SET_FLAG(CF, false);
		SET_FLAG(AF, false);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		break;
	case t_TESTd:
	case t_ANDd:
		SET_FLAG(CF, false);
		SET_FLAG(AF, false);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		break;


	case t_XORb:
		SET_FLAG(CF, false);
		SET_FLAG(AF, false);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		break;
	case t_XORw:
		SET_FLAG(CF, false);
		SET_FLAG(AF, false);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		break;
	case t_XORd:
		SET_FLAG(CF, false);
		SET_FLAG(AF, false);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		break;


	case t_SHLb:
		if (lf_var2b > 8) SET_FLAG(CF, false);
		else SET_FLAG(CF, (lf_var1b >> (8 - lf_var2b)) & 1);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, ((unsigned int)lf_resb >> 7U) ^ GETFLAG(CF)); /* MSB of result XOR CF. WARNING: This only works because FLAGS_CF == 1 */
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2b & 0x1f));
		break;
	case t_SHLw:
		if (lf_var2b > 16) SET_FLAG(CF, false);
		else SET_FLAG(CF, (lf_var1w >> (16 - lf_var2b)) & 1);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, ((unsigned int)lf_resw >> 15U) ^ GETFLAG(CF)); /* MSB of result XOR CF. WARNING: This only works because FLAGS_CF == 1 */
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2w & 0x1f));
		break;
	case t_SHLd:
		SET_FLAG(CF, (lf_var1d >> (32 - lf_var2b)) & 1);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, ((unsigned int)lf_resd >> 31U) ^ GETFLAG(CF)); /* MSB of result XOR CF. WARNING: This only works because FLAGS_CF == 1 */
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2d & 0x1f));
		break;


	case t_DSHLw:
		SET_FLAG(CF, (lf_var1d >> (32 - lf_var2b)) & 1);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, (lf_resw ^ lf_var1w) & 0x8000);
		DOFLAG_PF;
		break;
	case t_DSHLd:
		SET_FLAG(CF, (lf_var1d >> (32 - lf_var2b)) & 1);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, (lf_resd ^ lf_var1d) & 0x80000000);
		DOFLAG_PF;
		break;


	case t_SHRb:
		SET_FLAG(CF, (lf_var1b >> (lf_var2b - 1)) & 1);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		if ((lf_var2b & 0x1f) == 1) SET_FLAG(OF, (lf_var1b >= 0x80));
		else SET_FLAG(OF, false);
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2b & 0x1f));
		break;
	case t_SHRw:
		SET_FLAG(CF, (lf_var1w >> (lf_var2b - 1)) & 1);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		if ((lf_var2w & 0x1f) == 1) SET_FLAG(OF, (lf_var1w >= 0x8000));
		else SET_FLAG(OF, false);
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2w & 0x1f));
		break;
	case t_SHRd:
		SET_FLAG(CF, (lf_var1d >> (lf_var2b - 1)) & 1);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		if ((lf_var2d & 0x1f) == 1) SET_FLAG(OF, (lf_var1d >= 0x80000000));
		else SET_FLAG(OF, false);
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2d & 0x1f));
		break;


	case t_DSHRw:	/* Hmm this is not correct for shift higher than 16 */
		SET_FLAG(CF, (lf_var1d >> (lf_var2b - 1)) & 1);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, (lf_resw ^ lf_var1w) & 0x8000);
		DOFLAG_PF;
		break;
	case t_DSHRd:
		SET_FLAG(CF, (lf_var1d >> (lf_var2b - 1)) & 1);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, (lf_resd ^ lf_var1d) & 0x80000000);
		DOFLAG_PF;
		break;


	case t_SARb:
		SET_FLAG(CF, (((int8_t)lf_var1b) >> (lf_var2b - 1)) & 1);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2b & 0x1f));
		break;
	case t_SARw:
		SET_FLAG(CF, (((int16_t)lf_var1w) >> (lf_var2b - 1)) & 1);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2w & 0x1f));
		break;
	case t_SARd:
		SET_FLAG(CF, (((int32_t)lf_var1d) >> (lf_var2b - 1)) & 1);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, false);
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2d & 0x1f));
		break;

	case t_INCb:
		SET_FLAG(AF, (lf_resb & 0x0f) == 0);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, (lf_resb == 0x80));
		DOFLAG_PF;
		break;
	case t_INCw:
		SET_FLAG(AF, (lf_resw & 0x0f) == 0);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, (lf_resw == 0x8000));
		DOFLAG_PF;
		break;
	case t_INCd:
		SET_FLAG(AF, (lf_resd & 0x0f) == 0);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, (lf_resd == 0x80000000));
		DOFLAG_PF;
		break;

	case t_DECb:
		SET_FLAG(AF, (lf_resb & 0x0f) == 0x0f);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, (lf_resb == 0x7f));
		DOFLAG_PF;
		break;
	case t_DECw:
		SET_FLAG(AF, (lf_resw & 0x0f) == 0x0f);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, (lf_resw == 0x7fff));
		DOFLAG_PF;
		break;
	case t_DECd:
		SET_FLAG(AF, (lf_resd & 0x0f) == 0x0f);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, (lf_resd == 0x7fffffff));
		DOFLAG_PF;
		break;

	case t_NEGb:
		SET_FLAG(CF, (lf_var1b != 0));
		SET_FLAG(AF, (lf_resb & 0x0f) != 0);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		SET_FLAG(OF, (lf_var1b == 0x80));
		DOFLAG_PF;
		break;
	case t_NEGw:
		SET_FLAG(CF, (lf_var1w != 0));
		SET_FLAG(AF, (lf_resw & 0x0f) != 0);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		SET_FLAG(OF, (lf_var1w == 0x8000));
		DOFLAG_PF;
		break;
	case t_NEGd:
		SET_FLAG(CF, (lf_var1d != 0));
		SET_FLAG(AF, (lf_resd & 0x0f) != 0);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		SET_FLAG(OF, (lf_var1d == 0x80000000));
		DOFLAG_PF;
		break;


	case t_DIV:
	case t_MUL:
		break;

	default:
		LOG(LOG_CPU, LOG_ERROR)("Unhandled flag type %d", (int)lflags.type);
		return 0;
	}
	lflags.type = t_UNKNOWN;
	return reg_flags;
}

void FillFlagsNoCFOF(void) {
	switch (lflags.type) {
	case t_UNKNOWN:
		return;
	case t_ADDb:
		DOFLAG_AF;
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		break;
	case t_ADDw:
		DOFLAG_AF;
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_ADDd:
		DOFLAG_AF;
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;
	case t_ADCb:
		DOFLAG_AF;
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		break;
	case t_ADCw:
		DOFLAG_AF;
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_ADCd:
		DOFLAG_AF;
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;


	case t_SBBb:
		DOFLAG_AF;
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		break;
	case t_SBBw:
		DOFLAG_AF;
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_SBBd:
		DOFLAG_AF;
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;


	case t_SUBb:
	case t_CMPb:
		DOFLAG_AF;
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		break;
	case t_SUBw:
	case t_CMPw:
		DOFLAG_AF;
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_SUBd:
	case t_CMPd:
		DOFLAG_AF;
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;


	case t_ORb:
		SET_FLAG(AF, false);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		break;
	case t_ORw:
		SET_FLAG(AF, false);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_ORd:
		SET_FLAG(AF, false);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;


	case t_TESTb:
	case t_ANDb:
		SET_FLAG(AF, false);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		break;
	case t_TESTw:
	case t_ANDw:
		SET_FLAG(AF, false);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_TESTd:
	case t_ANDd:
		SET_FLAG(AF, false);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;


	case t_XORb:
		SET_FLAG(AF, false);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		break;
	case t_XORw:
		SET_FLAG(AF, false);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_XORd:
		SET_FLAG(AF, false);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;


	case t_SHLb:
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2b & 0x1f));
		break;
	case t_SHLw:
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2w & 0x1f));
		break;
	case t_SHLd:
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2d & 0x1f));
		break;


	case t_DSHLw:
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_DSHLd:
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;


	case t_SHRb:
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2b & 0x1f));
		break;
	case t_SHRw:
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2w & 0x1f));
		break;
	case t_SHRd:
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2d & 0x1f));
		break;


	case t_DSHRw:	/* Hmm this is not correct for shift higher than 16 */
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_DSHRd:
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;


	case t_SARb:
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2b & 0x1f));
		break;
	case t_SARw:
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2w & 0x1f));
		break;
	case t_SARd:
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		SET_FLAG(AF, (lf_var2d & 0x1f));
		break;

	case t_INCb:
		SET_FLAG(AF, (lf_resb & 0x0f) == 0);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		break;
	case t_INCw:
		SET_FLAG(AF, (lf_resw & 0x0f) == 0);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_INCd:
		SET_FLAG(AF, (lf_resd & 0x0f) == 0);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;

	case t_DECb:
		SET_FLAG(AF, (lf_resb & 0x0f) == 0x0f);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		break;
	case t_DECw:
		SET_FLAG(AF, (lf_resw & 0x0f) == 0x0f);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_DECd:
		SET_FLAG(AF, (lf_resd & 0x0f) == 0x0f);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;

	case t_NEGb:
		SET_FLAG(AF, (lf_resb & 0x0f) != 0);
		DOFLAG_ZFb;
		DOFLAG_SFb;
		DOFLAG_PF;
		break;
	case t_NEGw:
		SET_FLAG(AF, (lf_resw & 0x0f) != 0);
		DOFLAG_ZFw;
		DOFLAG_SFw;
		DOFLAG_PF;
		break;
	case t_NEGd:
		SET_FLAG(AF, (lf_resd & 0x0f) != 0);
		DOFLAG_ZFd;
		DOFLAG_SFd;
		DOFLAG_PF;
		break;


	case t_DIV:
	case t_MUL:
		break;

	default:
		LOG(LOG_CPU, LOG_ERROR)("Unhandled flag type %d", (int)lflags.type);
		break;
	}
	lflags.type = t_UNKNOWN;
}

/*void DestroyConditionFlags(void) {
	lflags.type = t_UNKNOWN;
}*/
#endif

PageHandler* jitcache[1024*1024];
PageHandler nojittedtemp;

UINT64 exec_jit_internal() {
	if (cache_initialized == false) {
		//exec_allstep();
		cache_init(true);
		return 0;
	}
	if (CPU_Cycles <= 0)
		return CBRET_NONE;

#if defined(WIN32) && defined(_M_ARM)
	if (!is_win10) {
		if (winrt_warning) {
			LOG_MSG("Dynamic core warning: Windows RT 8.x requires ARMv7 Thumb-2 dynarec core, which is not supported yet.");
			winrt_warning = false;
		}
		exec_allstep();
		return CBRET_NONE;
	}
#endif

	/* Dynamic core is NOT compatible with the way page faults
	 * in the guest are handled in this emulator. Do not use
	 * dynamic core if paging is enabled. Do not comment this
	 * out, even if it happens to work for a minute, a half
	 * hour, a day, because it will turn around and cause
	 * Windows 95 to crash when you've become most comfortable
	 * with the idea that it works. This code cannot handle
	 * the sudden context switch of a page fault and it never
	 * will. Don't do it. You have been warned. */
	if (CPU_STAT_PAGING && !use_dynamic_core_with_paging) {
		if (paging_warning) {
			LOG_MSG("Dynamic core warning: The guest OS/Application has just switched on 80386 paging, which is not supported by the dynamic core. The normal core will be used until paging is switched off again.");
			paging_warning = false;
		}

		exec_1step();
		return CBRET_NONE;
	}

	for (;;) {
		dosbox_allow_nonrecursive_page_fault = false;
		// Determine the linear address of CS:EIP
		PhysPt ip_point = SegPhys(cs) + reg_eip;
#if C_HEAVY_DEBUG
		if (DEBUG_HeavyIsBreakpoint()) return (Bits)debugCallback;
#endif

		CodePageHandlerDynRec* chandler = nullptr;
#if 1
		if (jitcache[(ip_point >> 12)] == 0 || jitcache[(ip_point >> 12)] == (&nojittedtemp)) {
			jitcache[(ip_point >> 12)] = (PageHandler*)new CodePageHandlerDynRec();
			((CodePageHandlerDynRec*)jitcache[(ip_point >> 12)])->SetupAt((ip_point >> 12), &nojittedtemp);
		}

		chandler = (CodePageHandlerDynRec*)jitcache[(ip_point >> 12)];
#else
		// see if the current page is present and contains code
		if (GCC_UNLIKELY(MakeCodePage(ip_point, chandler))) {
			// page not present, throw the exception
			CPU_Exception(cpu.exception.which, cpu.exception.error);
			continue;
		}
#endif

		// page doesn't contain code or is special
		if (GCC_UNLIKELY(!chandler)) {
			dosbox_allow_nonrecursive_page_fault = true;
			exec_allstep();
			return CBRET_NONE;
		}

		// find correct Dynamic Block to run
		CacheBlockDynRec* block = chandler->FindCacheBlock(ip_point & 4095);
		if (!block) {
			// no block found, thus translate the instruction stream
			// unless the instruction is known to be modified
			if (!chandler->invalidation_map || (chandler->invalidation_map[ip_point & 4095] < 4)) {
				// translate up to 32 instructions
				block = CreateCacheBlock(chandler, ip_point, 32);
			}
			else {
				dosbox_allow_nonrecursive_page_fault = true;
				// let the normal core handle this instruction to avoid zero-sized blocks
				cpu_cycles_count_t old_cycles = CPU_Cycles;
				CPU_Cycles = 1;
				CPU_CycleLeft += old_cycles;
				exec_allstep();
				Bits nc_retcode = 0;//CPU_Core_Normal_Run();
				if (!nc_retcode) {
					CPU_Cycles = old_cycles - 1;
					CPU_CycleLeft -= old_cycles;
					continue;
				}
				return nc_retcode;
			}
		}

	run_block:
		cache.block.running = nullptr;
		// now we're ready to run the dynamic code block
//		BlockReturnDynRec ret=((BlockReturnDynRec (*)(void))(block->cache.start))();
		BlockReturnDynRec ret = core_dynrec.runcode(block->cache.xstart);

		if (sizeof(CPU_Cycles) > 4) {
			// HACK: All dynrec cores for each processor assume CPU_Cycles is 32-bit wide.
			//       The purpose of this hack is to sign-extend the lower 32 bits so that
			//       when CPU_Cycles goes negative it doesn't suddenly appear as a very
			//       large integer value.
			//
			//       This hack is needed for dynrec to work on x86_64 targets.
			CPU_Cycles = (Bits)((int32_t)CPU_Cycles);
		}

		switch (ret) {
		case BR_Iret:
#if C_DEBUG
#if C_HEAVY_DEBUG
			if (DEBUG_HeavyIsBreakpoint()) return (Bits)debugCallback;
#endif
#endif
			if (!GETFLAG(TF)) {
				if (GETFLAG(IF) && PIC_IRQCheck) return CBRET_NONE;
				break;
			}
			// trapflag is set, switch to the trap-aware decoder
			cpudecoder = CPU_Core_Dynrec_Trap_Run;
			return CBRET_NONE;

		case BR_Normal:
			// the block was exited due to a non-predictable control flow
			// modifying instruction (like ret) or some nontrivial cpu state
			// changing instruction (for example switch to/from pmode),
			// or the maximum number of instructions to translate was reached
#if C_DEBUG
#if C_HEAVY_DEBUG
			if (DEBUG_HeavyIsBreakpoint()) return (Bits)debugCallback;
#endif
#endif
			break;

		case BR_Cycles:
			// cycles went negative, return from the core to handle
			// external events, schedule the pic...
#if C_DEBUG
#if C_HEAVY_DEBUG
			if (DEBUG_HeavyIsBreakpoint()) return (Bits)debugCallback;
#endif
#endif
			return CBRET_NONE;

		case BR_CallBack:
			// the callback code is executed in dosbox-x.conf, return the callback number
			FillFlags();
			return (Bits)core_dynrec.callback;

		case BR_SMCBlock:
			//			LOG_MSG("selfmodification of running block at %x:%x",SegValue(cs),reg_eip);
			cpu.exception.which = 0;
			// fallthrough, let the normal core handle the block-modifying instruction
		case BR_Opcode:
#if (C_DEBUG)
		case BR_OpcodeFull:
#endif
			// some instruction has been encountered that could not be translated
			// (thus it is not part of the code block), the normal core will
			// handle this instruction
			CPU_CycleLeft += CPU_Cycles;
			CPU_Cycles = 1;
			dosbox_allow_nonrecursive_page_fault = true;
			exec_1step();
			return CBRET_NONE;

		case BR_Link1:
		case BR_Link2:
			block = LinkBlocks(ret);
			if (block) goto run_block;
			break;

		case BR_Trap:
			// trapflag is set, switch to the trap-aware decoder
#if C_DEBUG
#if C_HEAVY_DEBUG
			if (DEBUG_HeavyIsBreakpoint()) {
				return debugCallback;
			}
#endif
#endif
			cpudecoder = CPU_Core_Dynrec_Trap_Run;
			return CBRET_NONE;

		default:
			E_Exit("Invalid return code %d", ret);
		}
	}
	return CBRET_NONE;
	//NtFreeVirtualMemory(GetCurrentProcess(),(PVOID*) &jitptx, 0, 0x8000);
}

Bits CPU_Core_Dynrec_Trap_Run(void) {
	exec_1step();

	return CBRET_NONE;
}

void CPU_Core_Dynrec_Init(void) {
#if defined(WIN32) && defined(_M_ARM)
	is_win10 = IsWin10OrGreater();
#endif
}

void CPU_Core_Dynrec_Cache_Init(bool enable_cache) {
	// Initialize code cache and dynamic blocks
	cache_init(enable_cache);
}

void CPU_Core_Dynrec_Cache_Close(void) {
	cache_close();
}

void CPU_Core_Dynrec_Cache_Reset(void) {
	cache_reset();
}

bool CPU_STI(void) {
	return true;
}

bool CPU_CLI(void) {
	return true;
}

bool CPU_LMSW(Bitu word) {
	UINT32 cr0;
	if (!CPU_STAT_PM || CPU_STAT_CPL == 0) {
		cr0 = CPU_CR0;
		CPU_CR0 &= ~0xe; /* can't switch back from protected mode */
		CPU_CR0 |= (word & 0xf);	/* TS, EM, MP, PE */
		if (!(cr0 & CPU_CR0_PE) && (word & CPU_CR0_PE)) {
			change_pm(1);	/* switch to protected mode */
		}
		return false;
	}
	VERBOSE(("LMSW: CPL(%d) != 0", CPU_STAT_CPL));
	EXCEPTION(GP_EXCEPTION, 0);
	return true;
}

uint8_t IO_ReadB(Bitu port) {
	return cpu_in(port);
}
uint16_t IO_ReadW(Bitu port) {
	return cpu_in_w(port);
}
uint32_t IO_ReadD(Bitu port) {
	return cpu_in_d(port);
}

void CPU_JMP(bool use32, Bitu selector, Bitu offset, uint32_t oldeip) {
	descriptor_t sd;
	UINT32 new_ip;
	UINT16 new_cs;
	UINT16 sreg;
	CPU_WORKCLOCK(11);
	new_ip = offset;
	new_cs = selector;
	if (!use32) {
		//16bit
		if (!CPU_STAT_PM || CPU_STAT_VM86) {
			/* Real mode or VM86 mode */
			/* check new instrunction pointer with new code segment */
			load_segreg(CPU_CS_INDEX, new_cs, &sreg, &sd, GP_EXCEPTION);
			if (new_ip > sd.u.seg.limit) {
				EXCEPTION(GP_EXCEPTION, 0);
			}
			LOAD_SEGREG(CPU_CS_INDEX, new_cs);
			CPU_EIP = new_ip;
		}
		else {
			/* Protected mode */
			JMPfar_pm(new_cs, new_ip);
		}
	}
	else {
		//32bit
		if (!CPU_STAT_PM || CPU_STAT_VM86) {
			/* Real mode or VM86 mode */
			/* check new instrunction pointer with new code segment */
			load_segreg(CPU_CS_INDEX, new_cs, &sreg, &sd, GP_EXCEPTION);
			if (new_ip > sd.u.seg.limit) {
				EXCEPTION(GP_EXCEPTION, 0);
			}
			LOAD_SEGREG(CPU_CS_INDEX, new_cs);
			CPU_EIP = new_ip;
		}
		else {
			/* Protected mode */
			JMPfar_pm(new_cs, new_ip);
		}
	}
}

void CPU_CALL(bool use32, Bitu selector, Bitu offset, uint32_t oldeip) {
	descriptor_t sd;
	UINT32 new_ip;
	UINT16 new_cs;
	UINT16 sreg;
	CPU_WORKCLOCK(13);
	new_ip = offset;
	new_cs = selector;
	if (!use32) {
		//16bit
		if (!CPU_STAT_PM || CPU_STAT_VM86) {
			/* Real mode or VM86 mode */
			CPU_SET_PREV_ESP();
			load_segreg(CPU_CS_INDEX, new_cs, &sreg, &sd, GP_EXCEPTION);
			if (new_ip > sd.u.seg.limit) {
				EXCEPTION(GP_EXCEPTION, 0);
			}

			PUSH0_16(CPU_CS);
			PUSH0_16(CPU_IP);

			LOAD_SEGREG(CPU_CS_INDEX, new_cs);
			CPU_EIP = new_ip;
			CPU_CLEAR_PREV_ESP();
		}
		else {
			/* Protected mode */
			CALLfar_pm(new_cs, new_ip);
		}
	}
	else {
		//32bit
		if (!CPU_STAT_PM || CPU_STAT_VM86) {
			/* Real mode or VM86 mode */
			CPU_SET_PREV_ESP();
			load_segreg(CPU_CS_INDEX, new_cs, &sreg, &sd, GP_EXCEPTION);
			if (new_ip > sd.u.seg.limit) {
				EXCEPTION(GP_EXCEPTION, 0);
			}

			PUSH0_32(CPU_CS);
			PUSH0_32(CPU_EIP);

			LOAD_SEGREG(CPU_CS_INDEX, new_cs);
			CPU_EIP = new_ip;
			CPU_CLEAR_PREV_ESP();
		}
		else {
			/* Protected mode */
			CALLfar_pm(new_cs, new_ip);
		}
	}
}

bool CPU_LTR(Bitu selector) {
	UINT32 madr;
	UINT16 src;
	if (CPU_STAT_PM && !CPU_STAT_VM86) {
		if (CPU_STAT_CPL == 0) {
			load_tr(selector);
			return false;
		}
		VERBOSE(("LTR: CPL(%d) != 0", CPU_STAT_CPL));
		EXCEPTION(GP_EXCEPTION, 0);
		return true;
	}
	VERBOSE(("LTR: real-mode or VM86"));
	EXCEPTION(UD_EXCEPTION, 0);
	return true;
}

bool CPU_POPF(Bitu use32) {
	UINT32 flags, mask;
	if (!use32) {
		//16bit
		CPU_WORKCLOCK(3);
		CPU_SET_PREV_ESP();
		if (!CPU_STAT_PM) {
			/* Real Mode */
			POP0_16(flags);
			mask = I_FLAG | IOPL_FLAG;
		}
		else if (!CPU_STAT_VM86) {
			/* Protected Mode */
			POP0_16(flags);
			if (CPU_STAT_CPL == 0) {
				mask = I_FLAG | IOPL_FLAG;
			}
			else if (CPU_STAT_CPL <= CPU_STAT_IOPL) {
				mask = I_FLAG;
			}
			else {
				mask = 0;
			}
		}
		else if (CPU_STAT_IOPL == CPU_IOPL3) {
			/* Virtual-8086 Mode, IOPL == 3 */
			POP0_16(flags);
			mask = I_FLAG;
		}
		else {
			EXCEPTION(GP_EXCEPTION, 0);
			flags = 0;
			mask = 0;
			/* compiler happy */
			return true;
		}
		set_eflags(flags, mask);
		CPU_CLEAR_PREV_ESP();
		IRQCHECKTERM();
		return false;
	}
	else {
		//32bit
		CPU_WORKCLOCK(3);
		CPU_SET_PREV_ESP();
		if (!CPU_STAT_PM) {
			/* Real Mode */
			POP0_32(flags);
			flags &= ~(RF_FLAG | VIF_FLAG | VIP_FLAG);
			mask = I_FLAG | IOPL_FLAG | RF_FLAG | VIF_FLAG | VIP_FLAG;
		}
		else if (!CPU_STAT_VM86) {
			/* Protected Mode */
			POP0_32(flags);
			flags &= ~RF_FLAG;
			if (CPU_STAT_CPL == 0) {
				flags &= ~(VIP_FLAG | VIF_FLAG);
				mask = I_FLAG | IOPL_FLAG | RF_FLAG | VIF_FLAG | VIP_FLAG;
			}
			else if (CPU_STAT_CPL <= CPU_STAT_IOPL) {
				flags &= ~(VIP_FLAG | VIF_FLAG);
				mask = I_FLAG | RF_FLAG | VIF_FLAG | VIP_FLAG;
			}
			else {
				mask = RF_FLAG;
			}
		}
		else if (CPU_STAT_IOPL == CPU_IOPL3) {
			/* Virtual-8086 Mode, IOPL == 3 */
			POP0_32(flags);
			mask = I_FLAG;
		}
		else {
			/* Virtual-8086 Mode, IOPL != 3 */
#if defined(USE_VME)
			if (CPU_CR4 & CPU_CR4_VME) {
				if ((CPU_EFLAG & VIP_FLAG) || (CPU_EFLAG & T_FLAG)) {
					EXCEPTION(GP_EXCEPTION, 0);
					flags = 0;
					mask = 0;
					return true;
				}
				else {
					POP0_32(flags);
					flags = (flags & ~VIF_FLAG) | ((flags & I_FLAG) << 10); // IF → VIFにコピー
					mask = I_FLAG | IOPL_FLAG; // IF, IOPLは変更させない
				}
			}
			else {
				EXCEPTION(GP_EXCEPTION, 0);
				flags = 0;
				mask = 0;
				return true;
			}
#else
			EXCEPTION(GP_EXCEPTION, 0);
			flags = 0;
			mask = 0;
			/* compiler happy */
			return true;
#endif
		}
		set_eflags(flags, mask);
		CPU_CLEAR_PREV_ESP();
		IRQCHECKTERM();
		return false;
	}
	return false;
}

void IO_WriteB(Bitu port, uint8_t val) {
	cpu_out(port,val);
	return;
}
void IO_WriteW(Bitu port, uint16_t val) {
	cpu_out_w(port, val);
	return;
}
void IO_WriteD(Bitu port, uint32_t val) {
	cpu_out_d(port, val);
	return;
}

Bitu CPU_STR(void) {
	return CPU_TR;
}

Bitu CPU_SLDT(void) {
	return CPU_LDTR;
}

void CPU_LGDT(Bitu limit, Bitu base) {
	if (!CPU_STAT_PM || (CPU_STAT_CPL == 0 && !CPU_STAT_VM86)) {
		CPU_WORKCLOCK(11);
		if (!CPU_INST_OP32) {
			base &= 0x00ffffff;
		}

#if defined(MORE_DEBUG)
		gdtr_dump(base, limit);
#endif

		CPU_GDTR_BASE = base;
		CPU_GDTR_LIMIT = limit;
		return;
	}
	EXCEPTION(GP_EXCEPTION, 0);
}
void CPU_LIDT(Bitu limit, Bitu base) {
	if (!CPU_STAT_PM || (CPU_STAT_CPL == 0 && !CPU_STAT_VM86)) {
		CPU_WORKCLOCK(11);
		if (!CPU_INST_OP32) {
			base &= 0x00ffffff;
		}

#if defined(MORE_DEBUG)
		idtr_dump(base, limit);
#endif

		CPU_IDTR_BASE = base;
		CPU_IDTR_LIMIT = limit;
		return;
	}
	EXCEPTION(GP_EXCEPTION, 0);
}

Bitu CPU_SGDT_base(void) {
	return CPU_GDTR_BASE;
}
Bitu CPU_SGDT_limit(void) {
	return CPU_GDTR_LIMIT;
}

Bitu CPU_SIDT_base(void) {
	return CPU_IDTR_BASE;
}
Bitu CPU_SIDT_limit(void) {
	return CPU_IDTR_LIMIT;
}

bool CPU_LLDT(Bitu selector) {
	int exc = GP_EXCEPTION;
	selector_t sel;
	int rv;
	if (CPU_STAT_PM && !CPU_STAT_VM86) {
		if (CPU_STAT_CPL == 0) {

			memset(&sel, 0, sizeof(sel));

			rv = parse_selector(&sel, selector);
			if (rv < 0 || sel.ldt) {
				if (rv == -2) {
					/* null segment */
					VERBOSE(("load_ldtr: null segment"));
					CPU_LDTR = 0;
					memset(&CPU_LDTR_DESC, 0, sizeof(CPU_LDTR_DESC));
					return false;
				}
				EXCEPTION(exc, sel.selector);
				return true;
			}

			/* check descriptor type */
			if (!SEG_IS_SYSTEM(&sel.desc)
				|| (sel.desc.type != CPU_SYSDESC_TYPE_LDT)) {
				EXCEPTION(exc, sel.selector);
				return true;
			}

			/* not present */
			rv = selector_is_not_present(&sel);
			if (rv < 0) {
				EXCEPTION((exc == TS_EXCEPTION) ? TS_EXCEPTION : NP_EXCEPTION, sel.selector);
				return true;
			}

#if defined(MORE_DEBUG)
			ldtr_dump(sel.desc.u.seg.segbase, sel.desc.u.seg.limit);
#endif

			CPU_LDTR = sel.selector;
			CPU_LDTR_DESC = sel.desc;
			return false;
		}
		VERBOSE(("LLDT: CPL(%d) != 0", CPU_STAT_CPL));
		EXCEPTION(GP_EXCEPTION, 0);
		return true;
	}
	VERBOSE(("LLDT: real-mode or VM86"));
	EXCEPTION(UD_EXCEPTION, 0);
	return true;
}

void CPU_VERW(Bitu selector) {
	selector_t sel;
	int rv;
	if (CPU_STAT_PM && !CPU_STAT_VM86) {
		rv = parse_selector(&sel, selector);
		if (rv < 0) {
			CPU_FLAGL &= ~Z_FLAG;
			return;
		}

		/* system segment || code segment */
		if (SEG_IS_SYSTEM(&sel.desc) || SEG_IS_CODE(&sel.desc)) {
			CPU_FLAGL &= ~Z_FLAG;
			return;
		}
		/* data segment is not writable */
		if (!SEG_IS_WRITABLE_DATA(&sel.desc)) {
			CPU_FLAGL &= ~Z_FLAG;
			return;
		}
		/* privilege level */
		if ((CPU_STAT_CPL > sel.desc.dpl) || (sel.rpl > sel.desc.dpl)) {
			CPU_FLAGL &= ~Z_FLAG;
			return;
		}

		CPU_FLAGL |= Z_FLAG;
		return;
	}
	VERBOSE(("VERW: real-mode or VM86"));
	EXCEPTION(UD_EXCEPTION, 0);
}

void CPU_SetFlags(Bitu word, Bitu mask) {
	set_eflags(word, mask);
}

void E_Exit(const char* format, ...) {
}

bool mem_unalignedreadw_checked(PhysPt address, uint16_t* val) {
	uint8_t rval1, rval2;
	if (mem_readb_checked(address + 0, &rval1)) return true;
	if (mem_readb_checked(address + 1, &rval2)) return true;
	*val = (uint16_t)(rval1 | (rval2 << 8));
	return false;
}

bool mem_unalignedreadd_checked(PhysPt address, uint32_t* val) {
	uint8_t rval1, rval2, rval3, rval4;
	if (mem_readb_checked(address + 0, &rval1)) return true;
	if (mem_readb_checked(address + 1, &rval2)) return true;
	if (mem_readb_checked(address + 2, &rval3)) return true;
	if (mem_readb_checked(address + 3, &rval4)) return true;
	*val = (uint32_t)(rval1 | (rval2 << 8) | (rval3 << 16) | (rval4 << 24));
	return false;
}

bool mem_unalignedwritew_checked(PhysPt address, uint16_t val) {
	if (mem_writeb_checked(address, (uint8_t)(val & 0xff))) return true;
	val >>= 8;
	if (mem_writeb_checked(address + 1, (uint8_t)(val & 0xff))) return true;
	return false;
}

bool mem_unalignedwrited_checked(PhysPt address, uint32_t val) {
	if (mem_writeb_checked(address, (uint8_t)(val & 0xff))) return true;
	val >>= 8;
	if (mem_writeb_checked(address + 1, (uint8_t)(val & 0xff))) return true;
	val >>= 8;
	if (mem_writeb_checked(address + 2, (uint8_t)(val & 0xff))) return true;
	val >>= 8;
	if (mem_writeb_checked(address + 3, (uint8_t)(val & 0xff))) return true;
	return false;
}

bool CPU_IO_Exception(Bitu port, Bitu len) {
	UINT off;
	UINT16 map;
	UINT16 mask;

	if (CPU_STAT_IOLIMIT == 0) {
		VERBOSE(("check_io: CPU_STAT_IOLIMIT == 0 (port = %04x, len = %d)", port, len));
		EXCEPTION(GP_EXCEPTION, 0);
		return true;
	}

	if ((port + len) / 8 >= CPU_STAT_IOLIMIT) {
		VERBOSE(("check_io: out of range: CPU_STAT_IOLIMIT(%08x) (port = %04x, len = %d)", CPU_STAT_IOLIMIT, port, len));
		EXCEPTION(GP_EXCEPTION, 0);
		return true;
	}

	off = port / 8;
	mask = ((1 << len) - 1) << (port % 8);
	map = cpu_kmemoryread_w(CPU_STAT_IOADDR + off);
	if (map & mask) {
		VERBOSE(("check_io: (bitmap(0x%04x) & bit(0x%04x)) != 0 (CPU_STAT_IOADDR=0x%08x, offset=0x%04x, port = 0x%04x, len = %d)", map, mask, CPU_STAT_IOADDR, off, port, len));
		EXCEPTION(GP_EXCEPTION, 0);
		return true;
	}
	return false;
}

void CPU_VERR(Bitu selector) {
	selector_t sel;
	int rv;
	if (CPU_STAT_PM && !CPU_STAT_VM86) {
		rv = parse_selector(&sel, selector);
		if (rv < 0) {
			CPU_FLAGL &= ~Z_FLAG;
			return;
		}

		/* system segment */
		if (SEG_IS_SYSTEM(&sel.desc)) {
			CPU_FLAGL &= ~Z_FLAG;
			return;
		}

		/* data or non-conforming code segment */
		if ((SEG_IS_DATA(&sel.desc)
			|| !SEG_IS_CONFORMING_CODE(&sel.desc))) {
			if ((sel.desc.dpl < CPU_STAT_CPL)
				|| (sel.desc.dpl < sel.rpl)) {
				CPU_FLAGL &= ~Z_FLAG;
				return;
			}
		}
		/* code segment is not readable */
		if (SEG_IS_CODE(&sel.desc)
			&& !SEG_IS_READABLE_CODE(&sel.desc)) {
			CPU_FLAGL &= ~Z_FLAG;
			return;
		}

		CPU_FLAGL |= Z_FLAG;
		return;
	}
	VERBOSE(("VERR: real-mode or VM86"));
	EXCEPTION(UD_EXCEPTION, 0);
}

void CPU_Push16(uint16_t value) {
	CPU_WORKCLOCK(3); PUSH0_16(value);
}
void CPU_Push32(uint32_t value) {
	CPU_WORKCLOCK(3); PUSH0_32(value);
}

Bitu CPU_SMSW(void) {
	return CPU_CR0;
}

void CPU_RET(bool use32, Bitu bytes, uint32_t oldeip) {
	descriptor_t sd;
	UINT32 new_ip;
	UINT32 new_cs;
	UINT16 sreg;
	if (!use32) {
		//16bit
		CPU_WORKCLOCK(15);
		if (!CPU_STAT_PM || CPU_STAT_VM86) {
			/* Real mode or VM86 mode */
			CPU_SET_PREV_ESP();
			POP0_16(new_ip);
			POP0_16(new_cs);

			/* check new instrunction pointer with new code segment */
			load_segreg(CPU_CS_INDEX, new_cs, &sreg, &sd, GP_EXCEPTION);
			if (new_ip > sd.u.seg.limit) {
				EXCEPTION(GP_EXCEPTION, 0);
			}

			LOAD_SEGREG(CPU_CS_INDEX, new_cs);
			CPU_EIP = new_ip;
			CPU_CLEAR_PREV_ESP();
		}
		else {
			/* Protected mode */
			RETfar_pm(0);
		}
	}
	else {
		//32bit
		CPU_WORKCLOCK(15);
		if (!CPU_STAT_PM || CPU_STAT_VM86) {
			/* Real mode or VM86 mode */
			CPU_SET_PREV_ESP();
			POP0_32(new_ip);
			POP0_32(new_cs);

			/* check new instrunction pointer with new code segment */
			load_segreg(CPU_CS_INDEX, (UINT16)new_cs, &sreg, &sd, GP_EXCEPTION);
			if (new_ip > sd.u.seg.limit) {
				EXCEPTION(GP_EXCEPTION, 0);
			}

			LOAD_SEGREG(CPU_CS_INDEX, (UINT16)new_cs);
			CPU_EIP = new_ip;
			CPU_CLEAR_PREV_ESP();
		}
		else {
			/* Protected mode */
			RETfar_pm(0);
		}
	}
}

uint16_t CPU_Pop16(void) {
	uint16_t Retvalue = 0;
	CPU_WORKCLOCK(5); POP0_16(Retvalue);
	return Retvalue;
}
uint32_t CPU_Pop32(void) {
	uint32_t Retvalue = 0;
	CPU_WORKCLOCK(5); POP0_32(Retvalue);
	return Retvalue;
}

/* dynamic core, policy, method, and flags.
 * We're going to make dynamic core more flexible, AND make sure
 * that both dynx86 and dynrec are using common memory mapping
 * code to reduce copy-pasta */
dyncore_alloc_t         dyncore_alloc = DYNCOREALLOC_NONE;
dyncore_method_t        dyncore_method = DYNCOREM_NONE;
dyncore_flags_t         dyncore_flags = 0;

void CPU_Interrupt(Bitu num, Bitu type, uint32_t oldeip) {
	CPU_EIP = oldeip;
	switch (type) {
	case CPU_INT_SOFTWARE:
		INTERRUPT(num, INTR_TYPE_SOFTINTR);
		break;
	case CPU_INT_EXCEPTION:
		INTERRUPT(num, INTR_TYPE_EXCEPTION);
		break;
	default:
		INTERRUPT(num, INTR_TYPE_EXTINTR);
		break;
	}
}

bool CPU_WRITE_CRX(Bitu cr, Bitu value) {
	UINT32 op, src;
	UINT32 reg;
	int idx;

	CPU_WORKCLOCK(11);
	if (CPU_STAT_PM && (CPU_STAT_VM86 || CPU_STAT_CPL != 0)) {
		VERBOSE(("MOV_CdRd: VM86(%s) or CPL(%d) != 0", CPU_STAT_VM86 ? "true" : "false", CPU_STAT_CPL));
		EXCEPTION(GP_EXCEPTION, 0);
		return true;
	}

	src = *(reg32_b20[op]);
	idx = (op >> 3) & 7;

	switch (idx) {
	case 0: /* CR0 */
		/*
		 * 0 = PE (protect enable)
		 * 1 = MP (monitor coprocesser)
		 * 2 = EM (emulation)
		 * 3 = TS (task switch)
		 * 4 = ET (extend type, FPU present = 1)
		 * 5 = NE (numeric error)
		 * 16 = WP (write protect)
		 * 18 = AM (alignment mask)
		 * 29 = NW (not write-through)
		 * 30 = CD (cache diable)
		 * 31 = PG (pageing)
		 */

		 /* 下巻 p.182 割り込み 13 - 一般保護例外 */
		if ((src & (CPU_CR0_PE | CPU_CR0_PG)) == (UINT32)CPU_CR0_PG) {
			EXCEPTION(GP_EXCEPTION, 0);
			return true;
		}
		if ((src & (CPU_CR0_NW | CPU_CR0_CD)) == CPU_CR0_NW) {
			EXCEPTION(GP_EXCEPTION, 0);
			return true;
		}

		reg = CPU_CR0;
		src &= CPU_CR0_ALL;
#if defined(USE_FPU)
		if (i386cpuid.cpu_feature & CPU_FEATURE_FPU) {
			src |= CPU_CR0_ET;	/* FPU present */
			//src &= ~CPU_CR0_EM;
		}
		else {
			src |= CPU_CR0_EM | CPU_CR0_NE;
			src &= ~(CPU_CR0_MP | CPU_CR0_ET);
		}
#else
		src |= CPU_CR0_EM | CPU_CR0_NE;
		src &= ~(CPU_CR0_MP | CPU_CR0_ET);
#endif
		CPU_CR0 = src;
		VERBOSE(("MOV_CdRd: %04x:%08x: cr0: 0x%08x <- 0x%08x(%s)", CPU_CS, CPU_PREV_EIP, reg, CPU_CR0, reg32_str[op & 7]));

		if ((reg ^ CPU_CR0) & (CPU_CR0_PE | CPU_CR0_PG)) {
			tlb_flush_all();
		}
		if ((reg ^ CPU_CR0) & CPU_CR0_PE) {
			if (CPU_CR0 & CPU_CR0_PE) {
				change_pm(1);
			}
		}
		if ((reg ^ CPU_CR0) & CPU_CR0_PG) {
			if (CPU_CR0 & CPU_CR0_PG) {
				change_pg(1);
			}
			else {
				change_pg(0);
			}
		}
		if ((reg ^ CPU_CR0) & CPU_CR0_PE) {
			if (!(CPU_CR0 & CPU_CR0_PE)) {
				change_pm(0);
			}
		}

		CPU_STAT_WP = (CPU_CR0 & CPU_CR0_WP) ? 0x10 : 0;
		break;

	case 2: /* CR2 */
		reg = CPU_CR2;
		CPU_CR2 = src;	/* page fault linear address */
		VERBOSE(("MOV_CdRd: %04x:%08x: cr2: 0x%08x <- 0x%08x(%s)", CPU_CS, CPU_PREV_EIP, reg, CPU_CR2, reg32_str[op & 7]));
		break;

	case 3: /* CR3 */
		/*
		 * 31-12 = page directory base
		 * 4 = PCD (page level cache diable)
		 * 3 = PWT (page level write throgh)
		 */
		reg = CPU_CR3;
		set_cr3(src);
		VERBOSE(("MOV_CdRd: %04x:%08x: cr3: 0x%08x <- 0x%08x(%s)", CPU_CS, CPU_PREV_EIP, reg, CPU_CR3, reg32_str[op & 7]));
		break;

	case 4: /* CR4 */
		/*
		 * 10 = OSXMMEXCPT (support non masking exception by OS)
		 * 9 = OSFXSR (support FXSAVE, FXRSTOR by OS)
		 * 8 = PCE (performance monitoring counter enable)
		 * 7 = PGE (page global enable)
		 * 6 = MCE (machine check enable)
		 * 5 = PAE (physical address extention)
		 * 4 = PSE (page size extention)
		 * 3 = DE (debug extention)
		 * 2 = TSD (time stamp diable)
		 * 1 = PVI (protected mode virtual interrupt)
		 * 0 = VME (VM8086 mode extention)
		 */
		reg = 0		/* allow bit */
#if (CPU_FEATURES & CPU_FEATURE_PGE) == CPU_FEATURE_PGE
			| CPU_CR4_PGE
#endif
#if (CPU_FEATURES & CPU_FEATURE_VME) == CPU_FEATURE_VME
			| CPU_CR4_PVI | CPU_CR4_VME
#endif
#if (CPU_FEATURES & CPU_FEATURE_FXSR) == CPU_FEATURE_FXSR
			| CPU_CR4_OSFXSR
#endif
#if (CPU_FEATURES & CPU_FEATURE_SSE) == CPU_FEATURE_SSE
			| CPU_CR4_OSXMMEXCPT
#endif
			| CPU_CR4_PCE
			;
		if (src & ~reg) {
			//if (src & 0xfffffc00) {
			if (src & 0xfffff800) {
				EXCEPTION(GP_EXCEPTION, 0);
				return true;
			}
			if ((src & ~reg) != CPU_CR4_DE) { // XXX: debug extentionは警告しない
				ia32_warning("MOV_CdRd: CR4 <- 0x%08x", src);
			}
		}

		reg = CPU_CR4;
		CPU_CR4 = src;
		VERBOSE(("MOV_CdRd: %04x:%08x: cr4: 0x%08x <- 0x%08x(%s)", CPU_CS, CPU_PREV_EIP, reg, CPU_CR4, reg32_str[op & 7]));

		if ((reg ^ CPU_CR4) & (CPU_CR4_PSE | CPU_CR4_PGE | CPU_CR4_PAE | CPU_CR4_PVI | CPU_CR4_VME | CPU_CR4_OSFXSR | CPU_CR4_OSXMMEXCPT)) {
			tlb_flush_all();
		}
		break;

	default:
		ia32_panic("MOV_CdRd: CR reg index (%d)", idx);
		/*NOTREACHED*/
		return true;
		break;
	}
	return false;
}

void CPU_ENTER(bool use32, Bitu bytes, Bitu level) {
	UINT32 sp, bp;
	UINT32 new_bp;
	UINT32 val;
	UINT16 dimsize = bytes;
	if (!use32) {
		//16bit
		level &= 0x1f;

		CPU_SET_PREV_ESP();
		PUSH0_16(CPU_BP);
		if (level == 0) {			/* enter level=0 */
			CPU_WORKCLOCK(11);
			CPU_BP = CPU_SP;
			if (!CPU_STAT_SS32) {
				CPU_SP -= dimsize;
			}
			else {
				CPU_ESP -= dimsize;
			}
		}
		else {
			--level;
			if (level == 0) {		/* enter level=1 */
				CPU_WORKCLOCK(15);
				sp = CPU_SP;
				PUSH0_16(sp);
				CPU_BP = (UINT16)sp;
				if (!CPU_STAT_SS32) {
					CPU_SP -= dimsize;
				}
				else {
					CPU_ESP -= dimsize;
				}
			}
			else {			/* enter level=2-31 */
				CPU_WORKCLOCK(12 + level * 4);
				if (!CPU_STAT_SS32) {
					bp = CPU_BP;
					new_bp = CPU_SP;
					while (level--) {
						bp -= 2;
						CPU_SP -= 2;
						val = cpu_vmemoryread_w(CPU_SS_INDEX, bp);
						cpu_vmemorywrite_w(CPU_SS_INDEX, CPU_SP, (UINT16)val);
					}
					REGPUSH0(new_bp);
					CPU_BP = new_bp;
					CPU_SP -= dimsize;
				}
				else {
					bp = CPU_EBP;
					new_bp = CPU_SP;
					while (level--) {
						bp -= 2;
						CPU_ESP -= 2;
						val = cpu_vmemoryread_w(CPU_SS_INDEX, bp);
						cpu_vmemorywrite_w(CPU_SS_INDEX, CPU_ESP, (UINT16)val);
					}
					REGPUSH0_16_32(new_bp);
					CPU_BP = new_bp;
					CPU_ESP -= dimsize;
				}
			}
		}
		CPU_CLEAR_PREV_ESP();
	}
	else {
		//32bit
		level &= 0x1f;

		CPU_SET_PREV_ESP();
		PUSH0_32(CPU_EBP);
		if (level == 0) {			/* enter level=0 */
			CPU_WORKCLOCK(11);
			CPU_EBP = CPU_ESP;
			if (!CPU_STAT_SS32) {
				CPU_SP -= dimsize;
			}
			else {
				CPU_ESP -= dimsize;
			}
		}
		else {
			--level;
			if (level == 0) {		/* enter level=1 */
				CPU_WORKCLOCK(15);
				sp = CPU_ESP;
				PUSH0_32(sp);
				CPU_EBP = sp;
				if (CPU_STAT_SS32) {
					CPU_ESP -= dimsize;
				}
				else {
					CPU_SP -= dimsize;
				}
			}
			else {			/* enter level=2-31 */
				CPU_WORKCLOCK(12 + level * 4);
				if (CPU_STAT_SS32) {
					bp = CPU_EBP;
					new_bp = CPU_ESP;
					while (level--) {
						bp -= 4;
						CPU_ESP -= 4;
						val = cpu_vmemoryread_d(CPU_SS_INDEX, bp);
						cpu_vmemorywrite_d(CPU_SS_INDEX, CPU_ESP, val);
					}
					REGPUSH0_32(new_bp);
					CPU_EBP = new_bp;
					CPU_ESP -= dimsize;
				}
				else {
					bp = CPU_BP;
					new_bp = CPU_ESP;
					while (level--) {
						bp -= 4;
						CPU_SP -= 4;
						val = cpu_vmemoryread_d(CPU_SS_INDEX, bp);
						cpu_vmemorywrite_d(CPU_SS_INDEX, CPU_SP, val);
					}
					REGPUSH0_32_16(new_bp);
					CPU_EBP = new_bp;
					CPU_SP -= dimsize;
				}
			}
		}
		CPU_CLEAR_PREV_ESP();
	}
}

void CPU_IRET(bool use32, uint32_t oldeip) {
	descriptor_t sd;
	UINT32 new_ip;
	UINT32 new_flags;
	UINT32 new_cs;
	UINT32 mask;
	UINT16 sreg;

	UINT8 op32bak = CPU_INST_OP32;
	CPU_INST_OP32 = (use32 ? 1 : 0);

	CPU_WORKCLOCK(22);
	if (!CPU_STAT_PM) {
		/* Real mode */
		CPU_SET_PREV_ESP();
		mask = I_FLAG | IOPL_FLAG;
		if (!CPU_INST_OP32) {
			POP0_16(new_ip);
			POP0_16(new_cs);
			POP0_16(new_flags);
		}
		else {
			POP0_32(new_ip);
			POP0_32(new_cs);
			POP0_32(new_flags);
			mask |= RF_FLAG;
		}

		/* check new instrunction pointer with new code segment */
		load_segreg(CPU_CS_INDEX, (UINT16)new_cs, &sreg, &sd, GP_EXCEPTION);
		if (new_ip > sd.u.seg.limit) {
			EXCEPTION(GP_EXCEPTION, 0);
		}

		LOAD_SEGREG(CPU_CS_INDEX, (UINT16)new_cs);
		CPU_EIP = new_ip;

		set_eflags(new_flags, mask);
		CPU_CLEAR_PREV_ESP();
	}
	else {
		/* Protected mode */
		IRET_pm();
	}
	CPU_INST_OP32 = op32bak;
	IRQCHECKTERM();

#if 0
	// Emulate system call on MS-DOS Player
	if (IRET_TOP <= CPU_PREV_PC && CPU_PREV_PC < (IRET_TOP + IRET_SIZE)) {
#ifdef USE_DEBUGGER
		// Disallow reentering CPU_EXECUTE() in msdos_syscall()
		msdos_int_num = (CPU_PREV_PC - IRET_TOP);
#else
		// Call msdos_syscall() here for better processing speed
		msdos_syscall(CPU_PREV_PC - IRET_TOP);
#endif
	}
#endif
}

void CPU_Exception(Bitu which, Bitu error) {
	EXCEPTION(which, error);
}

bool CPU_READ_CRX(Bitu cr, uint32_t& retvalue) {
	CPU_WORKCLOCK(11);
	if (CPU_STAT_PM && (CPU_STAT_VM86 || CPU_STAT_CPL != 0)) {
		VERBOSE(("MOV_RdCd: VM86(%s) or CPL(%d) != 0", CPU_STAT_VM86 ? "true" : "false", CPU_STAT_CPL));
		EXCEPTION(GP_EXCEPTION, 0);
		return true;
	}

	switch (cr) {
	case 0:
		retvalue = CPU_CR0;
		break;

	case 2:
		retvalue = CPU_CR2;
		break;

	case 3:
		retvalue = CPU_CR3;
		break;

	case 4:
		retvalue = CPU_CR4;
		break;

	default:
		ia32_panic("MOV_RdCd: CR reg index (%d)", cr);
		/*NOTREACHED*/
		return true;
		break;
	}
	VERBOSE(("MOV_RdCd: %04x:%08x: cr%d: 0x%08x -> %s", CPU_CS, CPU_PREV_EIP, idx, *out, reg32_str[op & 7]));
	return false;
}

bool CPU_PrepareException(Bitu which, Bitu error) {
	EXCEPTION(which, error);
	return true;
}

Bitu PIC_IRQCheck;

bool CPU_SetSegGeneral(SegNames seg, uint16_t value) {
	if ((seg != CPU_CS_INDEX || i386cpuid.allow_movCS) && seg < CPU_SEGREG_NUM) {
		if (!CPU_STAT_PM || CPU_STAT_VM86) {
			CPU_STATSAVE.cpu_regs.sreg[seg] = value;
			CPU_STATSAVE.cpu_stat.sreg[seg].u.seg.segbase = value << 4;
			return false;
		}
		else {
			LOAD_SEGREG(seg, (UINT16)value);
			if (seg == CPU_SS_INDEX) {
				exec_1step();
				return true;
			}
			return false;
		}
	}
	EXCEPTION(UD_EXCEPTION, 0);
	return true;
}

void MEM_SetPageHandler(Bitu phys_page, Bitu pages, PageHandler* handler) {
	for (; pages > 0; pages--) {
		//memory.phandlers[phys_page] = handler;
		jitcache[phys_page] = (PageHandler*)handler;
		phys_page++;
	}
}

bool CPU_PopSeg(SegNames seg, bool use32) {
	UINT32 src;

	if (!use32) {
		//16bit
		CPU_WORKCLOCK(5);

		CPU_SET_PREV_ESP();
		POP0_16(src);
		LOAD_SEGREG(seg, (UINT16)src);
		CPU_CLEAR_PREV_ESP();
	}
	else {
		//32bit
		CPU_WORKCLOCK(5);

		CPU_SET_PREV_ESP();
		POP0_32(src);
		LOAD_SEGREG(seg, (UINT16)src);
		CPU_CLEAR_PREV_ESP();
	}
	if (seg == CPU_SS_INDEX) {
		exec_1step();
		return true;
	}
	return true;
}

bool CPU_PUSHF(Bitu use32) {
	if (!use32) {
		//16bit
		CPU_WORKCLOCK(3);
		if (!CPU_STAT_PM || !CPU_STAT_VM86 || (CPU_STAT_IOPL == CPU_IOPL3)) {
			UINT16 flags = REAL_FLAGREG;
			flags = (flags & ALL_FLAG) | 2;
			PUSH0_16(flags);
			return false;
		}
		/* VM86 && IOPL != 3 */
		EXCEPTION(GP_EXCEPTION, 0);
		return true;
	}
	else {
		//32bit
		CPU_WORKCLOCK(3);
		if (!CPU_STAT_PM || !CPU_STAT_VM86 || (CPU_STAT_IOPL == CPU_IOPL3)) {
			UINT32 flags = REAL_EFLAGREG & ~(RF_FLAG | VM_FLAG);
			flags = (flags & ALL_EFLAG) | 2;
			PUSH0_32(flags);
			return false;
		}
		/* VM86 && IOPL != 3 */
#if defined(USE_VME)
		if (CPU_CR4 & CPU_CR4_VME) {
			if (CPU_EFLAG & VIP_FLAG) {
				EXCEPTION(GP_EXCEPTION, 0);
				return true;
			}
			else {
				UINT32 flags = REAL_EFLAGREG & ~(RF_FLAG | VM_FLAG);
				flags = (flags & ALL_EFLAG) | 2;
				flags = (flags & ~I_FLAG) | ((flags & VIF_FLAG) >> 10); // VIF → IFにコピー
				flags |= IOPL_FLAG; // IOPL == 3 の振りをする
				PUSH0_32(flags);
				return false;
			}
		}
		else {
			EXCEPTION(GP_EXCEPTION, 0);
			return true;
		}
#else
		EXCEPTION(GP_EXCEPTION, 0);
		return true;
#endif
	}
	return false;
}

Bitu MEM_TotalPages(void) {
	return 0;
}
HostPt MemBase = NULL;

bool dos_kernel_disabled;

PageHandler* MEM_GetPageHandler(Bitu phys_page) {
	return (PageHandler*)jitcache[phys_page];
}

void DOSBOX_RunMachine(void) {
	return;
}

void DEBUG_ShowMsg(char const* format, ...) {
	return;
}

int CPU_IsDynamicCore(void) {
	return 2;
}

CPUBlock cpu;
unsigned char CPU_ArchitectureType;
CPU_Decoder* cpudecoder;

UINT64 exec_jit() {
	UINT64 ret = exec_jit_internal();
	exec_1step();
	return ret;
}

#endif
