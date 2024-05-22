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

 //#include "compiler.h"
 //#include "dosio.h"

//syntax : 2byte|prefix|callgate|opcodetogetimm|unused9|unused8|imm16/32bytest|imm8bytest|7|6|5|4|3|2|1|0|unused7|unused6|unused5|unused4|unused3|unused2|unused1|unused0|sse+|imm16|offset16|offset8|imm16/32|imm8|modrm16/32|modrm8
UINT32 opcodeoperantsizedesc[4][256] = {
	{
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
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,// 0x6f
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
		0x00000004,
		0x00000004,
		0x00000004,
		0x00000004,// 0xe7
		0x00000008,
		0x00000008,
		0x20000000,
		0x00000004,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,// 0xef
		0x40000000,
		0x00000000,
		0x40000000,
		0x40000000,
		0x00000000,
		0x00000000,
		0x11030001,
		0x12030002,// 0xf7
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000001,
		0x00000002 // 0xff
	},
};

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

#ifdef _M_X64
void cpuidhost(UINT32 prm_0, UINT32 prm_1, void* prm_2) {
	__cpuidex((int*)prm_2, prm_0, prm_1);
	return;
}
#else
#ifdef _M_IX86
__declspec(naked) void cpuidhost(UINT32, UINT32, void*) {
	__asm {
		push eax
		push ebx
		push ecx
		push edx
		push esi
		push edi
		mov esi, esp + 8
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
		pop esi
		pop edx
		pop ecx
		pop ebx
		pop eax
		ret
nocpuid:
		xor eax, eax
		mov edi, [esi + 8]
		mov[edi + 0], eax
		mov[edi + 4], eax
		mov[edi + 8], eax
		mov[edi + 12], eax
		pop edi
		pop esi
		pop edx
		pop ecx
		pop ebx
		pop eax
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
#define GEN_LOADINT_FUNCTION(__reg,__val) *(UINT32*)(*pos) = 0xd2800000|(__reg&0x1f)|(((__val>>(16*0))&0xFFFF)<<5); (*pos) += 4; if (((__val>>(16*1))&0xFFFF) != 0){*(UINT32*)(*pos) = 0xF2A00000|(__reg&0x1f)|(((__val>>(16*1))&0xFFFF)<<5); (*pos) += 4;} if (((__val>>(16*2))&0xFFFF) != 0){*(UINT32*)(*pos) = 0xF2C00000|(__reg&0x1f)|(((__val>>(16*2))&0xFFFF)<<5); (*pos) += 4;} if (((__val>>(16*3))&0xFFFF) != 0){*(UINT32*)(*pos) = 0xF2E00000|(__reg&0x1f)|(((__val>>(16*3))&0xFFFF)<<5); (*pos) += 4;}
#define GEN_PUSHNATIVE_FUNCTION(__reg1,__reg2) *(UINT32*)(*pos) = 0xA9BF03E0|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<11); (*pos) += 4;
#define GEN_POPNATIVE_FUNCTION(__reg1,__reg2) *(UINT32*)(*pos) = 0xA8C103E0|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<11); (*pos) += 4;
#define GEN_LOAD_FUNCTION(__bitsz,__reg1,__reg2,__reg3) *(UINT32*)(*pos) = 0x38606800|((__bitsz&3)<<30)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); (*pos) += 4;
#define GEN_STOR_FUNCTION(__bitsz,__reg1,__reg2,__reg3) *(UINT32*)(*pos) = 0x38206800|((__bitsz&3)<<30)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); (*pos) += 4;
#define GEN_ADD_FUNCTION(__setflg,__bitsz,__reg1,__reg2,__reg3) *(UINT32*)(*pos) = 0x0b000000|((__bitsz&1)<<31)|((__setflg&1)<<29)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); (*pos) += 4;
#define GEN_SUB_FUNCTION(__setflg,__bitsz,__reg1,__reg2,__reg3) *(UINT32*)(*pos) = 0x4b000000|((__bitsz&1)<<31)|((__setflg&1)<<29)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); (*pos) += 4;
#define GEN_AND_FUNCTION(__setflg,__bitsz,__reg1,__reg2,__reg3) *(UINT32*)(*pos) = 0x0a000000|((__bitsz&1)<<31)|((__setflg&1)<<29)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); (*pos) += 4;
#define GEN_XOR_FUNCTION(__setflg,__bitsz,__reg1,__reg2,__reg3) *(UINT32*)(*pos) = 0x4a000000|((__bitsz&1)<<31)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); (*pos) += 4;if ((__reg1&0x1f) != 31 || (__setflg&1) != 0){GEN_AND_FUNCTION(__setflg,__bitsz,31,__reg1,__reg1);}
#define GEN_OR_FUNCTION(__setflg,__bitsz,__reg1,__reg2,__reg3) *(UINT32*)(*pos) = 0x2a000000|((__bitsz&1)<<31)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5)|((__reg3&0x1f)<<16); (*pos) += 4;if ((__reg1&0x1f) != 31 || (__setflg&1) != 0){GEN_AND_FUNCTION(__setflg,__bitsz,31,__reg1,__reg1);}
#define GEN_CMP_FUNCTION(__bitsz,__reg1,__reg2) GEN_SUB_FUNCTION(1,__bitsz,31,__reg1,__reg2);
#define GEN_TST_FUNCTION(__bitsz,__reg1,__reg2) GEN_AND_FUNCTION(1,__bitsz,31,__reg1,__reg2);
#define GEN_MOV_FUNCTION(__bitsz,__reg1,__reg2) if (__reg2==31 || __reg1==31){*(UINT32*)(*pos) = 0x11000000|((__bitsz&1)<<31)|((__reg1&0x1f)<<0)|((__reg2&0x1f)<<5); (*pos) += 4;}else{GEN_OR_FUNCTION(0,__bitsz,__reg1,31,__reg2);}
#define GEN_BLR_FUNCTION(__reg) *(UINT32*)(*pos) = 0xD63F0000|((__reg&0x1f)<<5); (*pos) += 4
#define GEN_BR_FUNCTION(__reg) *(UINT32*)(*pos) = 0xD61F0000|((__reg&0x1f)<<5); (*pos) += 4
#define GEN_CALL_FUNCTION(__addr) GEN_PUSHNATIVE_FUNCTION(9,30); GEN_LOADINT_FUNCTION(9,__addr); GEN_BLR_FUNCTION(9); GEN_POPNATIVE_FUNCTION(9,30);
#define GEN_JP_FUNCTION(__addr) GEN_LOADINT_FUNCTION(9,__addr); GEN_BR_FUNCTION(9);
#define GEN_RET_FUNCTION() *(UINT32*)(*pos) = 0xD65F03C0; (*pos) += 4
#else
#endif
#endif
#endif

void GenNativecode(UINT64* pos, UINT32 targetopcode,UINT8 targetbitlength) {
	int prefix;
	UINT32 op;

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
		GEN_LOADINT_FUNCTION(0, UD_EXCEPTION);
		GEN_LOADINT_FUNCTION(1, 0);
		GEN_CALL_FUNCTION((UINT64)&exception);
		GEN_RET_FUNCTION();
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
		GEN_CALL_FUNCTION((UINT64)*insttable_1byte[CPU_INST_OP32][op]);
		return;
	}

	/* rep */
	CPU_WORKCLOCK(5);
#if defined(DEBUG)
	if (!cpu_debug_rep_cont) {
		cpu_debug_rep_cont = 1;
		cpu_debug_rep_regs = CPU_STATSAVE.cpu_regs;
	}
#endif
	if (!CPU_INST_AS32) {
		if (CPU_CX != 0) {
			if (!(insttable_info[op] & REP_CHECKZF)) {
				/* rep */
				for (;;) {
					(*insttable_1byte[CPU_INST_OP32][op])();
					if (--CPU_CX == 0) {
#if defined(DEBUG)
						cpu_debug_rep_cont = 0;
#endif
						break;
					}
					if (CPU_REMCLOCK <= 0) {
						CPU_EIP = CPU_PREV_EIP;
						break;
					}
				}
			}
			else if (CPU_INST_REPUSE != 0xf2) {
				/* repe */
				for (;;) {
					(*insttable_1byte[CPU_INST_OP32][op])();
					if (--CPU_CX == 0 || CC_NZ) {
#if defined(DEBUG)
						cpu_debug_rep_cont = 0;
#endif
						break;
					}
					if (CPU_REMCLOCK <= 0) {
						CPU_EIP = CPU_PREV_EIP;
						break;
					}
				}
			}
			else {
				/* repne */
				for (;;) {
					(*insttable_1byte[CPU_INST_OP32][op])();
					if (--CPU_CX == 0 || CC_Z) {
#if defined(DEBUG)
						cpu_debug_rep_cont = 0;
#endif
						break;
					}
					if (CPU_REMCLOCK <= 0) {
						CPU_EIP = CPU_PREV_EIP;
						break;
					}
				}
			}
		}
	}
	else {
		if (CPU_ECX != 0) {
			if (!(insttable_info[op] & REP_CHECKZF)) {
				/* rep */
				for (;;) {
					(*insttable_1byte[CPU_INST_OP32][op])();
					if (--CPU_ECX == 0) {
#if defined(DEBUG)
						cpu_debug_rep_cont = 0;
#endif
						break;
					}
					if (CPU_REMCLOCK <= 0) {
						CPU_EIP = CPU_PREV_EIP;
						break;
					}
				}
			}
			else if (CPU_INST_REPUSE != 0xf2) {
				/* repe */
				for (;;) {
					(*insttable_1byte[CPU_INST_OP32][op])();
					if (--CPU_ECX == 0 || CC_NZ) {
#if defined(DEBUG)
						cpu_debug_rep_cont = 0;
#endif
						break;
					}
					if (CPU_REMCLOCK <= 0) {
						CPU_EIP = CPU_PREV_EIP;
						break;
					}
				}
			}
			else {
				/* repne */
				for (;;) {
					(*insttable_1byte[CPU_INST_OP32][op])();
					if (--CPU_ECX == 0 || CC_Z) {
#if defined(DEBUG)
						cpu_debug_rep_cont = 0;
#endif
						break;
					}
					if (CPU_REMCLOCK <= 0) {
						CPU_EIP = CPU_PREV_EIP;
						break;
					}
				}
			}
		}
	}
}
