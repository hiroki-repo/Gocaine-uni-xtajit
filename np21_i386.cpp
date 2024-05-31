#include "windows.h"
/*BOOL APIENTRY DllMain (HMODULE hModule,
	DWORD  ul_reason_for_call,
	LPVOID lpReserved
)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
};*/

#ifdef VC_DLL_EXPORTS
#undef VC_DLL_EXPORTS
#define VC_DLL_EXPORTS extern "C" __declspec(dllexport)
#else
#define VC_DLL_EXPORTS extern "C" __declspec(dllimport)
#endif

VC_DLL_EXPORTS void CPU_SET_EIP(UINT32 value);
VC_DLL_EXPORTS void CPU_SET_C_FLAG(UINT8 value);
VC_DLL_EXPORTS void CPU_SET_Z_FLAG(UINT8 value);
VC_DLL_EXPORTS void CPU_SET_S_FLAG(UINT8 value);
VC_DLL_EXPORTS void CPU_SET_I_FLAG(UINT8 value);
VC_DLL_EXPORTS void CPU_SET_IOP_FLAG(UINT8 value);
VC_DLL_EXPORTS void CPU_SET_NT_FLAG(UINT8 value);
//VC_DLL_EXPORTS void CPU_SET_VM_FLAG(UINT8 value);
//VC_DLL_EXPORTS void CPU_SET_CR0(UINT32 src);
VC_DLL_EXPORTS void CPU_SET_CR3(UINT32 value);
VC_DLL_EXPORTS void CPU_SET_CPL(int value);
//VC_DLL_EXPORTS void CPU_SET_EFLAG(UINT32 new_flags);
VC_DLL_EXPORTS void CPU_A20_LINE(UINT8 value);
VC_DLL_EXPORTS void CPU_IRQ_LINE(BOOL state);
VC_DLL_EXPORTS void CPU_INIT();
VC_DLL_EXPORTS void CPU_RELEASE();
VC_DLL_EXPORTS void CPU_FINISH();
VC_DLL_EXPORTS void CPU_RESET();
VC_DLL_EXPORTS int CPU_EXECUTE();
VC_DLL_EXPORTS void CPU_SOFT_INTERRUPT(int vect);
VC_DLL_EXPORTS void CPU_JMP_FAR(UINT16 new_cs, UINT32 new_ip);
VC_DLL_EXPORTS void CPU_CALL_FAR(UINT16 new_cs, UINT32 new_ip);
VC_DLL_EXPORTS void CPU_IRET();
VC_DLL_EXPORTS void CPU_PUSH(UINT16 reg);
VC_DLL_EXPORTS UINT16 CPU_POP();
VC_DLL_EXPORTS void CPU_PUSHF();
VC_DLL_EXPORTS UINT16 CPU_READ_STACK();
VC_DLL_EXPORTS void CPU_WRITE_STACK(UINT16 reg);
VC_DLL_EXPORTS void CPU_LOAD_LDTR(UINT16 selector);
VC_DLL_EXPORTS void CPU_LOAD_TR(UINT16 selector);
VC_DLL_EXPORTS UINT32 CPU_TRANS_PAGING_ADDR(UINT32 addr);
VC_DLL_EXPORTS void CPU_SET_MACTLFC(UINT32(*ptrformaf) (int, int, int));
VC_DLL_EXPORTS UINT32 CPU_GET_REG(int regid);
VC_DLL_EXPORTS void CPU_SET_REG(int regid, UINT32 regdata);
VC_DLL_EXPORTS void CPU_SET_IRQ(BOOL statforirq);
VC_DLL_EXPORTS void CPU_SET_A20(UINT8 statfora20);
VC_DLL_EXPORTS void CPU_REQ_INTERRUPT(int vect);
VC_DLL_EXPORTS void CPU_REQ_NMINTERRUPT();
VC_DLL_EXPORTS int CPU_EXECUTE_CC(int clockcount);
VC_DLL_EXPORTS UINT32 CPU_GET_SYSREG(int regid);
VC_DLL_EXPORTS void CPU_SET_SYSREG(int regid, UINT32 regdata);
VC_DLL_EXPORTS UINT32 CPU_GET_SYSREG_DESC(int regid);
VC_DLL_EXPORTS void CPU_SET_SYSREG_DESC(int regid, UINT32 regdata);
VC_DLL_EXPORTS UINT32 CPU_GET_SYSREG_LIMIT(int regid);
VC_DLL_EXPORTS void CPU_SET_SYSREG_LIMIT(int regid, UINT32 regdata);
VC_DLL_EXPORTS UINT32 CPU__GET_CR0();
VC_DLL_EXPORTS void CPU__SET_CR0(UINT32 regdata);
VC_DLL_EXPORTS UINT32 CPU__GET_CR1();
VC_DLL_EXPORTS void CPU__SET_CR1(UINT32 regdata);
VC_DLL_EXPORTS UINT32 CPU__GET_CR2();
VC_DLL_EXPORTS void CPU__SET_CR2(UINT32 regdata);
VC_DLL_EXPORTS UINT32 CPU__GET_CR3();
VC_DLL_EXPORTS void CPU__SET_CR3(UINT32 regdata);
VC_DLL_EXPORTS UINT32 CPU__GET_CR4();
VC_DLL_EXPORTS void CPU__SET_CR4(UINT32 regdata);
VC_DLL_EXPORTS UINT16 CPU_GET_LDTR();
VC_DLL_EXPORTS void CPU_SET_LDTR(UINT16 regdata);
VC_DLL_EXPORTS UINT16 CPU_GET_TR();
VC_DLL_EXPORTS void CPU_SET_TR(UINT16 regdata);
VC_DLL_EXPORTS UINT32 CPU__GET_MXCSR();
VC_DLL_EXPORTS void CPU__SET_MXCSR(UINT32 regdata);
VC_DLL_EXPORTS void CPU_SET_SS32(UINT8 statforsq);
VC_DLL_EXPORTS void CPU_SET_PM(UINT8 statforsq);
VC_DLL_EXPORTS void CPU_SET_OP32(UINT8 regdata);
VC_DLL_EXPORTS void CPU_SET_AS32(UINT8 regdata);
VC_DLL_EXPORTS void CPU_SWITCH_PM(BOOL onoff);
VC_DLL_EXPORTS UINT16 CPU_GET_IDTR_LIMIT();
VC_DLL_EXPORTS void CPU_SET_IDTR_LIMIT(UINT16 regdata);
VC_DLL_EXPORTS UINT32 CPU_GET_IDTR_BASE();
VC_DLL_EXPORTS void CPU_SET_IDTR_BASE(UINT32 regdata);
VC_DLL_EXPORTS UINT16 CPU_GET_GDTR_LIMIT();
VC_DLL_EXPORTS void CPU_SET_GDTR_LIMIT(UINT16 regdata);
VC_DLL_EXPORTS UINT32 CPU_GET_GDTR_BASE();
VC_DLL_EXPORTS void CPU_SET_GDTR_BASE(UINT32 regdata);
VC_DLL_EXPORTS void CPU_BUS_SIZE_CHANGE(int size);
VC_DLL_EXPORTS void CPU_REQ_INTERRUPT_IN(int vect);
VC_DLL_EXPORTS void CPU_REQ_NMINTERRUPT_IN();
VC_DLL_EXPORTS void CPU__SET_EFLAG(UINT32 new_flags);
VC_DLL_EXPORTS void CPU__SET_VM_FLAG(UINT8 value);

UINT32(*i386memaccess) (int, int, int);

void CPU_SET_MACTLFC(UINT32 (*ptrformaf) (int, int, int))
{
	i386memaccess = ptrformaf;
}

int cpubussize = 0;

void CPU_BUS_SIZE_CHANGE(int size) {
	cpubussize = size;
}


UINT8 read_byte(UINT32 byteaddress)
{
	if (((cpubussize >> 16) & 0xF) == 0) {
		return ((i386memaccess(((int)byteaddress) + 0, 0, 1) & 0xFF) << (8 * 0));
	}
	else {
		return ((i386memaccess(((int)byteaddress) + 0, 0, 1) >> (8 * (byteaddress & 3))) & 0xFF);
	}
}
UINT16 read_word(UINT32 byteaddress)
{
	if (((cpubussize >> 16) & 0xF) == 0) {
		if (((cpubussize >> 0) & 0xFF) == 0) {
			return ((i386memaccess(((int)byteaddress) + 0, 0, 1) & 0xFF) << (8 * 0)) | ((i386memaccess(((int)byteaddress) + 1, 0, 1) & 0xFF) << (8 * 1));
		}
		else if (((cpubussize >> 0) & 0xFF) >= 1) {
			return ((i386memaccess(((int)byteaddress) + 0, 0, 1 | 0x10) & 0xFFFF) << (16 * 0));
		}
	}
	else {
		if ((byteaddress & 3) == 3) { 
			return ((((i386memaccess(((int)byteaddress) + 0, 0, 1 | 0x10) >> (8 * 3)) & 0xFF) | ((i386memaccess(((int)byteaddress) + 1, 0, 1 | 0x10) << (8 * 1)) & 0xFF00)) & 0xFFFF);
		}
		else {
			return ((i386memaccess(((int)byteaddress) + 0, 0, 1 | 0x10) >> (8 * (byteaddress & 3))) & 0xFFFF);
		}
	}
}
UINT32 read_dword(UINT32 byteaddress)
{
	if (((cpubussize >> 16) & 0xF) == 0) {
		if (((cpubussize >> 0) & 0xFF) == 0) {
			return ((i386memaccess(((int)byteaddress) + 0, 0, 1) & 0xFF) << (8 * 0)) | ((i386memaccess(((int)byteaddress) + 1, 0, 1) & 0xFF) << (8 * 1)) | ((i386memaccess(((int)byteaddress) + 2, 0, 1) & 0xFF) << (8 * 2)) | ((i386memaccess(((int)byteaddress) + 3, 0, 1) & 0xFF) << (8 * 3));
		}
		else if (((cpubussize >> 0) & 0xFF) == 1) {
			return ((i386memaccess(((int)byteaddress) + 0, 0, 1 | 0x10) & 0xFFFF) << (16 * 0)) | ((i386memaccess(((int)byteaddress) + 2, 0, 1 | 0x10) & 0xFFFF) << (16 * 1));
		}
		else if (((cpubussize >> 0) & 0xFF) >= 2) {
			return i386memaccess(((int)byteaddress) + 0, 0, 1 | 0x20);
		}
	}
	else {
		if ((byteaddress & 3) == 0) {
			return i386memaccess(((int)byteaddress) + 0, 0, 1 | 0x20);
		}
		else {
			return (((i386memaccess(((int)byteaddress) + 0, 0, 1 | 0x20)>>(8 * (byteaddress & 3)))&((1<<(8 * (byteaddress & 3)))-1))|(((i386memaccess(((int)byteaddress) + 4, 0, 1 | 0x20)>>(8 * 0))&((1<<(8 * (4-(byteaddress & 3))))-1))<<(8 * (byteaddress & 3))))&0xFFFFFFFF;
		}
	}
}

void write_byte(UINT32 byteaddress, UINT8 data)
{
	if (((cpubussize >> 16) & 0xF) == 0) {
		i386memaccess(((int)byteaddress) + 0, (UINT8)data, 0);
	}
	else {
		i386memaccess(((int)byteaddress) + 0, (read_dword(byteaddress&0xFFFFFFFC)&(~(0xFF<<(8*(byteaddress&3)))))|(((UINT8)data)<<(8*(byteaddress&3))), 0);
	}
}
void write_word(UINT32 byteaddress, UINT16 data)
{
	if (((cpubussize >> 16) & 0xF) == 0) {
		if (((cpubussize >> 0) & 0xFF) == 0) {
		i386memaccess(((int)byteaddress) + 0, (UINT8)(data >> (8 * 0)), 0);
		i386memaccess(((int)byteaddress) + 1, (UINT8)(data >> (8 * 1)), 0);
	}
	else if (((cpubussize >> 0) & 0xFF) >= 1) {
		i386memaccess(((int)byteaddress) + 0, (data >> (16 * 0)), 0 | 0x10);
	}
	}
	else {
		if ((byteaddress & 3) == 3) {
			i386memaccess(((int)byteaddress) + 0, (read_dword(byteaddress&0xFFFFFFFC)&(~(0xFF<<(8*(byteaddress&3)))))|((data << (8 * 3))&0xFF000000), 0 | 0x10);
			i386memaccess(((int)byteaddress) + 1, (read_dword((byteaddress+1)&0xFFFFFFFC)&(~(0xFF<<(8*0))))|((data >> (8 * 1))&0xFF), 0 | 0x10);
		}
		else {
			i386memaccess(((int)byteaddress) + 0, (read_dword(byteaddress&0xFFFFFFFC)&(~(0xFFFF<<(8*(byteaddress&3)))))|(data << (8 * (byteaddress&3))), 0 | 0x10);
		}
	}
}
void write_dword(UINT32 byteaddress, UINT32 data)
{
	if (((cpubussize >> 16) & 0xF) == 0) {
		if (((cpubussize >> 0) & 0xFF) == 0) {
			i386memaccess(((int)byteaddress) + 0, (UINT8)(data >> (8 * 0)), 0);
			i386memaccess(((int)byteaddress) + 1, (UINT8)(data >> (8 * 1)), 0);
			i386memaccess(((int)byteaddress) + 2, (UINT8)(data >> (8 * 2)), 0);
			i386memaccess(((int)byteaddress) + 3, (UINT8)(data >> (8 * 3)), 0);
		}
		else if (((cpubussize >> 0) & 0xFF) == 1) {
			i386memaccess(((int)byteaddress) + 0, (data >> (16 * 0)), 0 | 0x10);
			i386memaccess(((int)byteaddress) + 2, (data >> (16 * 1)), 0 | 0x10);
		}
		else if (((cpubussize >> 0) & 0xFF) >= 2) {
			i386memaccess(((int)byteaddress) + 0, (data), 0 | 0x20);
		}
	}
	else {
		if ((byteaddress & 3) == 0) {
			i386memaccess(((int)byteaddress) + 0, (data), 0 | 0x20);
		}
		else {
			i386memaccess(((int)byteaddress) + 0, ((read_dword(byteaddress&0xFFFFFFFC)&((1<<(8*(byteaddress&3)))-1))|((data<<(8*(byteaddress & 3)))&(~((1<<(8*(byteaddress & 3)))-1)))), 0 | 0x20);
			i386memaccess(((int)byteaddress) + 4, ((read_dword((byteaddress+4)&0xFFFFFFFC)&(((1<<(8*(4-(byteaddress&3))))-1)<<(8*(byteaddress&3))))|((data>>(8*(4-(byteaddress & 3))))&((1<<(8*(byteaddress & 3)))-1))), 0 | 0x20);
		}
	}
}

UINT8 read_io_byte(UINT32 byteaddress)
{
	return (i386memaccess(((int)byteaddress) + 0, 0, 3) & 0xFF);
}
UINT16 read_io_word(UINT32 byteaddress)
{
	if (((cpubussize >> 8) & 0xFF) == 0) {
		return ((i386memaccess(((int)byteaddress) + 0, 0, 3) & 0xFF) << (8 * 0)) | ((i386memaccess(((int)byteaddress) + 1, 0, 3) & 0xFF) << (8 * 1));
	}
	else if (((cpubussize >> 8) & 0xFF) >= 1) {
		return (i386memaccess(((int)byteaddress) + 0, 0, 3 | 0x10) & 0xFFFF);
	}
}
UINT32 read_io_dword(UINT32 byteaddress)
{
	if (((cpubussize >> 8) & 0xFF) == 0) {
		return ((i386memaccess(((int)byteaddress) + 0, 0, 3) & 0xFF) << (8 * 0)) | ((i386memaccess(((int)byteaddress) + 1, 0, 3) & 0xFF) << (8 * 1)) | ((i386memaccess(((int)byteaddress) + 2, 0, 3) & 0xFF) << (8 * 2)) | ((i386memaccess(((int)byteaddress) + 3, 0, 3) & 0xFF) << (8 * 3));
	}
	else if (((cpubussize >> 8) & 0xFF) == 1) {
		return ((i386memaccess(((int)byteaddress) + 0, 0, 3 | 0x10) & 0xFFFF) << (16 * 0))| ((i386memaccess(((int)byteaddress) + 0, 0, 3 | 0x10) & 0xFFFF) << (16 * 1));
	}
	else if (((cpubussize >> 8) & 0xFF) >= 2) {
		return (i386memaccess(((int)byteaddress) + 0, 0, 3 | 0x20) & 0xFFFFFFFF);
	}
}

void write_io_byte(UINT32 byteaddress, UINT8 data)
{
	i386memaccess(((int)byteaddress) + 0, (UINT8)data, 2);
}
void write_io_word(UINT32 byteaddress, UINT16 data)
{
	if (((cpubussize >> 8) & 0xFF) == 0) {
		i386memaccess(((int)byteaddress) + 0, (UINT8)(data >> (8 * 0)), 2);
		i386memaccess(((int)byteaddress) + 1, (UINT8)(data >> (8 * 1)), 2);
	}
	else {
		i386memaccess(((int)byteaddress) + 0, (UINT16)(data >> 16 * 0), 2 | 0x10);
	}
	//i386memaccess(((int)byteaddress) + 0, (UINT16)data, 2);
}
void write_io_dword(UINT32 byteaddress, UINT32 data)
{
	if (((cpubussize >> 8) & 0xFF) == 0) {
		i386memaccess(((int)byteaddress) + 0, (UINT8)(data >> (8 * 0)), 2);
		i386memaccess(((int)byteaddress) + 1, (UINT8)(data >> (8 * 1)), 2);
		i386memaccess(((int)byteaddress) + 2, (UINT8)(data >> (8 * 2)), 2);
		i386memaccess(((int)byteaddress) + 3, (UINT8)(data >> (8 * 3)), 2);
	}else if (((cpubussize >> 8) & 0xFF) == 1) {
		i386memaccess(((int)byteaddress) + 0, (UINT16)(data >> 16 * 0), 2 | 0x10);
		i386memaccess(((int)byteaddress) + 2, (UINT16)(data >> 16 * 2), 2 | 0x10);
	}
	else {
		i386memaccess(((int)byteaddress) + 0, (UINT32)(data), 2 | 0x20);
	}
	//i386memaccess(((int)byteaddress) + 0, (UINT32)data, 2);
}


#if defined(_MSC_VER) && (_MSC_VER >= 1400)
#pragma warning( disable : 4018 )
#pragma warning( disable : 4244 )
#pragma warning( disable : 4309 )
#pragma warning( disable : 4819 )
#pragma warning( disable : 4995 )
#pragma warning( disable : 4996 )
#endif

#pragma warning( disable : 4703 )
#pragma warning( disable : 4146 )

UINT32 msdos_int6h_eip;
int cpu_type, cpu_step;


//#define ULONG unsigned long
//#define uintptr_t (*char)unsigned int
#include <stdint.h>
//#define uintptr_t intptr_t



#ifdef USE_DEBUGGER

/* ----------------------------------------------------------------------------
	MAME i386 disassembler
---------------------------------------------------------------------------- */

#if defined(_MSC_VER) && (_MSC_VER >= 1400)
#pragma warning( disable : 4146 )
#pragma warning( disable : 4244 )
#endif

/*****************************************************************************/
/* src/emu/devcpu.h */

// CPU interface functions
#define CPU_DISASSEMBLE_NAME(name)		cpu_disassemble_##name
#define CPU_DISASSEMBLE(name)			int CPU_DISASSEMBLE_NAME(name)(char *buffer, offs_t eip, const UINT8 *oprom)
#define CPU_DISASSEMBLE_CALL(name)		CPU_DISASSEMBLE_NAME(name)(buffer, eip, oprom)

/*****************************************************************************/
/* src/emu/didisasm.h */

// Disassembler constants
const UINT32 DASMFLAG_SUPPORTED     = 0x80000000;   // are disassembly flags supported?
const UINT32 DASMFLAG_STEP_OUT      = 0x40000000;   // this instruction should be the end of a step out sequence
const UINT32 DASMFLAG_STEP_OVER     = 0x20000000;   // this instruction should be stepped over by setting a breakpoint afterwards
const UINT32 DASMFLAG_OVERINSTMASK  = 0x18000000;   // number of extra instructions to skip when stepping over
const UINT32 DASMFLAG_OVERINSTSHIFT = 27;           // bits to shift after masking to get the value
const UINT32 DASMFLAG_LENGTHMASK    = 0x0000ffff;   // the low 16-bits contain the actual length

/*****************************************************************************/
/* src/emu/memory.h */

// offsets and addresses are 32-bit (for now...)
typedef UINT32	offs_t;

/*****************************************************************************/
/* src/osd/osdcomm.h */

/* Highly useful macro for compile-time knowledge of an array size */
#define ARRAY_LENGTH(x)     (sizeof(x) / sizeof(x[0]))

#ifndef INLINE
#define INLINE inline
#endif

#include "mame/emu/cpu/i386/i386dasm.c"

#undef CPU_DISASSEMBLE

int CPU_DISASSEMBLE(UINT8 *oprom, offs_t eip, bool is_ia32, char *buffer, size_t buffer_len)
{
	if(is_ia32) {
		return CPU_DISASSEMBLE_CALL(x86_32) & DASMFLAG_LENGTHMASK;
	} else {
		return CPU_DISASSEMBLE_CALL(x86_16) & DASMFLAG_LENGTHMASK;
	}
}
#endif

#include "np21_i386c/cpucore.h"
#include "np21_i386c/ia32/ia32.mcr"
#include "np21_i386c/ia32/ctrlxfer.h"
#include "np21_i386c/ia32/instructions/ctrl_trans.h"
#include "np21_i386c/ia32/instructions/flag_ctrl.h"
#include "np21_i386c/ia32/instructions/fpu/fp.h"

void CPU_SWITCH_PM(BOOL onoff)
{

	if (onoff) {
		VERBOSE(("change_pm: Entering to Protected-Mode..."));
	}
	else {
		VERBOSE(("change_pm: Leaveing from Protected-Mode..."));
	}

	CPU_INST_OP32 = CPU_INST_AS32 =
		CPU_STATSAVE.cpu_inst_default.op_32 =
		CPU_STATSAVE.cpu_inst_default.as_32 = 0;
	CPU_STAT_SS32 = 0;
	set_cpl(0);
	CPU_STAT_PM = onoff;
}


void CPU_SET_OP32(UINT8 regdata)
{
	CPU_INST_OP32 = regdata;
	CPU_STATSAVE.cpu_inst_default.op_32 = regdata;
}
void CPU_SET_AS32(UINT8 regdata)
{
	CPU_INST_AS32 = regdata;
	CPU_STATSAVE.cpu_inst_default.as_32 = regdata;
}

UINT32 CPU_GET_REG(int regid)
{
	return CPU_REGS_DWORD(regid);
}

void CPU_SET_REG(int regid, UINT32 regdata)
{
	CPU_REGS_DWORD(regid)=regdata;
}

UINT32 CPU_GET_SYSREG(int regid)
{
	return CPU_STAT_SREGBASE(regid);
}

void CPU_SET_SYSREG(int regid, UINT32 regdata)
{
	CPU_STAT_SREGBASE(regid) = regdata;
}

UINT32 CPU_GET_SYSREG_DESC(int regid)
{
	return CPU_REGS_SREG(regid);
}

void CPU_SET_SYSREG_DESC(int regid, UINT32 regdata)
{
	CPU_REGS_SREG(regid) = regdata;
}

UINT32 CPU_GET_SYSREG_LIMIT(int regid)
{
	return CPU_STAT_SREGLIMIT(regid);
}

void CPU_SET_SYSREG_LIMIT(int regid, UINT32 regdata)
{
	CPU_STAT_SREGLIMIT(regid) = regdata;
}

UINT32 CPU__GET_CR0()
{
	return CPU_CR0;
}

UINT32 CPU__GET_CR1()
{
	return CPU_CR1;
}

void CPU__SET_CR1(UINT32 regdata)
{
	CPU_CR1 = regdata;
}

UINT32 CPU__GET_CR2()
{
	return CPU_CR2;
}

void CPU__SET_CR2(UINT32 regdata)
{
	CPU_CR2 = regdata;
}

UINT32 CPU__GET_CR3()
{
	return CPU_CR3;
}

UINT32 CPU__GET_CR4()
{
	return CPU_CR4;
}

void CPU__SET_CR4(UINT32 regdata)
{
	CPU_CR4 = regdata;
}

UINT16 CPU_GET_LDTR()
{
	return CPU_LDTR;
}

void CPU_SET_LDTR(UINT16 regdata)
{
	CPU_LDTR = regdata;
}

UINT16 CPU_GET_IDTR_LIMIT()
{
	return CPU_IDTR_LIMIT;
}

void CPU_SET_IDTR_LIMIT(UINT16 regdata)
{
	CPU_IDTR_LIMIT = regdata;
}

UINT32 CPU_GET_IDTR_BASE()
{
	return CPU_IDTR_BASE;
}

void CPU_SET_IDTR_BASE(UINT32 regdata)
{
	CPU_IDTR_BASE = regdata;
}

UINT16 CPU_GET_GDTR_LIMIT()
{
	return CPU_GDTR_LIMIT;
}

void CPU_SET_GDTR_LIMIT(UINT16 regdata)
{
	CPU_GDTR_LIMIT = regdata;
}

UINT32 CPU_GET_GDTR_BASE()
{
	return CPU_GDTR_BASE;
}

void CPU_SET_GDTR_BASE(UINT32 regdata)
{
	CPU_GDTR_BASE = regdata;
}

UINT16 CPU_GET_TR()
{
	return CPU_TR;
}

void CPU_SET_TR(UINT16 regdata)
{
	CPU_TR = regdata;
}

UINT32 CPU__GET_MXCSR()
{
	return CPU_MXCSR;
}

void CPU__SET_MXCSR(UINT32 regdata)
{
	CPU_MXCSR = regdata;
}

void CPU_SET_SS32(UINT8 statforsq)
{
	CPU_STAT_SS32 = statforsq;
}

void CPU_SET_PM(UINT8 statforsq)
{
	CPU_STAT_PM = statforsq;
}


#define CPU_ES_BASE			ES_BASE
#define CPU_CS_BASE			CS_BASE
#define CPU_SS_BASE			SS_BASE
#define CPU_DS_BASE			DS_BASE
#define CPU_FS_BASE			FS_BASE
#define CPU_GS_BASE			GS_BASE

#define	CPU_LOAD_SREG(idx, selector)	LOAD_SEGREG(idx, selector)

#define CPU_EIP_CHANGED			(CPU_EIP != CPU_PREV_EIP)

inline void CPU_SET_EIP(UINT32 value)
{
	CPU_EIP = value;
}

#define CPU_C_FLAG			((CPU_FLAGL & C_FLAG) != 0)
#define CPU_Z_FLAG			((CPU_FLAGL & Z_FLAG) != 0)
#define CPU_S_FLAG			((CPU_FLAGL & S_FLAG) != 0)
#define CPU_I_FLAG			((CPU_FLAGH & I_FLAG) != 0)

inline void CPU_SET_C_FLAG(UINT8 value)
{
	if(value) {
		CPU_FLAG |=  C_FLAG;
	} else {
		CPU_FLAG &= ~C_FLAG;
	}
}

inline void CPU_SET_Z_FLAG(UINT8 value)
{
	if(value) {
		CPU_FLAG |=  Z_FLAG;
	} else {
		CPU_FLAG &= ~Z_FLAG;
	}
}

inline void CPU_SET_S_FLAG(UINT8 value)
{
	if(value) {
		CPU_FLAG |=  S_FLAG;
	} else {
		CPU_FLAG &= ~S_FLAG;
	}
}

inline void CPU_SET_I_FLAG(UINT8 value)
{
	if(value) {
		CPU_FLAG |=  I_FLAG;
	} else {
		CPU_FLAG &= ~I_FLAG;
	}
}

inline void CPU_SET_IOP_FLAG(UINT8 value)
{
	CPU_FLAG &= ~IOPL_FLAG;
	CPU_FLAG |= value << 12;
}

inline void CPU_SET_NT_FLAG(UINT8 value)
{
	if(value) {
		CPU_FLAG |=  NT_FLAG;
	} else {
		CPU_FLAG &= ~NT_FLAG;
	}
}

inline void CPU_SET_VM_FLAG(UINT8 value)
{
	if(value) {
		if(!(CPU_EFLAG & VM_FLAG)) {
			CPU_EFLAG |=  VM_FLAG;
			change_vm(1);
		}
	} else {
		if(CPU_EFLAG & VM_FLAG) {
			CPU_EFLAG &= ~VM_FLAG;
			change_vm(0);
		}
	}
}

void CPU__SET_VM_FLAG(UINT8 value) {
	CPU_SET_VM_FLAG(value);
}

inline void CPU_SET_CR0(UINT32 src)
{
	// from MOV_CdRd(void) in ia32/instructions/system_inst.c
	UINT32 reg = CPU_CR0;
	src &= CPU_CR0_ALL;
#if defined(USE_FPU)
	if(i386cpuid.cpu_feature & CPU_FEATURE_FPU){
		src |= CPU_CR0_ET;	/* FPU present */
		//src &= ~CPU_CR0_EM;
	} else {
		src |= CPU_CR0_EM | CPU_CR0_NE;
		src &= ~(CPU_CR0_MP | CPU_CR0_ET);
	}
#else
	src |= CPU_CR0_EM | CPU_CR0_NE;
	src &= ~(CPU_CR0_MP | CPU_CR0_ET);
#endif
	CPU_CR0 = src;

	if ((reg ^ CPU_CR0) & (CPU_CR0_PE|CPU_CR0_PG)) {
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
		} else {
			change_pg(0);
		}
	}
	if ((reg ^ CPU_CR0) & CPU_CR0_PE) {
		if (!(CPU_CR0 & CPU_CR0_PE)) {
			change_pm(0);
		}
	}

	CPU_STAT_WP = (CPU_CR0 & CPU_CR0_WP) ? 0x10 : 0;
}

void CPU__SET_CR0(UINT32 regdata)
{
	CPU_SET_CR0(regdata);
	//CPU_CR0 = regdata;
}

inline void CPU_SET_CR3(UINT32 value)
{
	set_cr3(value);
}

void CPU__SET_CR3(UINT32 regdata)
{
	set_cr3(regdata);
}

inline void CPU_SET_CPL(int value)
{
	set_cpl(value);
}

inline void CPU_SET_EFLAG(UINT32 new_flags)
{
	// from modify_eflags(UINT32 new_flags, UINT32 mask) in ia32/ia32.cpp
	UINT32 orig = CPU_EFLAG;

	new_flags &= ALL_EFLAG;
//	mask &= ALL_EFLAG;
//	CPU_EFLAG = (REAL_EFLAGREG & ~mask) | (new_flags & mask);
	CPU_EFLAG = new_flags;

	CPU_OV = CPU_FLAG & O_FLAG;
	CPU_TRAP = (CPU_FLAG & (I_FLAG|T_FLAG)) == (I_FLAG|T_FLAG);
	if (CPU_STAT_PM) {
		if ((orig ^ CPU_EFLAG) & VM_FLAG) {
			if (CPU_EFLAG & VM_FLAG) {
				change_vm(1);
			} else {
				change_vm(0);
			}
		}
	}
}

void CPU__SET_EFLAG(UINT32 new_flags) {
	CPU_SET_EFLAG(new_flags);
}

inline void CPU_A20_LINE(UINT8 value)
{
	ia32a20enable(value != 0);
}

#ifdef USE_DEBUGGER
UINT16 CPU_PREV_CS;
#endif
UINT32 CPU_PREV_PC;
BOOL irq_pending = FALSE;
BOOL nmi_pending = FALSE;
UINT8 pic_ack_vector = 0;

inline void CPU_IRQ_LINE(BOOL state)
{
	irq_pending = state;
}

void CPU_INIT()
{
	CPU_INITIALIZE();
	CPU_ADRSMASK = ~0;
//	CPU_ADRSMASK = ~(1 << 20);
	irq_pending = false;
}

void CPU_RELEASE()
{
	CPU_DEINITIALIZE();
}

void CPU_FINISH()
{
	CPU_RELEASE();
}

void CPU_RESET()
{
	// from pccore_reset(void) in pccore.c
	
	// enable all features
	i386cpuid.cpu_family = CPU_FAMILY;
	i386cpuid.cpu_model = CPU_MODEL;
	i386cpuid.cpu_stepping = CPU_STEPPING;
	i386cpuid.cpu_feature = CPU_FEATURES_ALL;
	i386cpuid.cpu_feature_ex = CPU_FEATURES_EX_ALL;
	i386cpuid.cpu_feature_ex_ecx = CPU_FEATURES_EX_ECX_ALL;
	i386cpuid.cpu_feature_ecx = CPU_FEATURES_ECX_ALL;
	i386cpuid.cpu_eflags_mask = CPU_EFLAGS_MASK;
	i386cpuid.cpu_brandid = CPU_BRAND_ID_NEKOPRO2;
	strcpy(i386cpuid.cpu_vendor, CPU_VENDOR_NEKOPRO);
	strcpy(i386cpuid.cpu_brandstring, CPU_BRAND_STRING_NEKOPRO2);

	i386cpuid.fpu_type = FPU_TYPE_SOFTFLOAT;
//	i386cpuid.fpu_type = FPU_TYPE_DOSBOX;
//	i386cpuid.fpu_type = FPU_TYPE_DOSBOX2;
	fpu_initialize();

	UINT32 PREV_CPU_ADRSMASK = CPU_ADRSMASK;
//	CPU_RESET();
	ia32reset();
	CPU_TYPE = 0;
	//CS_BASE = 0xf0000;
	CPU_CS = 0xf000;
	CPU_IP = 0xfff0;
	CPU_CLEARPREFETCH();
	CPU_ADRSMASK = PREV_CPU_ADRSMASK;
}

UINT32 CPU_GET_NEXT_PC();

int CPU_EXECUTE()
{
#ifdef USE_DEBUGGER
	if(now_debugging) {
		if(force_suspend) {
			force_suspend = false;
			now_suspended = true;
		} else {
			UINT32 next_pc = CPU_GET_NEXT_PC();
			for(int i = 0; i < MAX_BREAK_POINTS; i++) {
				if(break_point.table[i].status == 1 && break_point.table[i].addr == next_pc) {
					break_point.hit = i + 1;
					now_suspended = true;
					break;
				}
			}
		}
		while(now_debugging && now_suspended) {
			Sleep(10);
		}
	}
#endif

	CPU_REMCLOCK = CPU_BASECLOCK = 1;
	CPU_EXEC();
	if(nmi_pending) {
		CPU_INTERRUPT(2, 0);
		nmi_pending = false;
	} else
	if(irq_pending && CPU_isEI) {
		CPU_INTERRUPT(pic_ack_vector, 0);
		irq_pending = false;
		//pic_update();
	}
//	return CPU_BASECLOCK - CPU_REMCLOCK;

#ifdef USE_DEBUGGER
	if(now_debugging) {
		if(!now_going) {
			now_suspended = true;
		}
	}
#endif
	return CPU_BASECLOCK - CPU_REMCLOCK;
}

extern "C" __declspec(dllexport) int CPU_EXECUTE_BC(int clockcount)
{
#ifdef USE_DEBUGGER
	if (now_debugging) {
		if (force_suspend) {
			force_suspend = false;
			now_suspended = true;
		}
		else {
			UINT32 next_pc = CPU_GET_NEXT_PC();
			for (int i = 0; i < MAX_BREAK_POINTS; i++) {
				if (break_point.table[i].status == 1 && break_point.table[i].addr == next_pc) {
					break_point.hit = i + 1;
					now_suspended = true;
					break;
				}
			}
		}
		while (now_debugging && now_suspended) {
			Sleep(10);
		}
	}
#endif

	CPU_REMCLOCK = 1;
	CPU_BASECLOCK = clockcount;
	CPU_EXEC();
	if (nmi_pending) {
		CPU_INTERRUPT(2, 0);
		nmi_pending = false;
	}
	else
		if (irq_pending && CPU_isEI) {
			CPU_INTERRUPT(pic_ack_vector, 0);
			irq_pending = false;
			//pic_update();
		}
	//	return CPU_BASECLOCK - CPU_REMCLOCK;

#ifdef USE_DEBUGGER
	if (now_debugging) {
		if (!now_going) {
			now_suspended = true;
		}
	}
#endif
	return - CPU_REMCLOCK;
}
extern "C" __declspec(dllexport) int CPU_EXECUTE_BCC(int clockcount) {
	int cc4internal = clockcount;
	do { cc4internal -= CPU_EXECUTE_BC(clockcount); } while (cc4internal > 0);
	return cc4internal;
}

int CPU_EXECUTE_CC(int clockcount)
{
	CPU_REMCLOCK = CPU_BASECLOCK = clockcount;
#ifdef __cplusplus
	try {
#else
	switch (sigsetjmp(exec_1step_jmpbuf, 1)) {
	case 0:
		break;

	case 1:
		VERBOSE(("ia32: return from exception"));
		break;

	case 2:
		VERBOSE(("ia32: return from panic"));
		return;

	default:
		VERBOSE(("ia32: return from unknown cause"));
		break;
	}
#endif
		do {
			exec_1step();
			if (CPU_TRAP) {
				CPU_DR6 |= CPU_DR6_BS;
				INTERRUPT(1, INTR_TYPE_EXCEPTION);
			}
			else {
				if (nmi_pending) {
					INTERRUPT(2, 0);
					nmi_pending = false;
				} else if (irq_pending && CPU_isEI) {
						INTERRUPT(pic_ack_vector, 0);
						irq_pending = false;
						//pic_update();
				}
				else if (irq_pending && CPU_STAT_HLT) {
					VERBOSE(("interrupt: reset HTL in real mode"));
					CPU_EIP++;
					CPU_STAT_HLT = 0;
				}
			}
			dmax86();
		} while (CPU_REMCLOCK > 0);
#ifdef __cplusplus
	}
catch (int e) {
	switch (e) {
	case 0:
		break;

	case 1:
		VERBOSE(("ia32: return from exception"));
		break;

	case 2:
		VERBOSE(("ia32: return from panic"));
		return CPU_BASECLOCK - CPU_REMCLOCK;

	default:
		VERBOSE(("ia32: return from unknown cause"));
		break;
	}
}
#endif

		return CPU_BASECLOCK - CPU_REMCLOCK;
	/*int cc4internal = clockcount;
	while (cc4internal > 0) { cc4internal -= CPU_EXECUTE(); }
	return cc4internal;*/
#if 0
	CPU_REMCLOCK = clockcount;
	CPU_BASECLOCK = 0;
	CPU_EXEC();
		if(nmi_pending) {
			CPU_INTERRUPT(2, 0);
			nmi_pending = false;
		} else
	if (irq_pending && CPU_isEI) {
		CPU_INTERRUPT(pic_ack_vector, 0);
		irq_pending = false;
		//pic_update();
	}
	//	return CPU_BASECLOCK - CPU_REMCLOCK;

		return CPU_BASECLOCK - CPU_REMCLOCK;
#endif
}

extern "C" __declspec(dllexport) void* GET_CPU_exec_1step() { return &exec_1step; }

extern "C" __declspec(dllexport) int CPU_EXECUTE_CC_V2(int clockcount) {
	CPU_REMCLOCK = CPU_BASECLOCK = clockcount;
	exec_allstep();
	return CPU_BASECLOCK - CPU_REMCLOCK;
}

void CPU_SOFT_INTERRUPT(int vect)
{
	CPU_INTERRUPT(vect, 1);
}

void CPU_JMP_FAR(UINT16 new_cs, UINT32 new_ip)
{
	descriptor_t sd;
	UINT16 sreg;

	if (!CPU_STAT_PM || CPU_STAT_VM86) {
		/* Real mode or VM86 mode */
		/* check new instrunction pointer with new code segment */
		load_segreg(CPU_CS_INDEX, new_cs, &sreg, &sd, GP_EXCEPTION);
		if (new_ip > sd.u.seg.limit) {
			EXCEPTION(GP_EXCEPTION, 0);
		}
		LOAD_SEGREG(CPU_CS_INDEX, new_cs);
		CPU_EIP = new_ip;
	} else {
		/* Protected mode */
		JMPfar_pm(new_cs, new_ip);
	}
}

void CPU_CALL_FAR(UINT16 new_cs, UINT32 new_ip)
{
	descriptor_t sd;
	UINT16 sreg;

	if (!CPU_STAT_PM || CPU_STAT_VM86) {
		/* Real mode or VM86 mode */
		CPU_SET_PREV_ESP();
		load_segreg(CPU_CS_INDEX, new_cs, &sreg, &sd, GP_EXCEPTION);
		if (new_ip > sd.u.seg.limit) {
			EXCEPTION(GP_EXCEPTION, 0);
		}

		PUSH0_16(CPU_CS);
		PUSH0_16(CPU_EIP);

		LOAD_SEGREG(CPU_CS_INDEX, new_cs);
		CPU_EIP = new_ip;
		CPU_CLEAR_PREV_ESP();
	} else {
		/* Protected mode */
		CALLfar_pm(new_cs, new_ip);
	}
}

void CPU_IRET()
{
	// Don't call msdos_syscall() in iret routine
	UINT32 tmp = CPU_PREV_PC;
	CPU_PREV_PC = 0;
	IRET();
	CPU_PREV_PC = tmp;
}

void CPU_PUSH(UINT16 reg)
{
	if (!CPU_STAT_SS32) {
		UINT16 __new_sp = CPU_SP - 2;
		cpu_vmemorywrite_w(CPU_SS_INDEX, __new_sp, reg);
		CPU_SP = __new_sp;
	} else {
		UINT32 __new_esp = CPU_ESP - 2;
		cpu_vmemorywrite_w(CPU_SS_INDEX, __new_esp, reg);
		CPU_ESP = __new_esp;
	}
}

UINT16 CPU_POP()
{
	UINT16 reg;

	if (!CPU_STAT_SS32) {
		reg = cpu_vmemoryread_w(CPU_SS_INDEX, CPU_SP);
		CPU_SP += 2;
	} else {
		reg = cpu_vmemoryread_w(CPU_SS_INDEX, CPU_ESP);
		CPU_ESP += 2;
	}
	return reg;
}

void CPU_PUSHF()
{
	PUSHF_Fw();
}

UINT16 CPU_READ_STACK()
{
	UINT16 reg;

	if (!CPU_STAT_SS32) {
		reg = cpu_vmemoryread_w(CPU_SS_INDEX, CPU_SP);
//		CPU_SP += 2;
	} else {
		reg = cpu_vmemoryread_w(CPU_SS_INDEX, CPU_ESP);
//		CPU_ESP += 2;
	}
	return reg;
}

void CPU_WRITE_STACK(UINT16 reg)
{
	if (!CPU_STAT_SS32) {
		UINT16 __new_sp = CPU_SP - 2;
		cpu_vmemorywrite_w(CPU_SS_INDEX, __new_sp, reg);
//		CPU_SP = __new_sp;
	} else {
		UINT32 __new_esp = CPU_ESP - 2;
		cpu_vmemorywrite_w(CPU_SS_INDEX, __new_esp, reg);
//		CPU_ESP = __new_esp;
	}
}

void CPU_LOAD_LDTR(UINT16 selector)
{
	load_ldtr(selector, GP_EXCEPTION);
}

void CPU_LOAD_TR(UINT16 selector)
{
	load_tr(selector);
}

UINT32 CPU_TRANS_PAGING_ADDR(UINT32 addr)
{
	if(CPU_STAT_PM && CPU_STAT_PAGING) {
		UINT32 pde_addr = CPU_STAT_PDE_BASE + ((addr >> 20) & 0xffc);
		UINT32 pde = read_dword(pde_addr & CPU_ADRSMASK);
		/* XXX: check */
		UINT32 pte_addr = (pde & CPU_PDE_BASEADDR_MASK) + ((addr >> 10) & 0xffc);
		UINT32 pte = read_dword(pte_addr & CPU_ADRSMASK);
		/* XXX: check */
		addr = (pte & CPU_PTE_BASEADDR_MASK) + (addr & CPU_PAGE_MASK);
	}
	return addr;
}

void CPU_SET_A20(UINT8 statfora20)
{
	ia32a20enable(statfora20 != 0);
}

void CPU_REQ_INTERRUPT(int vect)
{
	/*irq_pending = TRUE;
	pic_ack_vector = vect;*/
	if (CPU_isEI) {
		CPU_INTERRUPT(vect, 0);
	}
}

void CPU_REQ_INTERRUPT_IN(int vect)
{
	irq_pending = TRUE;
	pic_ack_vector = vect;
}

void CPU_REQ_NMINTERRUPT()
{
	//nmi_pending = true;
	CPU_INTERRUPT(2, 0);
	//nmi_pending = false;
}

void CPU_REQ_NMINTERRUPT_IN()
{
	nmi_pending = true;
}

void CPU_SET_IRQ(BOOL statforirq)
{
	irq_pending = statforirq;
}

#ifdef USE_DEBUGGER
UINT32 CPU_TRANS_CODE_ADDR(UINT32 seg, UINT32 ofs)
{
	if(CPU_STAT_PM && !CPU_STAT_VM86) {
		UINT32 seg_base = 0;
		if(seg) {
			UINT32 base = (seg & 4) ? CPU_LDTR_BASE : CPU_GDTR_BASE;
			UINT32 v1 = read_dword(CPU_TRANS_PAGING_ADDR(base + (seg & ~7) + 0) & CPU_ADRSMASK);
			UINT32 v2 = read_dword(CPU_TRANS_PAGING_ADDR(base + (seg & ~7) + 4) & CPU_ADRSMASK);
			seg_base = (v2 & 0xff000000) | ((v2 & 0xff) << 16) | ((v1 >> 16) & 0xffff);
		}
		return seg_base + ofs;
	}
	return (seg << 4) + ofs;
}

UINT32 CPU_GET_PREV_PC()
{
	return CPU_PREV_PC;
}

UINT32 CPU_GET_NEXT_PC()
{
	return CPU_TRANS_CODE_ADDR(CPU_CS, CPU_EIP);
}
#endif

VC_DLL_EXPORTS void* CPU_GET_REGPTR(int reglno);

void* CPU_GET_REGPTR(int reglno) {
	switch(reglno){
	case 0:
		return (&(CPU_STATSAVE.cpu_regs.reg));
		break;
	case 1:
		return (&(CPU_STATSAVE.cpu_regs.sreg));
		break;
	case 2:
		return (&(CPU_STATSAVE.cpu_stat.sreg));
		break;
	case 3:
		return (&(CPU_STATSAVE.cpu_stat));
		break;
	case 4:
		return (&(CPU_STATSAVE));
		break;
	case 5:
		return (&(i386core));
		break;
	case 6:
		return (&(i386cpuid));
		break;
	case 7:
		return (&(i386msr));
		break;
	}
}


VC_DLL_EXPORTS void CPU_EXECUTE_INFINITY(void);
void CPU_EXECUTE_INFINITY(void) { while (true) { CPU_EXECUTE(); } }

#include "np21_i386c/ia32/inst_table.h"
#include "np21_i386c/cpumem.h"

__declspec(dllexport) void CPU_EXECUTE_BY_NUM_OF_INSTS(UINT32 noi4prm_0) {
	UINT32 noi = noi4prm_0;
	while (noi != 0) { exec_1step(); noi--; }
}


#include "np21_i386c/ia32/jit.h"
//extern UINT32 exec_jit();

extern "C" __declspec(dllexport) UINT64 CPU_EXECUTE_INJIT() {
	return exec_jit();
}
