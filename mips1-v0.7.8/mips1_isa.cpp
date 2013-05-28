/**
 * @file      mips1_isa.cpp
 * @author    Sandro Rigo
 *            Marcus Bartholomeu
 *            Alexandro Baldassin (acasm information)
 *
 *            The ArchC Team
 *            http://www.archc.org/
 *
 *            Computer Systems Laboratory (LSC)
 *            IC-UNICAMP
 *            http://www.lsc.ic.unicamp.br/
 *
 * @version   1.0
 * @date      Mon, 19 Jun 2006 15:50:52 -0300
 *
 * @brief     The ArchC i8051 functional model.
 *
 * @attention Copyright (C) 2002-2006 --- The ArchC Team
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include  "mips1_isa.H"
#include  "mips1_isa_init.cpp"
#include  "mips1_bhv_macros.H"

// estagios do pipeline, usados no vetor do historico
#define IFID 0
#define IDEX 1
#define EXMEM 2
#define MEMWB 3

// constantes da cache
#define LINE_NUMBER 128
#define BLOCK_NUMBER 4

//If you want debug information for this model, uncomment next line
//#define DEBUG_MODEL
#include "ac_debug_model.H"

// User defined macros to reference registers.
#define Ra 31
#define Sp 29

// 'using namespace' statement to allow access to all
// mips1-specific datatypes
using namespace mips1_parms;

// historico de registradores nos estagios do pipeline
int hist_rd[4]={-1,-1,-1,-1};
int hist_rs[4]={-1,-1,-1,-1};
int hist_rt[4]={-1,-1,-1,-1};

// historico de sinais reg_write e mem_read nos estagios do pipeline
int hist_rw[4]={0,0,0,0};
int hist_mr[4]={0,0,0,0};

// maquina de estados do branch predictor
typedef enum{no0, no1, yes0, yes1} state_machine;
state_machine predictor = no0; // inicia como not_taken

// struct da cache
typedef struct{
  int dirty; // algo foi escrito nesse endereco desde a leitura?
  int valid; // o endereco esta sendo usado ou vazio?
  int tag;   // tag para comparacao
} cache_t;
cache_t data_cache[LINE_NUMBER][BLOCK_NUMBER], inst_cache[LINE_NUMBER][BLOCK_NUMBER];


// declaração dos contadores
unsigned int instr_R = 0; // tipo r
unsigned int instr_I = 0; // tipo i
unsigned int instr_J = 0; // tipo j

unsigned int reg_write = 0; // escrita em registrador

unsigned int mem_write = 0; // escrita em memoria
unsigned int mem_read = 0;  // leitura em memoria

unsigned int branch = 0; // instrucao de branch
unsigned int jump = 0;   // instrucao de jump

unsigned int data_fwd = 0; // forward
unsigned int data_stl = 0; // stalls
unsigned int br_stl_1 = 0; // stalls com operacao R antecedendo branch
unsigned int br_stl_2 = 0; // stalls com operacao I antecedendo branch

unsigned int bp_win = 0;  // acertos do predictor
unsigned int bp_lose = 0; // erros do preditor

unsigned int cache_miss = 0; // misses do cache
unsigned int cache_hit = 0;  // hits do cache
unsigned int cache_wb = 0;   // write_backs na memoria


// declaracao dos headers
// atualiza estagios do pipeline
void hist_update(int rd, int rs, int rt, int rw, int mr);
// imprime estado dos estagios do pipeline
void hist_print();
// verifica se ocorreu hazard resolvido com forward
void fwd_update();
// verifica se ocorreu hazard resolvido com stall
void stl_update();
// verifica se ocorreu hazard em branch
void stl_branch();
// branch prediction state machine, atualiza o estado a cada instrucao
void bpsm_update(int ac_pc, int npc);
// inicializa o cache vazio
void cache_init(cache_t cache[LINE_NUMBER][BLOCK_NUMBER]);
// realiza leitura no cache, se necessario write-back
void cache_read(cache_t cache[LINE_NUMBER][BLOCK_NUMBER], int addr);
// realiza escrita no cache, atualiza dirty, se necessario write-back
void cache_write(cache_t cache[LINE_NUMBER][BLOCK_NUMBER], int addr);

//!Generic instruction behavior method.
void ac_behavior( instruction )
{
  dbg_printf("----- PC=%#x ----- %lld\n", (int) ac_pc, ac_instr_counter);
  //  dbg_printf("----- PC=%#x NPC=%#x ----- %lld\n", (int) ac_pc, (int)npc, ac_instr_counter);
#ifndef NO_NEED_PC_UPDATE

  // atualiza o estado do predictor
  bpsm_update(ac_pc,npc);

  ac_pc = npc;
  npc = ac_pc + 4;

  // tenta ler da cache de instrucoes
  cache_read(inst_cache, ac_pc);
#endif
};

//! Instruction Format behavior methods.

// _rw = operacoes RegWrite
void ac_behavior( Type_Rrw ){
  instr_R++;
  reg_write++;

  // atualiza estagios do pipeline
  hist_update (rs, rt, rd, 1, 0);
  // verifica forwards
  fwd_update();
}

void ac_behavior( Type_Irw ){
  instr_I++;
  reg_write++;

  // atualiza estagios do pipeline
  hist_update (rs, rt, -1, 1, 0);
  // verifica forwards
  fwd_update();
}

// _mr = operacoes MemRead
void ac_behavior( Type_Imr ){
  instr_I++;
  mem_read++;

  // atualiza estagios do pipeline
  hist_update (rs, rt, -1, 0, 1);
  // verifica forwards
  fwd_update();
  // verifica stalls
  stl_update();
  // leitura do cache
  cache_read(data_cache, RB[rs]);
}

// _mw = operacoes MemWrite
void ac_behavior( Type_Imw ){
  instr_I++;
  mem_write++;

  // escrita ao cache
  cache_write(data_cache, RB[rs]);
}
// _c = Operacoes de Controle
void ac_behavior( Type_Rc ){  instr_R++; stl_branch(); branch++;}
void ac_behavior( Type_Ic ){  instr_I++; stl_branch(); branch++;}
void ac_behavior( Type_Jc ){  instr_J++; jump++;}
// _c = Operacoes de Sistema
void ac_behavior( Type_Rs ){  instr_R++;}

//!Behavior called before starting simulation
void ac_behavior(begin)
{
  dbg_printf("@@@ begin behavior @@@\n");
  RB[0] = 0;
  npc = ac_pc + 4;

  // Is is not required by the architecture, but makes debug really easier
  for (int regNum = 0; regNum < 32; regNum ++)
    RB[regNum] = 0;
  hi = 0;
  lo = 0;

  // cache init
  cache_init(data_cache);
}

//!Behavior called after finishing simulation
void ac_behavior(end)
{
  dbg_printf("@@@ end behavior @@@\n");
  ofstream f;
  f.open ("mips_test.txt");;
  int instr_count = instr_R+instr_I+instr_J;

  cout << "Type R:\t" << instr_R << "\n";
  cout << "Type I:\t" << instr_I << "\n";
  cout << "Type J:\t" << instr_J << "\n";
  cout << "Total:\t" << instr_count << "\n";

  cout << "RegWrite:\t" << reg_write << "\n";
  cout << "Forwards:\t" << data_fwd << "\n";
  cout << "Stalls:\t" << data_stl + br_stl_1 + br_stl_2 << "\n";

  cout << "Branches:\t" << branch << "\n";
  cout << "Jumps:\t" << jump << "\n";
  cout << "SM success:\t" << bp_win << "\n";
  cout << "SM fail:\t" << bp_lose << "\n";
  cout << "SM fail rate:\t" << float(bp_lose)/float(bp_win+bp_lose) << "\n";

  cout << "MemRead:\t" << mem_read << "\n";
  cout << "MemWrite:\t" << mem_write << "\n";
  cout << "Cache Hit:\t" << cache_hit << "\n";
  cout << "Cache Miss:\t" << cache_miss << "\n";
  cout << "WB Count:\t" << cache_wb << "\n";
  cout << "Miss rate:\t" << float(cache_miss)/(float(cache_hit)+float(cache_miss)) << "\n";
  cout << "WB rate:\t" << float(cache_wb)/(float(mem_read)+float(mem_write)) << "\n";

  // ----------
  // Atribuit peso com numero de ciclos que cada tipo de hazard adiciona
  // ----------
  int cycle_count = instr_R + instr_I + instr_J + 1*data_stl + 1*br_stl_1 + 2*br_stl_2 + 1*bp_lose + 100*cache_miss + 100*cache_wb;
  // ----------

  f << "Type R:\t" << instr_R << "\n";
  f << "Type I:\t" << instr_I << "\n";
  f << "Type J:\t" << instr_J << "\n";
  f << "Total:\t" << instr_count << "\n";
  f << "RegWrite:\t" << reg_write << "\n";
  f << "Forwards:\t" << data_fwd << "\n";
  f << "Stalls:\t" << data_stl + br_stl_1 + br_stl_2 << "\n";
  f << "Branches:\t" << branch << "\n";
  f << "Jumps:\t" << jump << "\n";
  f << "SM success:\t" << bp_win << "\n";
  f << "SM fail:\t" << bp_lose << "\n";
  f << "SM fail rate:\t" << float(bp_lose)/float(bp_win+bp_lose) << "\n";
  f << "MemRead:\t" << mem_read << "\n";
  f << "MemWrite:\t" << mem_write << "\n";
  f << "Cache Hit:\t" << cache_hit << "\n";
  f << "Cache Miss:\t" << cache_miss << "\n";
  f << "WB Count:\t" << cache_wb << "\n";
  f << "Miss rate:\t" << float(cache_miss)/(float(cache_miss)+float(cache_hit)) << "\n";
  f << "WB rate:\t" << float(cache_wb)/(float(mem_read)+float(mem_write)) << "\n";
  f << "Cycles:\t" << cycle_count << "\n";
  f << "CPI:\t" << float(cycle_count)/(float)instr_count << "\n";

  f.close();
}


//!Instruction lb behavior method.
void ac_behavior( lb )
{
  char byte;
  dbg_printf("lb r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  byte = DM.read_byte(RB[rs]+ imm);
  RB[rt] = (ac_Sword)byte ;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lbu behavior method.
void ac_behavior( lbu )
{
  unsigned char byte;
  dbg_printf("lbu r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  byte = DM.read_byte(RB[rs]+ imm);
  RB[rt] = byte ;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lh behavior method.
void ac_behavior( lh )
{
  short int half;
  dbg_printf("lh r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  half = DM.read_half(RB[rs]+ imm);
  RB[rt] = (ac_Sword)half ;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lhu behavior method.
void ac_behavior( lhu )
{
  unsigned short int  half;
  half = DM.read_half(RB[rs]+ imm);
  RB[rt] = half ;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lw behavior method.
void ac_behavior( lw )
{
  dbg_printf("lw r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  RB[rt] = DM.read(RB[rs]+ imm);
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lwl behavior method.
void ac_behavior( lwl )
{
  dbg_printf("lwl r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  unsigned int addr, offset;
  ac_Uword data;

  addr = RB[rs] + imm;
  offset = (addr & 0x3) * 8;
  data = DM.read(addr & 0xFFFFFFFC);
  data <<= offset;
  data |= RB[rt] & ((1<<offset)-1);
  RB[rt] = data;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lwr behavior method.
void ac_behavior( lwr )
{
  dbg_printf("lwr r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  unsigned int addr, offset;
  ac_Uword data;

  addr = RB[rs] + imm;
  offset = (3 - (addr & 0x3)) * 8;
  data = DM.read(addr & 0xFFFFFFFC);
  data >>= offset;
  data |= RB[rt] & (0xFFFFFFFF << (32-offset));
  RB[rt] = data;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction sb behavior method.
void ac_behavior( sb )
{
  unsigned char byte;
  dbg_printf("sb r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  byte = RB[rt] & 0xFF;
  DM.write_byte(RB[rs] + imm, byte);
  dbg_printf("Result = %#x\n", (int) byte);
};

//!Instruction sh behavior method.
void ac_behavior( sh )
{
  unsigned short int half;
  dbg_printf("sh r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  half = RB[rt] & 0xFFFF;
  DM.write_half(RB[rs] + imm, half);
  dbg_printf("Result = %#x\n", (int) half);
};

//!Instruction sw behavior method.
void ac_behavior( sw )
{
  dbg_printf("sw r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  DM.write(RB[rs] + imm, RB[rt]);
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction swl behavior method.
void ac_behavior( swl )
{
  dbg_printf("swl r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  unsigned int addr, offset;
  ac_Uword data;

  addr = RB[rs] + imm;
  offset = (addr & 0x3) * 8;
  data = RB[rt];
  data >>= offset;
  data |= DM.read(addr & 0xFFFFFFFC) & (0xFFFFFFFF << (32-offset));
  DM.write(addr & 0xFFFFFFFC, data);
  dbg_printf("Result = %#x\n", data);
};

//!Instruction swr behavior method.
void ac_behavior( swr )
{
  dbg_printf("swr r%d, %d(r%d)\n", rt, imm & 0xFFFF, rs);
  unsigned int addr, offset;
  ac_Uword data;

  addr = RB[rs] + imm;
  offset = (3 - (addr & 0x3)) * 8;
  data = RB[rt];
  data <<= offset;
  data |= DM.read(addr & 0xFFFFFFFC) & ((1<<offset)-1);
  DM.write(addr & 0xFFFFFFFC, data);
  dbg_printf("Result = %#x\n", data);
};

//!Instruction addi behavior method.
void ac_behavior( addi )
{
  dbg_printf("addi r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  RB[rt] = RB[rs] + imm;
  dbg_printf("Result = %#x\n", RB[rt]);
  //Test overflow
  if ( ((RB[rs] & 0x80000000) == (imm & 0x80000000)) &&
       ((imm & 0x80000000) != (RB[rt] & 0x80000000)) ) {
    fprintf(stderr, "EXCEPTION(addi): integer overflow.\n"); exit(EXIT_FAILURE);
  }
};

//!Instruction addiu behavior method.
void ac_behavior( addiu )
{
  dbg_printf("addiu r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  RB[rt] = RB[rs] + imm;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction slti behavior method.
void ac_behavior( slti )
{
  dbg_printf("slti r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  // Set the RD if RS< IMM
  if( (ac_Sword) RB[rs] < (ac_Sword) imm )
    RB[rt] = 1;
  // Else reset RD
  else
    RB[rt] = 0;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction sltiu behavior method.
void ac_behavior( sltiu )
{
  dbg_printf("sltiu r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  // Set the RD if RS< IMM
  if( (ac_Uword) RB[rs] < (ac_Uword) imm )
    RB[rt] = 1;
  // Else reset RD
  else
    RB[rt] = 0;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction andi behavior method.
void ac_behavior( andi )
{
  dbg_printf("andi r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  RB[rt] = RB[rs] & (imm & 0xFFFF) ;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction ori behavior method.
void ac_behavior( ori )
{
  dbg_printf("ori r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  RB[rt] = RB[rs] | (imm & 0xFFFF) ;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction xori behavior method.
void ac_behavior( xori )
{
  dbg_printf("xori r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  RB[rt] = RB[rs] ^ (imm & 0xFFFF) ;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction lui behavior method.
void ac_behavior( lui )
{
  dbg_printf("lui r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  // Load a constant in the upper 16 bits of a register
  // To achieve the desired behaviour, the constant was shifted 16 bits left
  // and moved to the target register ( rt )
  RB[rt] = imm << 16;
  dbg_printf("Result = %#x\n", RB[rt]);
};

//!Instruction add behavior method.
void ac_behavior( add )
{
  dbg_printf("add r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] + RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
  //Test overflow
  if ( ((RB[rs] & 0x80000000) == (RB[rd] & 0x80000000)) &&
       ((RB[rd] & 0x80000000) != (RB[rt] & 0x80000000)) ) {
    fprintf(stderr, "EXCEPTION(add): integer overflow.\n"); exit(EXIT_FAILURE);
  }
};

//!Instruction addu behavior method.
void ac_behavior( addu )
{
  dbg_printf("addu r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] + RB[rt];
  //cout << "  RS: " << (unsigned int)RB[rs] << " RT: " << (unsigned int)RB[rt] << endl;
  //cout << "  Result =  " <<  (unsigned int)RB[rd] <<endl;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction sub behavior method.
void ac_behavior( sub )
{
  dbg_printf("sub r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] - RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
  //TODO: test integer overflow exception for sub
};

//!Instruction subu behavior method.
void ac_behavior( subu )
{
  dbg_printf("subu r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] - RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction slt behavior method.
void ac_behavior( slt )
{
  dbg_printf("slt r%d, r%d, r%d\n", rd, rs, rt);
  // Set the RD if RS< RT
  if( (ac_Sword) RB[rs] < (ac_Sword) RB[rt] )
    RB[rd] = 1;
  // Else reset RD
  else
    RB[rd] = 0;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction sltu behavior method.
void ac_behavior( sltu )
{
  dbg_printf("sltu r%d, r%d, r%d\n", rd, rs, rt);
  // Set the RD if RS < RT
  if( RB[rs] < RB[rt] )
    RB[rd] = 1;
  // Else reset RD
  else
    RB[rd] = 0;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction instr_and behavior method.
void ac_behavior( instr_and )
{
  dbg_printf("instr_and r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] & RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction instr_or behavior method.
void ac_behavior( instr_or )
{
  dbg_printf("instr_or r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] | RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction instr_xor behavior method.
void ac_behavior( instr_xor )
{
  dbg_printf("instr_xor r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = RB[rs] ^ RB[rt];
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction instr_nor behavior method.
void ac_behavior( instr_nor )
{
  dbg_printf("nor r%d, r%d, r%d\n", rd, rs, rt);
  RB[rd] = ~(RB[rs] | RB[rt]);
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction nop behavior method.
void ac_behavior( nop )
{
  dbg_printf("nop\n");
};

//!Instruction sll behavior method.
void ac_behavior( sll )
{
  dbg_printf("sll r%d, r%d, %d\n", rd, rs, shamt);
  RB[rd] = RB[rt] << shamt;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction srl behavior method.
void ac_behavior( srl )
{
  dbg_printf("srl r%d, r%d, %d\n", rd, rs, shamt);
  RB[rd] = RB[rt] >> shamt;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction sra behavior method.
void ac_behavior( sra )
{
  dbg_printf("sra r%d, r%d, %d\n", rd, rs, shamt);
  RB[rd] = (ac_Sword) RB[rt] >> shamt;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction sllv behavior method.
void ac_behavior( sllv )
{
  dbg_printf("sllv r%d, r%d, r%d\n", rd, rt, rs);
  RB[rd] = RB[rt] << (RB[rs] & 0x1F);
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction srlv behavior method.
void ac_behavior( srlv )
{
  dbg_printf("srlv r%d, r%d, r%d\n", rd, rt, rs);
  RB[rd] = RB[rt] >> (RB[rs] & 0x1F);
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction srav behavior method.
void ac_behavior( srav )
{
  dbg_printf("srav r%d, r%d, r%d\n", rd, rt, rs);
  RB[rd] = (ac_Sword) RB[rt] >> (RB[rs] & 0x1F);
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction mult behavior method.
void ac_behavior( mult )
{
  dbg_printf("mult r%d, r%d\n", rs, rt);

  long long result;
  int half_result;

  result = (ac_Sword) RB[rs];
  result *= (ac_Sword) RB[rt];

  half_result = (result & 0xFFFFFFFF);
  // Register LO receives 32 less significant bits
  lo = half_result;

  half_result = ((result >> 32) & 0xFFFFFFFF);
  // Register HI receives 32 most significant bits
  hi = half_result ;

  dbg_printf("Result = %#llx\n", result);
};

//!Instruction multu behavior method.
void ac_behavior( multu )
{
  dbg_printf("multu r%d, r%d\n", rs, rt);

  unsigned long long result;
  unsigned int half_result;

  result  = RB[rs];
  result *= RB[rt];

  half_result = (result & 0xFFFFFFFF);
  // Register LO receives 32 less significant bits
  lo = half_result;

  half_result = ((result>>32) & 0xFFFFFFFF);
  // Register HI receives 32 most significant bits
  hi = half_result ;

  dbg_printf("Result = %#llx\n", result);
};

//!Instruction div behavior method.
void ac_behavior( div )
{
  dbg_printf("div r%d, r%d\n", rs, rt);
  // Register LO receives quotient
  lo = (ac_Sword) RB[rs] / (ac_Sword) RB[rt];
  // Register HI receives remainder
  hi = (ac_Sword) RB[rs] % (ac_Sword) RB[rt];
};

//!Instruction divu behavior method.
void ac_behavior( divu )
{
  dbg_printf("divu r%d, r%d\n", rs, rt);
  // Register LO receives quotient
  lo = RB[rs] / RB[rt];
  // Register HI receives remainder
  hi = RB[rs] % RB[rt];
};

//!Instruction mfhi behavior method.
void ac_behavior( mfhi )
{
  dbg_printf("mfhi r%d\n", rd);
  RB[rd] = hi;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction mthi behavior method.
void ac_behavior( mthi )
{
  dbg_printf("mthi r%d\n", rs);
  hi = RB[rs];
  dbg_printf("Result = %#x\n", hi);
};

//!Instruction mflo behavior method.
void ac_behavior( mflo )
{
  dbg_printf("mflo r%d\n", rd);
  RB[rd] = lo;
  dbg_printf("Result = %#x\n", RB[rd]);
};

//!Instruction mtlo behavior method.
void ac_behavior( mtlo )
{
  dbg_printf("mtlo r%d\n", rs);
  lo = RB[rs];
  dbg_printf("Result = %#x\n", lo);
};

//!Instruction j behavior method.
void ac_behavior( j )
{
  dbg_printf("j %d\n", addr);
  addr = addr << 2;
#ifndef NO_NEED_PC_UPDATE
  npc =  (ac_pc & 0xF0000000) | addr;
#endif
  dbg_printf("Target = %#x\n", (ac_pc & 0xF0000000) | addr );
};

//!Instruction jal behavior method.
void ac_behavior( jal )
{
  dbg_printf("jal %d\n", addr);
  // Save the value of PC + 8 (return address) in $ra ($31) and
  // jump to the address given by PC(31...28)||(addr<<2)
  // It must also flush the instructions that were loaded into the pipeline
  RB[Ra] = ac_pc+4; //ac_pc is pc+4, we need pc+8

  addr = addr << 2;
#ifndef NO_NEED_PC_UPDATE
  npc = (ac_pc & 0xF0000000) | addr;
#endif

  dbg_printf("Target = %#x\n", (ac_pc & 0xF0000000) | addr );
  dbg_printf("Return = %#x\n", ac_pc+4);
};

//!Instruction jr behavior method.
void ac_behavior( jr )
{
  dbg_printf("jr r%d\n", rs);
  // Jump to the address stored on the register reg[RS]
  // It must also flush the instructions that were loaded into the pipeline
#ifndef NO_NEED_PC_UPDATE
  npc = RB[rs], 1;
#endif
  dbg_printf("Target = %#x\n", RB[rs]);
};

//!Instruction jalr behavior method.
void ac_behavior( jalr )
{
  dbg_printf("jalr r%d, r%d\n", rd, rs);
  // Save the value of PC + 8(return address) in rd and
  // jump to the address given by [rs]

#ifndef NO_NEED_PC_UPDATE
  npc = RB[rs], 1;
#endif
  dbg_printf("Target = %#x\n", RB[rs]);

  if( rd == 0 )  //If rd is not defined use default
    rd = Ra;
  RB[rd] = ac_pc+4;
  dbg_printf("Return = %#x\n", ac_pc+4);
};

//!Instruction beq behavior method.
void ac_behavior( beq )
{
  dbg_printf("beq r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  if( RB[rs] == RB[rt] ){
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm<<2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm<<2));
  }
};

//!Instruction bne behavior method.
void ac_behavior( bne )
{
  dbg_printf("bne r%d, r%d, %d\n", rt, rs, imm & 0xFFFF);
  if( RB[rs] != RB[rt] ){
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm<<2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm<<2));
  }
};

//!Instruction blez behavior method.
void ac_behavior( blez )
{
  dbg_printf("blez r%d, %d\n", rs, imm & 0xFFFF);
  if( (RB[rs] == 0 ) || (RB[rs]&0x80000000 ) ){
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm<<2), 1;
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm<<2));
  }
};

//!Instruction bgtz behavior method.
void ac_behavior( bgtz )
{
  dbg_printf("bgtz r%d, %d\n", rs, imm & 0xFFFF);
  if( !(RB[rs] & 0x80000000) && (RB[rs]!=0) ){
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm<<2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm<<2));
  }
};

//!Instruction bltz behavior method.
void ac_behavior( bltz )
{
  dbg_printf("bltz r%d, %d\n", rs, imm & 0xFFFF);
  if( RB[rs] & 0x80000000 ){
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm<<2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm<<2));
  }
};

//!Instruction bgez behavior method.
void ac_behavior( bgez )
{
  dbg_printf("bgez r%d, %d\n", rs, imm & 0xFFFF);
  if( !(RB[rs] & 0x80000000) ){
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm<<2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm<<2));
  }
};

//!Instruction bltzal behavior method.
void ac_behavior( bltzal )
{
  dbg_printf("bltzal r%d, %d\n", rs, imm & 0xFFFF);
  RB[Ra] = ac_pc+4; //ac_pc is pc+4, we need pc+8
  if( RB[rs] & 0x80000000 ){
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm<<2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm<<2));
  }
  dbg_printf("Return = %#x\n", ac_pc+4);
};

//!Instruction bgezal behavior method.
void ac_behavior( bgezal )
{
  dbg_printf("bgezal r%d, %d\n", rs, imm & 0xFFFF);
  RB[Ra] = ac_pc+4; //ac_pc is pc+4, we need pc+8
  if( !(RB[rs] & 0x80000000) ){
#ifndef NO_NEED_PC_UPDATE
    npc = ac_pc + (imm<<2);
#endif
    dbg_printf("Taken to %#x\n", ac_pc + (imm<<2));
  }
  dbg_printf("Return = %#x\n", ac_pc+4);
};

//!Instruction sys_call behavior method.
void ac_behavior( sys_call )
{
  dbg_printf("syscall\n");
  stop();
}

//!Instruction instr_break behavior method.
void ac_behavior( instr_break )
{
  fprintf(stderr, "instr_break behavior not implemented.\n");
  exit(EXIT_FAILURE);
}

// atualiza estagios dos pipelines
void hist_update(int rd, int rs, int rt, int rw, int mr)
{
  hist_rd[MEMWB]=hist_rd[EXMEM];
  hist_rs[MEMWB]=hist_rs[EXMEM];
  hist_rt[MEMWB]=hist_rt[EXMEM];
  hist_rw[MEMWB]=hist_rw[EXMEM];
  hist_mr[MEMWB]=hist_mr[EXMEM];

  hist_rd[EXMEM]=hist_rd[IDEX];
  hist_rs[EXMEM]=hist_rs[IDEX];
  hist_rt[EXMEM]=hist_rt[IDEX];
  hist_rw[EXMEM]=hist_rw[IDEX];
  hist_mr[EXMEM]=hist_mr[IDEX];

  hist_rd[IDEX]=hist_rd[IFID];
  hist_rs[IDEX]=hist_rs[IFID];
  hist_rt[IDEX]=hist_rt[IFID];
  hist_rw[IDEX]=hist_rw[IFID];
  hist_mr[IDEX]=hist_mr[IFID];


  hist_rd[IFID]=rd;
  hist_rs[IFID]=rs;
  hist_rt[IFID]=rt;
  hist_rw[IFID]=rw;
  hist_mr[IFID]=mr;
}

// imprime historico de registradoers
void hist_print()
{
   cout << "Rd: " << hist_rd[0] << "\t" << hist_rd[1] << "\t" << hist_rd[2] << hist_rd[3] << "\n";
    cout << "Rs: " << hist_rs[0] << "\t" << hist_rs[1] << "\t" << hist_rs[2] << hist_rs[3] << "\n";
    cout << "Rt: " << hist_rt[0] << "\t" << hist_rt[1] << "\t" << hist_rt[2] << "\t" << hist_rt[3] << "\n\n";
}

// verifica hazards de RegWrite tratados com Forward
void fwd_update()
{
  if ((hist_rw[EXMEM]) && (hist_rd[EXMEM] >= 0) &&
      (hist_rd[EXMEM] == hist_rs[IDEX]))
    data_fwd++;

  if ((hist_rw[EXMEM]) && (hist_rd[EXMEM] >= 0) &&
      (hist_rd[EXMEM] == hist_rt[IDEX]))
    data_fwd++;

  if ((hist_rw[MEMWB])
      && (hist_rd[MEMWB] >= 0)
      && !(hist_rw[EXMEM] && (hist_rd[EXMEM] > 0)
	   && (hist_rd[EXMEM] == hist_rs[IDEX]))
      && (hist_rd[MEMWB] == hist_rs[IDEX]))
    data_fwd++;

  if ((hist_rw[MEMWB])
      && (hist_rd[MEMWB] >= 0)
      && !(hist_rw[EXMEM] && (hist_rd[EXMEM] > 0)
	   && (hist_rd[EXMEM] == hist_rt[IDEX]))
      && (hist_rd[MEMWB] == hist_rt[IDEX]))
    data_fwd++;
}

// verifica hazards de RegWrite tratados com Stall
void stl_update()
{
  if ((hist_mr[IDEX])
      && ((hist_rt[IDEX] == hist_rs[IFID]) ||
	  (hist_rt[IDEX] == hist_rt[IFID])))
      data_stl;
}

// verifica hazards em branchs
void stl_branch()
{
  // se há fwd em operacoes aritmeticas, ha stall para branch
  // porque os branchs sao resolvidos 1 ciclo antes
  if ((hist_rw[EXMEM]) && (hist_rd[EXMEM] >= 0) &&
      (hist_rd[EXMEM] == hist_rs[IDEX]))
    br_stl_1++;

  if ((hist_rw[EXMEM]) && (hist_rd[EXMEM] >= 0) &&
      (hist_rd[EXMEM] == hist_rt[IDEX]))
    br_stl_1++;

  if ((hist_rw[MEMWB])
      && (hist_rd[MEMWB] >= 0)
      && !(hist_rw[EXMEM] && (hist_rd[EXMEM] > 0)
	   && (hist_rd[EXMEM] == hist_rs[IDEX]))
      && (hist_rd[MEMWB] == hist_rs[IDEX]))
    br_stl_1++;

  if ((hist_rw[MEMWB])
      && (hist_rd[MEMWB] >= 0)
      && !(hist_rw[EXMEM] && (hist_rd[EXMEM] > 0)
	   && (hist_rd[EXMEM] == hist_rt[IDEX]))
      && (hist_rd[MEMWB] == hist_rt[IDEX]))
    br_stl_1++;

  // se ha stall nas aritmeticas, ha 2 stalls nos branchs
  // mesmo motivo
  if ((hist_mr[IDEX])
      && ((hist_rt[IDEX] == hist_rs[IFID]) ||
	  (hist_rt[IDEX] == hist_rt[IFID])))
    br_stl_2++;
}

// inicializa o cache com enderecos invalidos e vazios
void cache_init(cache_t cache[LINE_NUMBER][BLOCK_NUMBER]){
  int i,j,k;
  for (i = 0 ; i < LINE_NUMBER ; i++){
    for (j = 0 ; j < BLOCK_NUMBER ; j++){
      cache[i][j].valid = 0;
      cache[i][j].dirty = 0;
      cache[i][j].tag = -1;
    }
  }
}

// faz leitura do cache
void cache_read(cache_t cache[LINE_NUMBER][BLOCK_NUMBER], int addr)
{
  int i,j;
  int hit = 0;

  // desloca 2 bits para dividir por 4 (e representar em bytes)
  int byte_offset = addr >> 2;
  // desloca o numero de bits para o posicionamento em blocos
  int block_offset = byte_offset >> int(sqrt(BLOCK_NUMBER));
  // identifica a tag de uma palavra a ser lida
  int tag = block_offset >> int(sqrt(LINE_NUMBER));
  // identifica a posicao de uma palavra a ser lida
  i = block_offset % LINE_NUMBER;

  // busca em todos os blocos de determinada posicao i
  for( j = 0 ; j < BLOCK_NUMBER ; j++ ){
    // caso esteja ocupado, verifica tag por hit
    if (cache[i][j].valid && cache[i][j].tag == tag){
      hit = 1;
      cache_hit++;
    }
  }

  // em caso de miss atualiza o cache
  if (!hit){
    cache_miss++;
    for( j = 0 ; j < BLOCK_NUMBER ; j++ ){
      // busca bloco vazio ou posiciona no ultimo bloco
      if( cache[i][j].valid == 0 || j == BLOCK_NUMBER-1 ){
	// se estiver dirty, escreve de volta na memoria
	if( cache[i][j].dirty ==1 )
	  cache_wb++;
	// carrega no cache endereco valido e limpo
	cache[i][j].valid = 1;
	cache[i][j].dirty = 0;
	cache[i][j].tag = tag;
      }
    }
  }
}

void cache_write(cache_t cache[LINE_NUMBER][BLOCK_NUMBER], int addr)
{
  int i,j;
  int hit = 0;

  // desloca 2 bits para dividir por 4 (e representar em bytes)
  int byte_offset = addr >> 2;
  // desloca o numero de bits para o posicionamento em blocos
  int block_offset = byte_offset >> int(sqrt(BLOCK_NUMBER));
  // identifica a tag de uma palavra a ser lida
  int tag = block_offset >> int(sqrt(LINE_NUMBER));
  // identifica a posicao de uma palavra a ser lida
  i = block_offset % LINE_NUMBER;

  // busca em todos os blocos de determinada posicao i
  for( j = 0 ; j < BLOCK_NUMBER ; j++ ){
    // caso esteja ocupado, verifica tag por hit
    if (cache[i][j].valid && cache[i][j].tag == tag){
      hit = 1;
      cache[i][j].dirty = 1;
      cache_hit++;
    }
  }

  // em caso de miss atualiza o cache
  if (!hit){
    cache_miss++;
    for( j = 0 ; j < BLOCK_NUMBER ; j++ ){
      // busca bloco vazio ou posiciona no ultimo bloco
      if( cache[i][j].valid == 0 || j == BLOCK_NUMBER-1 ){
	// se estiver dirty, escreve de volta na memoria
	if( cache[i][j].dirty == 1 )
	  cache_wb++;
	// carrega no cache endereco valido e atualiza como dirty
	cache[i][j].valid = 1;
	cache[i][j].dirty = 1;
	cache[i][j].tag = tag;
      }
    }
  }
}

// atualizadao dos estados do branch predictor
void bpsm_update(int ac_pc, int npc){
  // se a proxima posicao for a atual + 4, nao ocorreu branch
  if( npc == ac_pc + 4 ){
    switch( predictor ){
    case no0:
      // caso o estado seja not_taken, acertou a predicao
      bp_win++;
      break;
    case no1:
      // no modelo 2 bits, atualiza o not_taken1 para not_taken0
      predictor=no0;
      bp_win++;
      break;
    case yes0:
      // caso o estado seja taken, errou a predicao e atualiza
      predictor=no1;
      bp_lose++;
      break;
    case yes1:
      // caso o estado seja taken, errou a predicao e atualiza
      predictor=yes1;
      bp_lose++;
      break;
    }
  }
  // se a proxima posicao nao for a atual + 4, ocorreu branch
  else{
    switch( predictor ){
    case yes1:
      // caso o estado seja taken, acertou a predicao
      bp_win=yes1;
      break;
    case yes0:
      // no modelo 2 bits, atualiza o taken1 para taken0
      predictor=yes0;
      bp_win++;
      break;
    case no1:
      // caso o estado seja not_taken, errou a predicao e atualiza
      predictor=yes1;
      bp_lose++;
      break;
    case no0:
      // caso o estado seja not_taken, errou a predicao e atualiza
      predictor=no1;
      bp_lose++;
      break;
    }
  }
}
