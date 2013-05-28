/******************************************************
 * ArchC Resources Implementation file.               *
 * This file is automatically generated by ArchC      *
 * WITHOUT WARRANTY OF ANY KIND, either express       *
 * or implied.                                        *
 * For more information on ArchC, please visit:       *
 * http://www.archc.org                               *
 *                                                    *
 * The ArchC Team                                     *
 * Computer Systems Laboratory (LSC)                  *
 * IC-UNICAMP                                         *
 * http://www.lsc.ic.unicamp.br                       *
 ******************************************************/
 

#include "mips1_arch.H"

mips1_arch::mips1_arch() :
  ac_arch_dec_if<mips1_parms::ac_word, mips1_parms::ac_Hword>(mips1_parms::AC_MAX_BUFFER),
  ac_pc("ac_pc", 0),
  DM_stg("DM_stg", 5242880U),
  DM(*this, DM_stg),
  RB("RB"),
  npc("npc", 0),
  hi("hi", 0),
  lo("lo", 0) {

  ac_mt_endian = mips1_parms::AC_MATCH_ENDIAN;
  ac_tgt_endian = mips1_parms::AC_PROC_ENDIAN;

  IM = &DM;
  APP_MEM = &DM;

}

