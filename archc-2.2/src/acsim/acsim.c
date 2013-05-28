/* -*- Mode: C; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

/**
 * @file      acsim.c
 * @author    Sandro Rigo
 *
 *            The ArchC Team
 *            http://www.archc.org/
 *
 *            Computer Systems Laboratory (LSC)
 *            IC-UNICAMP
 *            http://www.lsc.ic.unicamp.br/
 *
 * @version   1.0
 * @date      Mon, 19 Jun 2006 15:33:20 -0300
 *
 * @brief     The ArchC pre-processor.
 *            This file contains functions to control the ArchC
 *            to emit the source files that compose the behavioral
 *            simulator.
 *
 * @attention Copyright (C) 2002-2006 --- The ArchC Team
 *
 */

#include "acsim.h"
#include "stdlib.h"
#include "string.h"


//#define DEBUG_STORAGE

//Defining Traces and Dasm strings
#define PRINT_TRACE "%strace_file << hex << decode_pc << dec <<\"\\n\";\n"

//Command-line options flags
int  ACABIFlag=0;                               //!<Indicates whether an ABI was provided or not
int  ACDebugFlag=0;                             //!<Indicates whether debugger option is turned on or not
int  ACDecCacheFlag=1;                          //!<Indicates whether the simulator will cache decoded instructions or not
int  ACDelayFlag=0;                             //!<Indicates whether delay option is turned on or not
int  ACDDecoderFlag=0;                          //!<Indicates whether decoder structures are dumped or not
//int  ACQuietFlag=0;                             //!<Indicates whether storage update logs are displayed during simulation or not
int  ACStatsFlag=0;                             //!<Indicates whether statistics collection is enable or not
int  ACVerboseFlag=0;                           //!<Indicates whether verbose option is turned on or not
int  ACVerifyFlag=0;                            //!<Indicates whether verification option is turned on or not
int  ACVerifyTimedFlag=0;                       //!<Indicates whether verification option is turned on for a timed behavioral model
int  ACGDBIntegrationFlag=0;                    //!<Indicates whether gdb support will be included in the simulator
int  ACWaitFlag=1;                              //!<Indicates whether the instruction execution thread issues a wait() call or not

//char *ACVersion = "2.0alpha1";                        //!<Stores ArchC version number.
char ACOptions[500];                            //!<Stores ArchC recognized command line options
char *ACOptions_p = ACOptions;                  //!<Pointer used to append options in ACOptions
char *arch_filename;                            //!<Stores ArchC arquitecture file

int ac_host_endian;                             //!<Indicates the endianess of the host machine
extern int ac_tgt_endian;                       //!<Indicates the endianess of the host machine
int ac_match_endian;                            //!<Indicates whether host and target endianess match on or not

//! This structure describes one command-line option mapping.
/*!  It is used to manage command line options, following gcc style. */
struct option_map
{
  const char *name;                          //!<The long option's name.
  const char *equivalent;                    //!<The equivalent short-name for options.
  const char *arg_desc;                      //!<The option description.
  /* Argument info.  A string of flag chars; NULL equals no options.
     r => argument required.
     o => argument optional.
     * => require other text after NAME as an argument.  */
  const char *arg_info;                      //!<Argument info.  A string of flag chars; NULL equals no options.
};

/*!Decoder object pointer */
ac_decoder_full *decoder;

/*!Storage device used for loading applications */
ac_sto_list* load_device=0;

/*! This is the table of mappings.  Mappings are tried sequentially
  for each option encountered; the first one that matches, wins.  */
struct option_map option_map[] = {
  {"--abi-included"  , "-abi"        ,"Indicate that an ABI for system call emulation was provided." ,"o"},
  {"--debug"         , "-g"          ,"Enable simulation debug features: traces, update logs." ,"o"},
  {"--delay"         , "-dy"          ,"Enable delayed assignments to storage elements." ,"o"},
  {"--dumpdecoder"   , "-dd"         ,"Dump the decoder data structure." ,"o"},
  {"--help"          , "-h"          ,"Display this help message."       , 0},
  {"--no-dec-cache"  , "-ndc"        ,"Disable cache of decoded instructions." ,"o"},
  {"--stats"         , "-s"          ,"Enable statistics collection during simulation." ,"o"},
  {"--verbose"       , "-vb"         ,"Display update logs for storage devices during simulation.", "o"},
  {"--version"       , "-vrs"        ,"Display ACSIM version.", 0},
  {"--gdb-integration", "-gdb"       ,"Enable support for debbuging programs running on the simulator.", 0},
  {"--no-wait"       , "-nw"        ,"Disable wait() at execution thread.", 0},
  0
};


/*! Display the command line options accepted by ArchC.  */
static void DisplayHelp (){
  int i;
  char line[]="====================";

  line[strlen(ACVERSION)+1] = '\0';

  printf ("===============================================%s\n", line);
  printf (" This is the ArchC Simulator Generator version %s\n", ACVERSION);
  printf ("===============================================%s\n\n", line);
  printf ("Usage: acsim input_file [options]\n");
  printf ("       Where input_file stands for your AC_ARCH description file.\n\n");
  printf ("Options:\n");

  for( i=0; i< ACNumberOfOptions; i++)
    printf ("    %-17s, %-11s %s\n", option_map[i].name, option_map[i].equivalent, option_map[i].arg_desc);

  printf ("\nFor more information please visit www.archc.org\n\n");
}

/*! Function for decoder to get bits from the instruction */
/*    PARSER-TIME VERSION: Should not be called */
unsigned long long GetBits(void *buffer, int *quant, int last, int quantity, int sign)
{
  AC_ERROR("GetBits(): This function should not be called in parser-time for interpreted simulator.\n");
  return 1;
};

/*!If target is little endian, this function inverts fields in each format. This
  is necessary in order to the decoder works in little endian architectures.
  \param formats The list of parsed formats containing fields to be inverted.
 */
void invert_fields(ac_dec_format *format)
{
  ac_dec_field *field;

  while (format) {
    int size = format->size;
    field = format->fields;
    while (field) {
      field->first_bit = size - 2 - field->first_bit + field->size;
      field = field->next;
    }

    format = format->next;
  }
}


/**********************************************************/
/*!Writes a standard comment at the begining of each file
   generated by ArchC.
   OBS: Description must have 50 characteres at most!!
  \param output The output file pointer.
  \param description A brief description of the file being emited.*/
/**********************************************************/
void print_comment( FILE* output, char* description ){
  fprintf( output, "/******************************************************\n");
  fprintf( output, " * %-50s *\n",description);
  fprintf( output, " * This file is automatically generated by ArchC      *\n");
  fprintf( output, " * WITHOUT WARRANTY OF ANY KIND, either express       *\n");
  fprintf( output, " * or implied.                                        *\n");
  fprintf( output, " * For more information on ArchC, please visit:       *\n");
  fprintf( output, " * http://www.archc.org                               *\n");
  fprintf( output, " *                                                    *\n");
  fprintf( output, " * The ArchC Team                                     *\n");
  fprintf( output, " * Computer Systems Laboratory (LSC)                  *\n");
  fprintf( output, " * IC-UNICAMP                                         *\n");
  fprintf( output, " * http://www.lsc.ic.unicamp.br                       *\n");
  fprintf( output, " ******************************************************/\n");
  fprintf( output, " \n\n");
}


//////////////////////////////////////////
/*!Main routine of  ArchC pre-processor.*/
//////////////////////////////////////////
int main(i