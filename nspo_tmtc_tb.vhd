------------------------------------------------------------------------------
--  Copyright (C) 2022, Cobham Gaisler AB - all rights reserved.
--
-- ANY USE OR REDISTRIBUTION IN PART OR IN WHOLE MUST BE HANDLED IN 
-- ACCORDANCE WITH THE GAISLER LICENSE AGREEMENT AND MUST BE APPROVED 
-- IN ADVANCE IN WRITING. 
--------------------------------------------------------------------------------

library  ieee;
use      ieee.std_logic_1164.all;

library  work;
use      work.config.all;

entity nspo_tmtc_tb is
  generic(
      fabtech:                Integer                    := CFG_FABTECH;
      memtech:                Integer                    := CFG_MEMTECH;
      padtech:                Integer                    := CFG_PADTECH;
      clktech:                Integer                    := CFG_CLKTECH;
      apid:                   Integer                    := 16#000#;
      audebug:                Integer                    := 1;

      sReedSolomon:           Std_ULogic                 := '1';
      sPseudo:                Std_ULogic                 := '1';
      sMark:                  Std_ULogic                 := '0';
      sConvolute:             Std_ULogic                 := '0';
      sTelemetry:             Std_ULogic                 := '1';
      sFECF:                  Std_ULogic                 := '0';
      sOCF:                   Std_ULogic                 := '1';
      stcscid:                Std_Logic_Vector           := "0100000010";
      stcvcid:                Std_Logic_Vector           := "000011";
      stcrfpos:               Std_ULogic                 := '1';
      stchigh:                Std_ULogic                 := '1';
      stcrise:                Std_ULogic                 := '1';
      stcpseudo:              Std_ULogic                 := '0';
      stcmark:                Std_ULogic                 := '0';

      sBoardPeriod:           Time                       :=   50 ns;
      sSpWBitPeriod:          Time                       :=  100 ns;
      sTMBitPeriod:           Time                       :=  84 ns;
      sTCBitPeriod:           Time                       := 1000 ns;
      cBaud:                  Integer                    := 115200;

      vc3vc6enable:           Boolean                    := True;

      prom8en:                Integer                    := 1;     -- 8-bit boot-prom
      romdepth:               Integer                    := 16;    -- rom address depth
      sramwidth:              Integer                    := 32;    -- ram data width (8/16/32)
      sramdepth:              Integer                    := 20;    -- ram address depth
      srambanks:              Integer                    := 2;     -- number of ram banks

      promfile:               String                     := "prom.srec";
      sramfile:               String                     := "sram.srec";

      sSynchronize:           Boolean                    := True;

      sOutputFile:            String                     := "NSPO";
      sScreenSource:          Boolean                    := False;
      sScreenDecoder:         Boolean                    := False);
end nspo_tmtc_tb;

--============================== Architecture ================================--

library  std;
use      std.standard.all;
use      std.textio.all;

library  ieee;
use      ieee.std_logic_1164.all;

library  grlib;
use      grlib.amba.all;
use      grlib.stdio.all;
use      grlib.stdlib.conv_integer;
use      grlib.stdlib.conv_std_logic_vector;
use      grlib.stdlib."+";

use      grlib.testlib.compare;
use      grlib.testlib.check;
use      grlib.testlib.print;
use      grlib.testlib.synchronise;
use      grlib.testlib.tinitialise;
use      grlib.testlib.tintermediate;
use      grlib.testlib.tterminate;
use      grlib.testlib.gen_rand_int;

library  tmtc;
use      tmtc.ccsdstypes.all;
use      tmtc.packetwirepackage.all;
use      tmtc.rs232package.all;
use      tmtc.ccsdsencoderpackage.all;
use      tmtc.ccsdsdecoderpackage.all;
use      tmtc.ccsdssourcepacket.all;
use      tmtc.ccsdssourcepackage.all;
use      tmtc.ccsdstelemetry.all;

library  gaisler;
use      gaisler.sim.all;

library  work;
use      work.rmap_tp.all;
use      work.astrium.all;
use      work.tcau_config.all;

architecture behavioural of nspo_tmtc_tb is

   type natural_vector is array (natural range <>) of natural;
   -----------------------------------------------------------------------------
   -- encoder and decoder settings
   -----------------------------------------------------------------------------
   constant framesize:           Integer                       := 1115;
   constant limitsize:           Integer                       := 1115;

   constant s_frame_length:      Std_Logic_Vector(11 downto 0) := Conv_Std_Logic_Vector(framesize, 12);
   constant s_ocf:               Std_ULogic                    := sOCF;
   constant s_ocf_ow:            Std_ULogic                    := '1';
   constant s_fsh_length:        Std_Logic_Vector(4 downto 0)  := "00000";
   constant s_fsh:               Std_ULogic                    := '0';
   constant s_insert:            Std_ULogic                    := '0';
   constant s_fecf:              Std_ULogic                    := sFECF;
   constant s_fhec:              Std_ULogic                    := '0';
   constant s_version:           Std_Logic_Vector(1 downto 0)  := "00";
   constant s_mc:                Std_ULogic                    := '1';
   constant s_fsh_ext_vc_cntr:   Std_ULogic                    := '0';
   constant s_vc_cntr_cycle:     Std_ULogic                    := '0';
   constant s_idle:              Std_ULogic                    := '1';
   constant s_idle_mc:           Std_ULogic                    := '0';
   constant s_idle_vcid:         Std_Logic_Vector(5 downto 0)  := "000111";
   constant s_idle_scid:         Std_Logic_Vector(9 downto 0)  := stcscid;
   constant s_asm:               Std_ULogic                    := '0';
   constant s_reed:              Std_ULogic                    := sReedSolomon;
   constant s_reed_depth:        Std_Logic_Vector(2 downto 0)  := "100";
   constant s_reed_e8:           Std_ULogic                    := '0';
   constant s_turbo:             Std_ULogic                    := '0';
   constant s_turbo_rate:        Std_Logic_Vector(1 downto 0)  := "00";
   constant s_chipher:           Std_ULogic                    := '0';
   constant s_pseudo:            Std_ULogic                    := sPseudo;
   constant s_mark:              Std_ULogic                    := sMark;
   constant s_conv:              Std_ULogic                    := sConvolute;
   constant s_conv_rate:         Std_Logic_Vector(2 downto 0)  := "000";
   constant s_split:             Std_ULogic                    := '0';
   constant s_sub:               Std_ULogic                    := '0';
   constant s_sub_fall:          Std_ULogic                    := '0';
   constant s_sub_rate:          Std_Logic_Vector(15 downto 0) := (others => '0');
   constant s_symbol_fall:       Std_ULogic                    := '0';
   constant s_symbol_rate:       Std_Logic_Vector(15 downto 0) := X"0001";

   -----------------------------------------------------------------------------
   -- Telemetry Frame Generation
   -----------------------------------------------------------------------------
   -- clock frequencies
   constant sBoardFrequency:  Integer := 1000000000 ns / (sBoardPeriod );
   constant sSpWBitFrequency: Integer := 1000000000 ns / (sSpWBitPeriod);
   constant sTMBitFrequency:  Integer := 1000000000 ns / (sTMBitPeriod );
   constant sTCBitFrequency:  Integer := 1000000000 ns / (sTcBitPeriod );

   -- memory size for transfer frames
   constant memorysize:    Integer := 2;
   constant MEMSIZE:       Integer := memorysize*1024;

   -- support for packet insertion in transfer frames
   constant packet:        Integer := 1;
   constant packetcrc:     Integer := 1;

   -- print requirements
   constant screen:        Boolean := False;

   -----------------------------------------------------------------------------
   -- Addressing constants
   -----------------------------------------------------------------------------
   constant ROM:           Std_Logic_Vector := X"00000000";
   constant RAM:           Std_Logic_Vector := X"40000000";
   constant APB_HADDR:     Integer          :=16#800#;
   constant AHBRAM4k:      Std_Logic_Vector := X"a0000000";
   constant AHBRAM16k:     Std_Logic_Vector := X"b0000000";
   constant GRTMDESC:      Std_Logic_Vector := X"c0000000";
   constant TCAUDEBUG:     Std_Logic_Vector := X"d0000000";
   constant IOADDR:        Integer          :=16#fff#;

   constant cIRQ:          Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#000#, 12);
   constant cIRQ_LEVEL:    Std_Logic_Vector := cIRQ & X"00";
   constant cIRQ_PENDING:  Std_Logic_Vector := cIRQ & X"04";
   constant cIRQ_FORCE:    Std_Logic_Vector := cIRQ & X"08";
   constant cIRQ_CLEAR:    Std_Logic_Vector := cIRQ & X"0C";
   constant cIRQ_MSTAT:    Std_Logic_Vector := cIRQ & X"10";
   constant cIRQ_BCAST:    Std_Logic_Vector := cIRQ & X"14";
   constant cIRQ_PMASK:    Std_Logic_Vector := cIRQ & X"40";
   constant cIRQ_PFORCE:   Std_Logic_Vector := cIRQ & X"80";
   constant cIRQ_EXTACK:   Std_Logic_Vector := cIRQ & X"C4";

   constant cGRGPIO:       Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#B00#, 12);
   constant cGPIO_IN:      Std_Logic_Vector := cGRGPIO & X"00";
   constant cGPIO_OUT:     Std_Logic_Vector := cGRGPIO & X"04";
   constant cGPIO_DIR:     Std_Logic_Vector := cGRGPIO & X"08";
   constant cGPIO_INTRMSK: Std_Logic_Vector := cGRGPIO & X"0C";
   constant cGPIO_INTRPOL: Std_Logic_Vector := cGRGPIO & X"10";
   constant cGPIO_INTREDG: Std_Logic_Vector := cGRGPIO & X"14";

   constant cGRGPREG:      Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#A00#, 12) & X"00";
   
   constant cFTSRCTRL:     Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#100#, 12);
   constant cMCFG1:        Std_Logic_Vector := cFTSRCTRL & X"00";
   constant cMCFG2:        Std_Logic_Vector := cFTSRCTRL & X"04";
   constant cMCFG3:        Std_Logic_Vector := cFTSRCTRL & X"08";

   constant cAHBSTATUS:    Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#200#, 12);
   constant cAHBSTAT:      Std_Logic_Vector := cAHBSTATUS & X"00";
   constant cAHBADDR:      Std_Logic_Vector := cAHBSTATUS & X"04";

   constant cAHBRAM4k:     Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                            Conv_Std_Logic_Vector(16#300#, 12);
   constant cAHBRAMCFG4k:  Std_Logic_Vector := cAHBRAM4k & X"00";

   constant cAHBRAM16k:    Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#400#, 12);
   constant cAHBRAMCFG16k: Std_Logic_Vector := cAHBRAM16k & X"00";

   constant cAHBUART:      Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#500#, 12);
   constant cAHBUARTSTAT:  Std_Logic_Vector := cAHBUART & X"04";
   constant cAHBUARTCTRL:  Std_Logic_Vector := cAHBUART & X"08";
   constant cAHBUARTSCALE: Std_Logic_Vector := cAHBUART & X"0C";

   constant cGRTM_DMA:     Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#600#, 12);
   constant cTMDMACtrl:    Std_Logic_Vector := cGRTM_DMA & '0' & "0000000";
   constant cTMDMAStat:    Std_Logic_Vector := cGRTM_DMA & '0' & "0000100";
   constant cTMDMALen:     Std_Logic_Vector := cGRTM_DMA & '0' & "0001000";
   constant cTMDMAPtr:     Std_Logic_Vector := cGRTM_DMA & '0' & "0001100";
   constant cTMDMAConf:    Std_Logic_Vector := cGRTM_DMA & '0' & "0010000";
   constant cTMDMARev:     Std_Logic_Vector := cGRTM_DMA & '0' & "0010100";
   constant cTMDMAExtCtrl: Std_Logic_Vector := cGRTM_DMA & '0' & "0100000";
   constant cTMDMAExtPtr:  Std_Logic_Vector := cGRTM_DMA & '0' & "0101100";

   constant cGRTM_TX:      Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#600#, 12);
   constant cTMCtrl:       Std_Logic_Vector := cGRTM_TX  & '1' & "0000000";
   constant cTMStat:       Std_Logic_Vector := cGRTM_TX  & '1' & "0000100";
   constant cTMConf:       Std_Logic_Vector := cGRTM_TX  & '1' & "0001000";
   constant cTMSize:       Std_Logic_Vector := cGRTM_TX  & '1' & "0001100";
   constant cTMPHY:        Std_Logic_Vector := cGRTM_TX  & '1' & "0010000";
   constant cTMCODE:       Std_Logic_Vector := cGRTM_TX  & '1' & "0010100";
   constant cTMASM:        Std_Logic_Vector := cGRTM_TX  & '1' & "0011000";
   constant cTMALL:        Std_Logic_Vector := cGRTM_TX  & '1' & "0100000";
   constant cTMMST:        Std_Logic_Vector := cGRTM_TX  & '1' & "0100100";
   constant cTMIDLE:       Std_Logic_Vector := cGRTM_TX  & '1' & "0101000";
   constant cTMFSH0:       Std_Logic_Vector := cGRTM_TX  & '1' & "1000000";
   constant cTMFSH1:       Std_Logic_Vector := cGRTM_TX  & '1' & "1000100";
   constant cTMFSH2:       Std_Logic_Vector := cGRTM_TX  & '1' & "1001000";
   constant cTMFSH3:       Std_Logic_Vector := cGRTM_TX  & '1' & "1001100";
   constant cTMOCF:        Std_Logic_Vector := cGRTM_TX  & '1' & "1010000";

   constant cTMVC:       Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                             Conv_Std_Logic_Vector(16#D00#, 12);
   constant cTMVCID:       Std_Logic_Vector := cTMVC  & X"E0";

   constant cGRTC:         Std_Logic_Vector := Conv_Std_Logic_Vector(IOADDR, 12) &
                                               Conv_Std_Logic_Vector(16#000#, 12);
   constant gTCGRR:        Std_Logic_Vector := cGRTC & X"00";
   constant gTCGCR:        Std_Logic_Vector := cGRTC & X"04";
   constant gTCPMR:        Std_Logic_Vector := cGRTC & X"08";
   constant gTCSIR:        Std_Logic_Vector := cGRTC & X"0C";
   constant gTCFAR:        Std_Logic_Vector := cGRTC & X"10";
   constant gTCCLCWR1:     Std_Logic_Vector := cGRTC & X"14";
   constant gTCCLCWR2:     Std_Logic_Vector := cGRTC & X"18";
   constant gTCPHIR:       Std_Logic_Vector := cGRTC & X"1C";
   constant gTCCOR:        Std_Logic_Vector := cGRTC & X"20";
   constant gTCSTR:        Std_Logic_Vector := cGRTC & X"24";
   constant gTCASR:        Std_Logic_Vector := cGRTC & X"28";
   constant gTCRRP:        Std_Logic_Vector := cGRTC & X"2C";
   constant gTCRWP:        Std_Logic_Vector := cGRTC & X"30";
   constant gTCPIMSR:      Std_Logic_Vector := cGRTC & X"60";
   constant gTCPIMR:       Std_Logic_Vector := cGRTC & X"64";
   constant gTCPISR:       Std_Logic_Vector := cGRTC & X"68";
   constant gTCPIR:        Std_Logic_Vector := cGRTC & X"6C";
   constant gTCIMR:        Std_Logic_Vector := cGRTC & X"70";
   constant gTCPICR:       Std_Logic_Vector := cGRTC & X"74";

   constant cGRTM_AHB3:    Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#700#, 12);
   constant gTMAHBSTAT3:   Std_Logic_Vector := cGRTM_AHB3 & X"04";
   constant gTMAHBCTRL3:   Std_Logic_Vector := cGRTM_AHB3 & X"08";

   constant cGRTM_AHB4:    Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#800#, 12);
   constant gTMAHBSTAT4:   Std_Logic_Vector := cGRTM_AHB4 & X"04";
   constant gTMAHBCTRL4:   Std_Logic_Vector := cGRTM_AHB4 & X"08";

   constant cGRTM_AHB5:    Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#900#, 12);
   constant gTMAHBSTAT5:   Std_Logic_Vector := cGRTM_AHB5 & X"04";
   constant gTMAHBCTRL5:   Std_Logic_Vector := cGRTM_AHB5 & X"08";

   constant cGRTM_AHB6:    Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#a00#, 12);
   constant gTMAHBSTAT6:   Std_Logic_Vector := cGRTM_AHB6 & X"04";
   constant gTMAHBCTRL6:   Std_Logic_Vector := cGRTM_AHB6 & X"08";

   constant cTCAU:         Std_Logic_Vector := Conv_Std_Logic_Vector(APB_HADDR, 12) &
                                               Conv_Std_Logic_Vector(16#c00#, 12);
   constant gTCAUFAR:      Std_Logic_Vector := cTCAU & X"00";
   constant gTCAUSR1:      Std_Logic_Vector := cTCAU & X"04";
   constant gTCAUSR2:      Std_Logic_Vector := cTCAU & X"08";
   constant gTCAUSR3:      Std_Logic_Vector := cTCAU & X"0c";
   constant gTCAUCFG:      Std_Logic_Vector := cTCAU & X"10";

   constant TMAHB3:        Std_Logic_Vector := Conv_Std_Logic_Vector(IOADDR, 12) &
                                               Conv_Std_Logic_Vector(16#700#, 12) &
                                               Conv_Std_Logic_Vector(16#00#, 8);
   constant TMAHB4:        Std_Logic_Vector := Conv_Std_Logic_Vector(IOADDR, 12) &
                                               Conv_Std_Logic_Vector(16#800#, 12) &
                                               Conv_Std_Logic_Vector(16#00#, 8);
   constant TMAHB5:        Std_Logic_Vector := Conv_Std_Logic_Vector(IOADDR, 12) &
                                               Conv_Std_Logic_Vector(16#900#, 12) &
                                               Conv_Std_Logic_Vector(16#00#, 8);
   constant TMAHB6:        Std_Logic_Vector := Conv_Std_Logic_Vector(IOADDR, 12) &
                                               Conv_Std_Logic_Vector(16#a00#, 12) &
                                               Conv_Std_Logic_Vector(16#00#, 8);

   -----------------------------------------------------------------------------
   -- Component declarations
   -----------------------------------------------------------------------------
   component nspo_tmtc is
   generic(
      fabtech:       in    integer := CFG_FABTECH;
      memtech:       in    integer := CFG_MEMTECH;
      padtech:       in    integer := CFG_PADTECH;
      clktech:       in    integer := CFG_CLKTECH;
      clkdiv10:      in    integer := 0;
      apid:          in    integer := 0;
      audebug:       in    integer := 0);
   port(
       -- system interface
      clk:           in    std_ulogic;                   -- system clock
      resetn:        in    std_ulogic;                   -- asynchronous reset
      irq:           out   std_ulogic;                   -- system interrupt

      -- gpio
      gp_io:          inout std_logic_vector(7 downto 0); -- GPIO
      
      -- debug interface
      dsurx:         in    std_ulogic;                   -- debug rx data
      dsutx:         out   std_ulogic;                   -- debug tx data

      -- telemetry encoder channel access data unit output interface
      transclk:      in    std_ulogic;                   -- transponder clock
      caduclk:       out   std_logic_vector(0 to 3);     -- cadu clock
      caduout:       out   std_logic_vector(0 to 3);     -- cadu data

      -- telecommand decoder configuration [static signals]
      tcscid:        in    std_logic_vector(0 to 9);     -- spacecraft id
      tcvcid:        in    std_logic_vector(0 to 5);     -- tc hardware vc id
      tcrfpos:       in    std_ulogic;                   -- polarity of rf
      tchigh:        in    std_ulogic;                   -- polarity of bit lock
      tcrise:        in    std_ulogic;                   -- edge of clock
      tcpseudo:      in    std_ulogic;                   -- pseudo derandomizer
      tcmark:        in    std_ulogic;                   -- nrz-m

      -- telecommand decoder physical layer interface
      tcrfa:         in    std_logic_vector(0 to 3);     -- rf available
      tcactive:      in    std_logic_vector(0 to 3);     -- input active
      tcclk:         in    std_logic_vector(0 to 3);     -- input clock
      tcdata:        in    std_logic_vector(0 to 3);     -- input data (sampled rising tcclk input)

      -- telecommand decoder output
      tcgpio:        out   std_logic_vector(0 to 23);    -- hardware command

      -- clcw interface
      clcwin:        in    std_logic_vector(0 to 1);     -- clcw in
      clcwout:       out   std_logic_vector(0 to 1);     -- clcw out

      -- spacewire input interfaces
      spw_clk:       in    std_ulogic;                   -- spw tx clock

      spwrxdp:       in    std_logic_vector(0 to 2);
      spwrxsp:       in    std_logic_vector(0 to 2);
      spwtxdp:       out   std_logic_vector(0 to 2);
      spwtxsp:       out   std_logic_vector(0 to 2);

      -- memory interface
      address:       out   std_logic_vector(27 downto 0);
      data:          inout std_logic_vector(31 downto 0);
      cb:            inout std_logic_vector(7 downto 0);
      ramsn:         out   std_logic_vector(7 downto 0);
      ramoen:        out   std_logic_vector(7 downto 0);
      rwen:          out   std_logic_vector(3 downto 0);
      romsn:         out   std_logic_vector(7 downto 0);
      oen:           out   std_ulogic;
      writen:        out   std_ulogic);
   end component nspo_tmtc;

   -----------------------------------------------------------------------------
   -- Signal declarations
   -----------------------------------------------------------------------------
   -- system interface
   signal   resetn:        std_ulogic := '0';            -- asynchronous reset
   signal   clk:           std_ulogic;                   -- system clock
   signal   irq:           std_ulogic;                   -- system interrupt

   -- gpio
   signal   gpio:          std_logic_vector(7 downto 0);
   
   -- debug interface
   signal   dsurx:         std_ulogic;                   -- debug rx data
   signal   dsutx:         std_ulogic;                   -- debug tx data

   -- channel access data unit output interface
   signal   transclk:      std_ulogic;                   -- transponder clock
   signal   caduclk:       std_logic_vector(0 to 3);     -- cadu clock
   signal   caduout:       std_logic_vector(0 to 3);     -- cadu data

   -- telecommand decoder configuration [static signals]
   signal   tcscid:        std_logic_vector(0 to 9);     -- spacecraft id
   signal   tcvcid:        std_logic_vector(0 to 5);     -- tc hardware vc id
   signal   tcrfpos:       std_ulogic;                   -- polarity of rf
   signal   tchigh:        std_ulogic;                   -- polarity of bit lock
   signal   tcrise:        std_ulogic;                   -- edge of clock
   signal   tcpseudo:      std_ulogic;                   -- pseudo derandomizer
   signal   tcmark:        std_ulogic;                   -- nrz-m

   -- telecommand decoder physical layer interface
   signal   tcrfa:         std_logic_vector(0 to 3);     -- rf available
   signal   tcactive:      std_logic_vector(0 to 3);     -- input active
   signal   tcclk:         std_logic_vector(0 to 3);     -- input clock
   signal   tcdata:        std_logic_vector(0 to 3);     -- input data

   -- telecommand decoder output
   signal   tcgpio:        std_logic_vector(0 to 23);    -- hardware command

   -- clcw interface
   signal   clcwin:        std_logic_vector(0 to 1);     -- clcw in
   signal   clcwout:       std_logic_vector(0 to 1);     -- clcw out

   -- spacewire input interfaces
   signal   spw_clk:       std_ulogic;                   -- spw tx clock

   signal   spwrxdp:       std_logic_vector(0 to 2);
   signal   spwrxsp:       std_logic_vector(0 to 2);
   signal   spwtxdp:       std_logic_vector(0 to 2);
   signal   spwtxsp:       std_logic_vector(0 to 2);
   signal   spwrxdp_del:   std_logic_vector(0 to 2);
   signal   spwrxsp_del:   std_logic_vector(0 to 2);
   signal   spwtxdp_del:   std_logic_vector(0 to 2);
   signal   spwtxsp_del:   std_logic_vector(0 to 2);

   -- memory interface
   signal   address:       std_logic_vector(27 downto 0);
   signal   data:          std_logic_vector(31 downto 0);
   signal   cb:            std_logic_vector(7 downto 0);
   signal   ramsn:         std_logic_vector(7 downto 0);
   signal   ramoen:        std_logic_vector(7 downto 0);
   signal   ramben:        std_logic_vector(3 downto 0) := (others => '0');
   signal   rwen:          std_logic_vector(3 downto 0);
   signal   romsn:         std_logic_vector (7 downto 0);
   signal   oen:           std_ulogic;
   signal   writen:        std_ulogic;
   --signal   mbe:           std_ulogic;

   -----------------------------------------------------------------------------
   -- Signal declarations for telemetry decoder
   -----------------------------------------------------------------------------
   signal   CADUClkIn:     Std_ULogic;
   signal   CADUDataIn:    Std_ULogic;
   signal   NRZMDClkOut:   Std_ULogic;
   signal   NRZMDDataOut:  Std_ULogic;
   signal   PSRDClkOut:    Std_ULogic;
   signal   PSRDDataOut:   Std_ULogic;
   signal   CDClkOut:      Std_ULogic;
   signal   CDDataOut:     Std_ULogic;
   signal   RSDClkOut:     Std_ULogic;
   signal   RSDDataOut:    Std_ULogic;
   signal   CRCDClkOut:    Std_ULogic;
   signal   CRCDDataOut:   Std_ULogic;

   signal   TMreset_n:     Std_ULogic := '0';            -- tm decoder reset

   signal   vc3vc6on:      Std_ULogic := '0';

   signal   frame_length:  Integer range 7 to 2048       := 1115;
   signal   code_length:   Integer range 0 to 256        := 160;

   signal   ReedSolomon:   Std_ULogic                    := sReedSolomon;
   signal   Pseudo:        Std_ULogic                    := sPseudo;
   signal   Mark:          Std_ULogic                    := sMark;
   signal   Convolute:     Std_ULogic                    := sConvolute;
   signal   Telemetry:     Std_ULogic                    := sTelemetry;
   signal   FECF:          Std_ULogic                    := sFECF;
   signal   OCF:           Std_ULogic                    := sOCF;

   signal   Session:       Integer                       := -1;
   signal   Logical_1:     Std_Logic                     := '1';
   signal   Logical_0:     Std_Logic                     := '0';
   signal   Logical_00:    Std_Logic_Vector(0 to 1)      := "00";
   signal   Logical_000:   Std_Logic_Vector(0 to 2)      := "000";

   signal   gnd:           Std_Logic_Vector(7 downto 0)  := (others => '0');

   signal   clcwen:        Std_Logic                     := '0';

   -----------------------------------------------------------------------------
   -- PacketWire block transmitter - without timing
   -----------------------------------------------------------------------------
   procedure TxPW_Local(
      signal   PWClk:         out   Std_ULogic;
      signal   PWValid:       out   Std_ULogic;
      signal   PWData:        out   Std_ULogic;
      signal   PWBusy_N:      in    Std_ULogic;
      signal   PWRdy:         in    Std_ULogic;
      constant DataBlock:     in    Octet_Vector;
      constant Comment:       in    String  := "";
      constant Screen:        in    Boolean := True;
      constant BitPeriod:     in    Time    := 100 ns;
      constant Instance:      in    String  := "TxPW") is
      variable L:                   Line;
   begin
      -- wait for ready on interface
      if PWRdy /= '1' then
         wait until PWRdy = '1';
      end if;

      -- diagnostics
      if Screen then
         Write(L, Now, Right, 15);
         Write(L, " : " & Instance & ": Size: ");
         Write(L, DataBlock'Length);
         if Comment /= "" then
            Write(L, " : " & Comment);
         end if;
         WriteLine(Output, L);
      end if;

      -- send data block
      for i in DataBlock'Left to DataBlock'Right loop
         TxPW(
            PWClk          => PWClk,
            PWValid        => PWValid,
            PWData         => PWData,
            PWBusy_N       => PWBusy_N,
            DataOctet      => DataBlock(i),
            Instance       => Instance,
            Comment        => Comment,
            Screen         => Screen,
            FirstOctet     => (i = DataBlock'Left),
            LastOctet      => (i = DataBlock'Right),
            BitPeriod      => BitPeriod);
      end loop;
   end procedure TxPW_Local;

   -----------------------------------------------------------------------------
   -- Simple routine to send a data block for transmission from PW transmitters
   -----------------------------------------------------------------------------
   procedure TxPW(
      constant Data_Block: in    Octet_Vector;
      signal   TxData:     out   Octet_Vector;
      signal   TxCount:    out   Natural;
      signal   TxRequest:  out   Std_ULogic;
      signal   TxAck:      in    Std_ULogic) is
   begin
      TxData(0 to Data_Block'Length-1) <= Data_Block;
      TxCount                          <= Data_Block'Length;
      TxRequest                        <= '1';
      wait until TxAck='1';
      TxRequest                        <= '0';
   end procedure TxPW;

   -----------------------------------------------------------------------------
   -- Multiple parallel transmitters for TC
   -----------------------------------------------------------------------------
   signal   Txreset_n:        Std_ULogic;                -- transmitter reset
   signal   TxData:           Octet_Vector(0 to 2047);   -- transmit data
   signal   TxCount:          Natural;                   -- transmit count
   signal   TxRequest:        Std_Logic_Vector(0 to 3);  -- transmit request
   signal   TxAck:            Std_ULogic;                -- transmit acknowledge

   signal   TxBitPeriod:      Time := sTCBitPeriod;

   -----------------------------------------------------------------------------
   -- Debug UART interface
   -----------------------------------------------------------------------------
   signal   UARTData:         Std_Logic_Vector(0 to 7);  -- received data
   signal   UARTReady:        Std_Logic;                 -- received data ready
   signal   UARTAck:          Std_Logic;                 -- received data ack

   -----------------------------------------------------------------------------
   -- SPW-A signal declarations
   -----------------------------------------------------------------------------
   constant spwa_hindex:      Integer := 0;
   constant spwa_ioaddr:      Integer := 0;
   constant spwa_iomask:      Integer := 16#F00#;
   constant spwa_htxindex:    Integer := 1;
   constant spwa_hrxindex:    Integer := 0;
   constant spwa_pindex:      Integer := 0;
   constant spwa_pirq:        Integer := 1;
   constant spwa_paddr:       Integer := 0;
   constant spwa_pmask:       Integer := 16#FFF#;

   signal   spwa0_apbi:       APB_Slv_In_Type;
   signal   spwa0_apbo:       APB_Slv_Out_Type;
   signal   spwa1_apbi:       APB_Slv_In_Type;
   signal   spwa1_apbo:       APB_Slv_Out_Type;
   signal   spwa2_apbi:       APB_Slv_In_Type;
   signal   spwa2_apbo:       APB_Slv_Out_Type;

   signal   spwa0_tx_ahbsi:   AHB_Slv_In_Type;
   signal   spwa0_tx_ahbso:   AHB_Slv_Out_Type;
   signal   spwa0_tx_ahbmi:   AHB_Mst_In_Type;
   signal   spwa0_tx_ahbmo:   AHB_Mst_Out_Type;
   signal   spwa0_rx_ahbmi:   AHB_Mst_In_Type;
   signal   spwa0_rx_ahbmo:   AHB_Mst_Out_Type;
   signal   spwa1_tx_ahbsi:   AHB_Slv_In_Type;
   signal   spwa1_tx_ahbso:   AHB_Slv_Out_Type;
   signal   spwa1_tx_ahbmi:   AHB_Mst_In_Type;
   signal   spwa1_tx_ahbmo:   AHB_Mst_Out_Type;
   signal   spwa1_rx_ahbmi:   AHB_Mst_In_Type;
   signal   spwa1_rx_ahbmo:   AHB_Mst_Out_Type;
   signal   spwa2_tx_ahbsi:   AHB_Slv_In_Type;
   signal   spwa2_tx_ahbso:   AHB_Slv_Out_Type;
   signal   spwa2_tx_ahbmi:   AHB_Mst_In_Type;
   signal   spwa2_tx_ahbmo:   AHB_Mst_Out_Type;
   signal   spwa2_rx_ahbmi:   AHB_Mst_In_Type;
   signal   spwa2_rx_ahbmo:   AHB_Mst_Out_Type;

   signal   spwa_reset0_n:    Std_Logic := '0';          -- reset
   signal   spwa_reset1_n:    Std_Logic := '0';          -- reset
   signal   spwa_reset2_n:    Std_Logic := '0';          -- reset
   signal   spwa_clk:         Std_Logic := '0';          -- clock
   signal   spwa_txclk:       Std_Logic := '0';          -- tx clock
   signal   spwa_tickin:      Std_Logic := '0';          -- time code to send

   constant ahb_mst_in_init: ahb_mst_in_type :=
      (hgrant => (others => '0'), hready => '0', hresp => "00",
       hrdata => (others => '0'), hirq => (others => '0'),
       testen => '0', testrst => '0', scanen => '0', testoen => '0', testin => (others => '0'));

   constant ahb_slv_in_init: ahb_slv_in_type :=
      (hsel => (others => '0'), haddr => (others => '0'),
       hwrite => '0', htrans => (others => '0'), hsize => (others => '0'),
       hburst => (others => '0'), hwdata => (others => '0'), hprot => (others => '0'),
       hready => '0', hmaster => (others => '0'), hmastlock => '0',
       hmbsel => (others => '0'), hirq => (others => '0'),
       testen => '0', testrst => '0', scanen => '0', testoen => '0', testin => (others => '0'));

   constant apb_slv_in_init: APB_Slv_In_Type :=
      (psel => (others => '0'), penable => '0',
       paddr => (others => '0'), pwrite => '0',
       pwdata => (others => '0'), pirq => (others => '0'),
       testen => '0', testrst => '0', scanen => '0', testoen => '0', testin => (others => '0'));

   -----------------------------------------------------------------------------
   -- Signals for passing status between processes
   -----------------------------------------------------------------------------
   signal vc3vc4done:      Boolean;
   signal vc5vc6done:      Boolean;
   signal vc3vc4tp:        Boolean;
   signal vc5vc6tp:        Boolean;
   signal vc3vc4tpcounter: Natural;
   signal vc5vc6tpcounter: Natural;
begin
   --=========================================================================--
   -- Clock generation
   --=========================================================================--
   ClockGenerator(clk,        sBoardFrequency);

   ClockGenerator(transclk,   sTMBitFrequency);

   ClockGenerator(spw_clk,    sSpWBitFrequency);
   ClockGenerator(spwa_txclk,    sSpWBitFrequency*2);

   spwa_clk <= clk;

   --=========================================================================--
   -- MEMORY
   --=========================================================================--
   prom8 : if prom8en = 1 generate
      sr0 : sram
         generic map (index => 6, abits => romdepth, fname => promfile)
         port map (address(romdepth-1 downto 0), data(31 downto 24), romsn(0), writen, oen);
   end generate;

   prom32 : if prom8en = 0 generate
      prom0 : for i in 0 to 3 generate
         sr0 : sram
            generic map (index => i, abits => romdepth, fname => promfile)
            port map (address(romdepth+1 downto 2), data(31-i*8 downto 24-i*8), romsn(0), rwen(i), oen);
      end generate;
      promcb0 : sramft
         generic map (index => 7, abits => romdepth, fname => promfile)
         port map (address(romdepth+1 downto 2), cb(7 downto 0), romsn(0), writen, oen);
   end generate;

   sram0 : sram16
      generic map (index => 0, abits => sramdepth, fname => sramfile)
      port map (address(sramdepth+1 downto 2), data(31 downto 16), ramben(1), ramben(0), ramsn(0), writen, ramoen(0));
   sram1 : sram16
      generic map (index => 2, abits => sramdepth, fname => sramfile)
      port map (address(sramdepth+1 downto 2), data(15 downto 0), ramben(3), ramben(2), ramsn(0), writen, ramoen(0));
   sramcb0 : sramft
      generic map (index => 7, abits => sramdepth, fname => sramfile)
      port map (address(sramdepth+1 downto 2), cb(7 downto 0), ramsn(0), writen, ramoen(0));

   sram2 : sram16
      generic map (index => 0, abits => sramdepth, fname => sramfile)
      port map (address(sramdepth+1 downto 2), data(31 downto 16), ramben(1), ramben(0), ramsn(1), writen, ramoen(1));
   sram3 : sram16
      generic map (index => 2, abits => sramdepth, fname => sramfile)
      port map (address(sramdepth+1 downto 2), data(15 downto 0), ramben(3), ramben(2), ramsn(1), writen, ramoen(1));
   sramcb1 : sramft
      generic map (index => 7, abits => sramdepth, fname => sramfile)
      port map (address(sramdepth+1 downto 2), cb(7 downto 0), ramsn(1), writen, ramoen(1));

   -----------------------------------------------------------------------------
   -- Mimics capacitance and pull-up on bus
   -----------------------------------------------------------------------------
   data     <= buskeep(data); --, (others => 'H') after 250 ns;
   cb       <= buskeep(cb); --,   (others => 'H') after 250 ns;

   data     <= (others => 'H') after 250 ns;
   cb       <= (others => 'H') after 250 ns;

   --=========================================================================--
   -- TELECOMMAND TEST HARNESS
   --=========================================================================--
   -----------------------------------------------------------------------------
   -- PacketWire transmitters used for sending data to the TCC. The transmitters
   -- send blocks of octet data.
   -----------------------------------------------------------------------------
   Transmitter: process is
      variable Data:    Octet_Vector(0 to 2047);
   begin
      -- initialise
      TCClk                   <= (others => 'L');
      TCActive                <= (others => 'L');
      TCData                  <= (others => 'L');
      TxAck                   <= 'L';
      loop
         if TxRequest(0)/='1' and TxRequest(1)/='1' and TxRequest(2)/='1' and TxRequest(3)/='1' then
            wait on TxRequest, Txreset_n;
         end if;
         if    Txreset_n='0' then
            TCClk             <= (others => 'L');
            TCActive          <= (others => 'L');
            TCData            <= (others => 'L');
            TxAck             <= 'L';
         else
            if TxRequest(0)='1' then
               Data           := TxData;
               TxAck          <= '1', '0' after 1 ns;
               TxPW_Local(
                  PWClk       => TCClk(0),
                  PWValid     => TCActive(0),
                  PWData      => TCData(0),
                  PWBusy_N    => Logical_1,
                  PWRdy       => Logical_1,
                  DataBlock   => Data(0 to TxCount-1),
                  Comment     => "",
                  Screen      => sScreenSource,
                  BitPeriod   => TxBitPeriod,
                  Instance    => "TxPW 0");
            end if;
            if TxRequest(1)='1' then
               Data           := TxData;
               TxAck          <= '1', '0' after 1 ns;
               TxPW_Local(
                  PWClk       => TCClk(1),
                  PWValid     => TCActive(1),
                  PWData      => TCData(1),
                  PWBusy_N    => Logical_1,
                  PWRdy       => Logical_1,
                  DataBlock   => Data(0 to TxCount-1),
                  Comment     => "",
                  Screen      => sScreenSource,
                  BitPeriod   => TxBitPeriod,
                  Instance    => "TxPW 1");
            end if;
            if TxRequest(2)='1' then
               Data           := TxData;
               TxAck          <= '1', '0' after 1 ns;
               TxPW_Local(
                  PWClk       => TCClk(2),
                  PWValid     => TCActive(2),
                  PWData      => TCData(2),
                  PWBusy_N    => Logical_1,
                  PWRdy       => Logical_1,
                  DataBlock   => Data(0 to TxCount-1),
                  Comment     => "",
                  Screen      => sScreenSource,
                  BitPeriod   => TxBitPeriod,
                  Instance    => "TxPW 2");
            end if;
            if TxRequest(3)='1' then
               Data           := TxData;
               TxAck          <= '1', '0' after 1 ns;
               TxPW_Local(
                  PWClk       => TCClk(3),
                  PWValid     => TCActive(3),
                  PWData      => TCData(3),
                  PWBusy_N    => Logical_1,
                  PWRdy       => Logical_1,
                  DataBlock   => Data(0 to TxCount-1),
                  Comment     => "",
                  Screen      => sScreenSource,
                  BitPeriod   => TxBitPeriod,
                  Instance    => "TxPW 3");
            end if;
         end if;
      end loop;
   end process Transmitter;

   -----------------------------------------------------------------------------
   -- RS232 receiver, optional parity and extra stop bit, frame and break
   -----------------------------------------------------------------------------
   Rx232(
      RxIn              => dsutx,
      Reset_N           => resetn,
      RxData            => UARTData,
      RxReady           => UARTReady,
      RxAck             => UARTAck,
      Baud              => 115200,
      Parity            => False,
      TwoStop           => False,
      Comment           => "UART");

   --=========================================================================--
   -- TELEMETRY TEST HARNESS
   --=========================================================================--
   -----------------------------------------------------------------------------
   -- CCSDS decoders
   -----------------------------------------------------------------------------
   TMD: TelemetryDecoder(
      Reset_N           => TMreset_n,
      ClkIn             => RSDClkOut,
      DataIn            => RSDDataOut,
      Len               => frame_length,
      CodeLen           => code_length,
      FECW              => FECF,
      OPCF              => OCF,
      ReedSolomon       => ReedSolomon,
      EnableDecoder     => Telemetry,
      Session           => Session,
      InstancePath      => "TelemetryDecoder",
      ScreenOutput      => sScreenDecoder,
      DatumPerLine      => 16,
      Diagnostic        => not sScreenDecoder,
      OutputFile        => sOutputFile&"_TM.txt",
      ASMComment        => False);

   CRCD: CRCDecoder(
      Reset_N           => TMreset_n,
      ClkIn             => RSDClkOut,
      DataIn            => RSDDataOut,
      ClkOut            => CRCDClkOut,
      DataOut           => CRCDDataOut,
      Len               => frame_length,
      EnableDecoder     => FECF,
      InstancePath      => "CRCDecoder",
      ScreenOutput      => False);

   RSD: ReedSolomonDecoder(
      Reset_N           => TMreset_n,
      ClkIn             => PSRDClkOut,
      DataIn            => PSRDDataOut,
      ClkOut            => RSDClkOut,
      DataOut           => RSDDataOut,
      Len               => frame_length,
      CodeLen           => code_length,
      EnableDecoder     => ReedSolomon,
      Session           => Session,
      InstancePath      => "ReedSolomonDecoder",
      ScreenOutput      => False,
      OutputFile        => sOutputFile&"_RS.txt");

   PSRD: PseudoRandomiserDecoder(
      Reset_N           => TMreset_n,
      ClkIn             => NRZMDClkOut,
      DataIn            => NRZMDDataOut,
      ClkOut            => PSRDClkOut,
      DataOut           => PSRDDataOut,
      Len               => frame_length,
      CodeLen           => code_length,
      ReedSolomon       => ReedSolomon,
      Turbo             => Logical_0,
      TurboRate         => Logical_00,
      EnableDecoder     => Pseudo,
      InstancePath      => "PseudoRandomiserDecoder",
      ScreenOutput      => False);

   NRZMD: NonReturnZeroMarkDecoder(
      Reset_N           => TMreset_n,
      ClkIn             => CDClkOut,
      DataIn            => CDDataOut,
      ClkOut            => NRZMDClkOut,
      DataOut           => NRZMDDataOut,
      EnableDecoder     => Mark,
      InstancePath      => "NonReturnZeroMarkDecoder",
      ScreenOutput      => False);

   CD: ConvolutionalDecoder(
      Reset_N           => TMreset_n,
      ClkIn             => CADUClkIn,
      DataIn            => CADUDataIn,
      ClkOut            => CDClkOut,
      DataOut           => CDDataOut,
      ConvoluteRate     => Logical_000,
      EnableDecoder     => Convolute,
      InstancePath      => "ConvolutionalDecoder",
      ScreenOutput      => False);

   -----------------------------------------------------------------------------
   -- Synchronize CADU outputs to remove glitches in gate-level simulation
   -----------------------------------------------------------------------------
   resync: process(transclk)
   begin
      if Rising_Edge(transclk) then
         CADUClkIn   <= CADUClk(0) and CADUClk(1) and CADUClk(2);
         CADUDataIn  <= CADUOut(0) and CADUOut(1) and CADUOut(2);
      end if;
   end process;

   -----------------------------------------------------------------------------
   -- Manchester decoding as per IEEE 802.3 (when symbol_fall=0):
   --
   -- original data XOR clock = Manchester value
   --      0            1                 1
   --      0            0                 0
   --      1            1                 0
   --      1            0                 1
   -----------------------------------------------------------------------------
   manchester: process(transclk)
   begin
      if Rising_Edge(transclk) then
         --## add manchester decoder
      end if;
   end process;

   --=========================================================================--
   -- SpaceWire interface
   --=========================================================================--
   -----------------------------------------------------------------------------
   -- ESA/ASTRIUM SpaceWire Link - SPW-A
   -----------------------------------------------------------------------------
   SPWA0: AstriumSpW
      generic map(
         hindex         => spwa_hindex,
         ioaddr         => spwa_ioaddr,
         iomask         => spwa_iomask,
         htxindex       => spwa_htxindex,
         hrxindex       => spwa_hrxindex,
         pindex         => spwa_pindex,
         pirq           => spwa_pirq,
         paddr          => spwa_paddr,
         pmask          => spwa_pmask)
      port map(
         rstn           => spwa_reset0_n,
         clk            => spwa_clk,
         test           => gnd(0),
         clk_txin       => spwa_txclk,
         clk_txout      => open,
         tickin         => spwa_tickin,
         tickout        => open,
         d_in           => spwtxdp(0),
         s_in           => spwtxsp(0),
         d_out          => spwrxdp(0),
         s_out          => spwrxsp(0),
         apbi           => spwa0_apbi,
         apbo           => spwa0_apbo,
         tx_ahbsi       => spwa0_tx_ahbsi,
         tx_ahbso       => spwa0_tx_ahbso,
         tx_ahbmi       => spwa0_tx_ahbmi,
         tx_ahbmo       => spwa0_tx_ahbmo,
         rx_ahbmi       => spwa0_rx_ahbmi,
         rx_ahbmo       => spwa0_rx_ahbmo);

   SPWA1: AstriumSpW
      generic map(
         hindex         => spwa_hindex,
         ioaddr         => spwa_ioaddr,
         iomask         => spwa_iomask,
         htxindex       => spwa_htxindex,
         hrxindex       => spwa_hrxindex,
         pindex         => spwa_pindex,
         pirq           => spwa_pirq,
         paddr          => spwa_paddr,
         pmask          => spwa_pmask)
      port map(
         rstn           => spwa_reset1_n,
         clk            => spwa_clk,
         test           => gnd(0),
         clk_txin       => spwa_txclk,
         clk_txout      => open,
         tickin         => spwa_tickin,
         tickout        => open,
         d_in           => spwtxdp(1),
         s_in           => spwtxsp(1),
         d_out          => spwrxdp(1),
         s_out          => spwrxsp(1),
         apbi           => spwa1_apbi,
         apbo           => spwa1_apbo,
         tx_ahbsi       => spwa1_tx_ahbsi,
         tx_ahbso       => spwa1_tx_ahbso,
         tx_ahbmi       => spwa1_tx_ahbmi,
         tx_ahbmo       => spwa1_tx_ahbmo,
         rx_ahbmi       => spwa1_rx_ahbmi,
         rx_ahbmo       => spwa1_rx_ahbmo);

   SPWA2: AstriumSpW
      generic map(
         hindex         => spwa_hindex,
         ioaddr         => spwa_ioaddr,
         iomask         => spwa_iomask,
         htxindex       => spwa_htxindex,
         hrxindex       => spwa_hrxindex,
         pindex         => spwa_pindex,
         pirq           => spwa_pirq,
         paddr          => spwa_paddr,
         pmask          => spwa_pmask)
      port map(
         rstn           => spwa_reset2_n,
         clk            => spwa_clk,
         test           => gnd(0),
         clk_txin       => spwa_txclk,
         clk_txout      => open,
         tickin         => spwa_tickin,
         tickout        => open,
         d_in           => spwtxdp(2),
         s_in           => spwtxsp(2),
         d_out          => spwrxdp(2),
         s_out          => spwrxsp(2),
         apbi           => spwa2_apbi,
         apbo           => spwa2_apbo,
         tx_ahbsi       => spwa2_tx_ahbsi,
         tx_ahbso       => spwa2_tx_ahbso,
         tx_ahbmi       => spwa2_tx_ahbmi,
         tx_ahbmo       => spwa2_tx_ahbmo,
         rx_ahbmi       => spwa2_rx_ahbmi,
         rx_ahbmo       => spwa2_rx_ahbmo);

   --=========================================================================--
   -- Device under test
   --=========================================================================--
   tmtc: nspo_tmtc
      generic map(
         fabtech        => CFG_FABTECH,
         memtech        => CFG_MEMTECH,
         padtech        => CFG_PADTECH,
         clktech        => CFG_ClkTECH,
         apid           => apid,
         audebug        => audebug)
      port map(
         clk            => clk,
         resetn         => resetn,
         irq            => irq,
         gp_io          => gpio,
         dsurx          => dsurx,
         dsutx          => dsutx,
         transclk       => transclk,
         caduclk        => caduclk,
         caduout        => caduout,

         tcscid         => tcscid,
         tcvcid         => tcvcid,
         tcrfpos        => tcrfpos,
         tchigh         => tchigh,
         tcrise         => tcrise,
         tcpseudo       => tcpseudo,
         tcmark         => tcmark,
         tcrfa          => tcrfa,
         tcactive       => tcactive,
         tcclk          => tcclk,
         tcdata         => tcdata,
         tcgpio         => tcgpio,

         clcwin         => clcwin,
         clcwout        => clcwout,

         spw_clk        => spw_clk,
         spwrxdp        => spwrxdp_del,
         spwrxsp        => spwrxsp_del,
         spwtxdp        => spwtxdp,
         spwtxsp        => spwtxsp,

         address        => address,
         data           => data,
         cb             => cb,
         ramsn          => ramsn,
         ramoen         => ramoen,
         rwen           => rwen,
         romsn          => romsn,
         oen            => oen,
         writen         => writen);

   clcwin      <= clcwout when clcwen='1' else "ZZ";

   spwrxdp_del <= transport spwrxdp after 10 ns;
   spwrxsp_del <= transport spwrxsp after 10 ns;

   --=========================================================================--
   -- Telemetry Virtual Channels 4 through 6
   --=========================================================================--
   MainVC3VC4: process
      --------------------------------------------------------------------------
      -- Test status variables
      --------------------------------------------------------------------------
      variable TP:               Boolean := True;        -- test passed
      variable TPCounter:        Natural := 0;           -- test error counter

      --------------------------------------------------------------------------
      -- Data variables
      --------------------------------------------------------------------------
      variable D:             Std_Logic_Vector(31 downto 0);

      --------------------------------------------------------------------------
      -- Write: Transmit RMAP commands with ESA/ASTRIUM SpaceWire Link - SPW-A
      --------------------------------------------------------------------------
      procedure RmapWrite1(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         constant Data:          in    Octet_Vector;
         constant DestLogAddr:   in    Std_Logic_Vector  := X"FE";
         constant DestinationKey:in    Std_Logic_Vector  := X"00";
         constant SourceLogAddr: in    Std_Logic_Vector  := X"FE";
         constant TransactionID: in    Std_Logic_Vector  := X"0000";
         constant ExtendedAddr:  in    Std_Logic_Vector  := X"00";
         constant Verify:        in    Std_Logic         := '0';
         constant Ack:           in    Std_Logic         := '0';
         constant Inc:           in    Std_Logic         := '1';
         constant InstancePath:  in    String            := "RmapWrite";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapWrite(spwa_clk, spwa1_tx_ahbsi, spwa1_tx_ahbso, 0, 0,
                   Address, Data, DestLogAddr, DestinationKey, SourceLogAddr,
                   TransactionID, ExtendedAddr, Verify, Ack, Inc,
                   InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Write: Transmit RMAP commands with ESA/ASTRIUM SpaceWire Link - SPW-A
      --------------------------------------------------------------------------
      procedure RmapW1(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         constant Data:          in    Std_Logic_Vector  := X"00000000";
         constant InstancePath:  in    String            := "RmapW";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapW(spwa_clk, spwa1_tx_ahbsi, spwa1_tx_ahbso, 0, 0,
               Address, Data, InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Read: Transmit/receive RMAP commands with ESA/ASTRIUM SpaceWire Link
      --------------------------------------------------------------------------
      procedure RmapR1(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         variable Data:          out   Std_Logic_Vector(31 downto 0);
         constant InstancePath:  in    String            := "RmapR";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapR(spwa_clk, spwa1_tx_ahbsi, spwa1_tx_ahbso, 0, 0,
               spwa1_rx_ahbmi, spwa1_rx_ahbmo, clk, spwa1_apbi, spwa1_apbo, 0, 0,
               Address, Data, InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Compare: Transmit/receive RMAP commands with ESA/ASTRIUM SpaceWire Link
      --------------------------------------------------------------------------
      procedure RmapC1(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         constant Data:          in    Std_Logic_Vector  := X"00000000";
         variable TP:            inout Boolean;
         constant InstancePath:  in    String            := "RmapC";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapC(spwa_clk, spwa1_tx_ahbsi, spwa1_tx_ahbso, 0, 0,
               spwa1_rx_ahbmi, spwa1_rx_ahbmo, clk, spwa1_apbi, spwa1_apbo, 0, 0,
               Address, Data, TP, InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- This procedure initialises test bench
      --------------------------------------------------------------------------
      procedure tReset is
      begin
         spwa1_tx_ahbsi <= ahb_slv_in_init;
         spwa1_tx_ahbmi <= ahb_mst_in_init;
         spwa1_rx_ahbmi <= ahb_mst_in_init;
         spwa1_apbi     <= apb_slv_in_init;

         Print("- Initialise ESA/ASTRIUM SpaceWire link 1");
         InitEsaSpw(spwa_clk, spwa1_rx_ahbmi, spwa1_rx_ahbmo,
                    spwa_clk, spwa_reset1_n, spwa1_apbi, spwa1_apbo,
                    sBoardFrequency, sSpWBitFrequency, True, TP);
         Print("- SpaceWire link 1 initialized");
      end procedure tReset;

      --------------------------------------------------------------------------
      -- Test procedure
      --------------------------------------------------------------------------
      procedure tVC3 is
      begin
         Print("--=========================================================--");
         Print("tVC3 --------------------------------------------------------");
         Print("--=========================================================--");
         RmapW1(gTMAHBCTRL3, X"00000301", "gTMAHBCTRL3", False);
         RmapR1(gTMAHBCTRL3, D, "gTMAHBCTRL3", False);

         RmapW1(gTMAHBSTAT3, X"00000000", "gTMAHBSTAT3", False);
         RmapR1(gTMAHBSTAT3, D, "gTMAHBSTAT3", False);

         for i in 0 to 127 loop
            -- prevent VC3 to be disabled when switching VCIDs
            RmapW1(gTMAHBCTRL3, X"00000301", "gTMAHBCTRL3", False); 
            Print("Wait for VC3 to be ready for packet");
            d := (others => '0');
            while d(0) /= '1' loop
               RmapR1(gTMAHBSTAT3, d, "gTMAHBSTAT3", False);
               wait for 1000 us;
            end loop;

            Print("Send packet on VC3");
            RmapWrite1(TMAHB3, SourcePacket(
               VersionId      => 0,
               TypeId         => 0,
               HeaderFlag     => 0,
               ApplicationId  => 3,
               SegmentFlags   => 3,
               SequenceCount  => i,
               PacketLength   => 16+7*i,
               DataOffset     => 0,
               ErrorControl   => 1));
         end loop;

         RmapR1(gTMAHBSTAT3, D, "gTMAHBSTAT3", False);

         tIntermediate(TP, TPCounter);
      end tVC3;

      procedure tVC4 is
      begin
         Print("--=========================================================--");
         Print("tVC4 --------------------------------------------------------");
         Print("--=========================================================--");

         RmapW1(gTMAHBCTRL4, X"00000301", "gTMAHBCTRL4", False);
         RmapR1(gTMAHBCTRL4, D, "gTMAHBCTRL4", False);

         RmapW1(gTMAHBSTAT4, X"00000000", "gTMAHBSTAT4", False);
         RmapR1(gTMAHBSTAT4, D, "gTMAHBSTAT4", False);

         for i in 0 to 127 loop
            Print("Wait for VC4 to be ready for packet");
            d := (others => '0');
            while d(0) /= '1' loop
               RmapR1(gTMAHBSTAT4, d, "gTMAHBSTAT4", False);
               wait for 100 us;
            end loop;

            Print("Send packet on VC4");
            RmapWrite1(TMAHB4, SourcePacket(
               VersionId      => 0,
               TypeId         => 0,
               HeaderFlag     => 0,
               ApplicationId  => 4,
               SegmentFlags   => 3,
               SequenceCount  => i,
               PacketLength   => 8+40*i,
               DataOffset     => 0,
               ErrorControl   => 1));
         end loop;

         RmapR1(gTMAHBSTAT4, D, "gTMAHBSTAT4", False);

         tIntermediate(TP, TPCounter);
      end tVC4;

   begin
      vc3vc4done      <= false;
      tReset;
      RmapW1(gTMAHBCTRL3, X"00000000", "gTMAHBCTRL3");
      RmapW1(gTMAHBSTAT3, X"00000000", "gTMAHBSTAT3");
      RmapR1(gTMAHBCTRL3, D, "gTMAHBCTRL3");
      RmapR1(gTMAHBSTAT3, D, "gTMAHBSTAT3");
      while not vc3vc6enable or vc3vc6on='0' loop
         wait on vc3vc6on;
      end loop;
      tVC3;
      tVC4;
      vc3vc4tp        <= TP;
      vc3vc4tpcounter <= TPCounter;
      vc3vc4done      <= true;
      wait;
   end process MainVC3VC4; -----------------------------------------------------

   MainVC5VC6: process
      --------------------------------------------------------------------------
      -- Test status variables
      --------------------------------------------------------------------------
      variable TP:               Boolean := True;        -- test passed
      variable TPCounter:        Natural := 0;           -- test error counter

      --------------------------------------------------------------------------
      -- Data variables
      --------------------------------------------------------------------------
      variable D:             Std_Logic_Vector(31 downto 0);

      --------------------------------------------------------------------------
      -- Write: Transmit RMAP commands with ESA/ASTRIUM SpaceWire Link - SPW-A
      --------------------------------------------------------------------------
      procedure RmapW2(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         constant Data:          in    Std_Logic_Vector  := X"00000000";
         constant InstancePath:  in    String            := "RmapW";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapW(spwa_clk, spwa2_tx_ahbsi, spwa2_tx_ahbso, 0, 0,
               Address, Data, InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Read: Transmit/receive RMAP commands with ESA/ASTRIUM SpaceWire Link
      --------------------------------------------------------------------------
      procedure RmapR2(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         variable Data:          out   Std_Logic_Vector(31 downto 0);
         constant InstancePath:  in    String            := "RmapR";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapR(spwa_clk, spwa2_tx_ahbsi, spwa2_tx_ahbso, 0, 0,
               spwa2_rx_ahbmi, spwa2_rx_ahbmo, clk, spwa2_apbi, spwa2_apbo, 0, 0,
               Address, Data, InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Compare: Transmit/receive RMAP commands with ESA/ASTRIUM SpaceWire Link
      --------------------------------------------------------------------------
      procedure RmapC2(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         constant Data:          in    Std_Logic_Vector  := X"00000000";
         variable TP:            inout Boolean;
         constant InstancePath:  in    String            := "RmapC";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapC(spwa_clk, spwa2_tx_ahbsi, spwa2_tx_ahbso, 0, 0,
               spwa2_rx_ahbmi, spwa2_rx_ahbmo, clk, spwa2_apbi, spwa2_apbo, 0, 0,
               Address, Data, TP, InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Write: Transmit RMAP commands with ESA/ASTRIUM SpaceWire Link - SPW-A
      --------------------------------------------------------------------------
      procedure RmapWrite2(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         constant Data:          in    Data_Vector;
         constant DestLogAddr:   in    Std_Logic_Vector  := X"FE";
         constant DestinationKey:in    Std_Logic_Vector  := X"00";
         constant SourceLogAddr: in    Std_Logic_Vector  := X"FE";
         constant TransactionID: in    Std_Logic_Vector  := X"0000";
         constant ExtendedAddr:  in    Std_Logic_Vector  := X"00";
         constant Verify:        in    Std_Logic         := '0';
         constant Ack:           in    Std_Logic         := '0';
         constant Inc:           in    Std_Logic         := '1';
         constant InstancePath:  in    String            := "RmapWrite";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapWrite(spwa_clk, spwa2_tx_ahbsi, spwa2_tx_ahbso, 0, 0,
                   Address, Data, DestLogAddr, DestinationKey, SourceLogAddr,
                   TransactionID, ExtendedAddr, Verify, Ack, Inc,
                   InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- This procedure initialises test bench
      --------------------------------------------------------------------------
      procedure tReset is
      begin
         spwa2_tx_ahbsi <= ahb_slv_in_init;
         spwa2_tx_ahbmi <= ahb_mst_in_init;
         spwa2_rx_ahbmi <= ahb_mst_in_init;
         spwa2_apbi     <= apb_slv_in_init;

         Print("- Initialise ESA/ASTRIUM SpaceWire link 2");
         InitEsaSpw(spwa_clk, spwa2_rx_ahbmi, spwa2_rx_ahbmo,
                    spwa_clk, spwa_reset2_n, spwa2_apbi, spwa2_apbo,
                    sBoardFrequency, sSpWBitFrequency, True, TP);
         Print("- SpaceWire link 2 initialized");
      end procedure tReset;

      procedure tVC5 is
      begin
         Print("--=========================================================--");
         Print("tVC5 --------------------------------------------------------");
         Print("--=========================================================--");

         RmapW2(gTMAHBCTRL5, X"00000301", "gTMAHBCTRL5", False);
         RmapR2(gTMAHBCTRL5, D, "gTMAHBCTRL5", False);

         RmapW2(gTMAHBSTAT5, X"00000000", "gTMAHBSTAT5", False);
         RmapR2(gTMAHBSTAT5, D, "gTMAHBSTAT5", False);

         for i in 0 to 127 loop
--            Print("Wait for VC5 to be ready for packet");
--            d := (others => '0');
--            while d(0) /= '1' loop
--               RmapR2(gTMAHBSTAT5, d, "gTMAHBSTAT5", False);
--               wait for 1000 us;
--            end loop;

            Print("Send packet on VC5");
            RmapWrite2(TMAHB5, SourcePacket32(
               VersionId      => 0,
               TypeId         => 0,
               HeaderFlag     => 0,
               ApplicationId  => 5,
               SegmentFlags   => 3,
               SequenceCount  => i,
               PacketLength   => 12+16*i,
               DataOffset     => 0,
               ErrorControl   => 1));
         end loop;

         RmapR2(gTMAHBSTAT5, D, "gTMAHBSTAT5", False);

         tIntermediate(TP, TPCounter);
      end tVC5;

      procedure tVC6 is
      begin
         Print("--=========================================================--");
         Print("tVC6 --------------------------------------------------------");
         Print("--=========================================================--");

         RmapW2(gTMAHBCTRL6, X"00000301", "gTMAHBCTRL6", False);
         RmapR2(gTMAHBCTRL6, D, "gTMAHBCTRL6", False);

         RmapW2(gTMAHBSTAT6, X"00000000", "gTMAHBSTAT6", False);
         RmapR2(gTMAHBSTAT6, D, "gTMAHBSTAT6", False);

         for i in 0 to 127 loop
            Print("Wait for VC6 to be ready for packet");
            d := (others => '0');
            while d(0) /= '1' loop
               RmapR2(gTMAHBSTAT6, d, "gTMAHBSTAT6", False);
               wait for 100 us;
            end loop;

            Print("Send packet on VC6");
            RmapWrite2(TMAHB6, SourcePacket32(
               VersionId      => 0,
               TypeId         => 0,
               HeaderFlag     => 0,
               ApplicationId  => 6,
               SegmentFlags   => 3,
               SequenceCount  => i,
               PacketLength   => 32+32*i,
               DataOffset     => 0,
               ErrorControl   => 1));
         end loop;

         RmapR2(gTMAHBSTAT6, D, "gTMAHBSTAT6", False);

         tIntermediate(TP, TPCounter);
      end tVC6;
   begin
      vc5vc6done      <= false;
      tReset;
      RmapW2(gTMAHBCTRL5, X"00000000", "gTMAHBCTRL5");
      RmapW2(gTMAHBSTAT5, X"00000000", "gTMAHBSTAT5");
      RmapR2(gTMAHBCTRL5, D, "gTMAHBCTRL5");
      RmapR2(gTMAHBSTAT5, D, "gTMAHBSTAT5");
      while not vc3vc6enable or vc3vc6on='0' loop
         wait on vc3vc6on;
      end loop;
      tVC5;
      tVC6;
      vc5vc6tp        <= TP;
      vc5vc6tpcounter <= TPCounter;
      vc5vc6done      <= true;
      wait;
   end process MainVC5VC6; -----------------------------------------------------


   --=========================================================================--
   -- THIS IS THE MAIN TEST HARNESS
   --=========================================================================--
   Main: process
      --------------------------------------------------------------------------
      -- Frame variables
      --------------------------------------------------------------------------
      variable MC:               Natural range 0 to 255        := 0;
      variable VC:               Natural range 0 to 2**24-1    := 0;
      variable EVC:              Natural range 0 to 16777215   := 0;
      variable PC:               Natural range 0 to 16383      := 0;

      --------------------------------------------------------------------------
      -- Test status variables
      --------------------------------------------------------------------------
      variable TP:               Boolean := True;        -- test passed
      variable TPCounter:        Natural := 0;           -- test error counter

      --------------------------------------------------------------------------
      -- Variables for time measurements
      --------------------------------------------------------------------------
      variable Time0:            Time;                   -- supporting time
      variable Time1:            Time;
      variable Time2:            Time;
      variable Time3:            Time;

      --------------------------------------------------------------------------
      -- Reporting variables
      --------------------------------------------------------------------------
      variable L:                Line;

      --------------------------------------------------------------------------
      -- Data variables
      --------------------------------------------------------------------------
      variable Data:             Octet_Vector(0 to 2047);

      variable DV:            Data_Vector(0 to 255);
      variable CV:            Data_Vector(0 to 255);
      variable TV:            Data_Vector(0 to 1023);

      variable A:             Std_Logic_Vector(31 downto 0);

      variable D:             Std_Logic_Vector(31 downto 0);
      variable C:             Std_Logic_Vector(31 downto 0);

      constant Z:             Std_Logic_Vector(31 downto 0) := (others => '0');
      constant O:             Std_Logic_Vector(31 downto 0) := (others => '1');

      variable OV:            Octet_Vector(0 to 1023);
      variable OCV:           Octet_Vector(0 to 1023);

      variable lac:           natural_vector(0 to 2);


      -----------------------------------------------------------------------------
      -- Write: Transmit RMAP commands with ESA/ASTRIUM SpaceWire Link - SPW-A
      -----------------------------------------------------------------------------
      procedure RmapWriteX(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         constant Data:          in    Octet_Vector;
         constant DestLogAddr:   in    Std_Logic_Vector  := X"FE";
         constant DestinationKey:in    Std_Logic_Vector  := X"00";
         constant SourceLogAddr: in    Std_Logic_Vector  := X"FE";
         constant TransactionID: in    Std_Logic_Vector  := X"0000";
         constant ExtendedAddr:  in    Std_Logic_Vector  := X"00";
         constant Verify:        in    Std_Logic         := '0';
         constant Ack:           in    Std_Logic         := '0';
         constant Inc:           in    Std_Logic         := '1';
         constant InstancePath:  in    String            := "RmapWrite";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapWrite(spwa_clk, spwa0_tx_ahbsi, spwa0_tx_ahbso, 0, 0,
                   Address, Data, DestLogAddr, DestinationKey, SourceLogAddr,
                   TransactionID, ExtendedAddr, Verify, Ack, Inc,
                   InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Write: Transmit RMAP commands with ESA/ASTRIUM SpaceWire Link - SPW-A
      --------------------------------------------------------------------------
      procedure RmapWrite0(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         constant Data:          in    Data_Vector;
         constant DestLogAddr:   in    Std_Logic_Vector  := X"FE";
         constant DestinationKey:in    Std_Logic_Vector  := X"00";
         constant SourceLogAddr: in    Std_Logic_Vector  := X"FE";
         constant TransactionID: in    Std_Logic_Vector  := X"0000";
         constant ExtendedAddr:  in    Std_Logic_Vector  := X"00";
         constant Verify:        in    Std_Logic         := '0';
         constant Ack:           in    Std_Logic         := '0';
         constant Inc:           in    Std_Logic         := '1';
         constant InstancePath:  in    String            := "RmapWrite";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapWrite(spwa_clk, spwa0_tx_ahbsi, spwa0_tx_ahbso, 0, 0,
                   Address, Data, DestLogAddr, DestinationKey, SourceLogAddr,
                   TransactionID, ExtendedAddr, Verify, Ack, Inc,
                   InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Write: Transmit RMAP commands with ESA/ASTRIUM SpaceWire Link - SPW-A
      --------------------------------------------------------------------------
      procedure RmapW0(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         constant Data:          in    Std_Logic_Vector  := X"00000000";
         constant InstancePath:  in    String            := "RmapW";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapW(spwa_clk, spwa0_tx_ahbsi, spwa0_tx_ahbso, 0, 0,
               Address, Data, InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Read: Transmit/receive RMAP commands with ESA/ASTRIUM SpaceWire Link
      --------------------------------------------------------------------------
      procedure RmapR0(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         variable Data:          out   Std_Logic_Vector(31 downto 0);
         constant InstancePath:  in    String            := "RmapR";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapR(spwa_clk, spwa0_tx_ahbsi, spwa0_tx_ahbso, 0, 0,
               spwa0_rx_ahbmi, spwa0_rx_ahbmo, clk, spwa0_apbi, spwa0_apbo, 0, 0,
               Address, Data, InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Compare: Transmit/receive RMAP commands with ESA/ASTRIUM SpaceWire Link
      --------------------------------------------------------------------------
      procedure RmapC0(
         constant Address:       in    Std_Logic_Vector  := X"00000000";
         constant Data:          in    Std_Logic_Vector  := X"00000000";
         variable TP:            inout Boolean;
         constant InstancePath:  in    String            := "RmapC";
         constant ScreenOutput:  in    Boolean           := False) is
      begin
         RmapC(spwa_clk, spwa0_tx_ahbsi, spwa0_tx_ahbso, 0, 0,
               spwa0_rx_ahbmi, spwa0_rx_ahbmo, clk, spwa0_apbi, spwa0_apbo, 0, 0,
               Address, Data, TP, InstancePath, ScreenOutput);
      end procedure;

      --------------------------------------------------------------------------
      -- Increase one of the AU LACs
      --------------------------------------------------------------------------
      procedure IncreaseLAC(
        constant lacid:          in    natural range 0 to 2) is
      begin
        if lacid < 2 then
          if lac(lacid) = 2**30-1 then
            lac(lacid) := 0;
          else
            lac(lacid) := lac(lacid) + 1;
          end if;
        else -- Recovery LAC
          if lac(lacid) mod 256 = 255 then
            lac(lacid) := 2**30-1 - 255;
          else
            lac(lacid) := lac(lacid) + 1;
          end if;
        end if;
      end procedure;

      --------------------------------------------------------------------------
      -- Increase one of the AU LACs
      --------------------------------------------------------------------------
      procedure SetLAC(
        constant lacid:          in    natural range 0 to 2;
        constant newcount:       in    natural range 0 to 2**30-1) is
      begin
        if lacid < 2 then
           lac(lacid) := newcount;
        else -- Recovery LAC
           lac(lacid) := 2**30-256 + newcount mod 256;
        end if;
      end procedure;

      --------------------------------------------------------------------------
      -- This procedure initialises test bench
      --------------------------------------------------------------------------
      procedure tReset is
      begin
         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("Initialise --------------------------------------------------");
         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("tReset");
         Print("--=========================================================--");

         Write(L, Now, Right, 15);
         Write(L, String'(" : System period [ns]:         "));
         Write(L, sBoardPeriod);
         WriteLine(Output, L);
         Write(L, Now, Right, 15);
         Write(L, String'(" : Telemetry period [ns]:      "));
         Write(L, sTMBitPeriod);
         WriteLine(Output, L);
         Write(L, Now, Right, 15);
         Write(L, String'(" : Telecommand period [ns]:    "));
         Write(L, sTCBitPeriod);
         WriteLine(Output, L);
         Write(L, Now, Right, 15);
         Write(L, String'(" : SpaceWire period [ns]:      "));
         Write(L, sSpWBitPeriod);
         WriteLine(Output, L);

         Write(L, Now, Right, 15);
         Write(L, String'(" : System frequency [Hz]:      "));
         Write(L, sBoardFrequency);
         WriteLine(Output, L);
         Write(L, Now, Right, 15);
         Write(L, String'(" : Telemetry frequency [Hz]:   "));
         Write(L, sTMBitFrequency);
         WriteLine(Output, L);
         Write(L, Now, Right, 15);
         Write(L, String'(" : Telecommand frequency [Hz]: "));
         Write(L, sTCBitFrequency);
         WriteLine(Output, L);
         Write(L, Now, Right, 15);
         Write(L, String'(" : SpaceWire frequency [Hz]:   "));
         Write(L, sSpWBitFrequency);
         WriteLine(Output, L);

         Print("-- Assert reset");
         -- Reset
         resetn         <= '0';
         clcwen         <= '0';
         clcwin         <= "11";

         spwa0_tx_ahbsi <= ahb_slv_in_init;
         spwa0_tx_ahbmi <= ahb_mst_in_init;
         spwa0_rx_ahbmi <= ahb_mst_in_init;
         spwa0_apbi     <= apb_slv_in_init;

         -- default values
         tcscid         <= stcscid;
         tcvcid         <= stcvcid;
         tcrfpos        <= stcrfpos;
         tchigh         <= stchigh;
         tcrise         <= stcrise;
         tcpseudo       <= stcpseudo;
         tcmark         <= stcmark;

         tcrfa          <= "0000";

         -- gpio
         gpio           <= "1HHHLL1H";
         
         -- debug uart
         dsurx          <= '1';

         -- tc transmitter
         Txreset_n      <= '1';
         TxRequest      <= (others => '0');
         TxBitPeriod    <= sTCBitPeriod;

         -- tm receiver and sources
         TMreset_n      <= '0';

         Synchronise(clk, 20 ns);
         resetn         <= '0', '1' after sBoardPeriod * 127;

         Print("- Initialise ESA/ASTRIUM SpaceWire link 0");
         InitEsaSpw(spwa_clk, spwa0_rx_ahbmi, spwa0_rx_ahbmo,
                    spwa_clk, spwa_reset0_n, spwa0_apbi, spwa0_apbo,
                    sBoardFrequency, sSpWBitFrequency, True, TP);
         Print("- SpaceWire link 0 initialized");


         Print("-- De-assert TM reset");
         Synchronise(clk, 20 ns);
         TMreset_n      <= '1';

         for i in 0 to 31 loop
            Synchronise(clk, 10 ns);
         end loop;


         lac(0) := 2**30-1;
         lac(1) := 2**30-1;
         lac(2) := 2**30-256;
      end procedure tReset;

      --------------------------------------------------------------------------
      -- Memory controller tests
      --------------------------------------------------------------------------
      procedure tMRAM is
      begin
         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("Memory controller access to MRAM");
         Print("--=========================================================--");
         Print("--=========================================================--");
         RmapW0(cMCFG1,  X"00000801", "cMCFG1");
         RmapC0(cMCFG1,  X"00000801", TP, "cMCFG1");

         RmapW0(cMCFG2,  X"00000041", "cMCFG2");
         RmapC0(cMCFG2,  X"00000041", TP, "cMCFG2");

         RmapW0(cMCFG3,  X"00000200", "cMCFG3");     -- EDAC for 8bit PROM is disabled
         RmapC0(cMCFG3,  X"00000200", TP, "cMCFG3");

         RmapW0(X"00000000", X"12345678", "00000000");
         RmapC0(X"00000000", X"12345678", TP, "00000000");
         
         RmapC0(cAHBSTAT, X"0000003A", TP, "cAHBSTAT");
         RmapC0(cAHBADDR, cAHBADDR, TP, "cAHBADDR");

         Print("Byte-write to MRAM without EDAC");
         OV(0) := X"EF";
         RmapWriteX(
            Address        => X"00000000",
            Data           => OV(0 to 0),
            ScreenOutput   => False);

         RmapC0(X"00000000", X"EF345678", TP, "00000000");
         
         RmapC0(cAHBSTAT, X"0000003A", TP, "cAHBSTAT");
         RmapC0(cAHBADDR, cAHBADDR, TP, "cAHBADDR");

         Print("Test MBE for AHBSTAT");
         RmapW0(cGRGPREG, X"00000001", "GRGPIO");
         RmapW0(X"00000004", X"12233445", "00000004");
         RmapR0(X"00000004", d, "00000004", False);
         -- Optionally for more precision use: 
         -- force -freeze sim:/nspo_tmtc_tb/tmtc/gp_io(1) 0 4046451 ns, 1  4046501 ns ;
         gpio(1) <= '0';  -- MBE on gpio1
         RmapC0(X"00000004", X"12233445", TP, "00000004");
         gpio(1) <= '1';  -- MBE
         RmapC0(cAHBADDR, X"00000004", TP, "cAHBADDR"); --Check MBE fault address
         RmapR0(cAHBSTAT, d, "cAHBSTAT", False);
         
         -- Clear Interrupt
         RmapW0(cAHBSTAT, X"0000003A", "cAHBSTAT");
         RmapW0(cIRQ_PENDING, X"00000000", "cIRQ_PENDING"); 

         Print("Wash MRAM with EDAC");
         OV := (others => X"00");
         RmapWriteX(
            Address        => X"00020000",
            Data           => OV(0 to 4*32-1),
            ScreenOutput   => False);

--         for i in 0 to 31  loop
--            RmapW0(Conv_Std_Logic_Vector(16#20000# + i*4, 32), X"00000000", "MRAM");
--         end loop;
--         for i in 0 to 31  loop
--            RmapR0(Conv_Std_Logic_Vector(16#20000# + i*4, 32), d, "MRAM");
--         end loop;

         RmapC0(cAHBSTAT, X"0000003A", TP, "cAHBSTAT");
         RmapC0(cAHBADDR, cAHBADDR, TP, "cAHBADDR");

         tIntermediate(TP, TPCounter);
      end tMRAM;

      --------------------------------------------------------------------------
      -- Telecommand decoder tests
      --------------------------------------------------------------------------
      procedure tTelecommand is
         variable Period:        Time;
         variable OV:            Octet_Vector(0 to 255);
         variable SeqCntr:       Natural := 0;           -- packet sequence
         constant NA:            Std_Logic_Vector := "----------------";
      begin
         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("CCSDS Telecomand Decoder: software commands from MRAM");
         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("Default values after reset ----------------------------------");
         RmapC0 (gTCSTR,  X"00000000", TP, "gTCSTR");
         RmapC0 (gTCASR,  X"00000000", TP, "gTCASR");
         RmapC0 (gTCRRP,  X"00000000", TP, "gTCRRP");
         RmapC0 (gTCRWP,  X"00000000", TP, "gTCRWP");

         Print("Initiate basic transfer -------------------------------------");
         RmapW0(gTCASR,  X"0002001F", "gTCASR");
         RmapC0(gTCSTR,  X"00000000", TP, "gTCSTR");
         RmapC0(gTCASR,  X"0002001F", TP, "gTCASR");
         RmapW0(gTCRRP,  X"00020000", "gTCRRP");
         RmapC0(gTCSTR,  X"00000000", TP, "gTCSTR");
         RmapC0(gTCASR,  X"0002001F", TP, "gTCASR");
         RmapC0(gTCRRP,  X"00020000", TP, "gTCRRP");
         RmapC0(gTCRWP,  X"00020000", TP, "gTCRWP");

         Print("Enable telecommand decoder ----------------------------------");
         RmapW0(gTCCOR,  X"55000001", "gTCCOR");
         RmapC0(gTCCOR,  X"00000001", TP, "gTCCOR");

         RmapW0(gTCIMR,  X"FFFFFFFF", "gTCIMR");

         Print("Create correct CLTU -----------------------------------------");
         Data (0 to 13) := SourcePacket(0, 1, 0, 5, 3, SeqCntr, 14, 0, 1);
         SeqCntr  := (SeqCntr+1) mod 2**14;

         Print("Transfer CLTU");
         TxPW(CLTU(Data(0 to 13)), TxData,TxCount,TxRequest(2),TxAck);

         wait for TxBitPeriod * 300;

         Print("Verify CLTU data");
         RmapC0(X"00020000", Data( 0) & X"01" & Data( 1) & X"00",
                TP, "");
         RmapC0(X"00020004", Data( 2) & X"00" & Data( 3) & X"00",
                TP, "");
         RmapC0(X"00020008", Data( 4) & X"00" & Data( 5) & X"00",
                TP, "");
         RmapC0(X"0002000C", Data( 6) & X"00" & Data( 7) & X"00",
                TP, "");
         RmapC0(X"00020010", Data( 8) & X"00" & Data( 9) & X"00",
                TP, "");
         RmapC0(X"00020014", Data(10) & X"00" & Data(11) & X"00",
                TP, "");
         RmapC0(X"00020018", Data(12) & X"00" & Data(13) & X"00",
                TP, "");
         RmapC0(X"0002001C", X"00"    & X"02" & NA,
                TP, "");

         Print("Verify Write Pointer");
         RmapC0 (gTCRWP, X"0002001E", TP, "gTCRWP");

         Print("Verify Frame Analysis Report");
         RmapC0 (gTCFAR,  '0' & "0000" & X"02" & "000" & "00" &
                  "010" & "00000000000", TP, "gTCFAR");

         RmapC0 (gTCFAR,  '1' & "0000" & X"02" & "000" & "00" &
                  "010" & "00000000000", TP, "gTCFAR");
         tIntermediate(TP, TPCounter);
      end tTelecommand;

      --------------------------------------------------------------------------
      -- Telecommand decoder test
      --------------------------------------------------------------------------
      procedure tTelecommandHardware is
         variable OV:            Octet_Vector(0 to 255);
         variable key:           key_type;
         variable mapid:         natural range 0 to 31 := 0;
         variable sflags:        natural range 0 to 3  := 3;
         variable cflag:         natural range 0 to 1  := 0;
         variable lacid:         natural range 0 to 2  := 0;

         -----------------------------------------------------------------------
         -- Hardware Telecommand Instruction
         -----------------------------------------------------------------------
         function HWCommand(
            constant OutputNumber:  in Std_Logic_Vector;
            constant PulseLength:   in Natural)
            return                     Octet_Vector is
            constant OutputVector:     Std_Logic_Vector(0 to 31) := OutputNumber;
            constant PulseVector:      Std_Logic_Vector(0 to 7) :=
                                          Conv_Std_Logic_Vector(PulseLength, 8);
            variable Result:           Octet_Vector(0 to 4);
         begin
            Result(0)      := OutputVector( 0 to  7);
            Result(1)      := OutputVector( 8 to 15);
            Result(2)      := OutputVector(16 to 23);
            Result(3)      := OutputVector(24 to 31);
            Result(4)      := PulseVector;
            return Result;
         end function HWCommand;
      begin
         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("CCSDS Telecomand Decoder: hardware commands");
         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("-------------------------------------------------------------");
         Print("Check reset value");
         Print("-------------------------------------------------------------");
         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");

         -- AU is disabled by default, no need to disable

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - TC 1 without pseudo -------------------------");
         Print("Set #13# ----------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '0';
         Synchronise(clk, 10 ns);

         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid,
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"00001300", 255),
               ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(1),TxAck);

         wait for TxBitPeriod * 450;
         Check(TP, TCGPIO, X"000013", "TC HW GPIO COMMAND");

         RmapC0(gTCAUFAR,      X"70188" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
         if lacid = 0 then
           RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         elsif lacid = 1 then
           RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         elsif lacid = 2 then
           RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end if;
         
         Print("Clear #02# --------------------------------------------------");
         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid,
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"00001100", 0),
               ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(1),TxAck);

         wait for TxBitPeriod * 450;
         Check(TP, TCGPIO, X"000011", "TC HW GPIO COMMAND");

         RmapC0(gTCAUFAR,      X"70188" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
         if lacid = 0 then
           RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         elsif lacid = 1 then
           RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         elsif lacid = 2 then
           RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end if;
         
         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - TC 0 with pseudo ----------------------------");
         Print("Pulse #0c# --------------------------------------------------");
         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid,
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"00000c00", 3),
               ErrorControl   => 1)
                    )
                            ))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         Check(TP, TCGPIO, X"00001d", "TC HW GPIO COMMAND");
         wait for 3 * 8192 * sBoardPeriod;
         Check(TP, TCGPIO, X"000011", "TC HW GPIO COMMAND");

         RmapC0(gTCAUFAR,      X"70188" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
         if lacid = 0 then
           RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         elsif lacid = 1 then
           RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         elsif lacid = 2 then
           RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end if;
         
         Print("Clear #00# --------------------------------------------------");
         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid,
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"00000000", 0),
               ErrorControl   => 1)
                    )
                            ))),
              TxData,TxCount,TxRequest(0),TxAck);
         wait for TxBitPeriod * 450;
         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");

         RmapC0(gTCAUFAR,      X"70188" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
         if lacid = 0 then
           RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         elsif lacid = 1 then
           RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         elsif lacid = 2 then
           RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end if;
         
         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - TC 3 without pseudo -------------------------");
         Print("Set #80000000# ----------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '0';
         Synchronise(clk, 10 ns);

         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid,
                    SourcePacket(
                      VersionId      => 0,
                      TypeId         => 1,
                      HeaderFlag     => 0,
                      ApplicationId  => apid,
                      SegmentFlags   => 3,
                      SequenceCount  => 0,
                      Data           => HWCommand(X"80000000", 255),
                      ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"800000", "TC HW GPIO COMMAND");
         Check(TP, TCGPIO, X"800000", "TC HW GPIO COMMAND");

         RmapC0(gTCAUFAR,      X"70189" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
         if lacid = 0 then
           RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         elsif lacid = 1 then
           RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         elsif lacid = 2 then
           RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end if;
         
         Print("Clear #00# --------------------------------------------------");
         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, 
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"00000000", 0),
               ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");
         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");

         RmapC0(gTCAUFAR,      X"70189" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
         if lacid = 0 then
           RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         elsif lacid = 1 then
           RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         elsif lacid = 2 then
           RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end if;
         
         wait for TxBitPeriod * 450;

         tIntermediate(TP, TPCounter);
      end tTelecommandHardware;

      procedure tTelecommandAuthentication is
         variable OV:            Octet_Vector(0 to 255);
         variable fkey, pkey:    key_type;
         variable mapid:         natural range 0 to 31 := 0;
         variable sflags:        natural range 0 to 3  := 3;
         variable cflag:         natural range 0 to 1  := 0;
         variable lacid:         natural range 0 to 2  := 0;
         variable seed1, seed2:  positive;
         variable tmp:           integer;
        
         -----------------------------------------------------------------------
         -- Hardware Telecommand Instruction
         -----------------------------------------------------------------------
         function HWCommand(
            constant OutputNumber:  in Std_Logic_Vector;
            constant PulseLength:   in Natural)
            return                     Octet_Vector is
            constant OutputVector:     Std_Logic_Vector(0 to 31) := OutputNumber;
            constant PulseVector:      Std_Logic_Vector(0 to 7) :=
                                          Conv_Std_Logic_Vector(PulseLength, 8);
            variable Result:           Octet_Vector(0 to 4);
         begin
            Result(0)      := OutputVector( 0 to  7);
            Result(1)      := OutputVector( 8 to 15);
            Result(2)      := OutputVector(16 to 23);
            Result(3)      := OutputVector(24 to 31);
            Result(4)      := PulseVector;
            return Result;
         end function HWCommand;
      begin
         seed1 := 23;
         seed2 := 124;
         
         -- Set key to fixed key
         for i in 0 to 59 loop
           fkey.w(i) := fixedkey_c.weights(i);
           fkey.c(i) := fixedkey_c.coeffs(i);
         end loop;

         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("CCSDS Telecomand Decoder: hardware authentication unit");
         Print("--=========================================================--");
         Print("--=========================================================--");

         -- Enable AU for all MAP IDs
         Print("-------------------------------------------------------------");
         Print("Enable authentication ---------------------------------------");
         Print("-------------------------------------------------------------");
         RmapW0(gTCAUCFG, X"0000009F", "gTCAUCFG");

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Copy fixed key to programmable key ----------");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    ccloadfixedkey)))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         IncreaseLAC(lacid);
         pkey := fkey;
         
         RmapC0(gTCAUFAR,      X"70188" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Set new LAC count value (Principal) ---------");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    ccsetnewlaccount(lacid,2**30-3))))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         SetLAC(lacid,2**30-3);
         
         RmapC0(gTCAUFAR,      X"70208" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
  
         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Set new LAC count value (Auxiliary) ---------");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;
         lacid  := 1;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    ccsetnewlaccount(lacid,16#123456#))))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         SetLAC(lacid,16#123456#);
         
         RmapC0(gTCAUFAR,      X"70208" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
  
         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Change programmable key block A -------------");
         Print("-------------------------------------------------------------");

         for i in 0 to 51 loop
            Synchronise(clk, 10 ns);
            TCPseudo <= '1';
            Synchronise(clk, 10 ns);

            if i < 10 then
               Print("Block " & integer'image(i+1) & " -----------------------------------------------------");
            else
               Print("Block " & integer'image(i+1) & " ----------------------------------------------------");
            end if;

            mapid  := 31;
            cflag  := 1;
            sflags := 3;

            for j in 0 to 6 loop
              gen_rand_int(255.0,seed1,seed2,tmp);
              OV(j) := Conv_Std_Logic_Vector(tmp,8);
            end loop;

            Print("Transfer CLTU");
            TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
               Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey, ccchangeblocka(i*5,OV(0 to 6)))))),
                 TxData,TxCount,TxRequest(0),TxAck);

            wait for TxBitPeriod * 450;
         
            changekey(Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey, ccchangeblocka(i*5,OV(0 to 6))), fkey, pkey);
         
            IncreaseLAC(lacid);

            RmapC0(gTCAUFAR,      X"70208" & '0' &
                   conv_std_logic_vector(32*cflag+mapid, 6) &
                   '0' & X"4", TP, "gTCAUFAR");
            RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
            RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
            RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end loop;

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Change programmable key block B -------------");
         Print("-------------------------------------------------------------");

         for i in 52 to 73 loop
            Synchronise(clk, 10 ns);
            TCPseudo <= '1';
            Synchronise(clk, 10 ns);

            Print("Block " & integer'image(i+1) & " ----------------------------------------------------");
            
            mapid  := 31;
            cflag  := 1;
            sflags := 3;

            for j in 0 to 6 loop
              gen_rand_int(255.0,seed1,seed2,tmp);
              OV(j) := Conv_Std_Logic_Vector(tmp,8);
            end loop;

            Print("Transfer CLTU");
            TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
               Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey, ccchangeblockb(i*5-256,OV(0 to 6)))))),
                 TxData,TxCount,TxRequest(0),TxAck);

            wait for TxBitPeriod * 450;
         
            changekey(Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey, ccchangeblockb(i*5-256,OV(0 to 6))), fkey, pkey);

            IncreaseLAC(lacid);
         
            RmapC0(gTCAUFAR,      X"70208" & '0' &
                   conv_std_logic_vector(32*cflag+mapid, 6) &
                   '0' & X"4", TP, "gTCAUFAR");
            RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
            RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
            RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end loop;

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Select programmable key ---------------------");
         Print("-------------------------------------------------------------");

         hwrite(l,pkey.c);
         writeline(output,l);
         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), pkey,
                    ccselectprogrammablekey)))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         IncreaseLAC(lacid);
         
         RmapC0(gTCAUFAR,      X"70188" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000080" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - TC 3 without pseudo -------------------------");
         Print("Set #80000000# ----------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '0';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 0;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), pkey,
                    SourcePacket(
                      VersionId      => 0,
                      TypeId         => 1,
                      HeaderFlag     => 0,
                      ApplicationId  => apid,
                      SegmentFlags   => 3,
                      SequenceCount  => 0,
                      Data           => HWCommand(X"80000000", 255),
                      ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"800000", "TC HW GPIO COMMAND");
         Check(TP, TCGPIO, X"800000", "TC HW GPIO COMMAND");
         IncreaseLAC(lacid);

         RmapC0(gTCAUFAR,      X"70289" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"2", TP, "gTCAUFAR");
         RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
         if lacid = 0 then
           RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         elsif lacid = 1 then
           RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         elsif lacid = 2 then
           RmapC0(gTCAUSR3,    X"000080" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end if;
         
         Print("Clear #00# --------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '0';
         Synchronise(clk, 10 ns);

         mapid  := 0;
         cflag  := 0;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), pkey,
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"00000000", 0),
               ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");
         IncreaseLAC(lacid);

         RmapC0(gTCAUFAR,      X"70289" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"2", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000080" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         
         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Incorrect segment length --------------------");
         Print("-------------------------------------------------------------");

         for i in 0 to 9 loop
            Print("Segment length " & integer'image(i) & " --------------------------------------------");
            
            Synchronise(clk, 10 ns);
            TCPseudo <= '0';
            Synchronise(clk, 10 ns);

            mapid  := 23;
            cflag  := 1;
            sflags := 3;

            Print("Transfer CLTU");
            TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
               Segment(sflags, cflag, mapid, lacid, lac(lacid), pkey,
                       SourcePacket(
                         VersionId      => 0,
                         TypeId         => 1,
                         HeaderFlag     => 0,
                         ApplicationId  => apid,
                         SegmentFlags   => 3,
                         SequenceCount  => 0,
                         Data           => HWCommand(X"FFFFFFFF", 255),
                         ErrorControl   => 1)
                       )(0 to i-1)
                           )),
                 TxData,TxCount,TxRequest(3),TxAck);

            wait for TxBitPeriod * 450;

            Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");
--            IncreaseLAC(lacid);

            if i = 0 then
               RmapC0(gTCAUFAR,      X"10089800", TP, "gTCAUFAR");
            elsif i < 8 then
               RmapC0(gTCAUFAR,      X"10109800", TP, "gTCAUFAR");
            else
               RmapC0(gTCAUFAR,      X"10189800", TP, "gTCAUFAR");
            end if;
            RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
            RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
            RmapC0(gTCAUSR3,    X"000080" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end loop;

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Error in packet CRC -------------------------");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '0';
         Synchronise(clk, 10 ns);

         mapid  := 15;
         cflag  := 1;
         sflags := 3;

         OV(0 to 6) := (others => X"FF");

         Print("Transfer CLTU with packet CRC error");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), pkey,
                    SourcePacket(
                      VersionId      => 0,
                      TypeId         => 1,
                      HeaderFlag     => 0,
                      ApplicationId  => apid,
                      SegmentFlags   => 3,
                      SequenceCount  => 0,
                      Data           => OV(0 to 6),
                      ErrorControl   => 0)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");
         IncreaseLAC(lacid);

         RmapC0(gTCAUFAR,      X"70289" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"2", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000080" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");


         
         wait for TxBitPeriod * 450;

         Print("-------------------------------------------------------------");
         Print("Disable authentication --------------------------------------");
         Print("-------------------------------------------------------------");
         RmapW0(gTCAUCFG, X"00000000", "gTCAUCFG");
         
         tIntermediate(TP, TPCounter);
      end tTelecommandAuthentication;

      procedure tTelecommandAuthenticationDisable is
         variable OV:            Octet_Vector(0 to 255);
         variable fkey, pkey:    key_type;
         variable mapid:         natural range 0 to 31 := 0;
         variable sflags:        natural range 0 to 3  := 3;
         variable cflag:         natural range 0 to 1  := 0;
         variable lacid:         natural range 0 to 2  := 0;
         variable seed1, seed2:  positive;
         variable tmp:           integer;
         variable expio:         Std_Logic_Vector(23 downto 0);
         variable aumap:         natural range 0 to 31 := 0;
        
         -----------------------------------------------------------------------
         -- Hardware Telecommand Instruction
         -----------------------------------------------------------------------
         function HWCommand(
            constant OutputNumber:  in Std_Logic_Vector;
            constant PulseLength:   in Natural)
            return                     Octet_Vector is
            constant OutputVector:     Std_Logic_Vector(0 to 31) := OutputNumber;
            constant PulseVector:      Std_Logic_Vector(0 to 7) :=
                                          Conv_Std_Logic_Vector(PulseLength, 8);
            variable Result:           Octet_Vector(0 to 4);
         begin
            Result(0)      := OutputVector( 0 to  7);
            Result(1)      := OutputVector( 8 to 15);
            Result(2)      := OutputVector(16 to 23);
            Result(3)      := OutputVector(24 to 31);
            Result(4)      := PulseVector;
            return Result;
         end function HWCommand;
      begin
         seed1 := 41;
         seed2 := 4123;

         -- Set key to fixed key
         for i in 0 to 59 loop
           fkey.w(i) := fixedkey_c.weights(i);
           fkey.c(i) := fixedkey_c.coeffs(i);
         end loop;

         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("CCSDS Telecomand Decoder: hardware authentication disabling");
         Print("--=========================================================--");
         Print("--=========================================================--");

         Print("-------------------------------------------------------------");
         Print("Enable authentication ---------------------------------------");
         Print("-------------------------------------------------------------");
         RmapW0(gTCAUCFG, X"0000009F", "gTCAUCFG");

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Select fixed key ----------------------------");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    ccselectfixedkey)))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         IncreaseLAC(lacid);
         
         RmapC0(gTCAUFAR,      X"70188" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");

         Print("-------------------------------------------------------------");
         Print("Disable authentication --------------------------------------");
         Print("-------------------------------------------------------------");
         RmapW0(gTCAUCFG, X"00000000", "gTCAUCFG");
         
         Print("-------------------------------------------------------------");
         Print("Transfer CLTU with AU tail - TC 3 without pseudo ------------");
         Print("Set #80000000# ----------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '0';
         Synchronise(clk, 10 ns);

         mapid  := 0;
         cflag  := 0;
         sflags := 3;

         lacid := 0;

         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"80000000", 255),
               ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");
--         IncreaseLAC(lacid);

         RmapC0(gTCAUFAR,      X"10289" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         
         Print("-------------------------------------------------------------");
         Print("Transfer CLTU without authentication - TC 3 without pseudo --");
         Print("Set #80000000# ----------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '0';
         Synchronise(clk, 10 ns);

         mapid  := 1;
         cflag  := 0;
         sflags := 3;

         lacid := 0;

         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid,
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"80000000", 255),
               ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"800000", "TC HW GPIO COMMAND");
         Check(TP, TCGPIO, X"800000", "TC HW GPIO COMMAND");
--         IncreaseLAC(lacid);

         RmapC0(gTCAUFAR,      X"70189" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         
         Print("Clear #00# --------------------------------------------------");
         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, 
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"00000000", 0),
               ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");
         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");

         RmapC0(gTCAUFAR,      X"70189" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
         if lacid = 0 then
           RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         elsif lacid = 1 then
           RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         elsif lacid = 2 then
           RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end if;
         
--         wait for TxBitPeriod * 450;

         Print("-------------------------------------------------------------");
         Print("Set AU MAP to 0 ---------------------------------------------");
         Print("-------------------------------------------------------------");
         RmapW0(gTCAUCFG, X"00000080", "gTCAUCFG");
         
         Print("-------------------------------------------------------------");
         Print("Transfer CLTU to MAP ID 0 - TC 3 without pseudo -------------");
         Print("Set #00FFFF00# ----------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '0';
         Synchronise(clk, 10 ns);

         mapid  := 0;
         cflag  := 0;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    SourcePacket(
                      VersionId      => 0,
                      TypeId         => 1,
                      HeaderFlag     => 0,
                      ApplicationId  => apid,
                      SegmentFlags   => 3,
                      SequenceCount  => 0,
                      Data           => HWCommand(X"00FFFF00", 255),
                      ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"00FFFF", "TC HW GPIO COMMAND");
         Check(TP, TCGPIO, X"00FFFF", "TC HW GPIO COMMAND");
         IncreaseLAC(lacid);

         RmapC0(gTCAUFAR,      X"70289" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"2", TP, "gTCAUFAR");
         RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
         if lacid = 0 then
           RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         elsif lacid = 1 then
           RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         elsif lacid = 2 then
           RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end if;
         
         Print("Clear #00# --------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '0';
         Synchronise(clk, 10 ns);

         mapid  := 0;
         cflag  := 1;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"00000000", 0),
               ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");
         IncreaseLAC(lacid);

         RmapC0(gTCAUFAR,      X"70289" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"2", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         
         Print("-------------------------------------------------------------");
         Print("Transfer CLTU to MAP ID 1 - TC 3 without pseudo -------------");
         Print("Set #AAAAAA00# ----------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '0';
         Synchronise(clk, 10 ns);

         mapid  := 1;
         cflag  := 0;
         sflags := 3;

         lacid := 0;

         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid,
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"AAAAAA00", 255),
               ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"AAAAAA", "TC HW GPIO COMMAND");
         Check(TP, TCGPIO, X"AAAAAA", "TC HW GPIO COMMAND");
--         IncreaseLAC(lacid);

         RmapC0(gTCAUFAR,      X"70189" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         
         mapid  := 1;
         cflag  := 1;
         sflags := 3;

         Print("Clear #00# --------------------------------------------------");
         Print("Transfer CLTU");
         TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, 
                    SourcePacket(
               VersionId      => 0,
               TypeId         => 1,
               HeaderFlag     => 0,
               ApplicationId  => apid,
               SegmentFlags   => 3,
               SequenceCount  => 0,
               Data           => HWCommand(X"00000000", 0),
               ErrorControl   => 1)
                    )
                        )),
              TxData,TxCount,TxRequest(3),TxAck);

         wait for TxBitPeriod * 450;

         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");
         Check(TP, TCGPIO, X"000000", "TC HW GPIO COMMAND");

         RmapC0(gTCAUFAR,      X"70189" & '1' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"0", TP, "gTCAUFAR");
         RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
         if lacid = 0 then
           RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         elsif lacid = 1 then
           RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         elsif lacid = 2 then
           RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         end if;

         -- Loop over MAP IDs
         expio := (others => '0');
         for i in 0 to 31 loop
           case i is
             when 0 =>
               aumap := 0;
             when 1 =>
               aumap := 1;
             when 2 =>
               aumap := 15;
             when 3 =>
               aumap := 30;
             when others =>
               aumap := 31;
           end case;
           Print("-------------------------------------------------------------");
           if aumap < 10 then
             Print("Set AU MAP to " & integer'image(aumap) & " ---------------------------------------------");
           else
             Print("Set AU MAP to " & integer'image(aumap) & " ---------------------------------------------");
           end if;
           Print("-------------------------------------------------------------");
           RmapW0(gTCAUCFG, X"000000" & "100" & Conv_Std_Logic_Vector(aumap,5), "gTCAUCFG");

           -- Send segments to MAPs that are to be authenticated
--           for j in 0 to aumap loop
           for j in 0 to 4 loop
             if j > aumap then
               next;
             end if;
             case j is
               when 0 => mapid := 0;
               when 1 => mapid := 1;
               when 2 => mapid := aumap/2;
               when 3 => mapid := aumap-1;
               when others => mapid := aumap;
             end case;
             for k in 0 to 1 loop
               if mapid = 31 and k = 1 then
                 next;
               end if;
               Print("-------------------------------------------------------------");
               if 32*k+mapid < 10 then
                 Print("Transfer CLTU to authenticated map address " & integer'image(32*k+mapid) & " ----------------");
               else
                 Print("Transfer CLTU to authenticated map address " & integer'image(32*k+mapid) & " ---------------");
               end if;
               Print("-------------------------------------------------------------");
--               mapid  := j;
               cflag  := k;
               sflags := 3;
               lacid := mapid mod 3;
               Print("Transfer CLTU");
               TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
                              Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                                      SourcePacket(
                                        VersionId      => 0,
                                        TypeId         => 1,
                                        HeaderFlag     => 0,
                                        ApplicationId  => apid,
                                        SegmentFlags   => 3,
                                        SequenceCount  => 0,
                                        Data           => HWCommand(Conv_Std_Logic_Vector(j, 24) & X"00", 255*(1-k)),
-- Set on k=0, clear on k=1
                                        ErrorControl   => 1)
                                      )
                              )),
                    TxData,TxCount,TxRequest(1),TxAck);

               wait for TxBitPeriod * 450;
               if k=0 then
                 expio := expio or Conv_Std_Logic_Vector(j, 24);
               else
                 expio := expio and Conv_Std_Logic_Vector(j,24);
               end if;
               Check(TP, TCGPIO, expio, "TC HW GPIO COMMAND");
               IncreaseLAC(lacid);

               RmapC0(gTCAUFAR,      X"70288" & '1' &
                      conv_std_logic_vector(32*cflag+mapid, 6) &
                      '0' & X"2", TP, "gTCAUFAR");
               RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
               if lacid = 0 then
                 RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
               elsif lacid = 1 then
                 RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
               elsif lacid = 2 then
                 RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
               end if;
             end loop;
           end loop;
           
           -- Send segments to MAPs that are to be forwarded
--           for j in aumap+1 to 31 loop
           for j in 0 to 4 loop
             if j >= 31-aumap then
               next;
             end if;
             case j is
               when 0 => mapid := aumap+1;
               when 1 => mapid := aumap+2;
               when 2 => mapid := aumap+1+(31-aumap)/2;
               when 3 => mapid := 30;
               when others => mapid := 31;
             end case;
             for k in 0 to 1 loop
               if mapid = 31 and k = 1 then
                 next;
               end if;
               Print("-------------------------------------------------------------");
               if 32*k+mapid < 10 then
                 Print("Transfer CLTU to unauthenticated map address " & integer'image(32*k+mapid) & " --------------");
               else
                 Print("Transfer CLTU to unauthenticated map address " & integer'image(32*k+mapid) & " -------------");
               end if;
               Print("-------------------------------------------------------------");
--               mapid  := j;
               cflag  := k;
               sflags := 3;
               lacid := mapid mod 3;
               Print("Transfer CLTU");
               TxPW(CLTU(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
                              Segment(sflags, cflag, mapid,
                                      SourcePacket(
                                        VersionId      => 0,
                                        TypeId         => 1,
                                        HeaderFlag     => 0,
                                        ApplicationId  => apid,
                                        SegmentFlags   => 3,
                                        SequenceCount  => 0,
                                        Data           => HWCommand(Conv_Std_Logic_Vector(j, 24) & X"00", 255*(1-k)),
-- Set on k=0, clear on k=1
                                        ErrorControl   => 1)
                                      )
                              )),
                    TxData,TxCount,TxRequest(1),TxAck);

               wait for TxBitPeriod * 450;
               if k=0 then
                 expio := expio or Conv_Std_Logic_Vector(j, 24);
               else
                 expio := expio and Conv_Std_Logic_Vector(j, 24);
               end if;
               Check(TP, TCGPIO, expio, "TC HW GPIO COMMAND");

               RmapC0(gTCAUFAR,      X"70188" & '1' &
                      conv_std_logic_vector(32*cflag+mapid, 6) &
                      '0' & X"0", TP, "gTCAUFAR");
               RmapR0(gTCAUSR1,      d, "cTCAUSR1", false);
               if lacid = 0 then
                 RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
               elsif lacid = 1 then
                 RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
               elsif lacid = 2 then
                 RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
               end if;
             end loop;
           end loop;
         end loop;
           
         wait for TxBitPeriod * 450;

         tIntermediate(TP, TPCounter);
      end tTelecommandAuthenticationDisable;

      procedure tTelecommandAuthenticationDebug is
         variable OV:            Octet_Vector(0 to 255);
         variable fkey, pkey:    key_type;
         variable mapid:         natural range 0 to 31 := 0;
         variable sflags:        natural range 0 to 3  := 3;
         variable cflag:         natural range 0 to 1  := 0;
         variable lacid:         natural range 0 to 2  := 0;
         variable seed1, seed2:  positive;
         variable tmp:           integer;
         variable expdata:       Std_Logic_Vector(31 downto 0);
         variable wnum, windex:  integer;
         
         -----------------------------------------------------------------------
         -- Hardware Telecommand Instruction
         -----------------------------------------------------------------------
         function HWCommand(
            constant OutputNumber:  in Std_Logic_Vector;
            constant PulseLength:   in Natural)
            return                     Octet_Vector is
            constant OutputVector:     Std_Logic_Vector(0 to 31) := OutputNumber;
            constant PulseVector:      Std_Logic_Vector(0 to 7) :=
                                          Conv_Std_Logic_Vector(PulseLength, 8);
            variable Result:           Octet_Vector(0 to 4);
         begin
            Result(0)      := OutputVector( 0 to  7);
            Result(1)      := OutputVector( 8 to 15);
            Result(2)      := OutputVector(16 to 23);
            Result(3)      := OutputVector(24 to 31);
            Result(4)      := PulseVector;
            return Result;
         end function HWCommand;
      begin
         seed1 := 23;
         seed2 := 124;
         
         -- Set key to fixed key
         for i in 0 to 59 loop
           fkey.w(i) := fixedkey_c.weights(i);
           fkey.c(i) := fixedkey_c.coeffs(i);
         end loop;

         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("CCSDS Telecomand Decoder: hardware authentication debug");
         Print("--=========================================================--");
         Print("--=========================================================--");

         -- Enable AU for all MAP IDs
         Print("-------------------------------------------------------------");
         Print("Enable authentication ---------------------------------------");
         Print("-------------------------------------------------------------");
         RmapW0(gTCAUCFG, X"0000009F", "gTCAUCFG");

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Select fixed key ----------------------------");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    ccselectfixedkey)))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         IncreaseLAC(lacid);
         
         RmapC0(gTCAUFAR,      X"70188" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Copy fixed key to programmable key ----------");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    ccloadfixedkey)))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         IncreaseLAC(lacid);
         pkey := fkey;
         
         RmapC0(gTCAUFAR,      X"70188" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Change programmable key block A, offset 6 ---");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;

         for j in 0 to 6 loop
           gen_rand_int(255.0,seed1,seed2,tmp);
           OV(j) := Conv_Std_Logic_Vector(tmp,8);
         end loop;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey, ccchangeblocka(6,OV(0 to 6)))))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         changekey(Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey, ccchangeblocka(6,OV(0 to 6))), fkey, pkey);
         
         IncreaseLAC(lacid);

         RmapC0(gTCAUFAR,      X"70208" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Change programmable key block B, offset 4 ---");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;

         for j in 0 to 6 loop
           gen_rand_int(255.0,seed1,seed2,tmp);
           OV(j) := Conv_Std_Logic_Vector(tmp,8);
         end loop;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey, ccchangeblockb(4,OV(0 to 6)))))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         changekey(Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey, ccchangeblockb(4,OV(0 to 6))), fkey, pkey);
         
         IncreaseLAC(lacid);

         RmapC0(gTCAUFAR,      X"70208" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");

         Print("-------------------------------------------------------------");
         Print("Set Authentication unit in debug mode -----------------------");
         Print("-------------------------------------------------------------");
         -- Disable debug mode to start the scrubber
         RmapW0(gTCAUCFG, X"8000009F", "gTCAUCFG");

         Print("-------------------------------------------------------------");
         Print("Check contents of programmable key memory -------------------");
         Print("-------------------------------------------------------------");
         for i in 0 to 89 loop
           -- Populate expected data
           for j in 0 to 3 loop
             wnum   := (4*i+j) / 6;
             windex := (4*i+j) mod 6;
             expdata(j*8+7 downto j*8) := pkey.W(wnum)((5-windex)*8 to (5-windex)*8+7);
           end loop;
           RmapC0(TCAUDEBUG+4*i, expdata, TP, "Key memory");
         end loop;

         Print("-------------------------------------------------------------");
         Print("Disable Authentication unit debug mode ----------------------");
         Print("-------------------------------------------------------------");
         -- Enable debug mode to stop the scrubber
         RmapW0(gTCAUCFG, X"0000009F", "gTCAUCFG");

         wait for TxBitPeriod * 450;

         tIntermediate(TP, TPCounter);
         
      end tTelecommandAuthenticationDebug;

      procedure tTelecommandNoDebug is
         variable OV:            Octet_Vector(0 to 255);
         variable fkey, pkey:    key_type;
         variable mapid:         natural range 0 to 31 := 0;
         variable sflags:        natural range 0 to 3  := 3;
         variable cflag:         natural range 0 to 1  := 0;
         variable lacid:         natural range 0 to 2  := 0;
         variable seed1, seed2:  positive;
         variable tmp:           integer;
         variable expdata:       Std_Logic_Vector(31 downto 0);
         variable wnum, windex:  integer;
         
         -----------------------------------------------------------------------
         -- Hardware Telecommand Instruction
         -----------------------------------------------------------------------
         function HWCommand(
            constant OutputNumber:  in Std_Logic_Vector;
            constant PulseLength:   in Natural)
            return                     Octet_Vector is
            constant OutputVector:     Std_Logic_Vector(0 to 31) := OutputNumber;
            constant PulseVector:      Std_Logic_Vector(0 to 7) :=
                                          Conv_Std_Logic_Vector(PulseLength, 8);
            variable Result:           Octet_Vector(0 to 4);
         begin
            Result(0)      := OutputVector( 0 to  7);
            Result(1)      := OutputVector( 8 to 15);
            Result(2)      := OutputVector(16 to 23);
            Result(3)      := OutputVector(24 to 31);
            Result(4)      := PulseVector;
            return Result;
         end function HWCommand;
      begin
         -- The test infrastructure doesn't support reading from an address
         -- that results in an error response
         seed1 := 23;
         seed2 := 124;
         
         -- Set key to fixed key
         for i in 0 to 59 loop
           fkey.w(i) := fixedkey_c.weights(i);
           fkey.c(i) := fixedkey_c.coeffs(i);
         end loop;

         Print("--=========================================================--");
         Print("--=========================================================--");
         Print("CCSDS Telecomand Decoder: no hardware authentication debug");
         Print("--=========================================================--");
         Print("--=========================================================--");

         -- Enable AU for all MAP IDs
         Print("-------------------------------------------------------------");
         Print("Enable authentication ---------------------------------------");
         Print("-------------------------------------------------------------");
         RmapW0(gTCAUCFG, X"0000009F", "gTCAUCFG");

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Select fixed key ----------------------------");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    ccselectfixedkey)))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         IncreaseLAC(lacid);
         
         RmapC0(gTCAUFAR,      X"70188" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");

         Print("-------------------------------------------------------------");
         Print("Transfer CLTU - Copy fixed key to programmable key ----------");
         Print("-------------------------------------------------------------");

         Synchronise(clk, 10 ns);
         TCPseudo <= '1';
         Synchronise(clk, 10 ns);

         mapid  := 31;
         cflag  := 1;
         sflags := 3;

         Print("Transfer CLTU");
         TxPW(CLTU(PSR(TCTF(0, 1, 0, Conv_Integer(tcSCID), Conv_Integer(TCVcId), 0,
            Segment(sflags, cflag, mapid, lacid, lac(lacid), fkey,
                    ccloadfixedkey)))),
              TxData,TxCount,TxRequest(0),TxAck);

         wait for TxBitPeriod * 450;
         
         IncreaseLAC(lacid);
         pkey := fkey;
         
         RmapC0(gTCAUFAR,      X"70188" & '0' &
                conv_std_logic_vector(32*cflag+mapid, 6) &
                '0' & X"4", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,    "00" & Conv_Std_Logic_Vector(lac(0),30), TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,    "01" & Conv_Std_Logic_Vector(lac(1),30), TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,    X"000000" & Conv_Std_Logic_Vector(lac(2) mod 256,8), TP, "gTCAUSR3");
         
         Print("-------------------------------------------------------------");
         Print("Try to set Authentication unit in debug mode ----------------");
         Print("-------------------------------------------------------------");
         -- Enable debug mode to stop the scrubber
         RmapW0(gTCAUCFG, X"8000009F", "gTCAUCFG");

         Print("-------------------------------------------------------------");
         Print("Try to check contents of programmable key memory ------------");
         Print("-------------------------------------------------------------");
         for i in 0 to 89 loop
           RmapC0(TCAUDEBUG+4*i, X"00000000", TP, "Dummy key memory", true);
         end loop;

         Print("-------------------------------------------------------------");
         Print("Disable Authentication unit debug mode ----------------------");
         Print("-------------------------------------------------------------");
         -- Disable debug mode to start the scrubber
         RmapW0(gTCAUCFG, X"0000009F", "gTCAUCFG");

         wait for TxBitPeriod * 450;

         tIntermediate(TP, TPCounter);
         
      end tTelecommandNoDebug;

      --------------------------------------------------------------------------
      -- Test procedure
      --------------------------------------------------------------------------
      procedure tSetup is
      begin
         Print("--=========================================================--");
         Print("tSetup ------------------------------------------------------");
         Print("--=========================================================--");

         Print("-------------------------------------------------------------");
         Print("Soft reset ");
         Print("-------------------------------------------------------------");
--
--         RmapW0 (cTMDMACtrl, X"000000" & "000" &
--                             '0' &                       -- TX FRAME IRQ ENABLE
--                             '1' &                       -- RESET
--                             '0' &                       -- TX RESET
--                             '0' &                       -- TX IRQ ENABLE
--                             '0',                        -- TX ENABLE
--               "cTMDMACtrl");
--
--         RmapC0(cTMDMACtrl, X"000000" & "000" &
--                             '0' &                       -- TX FRAME IRQ
--                             '0' &                       -- RESET
--                             '1' &                       -- TX RESET
--                             '0' &                       -- TX IRQ ENABLE
--                             '0',                        -- TX ENABLE
--               TP,  "cTMDMACtrl   ");


         RmapW0(cTMPHY,  s_symbol_fall & s_symbol_rate(14 downto 0) &
                         s_sub_fall & s_sub_rate(14 downto 0),
                     "cTMPHY     ");

--         RmapW0(cTMCODE, X"000" &
--                           s_chipher &
--                           "00" &
--                           s_asm &
--                           s_reed & s_reed_depth & s_reed_e8 &
--                           s_turbo & s_turbo_rate &
--                           s_pseudo &
--                           s_mark &
--                           s_conv & s_conv_rate &
--                           s_split &
--                           s_sub,
--                     "cTMCODE    ");
--
--         RmapW0(cTMALL,  X"00" & "00" &
--                           s_fsh_length &
--                           s_insert &
--                           s_fecf &
--                           s_fhec &
--                           s_version &
--                           s_frame_length,
--                     "cTMALL     ");
--
--         RmapW0(cTMMST,  X"0000000" &
--                           s_mc &                        -- mc
--                           s_fsh &                       -- mc_fsh
--                           s_ocf &                       -- mc_ocf
--                           s_ocf_ow,                     -- ocf_ow
--                     "cTMMST     ");
--
--         RmapW0(cTMIDLE, X"00" & "00" &
--                           s_idle &                      -- idle
--                           s_ocf &                       -- ocf
--                           s_fsh_ext_vc_cntr &           -- fsh_ext_vc_cntr
--                           s_fsh &                       -- fsh
--                           s_vc_cntr_cycle &             -- vc_cntr_cycle
--                           s_idle_mc &                   -- mc
--                           s_idle_vcid&                  -- vcid
--                           s_idle_scid ,                 -- scid
--                     "cTMIDLE    ");
--
--         RmapW0(cTMFSH0, X"0F010203",                  -- fsh_sdu( 0 to 31)
--                     "cTMFSH0     ");
--         RmapW0(cTMFSH1, X"04050607",                  -- fsh_sdu(32 to 63)
--                     "cTMFSH1     ");
--         RmapW0(cTMFSH2, X"08091011",                  -- fsh_sdu(64 to 95)
--                     "cTMFSH2     ");
--         RmapW0(cTMFSH3, X"12131415",                  -- fsh_sdu(96 to 127)
--                     "cTMFSH3     ");
--
--         RmapW0(cTMOCF,  X"BAADCEDE",                  -- ocf_sdu( 0 to 31)
--                     "cTMOCF     ");
--
--         RmapW0(cTMDMALen, Conv_Std_Logic_Vector(limitsize-1, 16) &     -- LIMIT
--                           Conv_Std_Logic_Vector(framesize-1, 16),      -- LEN
--                     "cTMDMALen  ");
--
--         Print("Un-reset telemetry encoder transmitter ----------------------");
         RmapW0 (cTMDMACtrl, X"000000" & "000" &
                             '1' &                       -- TX FRAME IRQ ENABLE
                             '0' &                       -- RESET
                             '0' &                       -- TX RESET
                             '1' &                       -- TX IRQ ENABLE
                             '0',                        -- TX ENABLE
               "cTMDMACtrl");

--         RmapC0(cTMDMACtrl, X"000000" & "000" &
--                             '0' &                       -- TX FRAME IRQ
--                             '0' &                       -- RESET
--                             '0' &                       -- TX RESET
--                             '0' &                       -- TX IRQ ENABLE
--                             '0',                        -- TX ENABLE
--               TP,  "cTMDMACtrl   ");

         RmapW0(cTMDMAStat,    X"FFFFFFFF", "cTMDMAStat");

         Print("Wait for TX part to become un-reset -------------------------");
         d := (others => '1');
         while d(7) /= '0' loop
            RmapR0(cTMDMAStat, d, "cTMDMAStat  ", False);
         end loop;
--
--         Print("Enable telemetry encoder transmitter ------------------------");
--         RmapW0(cTMCtrl, X"0000000" & "000" &
--                           '1',                          -- enable
--                     "cTMCtrl     ");

--         Print("-------------------------------------------------------------");
--         Print("Setup descriptor register hardware channels");
--         RmapW0(cTMDMAVCPtr,
--                     Conv_Std_Logic_Vector(GRTMDESC_HADDR, 12) & Conv_Std_Logic_Vector(0, 20),
--                     "cTMDMAPtr   ");

         Print("Enable telemetry encoder hardware channels ------------------");
         RmapW0(cAHBRAMCFG16k,X"00000080", "cAHBRAMCFG16k");
         RmapW0(cTMDMAExtCtrl, X"0000000" & "000" &
                           '1',                          -- enable
                     "cTMDMAExtCtrl     ");

         wait for 100 us;
         vc3vc6on <= '1';

         tIntermediate(TP, TPCounter);
      end tSetup;

      --------------------------------------------------------------------------
      -- Test procedure
      --------------------------------------------------------------------------
      procedure tClcw is
      begin
         Print("--=========================================================--");
         Print("tCLCW");
         Print("--=========================================================--");
         Synchronise(clk);
         Synchronise(clk);
         clcwen   <= '0';
         Synchronise(clk);
         clcwin   <= "11";
         Synchronise(clk);
         Synchronise(clk);
         Print("Send CLCW over UART -----------------------------------------");
         Tx232(clk, clcwin(0), X"01", cBaud, False, False, False, False, "", False, True, 10 ns);
         Tx232(clk, clcwin(0), X"23", cBaud, False, False, False, False, "", False, True, 10 ns);
         Tx232(clk, clcwin(0), X"45", cBaud, False, False, False, False, "", False, True, 10 ns);
         Tx232(clk, clcwin(0), X"67", cBaud, False, False, False, False, "", False, True, 10 ns);
         Tx232(clk, clcwin(0), X"00", cBaud, False, False, False, True, "", False, True, 10 ns);

         Tx232(clk, clcwin(1), X"89", cBaud, False, False, False, False, "", False, True, 10 ns);
         Tx232(clk, clcwin(1), X"ab", cBaud, False, False, False, False, "", False, True, 10 ns);
         Tx232(clk, clcwin(1), X"cd", cBaud, False, False, False, False, "", False, True, 10 ns);
         Tx232(clk, clcwin(1), X"ef", cBaud, False, False, False, False, "", False, True, 10 ns);
         Tx232(clk, clcwin(1), X"00", cBaud, False, False, False, True, "", False, True, 10 ns);

         clcwen   <= '1' after 1 ms;
         clcwin   <= "ZZ" after 1 ms;
      end tClcw;

      --------------------------------------------------------------------------
      -- Test procedure
      --------------------------------------------------------------------------
      procedure tSimSettings is

      begin
         Print("--=========================================================--");
         Print("tSimSettings ------------------------------------------------");
         Print("--=========================================================--");
         Print("Simulation settings");
         Print("-------------------------------------------------------------");
         Write(L, Now, Right, 15);
         Write(L, String'(" : System period [ns]:   "));
         Write(L, sBoardPeriod);
         WriteLine(Output, L);
         Write(L, Now, Right, 15);
         Write(L, String'(" : Symbol period [ns]:   "));
         Write(L, sTMBitPeriod);
         WriteLine(Output, L);
         Print("-------------------------------------------------------------");
         Write(L, Now, Right, 15);
         Write(L, String'(" : Memory size   [Byte]:  "));
         Write(L, memorysize*1024);
         WriteLine(Output, L);
         Write(L, Now, Right, 15);
         Write(L, String'(" : Limit size    [Byte]:  "));
         Write(L, limitsize);
         WriteLine(Output, L);
         Write(L, Now, Right, 15);
         Write(L, String'(" : Frame size    [Byte]:  "));
         Write(L, framesize);
         WriteLine(Output, L);
         Print("-------------------------------------------------------------");
         if    packet=1 and packetcrc=1 then
            Write(L, Now, Right, 15);
            Write(L, String'(" : CCSDS Space Packet generation with CRC: frame length = "));
            Write(L, framesize);
            WriteLine(Output, L);
         elsif packet=1 and packetcrc=0 then
            Write(L, Now, Right, 15);
            Write(L, String'(" : CCSDS Space Packet generation: frame length = "));
            Write(L, framesize);
            WriteLine(Output, L);
         end if;
         Print("--=========================================================--");
      end tSimSettings;

      --------------------------------------------------------------------------
      -- Test procedure
      --------------------------------------------------------------------------
      procedure tRegisters is
      begin
         Print("--=========================================================--");
         Print("tRegisters");
         Print("--=========================================================--");
         Print("-------------------------------------------------------------");
         Print("On-chip memory");
         Print("-------------------------------------------------------------");
         RmapW0(cAHBRAMCFG4k, X"00000080", "cAHBRAMCFG4k");
         RmapW0(cAHBRAMCFG16k,X"00000080", "cAHBRAMCFG16k");

         RmapW0(cAHBSTAT,      X"00000000", "cAHBSTAT");
         RmapW0(cAHBADDR,      X"00000000", "cAHBADDR");

         RmapW0(AHBRAM4k,  X"12345678", "AHBRAM4k");
         RmapC0(AHBRAM4k,  X"12345678", TP, "AHBRAM4k");
         RmapC0(cAHBSTAT,      X"0000003A", TP, "cAHBSTAT");
         RmapC0(cAHBADDR,      cAHBADDR, TP, "cAHBADDR");

         RmapW0(AHBRAM16k, X"9abcdef0", "AHBRAM16k");
         RmapC0(AHBRAM16k, X"9abcdef0", TP, "AHBRAM16k");
         RmapC0(cAHBSTAT,      X"0000003A", TP, "cAHBSTAT");
         RmapC0(cAHBADDR,      cAHBADDR, TP, "cAHBADDR");

         Print("-------------------------------------------------------------");
         Print("Interrupt test");
         Print("-------------------------------------------------------------");
         RmapW0(cIRQ_LEVEL, X"00000000", "cIRQ_LEVEL");
         RmapW0(cIRQ_PENDING, X"00000000", "cIRQ_PENDING");
         RmapW0(cIRQ_FORCE, X"00000000", "cIRQ_FORCE");
         RmapW0(cIRQ_CLEAR, X"00000000", "cIRQ_CLEAR");
         RmapW0(cIRQ_MSTAT, X"00000000", "cIRQ_MSTAT");
         RmapW0(cIRQ_BCAST, X"00000000", "cIRQ_BCAST");
         RmapW0(cIRQ_PMASK, X"00000000", "cIRQ_PMASK");
         RmapW0(cIRQ_PFORCE, X"00000000", "cIRQ_PFORCE");
         RmapW0(cIRQ_EXTACK, X"00000000", "cIRQ_EXTACK");

         RmapC0(cIRQ_LEVEL,   X"00000000", TP, "cIRQ_LEVEL");
         RmapC0(cIRQ_PENDING, X"00000000", TP, "cIRQ_PENDING");
         RmapC0(cIRQ_FORCE,   X"00000000", TP, "cIRQ_FORCE");
         RmapC0(cIRQ_CLEAR,   X"00000000", TP, "cIRQ_CLEAR");
         RmapC0(cIRQ_MSTAT,   X"00000000", TP, "cIRQ_MSTAT");
         RmapC0(cIRQ_BCAST,   X"00000000", TP, "cIRQ_BCAST");
         RmapC0(cIRQ_PMASK,   X"00000000", TP, "cIRQ_PMASK");
         RmapC0(cIRQ_PFORCE,  X"00000000", TP, "cIRQ_PFORCE");
         RmapC0(cIRQ_EXTACK,  X"00000000", TP, "cIRQ_EXTACK");

         Check(TP, IRQ, '0', "IRQ incorrectly asserted");

         RmapW0(cIRQ_PMASK, X"00000006", "cIRQ_PMASK");
         RmapC0(cIRQ_PMASK, X"00000006", TP, "cIRQ_PMASK");

         RmapW0(cIRQ_FORCE, X"00000002", "cIRQ_FORCE");
         RmapC0(cIRQ_FORCE, X"00000002", TP, "cIRQ_FORCE");

         RmapC0(cIRQ_PENDING, X"00000000", TP, "cIRQ_PENDING");
         RmapC0(cIRQ_LEVEL,   X"00000000", TP, "cIRQ_LEVEL");

         Check(TP, IRQ, '1', "IRQ incorrectly deasserted");

         RmapW0(cIRQ_FORCE, X"00000000", "cIRQ_FORCE");
         RmapC0(cIRQ_FORCE, X"00000000", TP, "cIRQ_FORCE");

         RmapC0(cIRQ_PENDING, X"00000000", TP, "cIRQ_PENDING");
         RmapC0(cIRQ_LEVEL,   X"00000000", TP, "cIRQ_LEVEL");

         Check(TP, IRQ, '0', "IRQ incorrectly asserted");


         RmapW0(cIRQ_PENDING, X"00000004", "cIRQ_PENDING");
         RmapC0(cIRQ_LEVEL,   X"00000000", TP, "cIRQ_LEVEL");

         Check(TP, IRQ, '1', "IRQ incorrectly deasserted");

         RmapW0(cIRQ_PENDING, X"00000000", "cIRQ_PENDING");
         RmapC0(cIRQ_LEVEL,   X"00000000", TP, "cIRQ_LEVEL");

         Check(TP, IRQ, '0', "IRQ incorrectly asserted");
         
         -- GPIO Interrupt
         Print("-- GPIO Interrupt test");
         
         RmapW0(cGPIO_OUT,     X"00000040", "cGPIO_OUT");
         RmapW0(cGPIO_DIR,     X"00000050", "cGPIO_DIR");
         RmapW0(cGPIO_INTREDG, X"00000000", "cGPIO_INTREDG");
         RmapW0(cGPIO_INTRPOL, X"000000FD", "cGPIO_INTRPOL");
         
         RmapW0(cGPIO_INTRMSK, X"00000002", "cGPIO_INTRMSK");     
         gpio(1) <= '0';  -- MBE on gpio1
         RmapC0(cIRQ_PENDING, X"00000002", TP, "cIRQ_PENDING");
         gpio(1) <= '1';  -- MBE on gpio1         
         Check(TP, IRQ, '1', "IRQ incorrectly deasserted");
                  
         RmapW0(cGPIO_INTRMSK, X"00000000", "cGPIO_INTRMSK");
         
         RmapW0(cIRQ_PENDING, X"00000000", "cIRQ_PENDING");
         RmapC0(cIRQ_LEVEL,   X"00000000", TP, "cIRQ_LEVEL");
         -- GPIO End
         
         RmapW0(cIRQ_PENDING, X"00000000", "cIRQ_PENDING");
         RmapC0(cIRQ_LEVEL,   X"00000000", TP, "cIRQ_LEVEL");

         Check(TP, IRQ, '0', "IRQ incorrectly asserted");

         RmapW0(cIRQ_PMASK, X"FFFFFFFF", "cIRQ_PMASK");

         Print("-------------------------------------------------------------");
         Print("Register accesses");
         Print("-------------------------------------------------------------");
         
         RmapW0(cGPIO_OUT,     X"00000040", "cGPIO_OUT");
         RmapW0(cGPIO_DIR,     X"00000050", "cGPIO_DIR");
         RmapW0(cGPIO_INTREDG, X"00000000", "cGPIO_INTREDG");
         RmapW0(cGPIO_INTRPOL, X"000000FF", "cGPIO_INTRPOL");
         RmapC0(cGPIO_OUT,     X"00000040", TP, "cGPIO_OUT");
         RmapC0(cGPIO_DIR,     X"00000050", TP, "cGPIO_DIR");
         RmapC0(cGPIO_INTREDG, X"00000000", TP, "cGPIO_INTREDG");
         RmapC0(cGPIO_INTRPOL, X"00000002", TP, "cGPIO_INTRPOL"); -- only IRQ 1 enabled
         
         RmapW0(cMCFG1,        X"00000000", "cMCFG1");
         RmapW0(cMCFG2,        X"00000000", "cMCFG2");
         RmapW0(cMCFG3,        X"00000000", "cMCFG3");
         RmapC0(cMCFG1,        X"00000000", TP, "cMCFG1");
         RmapC0(cMCFG2,        X"00000000", TP, "cMCFG2");
         RmapC0(cMCFG3,        X"00000000", TP, "cMCFG3");

         RmapW0(cAHBSTAT,      X"00000000", "cAHBSTAT");
         RmapW0(cAHBADDR,      X"00000000", "cAHBADDR");
         RmapC0(cAHBSTAT,      X"0000003A", TP, "cAHBSTAT");
         RmapC0(cAHBADDR,      cAHBADDR, TP, "cAHBADDR");

         RmapW0(cAHBRAMCFG4k,  X"00000000", "cAHBRAMCFG4k");
         RmapW0(cAHBRAMCFG16k, X"00000000", "cAHBRAMCFG16k");
         RmapC0(cAHBRAMCFG4k,  X"01000800", TP, "cAHBRAMCFG4k");  -- EDACEN changed
         RmapC0(cAHBRAMCFG16k, X"01001000", TP, "cAHBRAMCFG16k"); -- EDACEN changed

         RmapR0(cAHBUARTSTAT,  d, "cAHBUARTSTAT");
         RmapR0(cAHBUARTCTRL,  d, "cAHBUARTCTRL");
         RmapR0(cAHBUARTSCALE, d, "cAHBUARTSCALE:");

         RmapW0(cTMDMACtrl,    X"00000000", "cTMDMACtrl");
         RmapW0(cTMDMAPtr,     X"00000000", "cTMDMAPtr");
         RmapW0(cTMDMAStat,    X"FFFFFFFF", "cTMDMAStat");

         RmapR0(cTMDMACtrl,    d, "cTMDMACtrl");
         RmapR0(cTMDMAStat,    d, "cTMDMAStat");
         RmapC0(cTMDMALen,     X"045A045A", TP, "cTMDMALen");
         RmapR0(cTMDMAPtr,     d, "cTMDMAPtr");
         RmapC0(cTMDMAConf,    X"08000040", TP, "cTMDMAConf");
         RmapC0(cTMDMARev,     X"000E0004", TP, "cTMDMARev"); -- Latest
                                                              -- revision (=4)
         RmapR0(cTMDMAExtCtrl, d, "cTMDMAExtCtrl:");
         RmapR0(cTMDMAExtPtr,  d, "cTMDMAExtPtr");
         RmapR0(cTMCtrl,       d, "cTMCtrl");
         RmapR0(cTMStat,       d, "cTMStat");
         RmapC0(cTMConf,       X"0042931C", TP, "cTMConf");
         RmapR0(cTMSize,       d, "cTMSize");
         RmapR0(cTMPHY,        d, "cTMPHY");
         RmapR0(cTMCODE,       d, "cTMCODE");
         RmapR0(cTMASM,        d, "cTMASM");
         RmapR0(cTMALL,        d, "cTMALL");
         RmapR0(cTMMST,        d, "cTMMST");
         RmapC0(cTMIDLE,       X"0030FD02", TP, "cTMIDLE");
         RmapR0(cTMVCID,       d, "cTMVCID");

         RmapC0(gTCSIR,        X"00000102", TP, "gTCSIR");
         RmapC0(gTCFAR,        X"00003800", TP, "gTCFAR");
         RmapC0(gTCCLCWR1,     X"0000C000", TP, "gTCCLCWR1");
         RmapC0(gTCCLCWR2,     X"0000C000", TP, "gTCCLCWR2");
         RmapC0(gTCPHIR,       X"00000000", TP, "gTCPHIR");
         
         RmapC0(gTCAUFAR,      X"00007FE0", TP, "gTCAUFAR");
         RmapC0(gTCAUFAR,      X"80007FE0", TP, "gTCAUFAR");
         RmapC0(gTCAUSR1,      X"3FFFFFFF", TP, "gTCAUSR1");
         RmapC0(gTCAUSR2,      X"7FFFFFFF", TP, "gTCAUSR2");
         RmapC0(gTCAUSR3,      X"00000000", TP, "gTCAUSR3");
         RmapC0(gTCAUCFG,      X"0000001F", TP, "gTCAUCFG");

         tIntermediate(TP, TPCounter);
      end tRegisters;

      --------------------------------------------------------------------------
      -- Test procedure
      --------------------------------------------------------------------------
      procedure tDSU is
      begin
         Print("-------------------------------------------------------------");
         Print("DSU:");
         Print("-------------------------------------------------------------");

         Print("-------------------------------------------------------------");
         Print("DSU: Synchronize");
         Print("-------------------------------------------------------------");
         Tx232(clk, dsurx, X"55", 115200, False, False, False, False, "55",    False, True, 10 ns);
         Tx232(clk, dsurx, X"55", 115200, False, False, False, False, "55",    False, True, 10 ns);

         Print("-------------------------------------------------------------");
         Print("DSU: read TM configuration");
         Print("-------------------------------------------------------------");
         Tx232(clk, dsurx, X"80", 115200, False, False, False, False, "80",    False, True, 10 ns);
         Tx232(clk, dsurx, cTMConf(0 to 7), 115200, False, False, False, False, "",    False, True, 10 ns);
         Tx232(clk, dsurx, cTMConf(8 to 15), 115200, False, False, False, False, "",    False, True, 10 ns);
         Tx232(clk, dsurx, cTMConf(16 to 23), 115200, False, False, False, False, "",    False, True, 10 ns);
         Tx232(clk, dsurx, cTMConf(24 to 31), 115200, False, False, False, False, "",    False, True, 10 ns);

         Print("Receive debug output");
         for i in 0 to 4-1 loop
            Rx232r(TP, UARTReady, UARTAck, UARTData, OV(i));
         end loop;
         RmapR0(cTMConf, d, "cTMConf");

         tIntermediate(TP, TPCounter);
      end tDSU;

      --------------------------------------------------------------------------
      -- Telemetry test procedure
      --------------------------------------------------------------------------
      procedure tTransmitDMA (
                  HRESP1: Std_Logic_Vector(1  downto 0) := HRESP_OKAY;
                  HRESP2: Std_Logic_Vector(1  downto 0) := HRESP_OKAY) is
         variable loops:            Integer;
         variable descs:            Integer;
         variable framesizealigned:  Integer;
         variable NOV:              Octet_Vector(0 to -1);

         variable FSHV:             Octet_Vector(0 to 15);
         variable OCFV:             Octet_Vector(0 to 3);

         variable DV:               Data_Vector(0 to 511);
         variable add1:             Integer := 0;

         variable ZEROV:            Octet_Vector(0 to 255) := (others => X"00");

      begin
         Print("--=========================================================--");
         Print("tTransmitDMA ------------------------------------------------");
         Print("--=========================================================--");

         Print("-------------------------------------------------------------");
         Print("Setup memory");
         descs := (MEMSIZE / ((framesize/4) +1)) / 4;
         loops := framesize/4;

         Print("Setup number of frames");
         Write(L, Now, Right, 15);
         Write(L, String'(" : Descriptors:      "));
         Write(L, descs);
         WriteLine(Output, L);
         Write(L, Now, Right, 15);
         Write(L, String'(" : MEMSIZE:      "));
         Write(L, MEMSIZE);
         WriteLine(Output, L);

         if framesize > loops*4 then
            loops := loops +1;
         end if;

         -- calculate alignment
         framesizealigned      := framesize/4;
         framesizealigned      := framesizealigned * 4;
         if framesize > framesizealigned then
            framesizealigned   := framesizealigned + 4;    -- add word
            add1 := 1;
         end if;

         FSHV := (X"0F", X"10", X"11", X"12", X"13", X"14", X"15", X"16", X"17", X"18", X"19", X"1A", X"1B", X"1C", X"1D", X"1E");
         OCFV := (X"FE", X"ED", X"BA", X"CC");

         for j in 0 to descs -1  loop
            if s_fsh_ext_vc_cntr='1' and s_version="00" then
            DV(0 to framesize/4-1+add1) := Conv_Data_Vector(
               TMTF(
                  SpacecraftID   => Conv_Integer(stcscid),
                  VCID           => 1,
                  OCFFlag        => Conv_Integer(X"0" & s_ocf),
                  MCFrameCntr    => MC,
                  VCFrameCntr    => VC,
                  ExtVCFrameCntr => EVC,
                  SyncFlag       => 0,
                  PktOrderFlag   => 0,
                  SegmentID      => 3,
                  FHP            => 000,
                  DataField      => SourcePacket(0, 0, 0, 1, 3, PC, framesize-6-Conv_Integer(X"0" & s_fecf)*2-Conv_Integer(X"0" & s_ocf)*4-4, 0, packetcrc),
                  OCF            => OCFV(0 to Conv_Integer(X"0" & s_ocf)*4-1),
                  FECF           => Conv_Integer(X"0" & s_fecf)));
            elsif s_version="00" then
            DV(0 to framesize/4-1+add1) := Conv_Data_Vector(
               TMTF(
                  SpacecraftID   => Conv_Integer(stcscid),
                  VCID           => 1,
                  OCFFlag        => Conv_Integer(X"0" & s_ocf),
                  MCFrameCntr    => MC,
                  VCFrameCntr    => VC,
                  FSHFlag        => Conv_Integer(X"0" & s_fsh),
                  SyncFlag       => 0,
                  PktOrderFlag   => 0,
                  SegmentID      => 3,
                  FHP            => 000,
                  FSH            => FSHV(0 to Conv_Integer(s_fsh_length)-1),
                  DataField      => SourcePacket(0, 0, 0, 1, 3, PC, framesize-6-Conv_Integer(X"0" & s_fecf)*2-Conv_Integer(X"0" & s_ocf)*4-Conv_Integer(s_fsh_length), 0, packetcrc),
                  OCF            => OCFV(0 to Conv_Integer(X"0" & s_ocf)*4-1),
                  FECF           => Conv_Integer(X"0" & s_fecf)));
            elsif s_version="01" then
               DV(0 to framesize/4-1+add1) := Conv_Data_Vector(
                  AOSTF(
                     SpacecraftID   => Conv_Integer(stcscid),
                     VCID           => 1,
                     VCFrameCntr    => VC,
                     VCFrameCycle   => VC/(2**24),
                     ReplayFlag     => 0,
                     VCUsageFlag    => Conv_Integer(X"0" & s_vc_cntr_cycle),
                     SpareFlag      => 0,
                     FHEC           => Conv_Integer(X"0" & s_fhec),
                     InsertZone     => FSHV(0 to Conv_Integer(s_fsh_length)-1),
                     FHPSpare       => 0,
                     FHP            => 000,
                     DataField      => SourcePacket(0, 0, 0, 1, 3, PC, framesize-6-2-Conv_Integer(X"0" & s_fhec)*2-Conv_Integer(X"0" & s_fecf)*2-Conv_Integer(X"0" & s_ocf)*4-Conv_Integer(s_fsh_length), 0, packetcrc),
                     OCF            => OCFV(0 to Conv_Integer(X"0" & s_ocf)*4-1),
                     FECF           => Conv_Integer(X"0" & s_fecf)));
            end if;

            if s_version="00" then
               VC := (VC+1) mod 256;
            else
               VC := (VC+1) mod 2**24;
            end if;
            PC := (PC+1) mod 16383;
            MC := (MC+1) mod 256;
            ZEROV := (others => X"00");

            if screen then
               Write(L, Now, Right, 15);
               Write(L, String'(" : Frame start address: "));
               Write(L, j, Right, 5);
               Write(L, String'(" :  "));
               HWrite(L, RAM +
                         Conv_Std_Logic_Vector((framesizealigned)*j, 20));
               WriteLine(Output, L);
            end if;

--            for i in 0 to loops-1 loop
--               RmapW0(RAM + Conv_Std_Logic_Vector((framesizealigned)*j + i*4, 20),
--                      DV(i),
--                      "RAM", False);
--            end loop;

            Print("Write frame in memmory");
            RmapWrite0(
               Address        => RAM + Conv_Std_Logic_Vector(framesizealigned*j, 20),
               Data           => DV(0 to loops-1),
               ScreenOutput   => False);
         end loop;

         Print("-------------------------------------------------------------");
         Print("Setup descriptor table");

--         Print("Setup number of loops for descriptor table");
--         Write(L, Now, Right, 15);
--         Write(L, String'(" : framesize, bytes:  "));
--         Write(L, framesize);
--         WriteLine(Output, L);
--         Write(L, Now, Right, 15);
--         Write(L, String'(" : descriptors:      "));
--         Write(L, descs);
--         WriteLine(Output, L);

         for i in 0 to descs -1-1-1  loop
            RmapW0(AHBRAM4k +
                     Conv_Std_Logic_Vector(i*8+4, 20),
                     RAM +
                     Conv_Std_Logic_Vector((framesizealigned)*i, 20),
                     "AHBRAM4k", False);
            RmapW0(AHBRAM4k +
                     Conv_Std_Logic_Vector(i*8, 20),
                     X"0000" & '0' &
                     '0' &                               -- time_strobe
                     "0000" &                            --
                     '0' &                               -- vc_cntr_enable
                     '0' &                               -- mc_cntr_bypass
                     '0' &                               -- mc_fsh_bypass
                     '0' &                               -- mc_ocf_bypass
                     '0' &                               -- fhec_bypass
                     '0' &                               -- insert_bypass
                     '0' &                               -- fecf_bypass
                     '0' &                               -- irq enable
                     '0' &                               -- wrap
                     '1',                                -- enable
                     "AHBRAM4k");
         end loop;

         for i in descs-1-1 to descs-1-1  loop
            RmapW0(AHBRAM4k +
                     Conv_Std_Logic_Vector(i*8+4, 20),
                     RAM +
                     Conv_Std_Logic_Vector((framesizealigned)*i, 20),
                     "AHBRAM4k", False);
            RmapW0(AHBRAM4k +
                     Conv_Std_Logic_Vector(i*8, 20),
                     X"0000" & '0' &
                     '0' &                               -- time_strobe
                     "0000" &                            --
                     '0' &                               -- vc_cntr_enable
                     '0' &                               -- mc_cntr_bypass
                     '0' &                               -- mc_fsh_bypass
                     '0' &                               -- mc_ocf_bypass
                     '0' &                               -- fhec_bypass
                     '0' &                               -- insert_bypass
                     '0' &                               -- fecf_bypass
                     '0' &                               -- irq enable
                     '0' &                               -- wrap
                     '1',                                -- enable
                     "AHBRAM4k");
         end loop;

         for i in descs-1 to descs-1  loop
            RmapW0(AHBRAM4k +
                     Conv_Std_Logic_Vector(i*8+4, 20),
                     RAM +
                     Conv_Std_Logic_Vector((framesizealigned)*i, 20),
                     "AHBRAM4k", False);
            RmapW0(AHBRAM4k +
                     Conv_Std_Logic_Vector(i*8, 20),
                     X"0000" & '0' &
                     '1' &                               -- time_strobe
                     "0000" &                            --
                     '0' &                               -- vc_cntr_enable
                     '0' &                               -- mc_cntr_bypass
                     '0' &                               -- mc_fsh_bypass
                     '0' &                               -- mc_ocf_bypass
                     '0' &                               -- fhec_bypass
                     '0' &                               -- insert_bypass
                     '0' &                               -- fecf_bypass
                     '0' &                               -- irq enable
                     '1' &                               -- wrap
                     '1',                                -- enable
                     "AHBRAM4k");
         end loop;

         if screen then
            Print("----------------------------------------------------------");
            Print("Read descriptor table");

            for i in 0 to descs -1   loop
               RmapR0(AHBRAM4k +
                        Conv_Std_Logic_Vector(i*8+4, 20),
                        d, "AHBRAM4k", True);
               RmapR0(AHBRAM4k +
                        Conv_Std_Logic_Vector(i*8, 20),
                        d, "AHBRAM4k", True);
            end loop;
         end if;

         Print("-------------------------------------------------------------");
         Print("Setup descriptor register");
         RmapW0(cTMDMAPtr,
                     AHBRAM4k + Conv_Std_Logic_Vector(0, 20),
                     "cTMDMAPtr   ");

         Print("-------------------------------------------------------------");
         Print("Wait for TX part to become ready");
         d := (others => '0');
         while d(6) /= '1' loop
            RmapR0(cTMDMAStat, d, "cTMDMAStat  ", False);
         end loop;

         Print("Start descriptor --------------------------------------------");
         RmapW0(cTMDMACtrl, X"000000" & "000" &
                             '1' &                       -- TX FRAME IRQ ENABLE
                             '0' &                       -- RESET
                             '0' &                       -- TX RESET
                             '1' &                       -- TX IRQ ENABLE
                             '1',                        -- TX ENABLE
                     "cTMDMACtrl  ");

         RmapC0(cTMDMAStat, X"000000" &
                             '0' &                       -- TX RESET STATUS
                             '1' &                       -- TX READY STATUS
                             '-' &                       -- TX FRAME ONGOING
                             '-' &                       -- TX FRAME SENT
                             '0' &                       -- TX FAILURE
                             '0' &                       -- TX AHB ERROR
                             '-' &                       -- TX IRQ
                             '0',                        -- TX ERROR
               TP, "cTMDMAStat  ");


         if Conv_Integer(s_symbol_rate) > 8 and (s_sub='0') then
            Print("----------------------------------------------------------");
            Print("Change baud rate - up");
            RmapW0(cTMPHY,  s_symbol_fall & '0' & s_symbol_rate(14 downto 1) &
                              s_sub_fall & '0' & s_sub_rate(14 downto 1),
                        "cTMPHY     ");

            wait for 400 us;

            Print("Change baud rate - down");
            RmapW0(cTMPHY,  s_symbol_fall & s_symbol_rate(13 downto 0) & '0' &
                              s_sub_fall & s_sub_rate(13 downto 0) & '0',
                        "cTMPHY     ");

            wait for 400 us;

            Print("Change baud rate - back");
            RmapW0(cTMPHY,  s_symbol_fall & s_symbol_rate(14 downto 0)  &
                              s_sub_fall & s_sub_rate(14 downto 0),
                        "cTMPHY     ");
         end if;

         Print("-------------------------------------------------------------");
         Print("Wait for descriptor to be completed");
         d := (others => '1');
         while d(0) /= '0' loop
            RmapR0(cTMDMACtrl, d, "cTMDMACtrl  ", False);
            wait for 2 ms;
         end loop;

         Print("-------------------------------------------------------------");
         Print("Wait for frame to be completed");
         d := (others => '1');
         while d(5) /= '0' loop
            RmapR0(cTMDMAStat, d, "cTMDMAStat  ", False);
            wait for 2 ms;
         end loop;

         if False then
            Print("----------------------------------------------------------");
            Print("Read descriptor table");

            for i in 0 to descs -1   loop
               RmapR0(AHBRAM4k +
                        Conv_Std_Logic_Vector(i*8+4, 20),
                        d, "AHBRAM4k", True);
               RmapR0(AHBRAM4k +
                        Conv_Std_Logic_Vector(i*8, 20),
                        d, "AHBRAM4k", True);
            end loop;
         end if;

         Print("Check status ");

         RmapC0(cTMDMAStat, X"000000" &
                             '0' &                       -- TX RESET STATUS
                             '1' &                       -- TX READY STATUS
                             '0' &                       -- TX FRAME ONGOING
                             '1' &                       -- TX FRAME SENT
                             '0' &                       -- TX FAILURE
                             '0' &                       -- TX AHB ERROR
                             '1' &                       -- TX IRQ
                             '0',                        -- TX ERROR
               TP, "cTMDMAStat  ");
         RmapW0(cTMDMAStat, X"000000" &
                             '0' &                       -- TX RESET STATUS
                             '0' &                       -- TX READY STATUS
                             '1' &                       -- TX FRAME ONGOING
                             '1' &                       -- TX FRAME SENT
                             '1' &                       -- TX FAILURE
                             '1' &                       -- TX AHB ERROR
                             '1' &                       -- TX IRQ
                             '1',                        -- TX ERROR
                  "cTMDMAStat  ");
         RmapC0(cTMDMAStat, X"000000" &
                             '0' &                       -- TX RESET STATUS
                             '1' &                       -- TX READY STATUS
                             '0' &                       -- TX FRAME ONGOING
                             '-' &                       -- TX FRAME SENT
                             '0' &                       -- TX FAILURE
                             '0' &                       -- TX AHB ERROR
                             '0' &                       -- TX IRQ
                             '0',                        -- TX ERROR
               TP, "cTMDMAStat  ");

         tIntermediate(TP, TPCounter);
      end tTransmitDMA;

   begin
      tInitialise(TP, TPCounter);
      tReset;
      tRegisters;
      tMRAM;
      tSetup;
      tClcw;
      tDSU;
      tTelecommand;
      tTelecommandHardware;
      tTelecommandAuthentication;
      tTelecommandAuthenticationDisable;
      if audebug /= 0 then
        tTelecommandAuthenticationDebug;
--      else
--        The test infrastructure doesn't support reading an area that
--        results in an error response
--        tTelecommandNoDebug;
      end if;

      for i in 0 to 0 loop
         tTransmitDMA(HRESP_OKAY, HRESP_OKAY);
      end loop;

      tClcw;

      Print("Disable telemetry encoder hardware channels and switch vcid------------------");
      RmapW0(cTMDMAExtCtrl, X"0000000" & "000" &
                        '0',                             -- enable
                  "cTMDMAExtCtrl     ");
      RmapW0(cTMVCID, X"85868384", "cTMVCID");
      RmapR0(cTMDMACtrl, d, "cTMDMACtrl  ", False);
      d:= d or X"00000020";
      RmapW0 (cTMDMACtrl, d, "cTMDMACtrl");              -- Reset EXT
      
      for i in 0 to 0 loop
         tTransmitDMA(HRESP_OKAY, HRESP_OKAY);
         RmapW0(cTMDMAStat, X"00000000",  "cTMDMAStat  ");
      end loop;

      Print("CLCW registers -------------------------------------------------");
      RmapW0(gTCCLCWR1,  X"AABBCCDD", "gTCCLCWR1");
      RmapW0(gTCCLCWR2,  X"EEFF0011", "gTCCLCWR2");

      Print("Re-enable telemetry encoder hardware channels ------------------");
      RmapW0(cTMDMAExtCtrl, X"0000000" & "000" &
                        '1',                             -- enable
                  "cTMDMAExtCtrl     ");

      for i in 0 to 0 loop
         tTransmitDMA(HRESP_OKAY, HRESP_OKAY);
      end loop;

      if false then
         --------------------------------------------------------------------------
         Print("Soft reset test");
         wait for 2500 us;
         Print("########### Reset");

         RmapW0 (cTMDMACtrl, X"000000" & "000" &
                              '0' &                       -- TX FRAME IRQ ENABLE
                              '1' &                       -- RESET
                              '0' &                       -- TX RESET
                              '0' &                       -- TX IRQ ENABLE
                              '0',                        -- TX ENABLE
                 "cTMDMACtrl");

         RmapC0(cTMDMACtrl, X"000000" & "000" &
                             '0' &                       -- TX FRAME IRQ
                             '0' &                       -- RESET
                             '1' &                       -- TX RESET
                             '0' &                       -- TX IRQ ENABLE
                             '0',                        -- TX ENABLE
                TP,  "cTMDMACtrl   ");


         RmapW0 (cTMDMACtrl, X"000000" & "000" &
                              '0' &                       -- TX FRAME IRQ ENABLE
                              '0' &                       -- RESET
                              '0' &                       -- TX RESET
                              '0' &                       -- TX IRQ ENABLE
                              '0',                        -- TX ENABLE
                 "cTMDMACtrl");

         Print("Enable telemetry encoder transmitter ------------------------");
         RmapW0(cTMCtrl, X"0000000" & "000" &
                          '1',                          -- enable
                "cTMCtrl     ");

         Print("Wait for TX part to become un-reset -------------------------");
         d := (others => '1');
         while d(7) /= '0' loop
            RmapR0(cTMDMAStat, d, "cTMDMAStat  ", False);
         end loop;

         Print("Enable telemetry encoder hardware channels ------------------");
         RmapW0(cAHBRAMCFG16k,X"00000080", "cAHBRAMCFG16k");
         RmapW0(cTMDMAExtCtrl, X"0000000" & "000" &
                                '1',                          -- enable
                "cTMDMAExtCtrl     ");

         for i in 0 to 7 loop
            tTransmitDMA(HRESP_OKAY, HRESP_OKAY);
         end loop;
      end if;

      if not vc3vc4done then
        wait until vc3vc4done for 2000 ms;
      end if;

      if not vc3vc4done then
        TP := false;
        TPCounter := TPCounter + 1;
      elsif not vc3vc4tp then
        TP := false;
        TPCounter := TPCounter + vc3vc4tpcounter;
      end if;

      if not vc5vc6done then
        wait until vc5vc6done for 1000 ms;
      end if;

      if not vc5vc6done then
        TP := false;
        TPCounter := TPCounter + 1;
      elsif not vc5vc6tp then
        TP := false;
        TPCounter := TPCounter + vc5vc6tpcounter;
      end if;

      tTerminate(TP, TPCounter);
   end process Main; -----------------------------------------------------------


end Behavioural; --===========================================================--
