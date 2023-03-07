// Copyright 2021 Thales DIS design services SAS
//
// Licensed under the Solderpad Hardware Licence, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.0
// You may obtain a copy of the License at https://solderpad.org/licenses/
//
// Original Author: Jean-Roch COULON - Thales


package cva6_config_pkg;

    localparam CVA6ConfigXlen = 32;

    localparam CVA6ConfigFpuEn = 0;
    localparam CVA6ConfigF16En = 0;
    localparam CVA6ConfigF16AltEn = 0;
    localparam CVA6ConfigF8En = 0;
    localparam CVA6ConfigFVecEn = 0;

    localparam CVA6ConfigCvxifEn = 0;
    localparam CVA6ConfigCExtEn = 0;
    localparam CVA6ConfigAExtEn = 1;

    localparam CVA6ConfigFetchUserEn = 0;
    localparam CVA6ConfigFetchUserWidth = CVA6ConfigXlen;
    localparam CVA6ConfigDataUserEn = 0;
    localparam CVA6ConfigDataUserWidth = CVA6ConfigXlen;

    localparam CVA6ConfigRenameEn = 0;

    localparam CVA6ConfigIcacheSetAssoc = 2;
    localparam CVA6ConfigIcacheLines = 4096;
    localparam CVA6ConfigIcacheLineWidth = 128;
    localparam CVA6ConfigDcacheSetAssoc = 2;
    localparam CVA6ConfigDcacheLines = 4096;
    localparam CVA6ConfigDcacheLineWidth = 128;

    localparam CVA6ConfigNrCommitPorts = 1;
    localparam CVA6ConfigNrScoreboardEntries = 4;

    localparam CVA6ConfigFPGAEn = 1;

    localparam CVA6ConfigNrLoadPipeRegs = 1;
    localparam CVA6ConfigNrStorePipeRegs = 0;

    localparam CVA6ConfigInstrTlbEntries = 2;
    localparam CVA6ConfigDataTlbEntries = 2;

    localparam CVA6ConfigRASDepth = 2;
    localparam CVA6ConfigBTBEntries = 32;
    localparam CVA6ConfigBHTEntries = 128;

    localparam CVA6ConfigNrPMPEntries = 0;

endpackage
