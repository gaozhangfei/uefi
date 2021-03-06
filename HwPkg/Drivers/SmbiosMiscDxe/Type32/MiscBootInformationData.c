
/**@file

Copyright (c) 2006 - 2009, Intel Corporation. All rights reserved.<BR>
This program and the accompanying materials                          
are licensed and made available under the terms and conditions of the BSD License         
which accompanies this distribution.  The full text of the license may be found at        
http://opensource.org/licenses/bsd-license.php                                            
                                                                                          
THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,                     
WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.             

Module Name:

  MiscBootInformationData.c
  
Abstract: 

  This driver parses the mMiscSubclassDataTable structure and reports
  any generated data to the DataHub.

**/

#include "SmbiosMisc.h"

//
// Static (possibly build generated) Bios Vendor data.
//
MISC_SMBIOS_TABLE_DATA(SMBIOS_TABLE_TYPE32, MiscBootInformation) = {
  {                                                     // Hdr
    EFI_SMBIOS_TYPE_SYSTEM_BOOT_INFORMATION,              // Type,
    0,                                                    // Length,
    0                                                     // Handle
  },
  {                                                     // Reserved[6]
    0,
    0,
    0,
    0,
    0,
    0
  },          
  BootInformationStatusNoError,                         // BootInformationStatus
  {                                                     // BootInformationData 
    0
  }
};

/* eof - MiscBootInformationData.c */
