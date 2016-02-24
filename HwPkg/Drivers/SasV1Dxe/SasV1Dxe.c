/** @file

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "SasV1Dxe.h"

/* HW dma structures */
/* Delivery queue header */
/* dw0 */
#define CMD_HDR_RESP_REPORT_OFF		5
#define CMD_HDR_RESP_REPORT_MSK		0x20
#define CMD_HDR_TLR_CTRL_OFF		6
#define CMD_HDR_TLR_CTRL_MSK		0xc0
#define CMD_HDR_PORT_OFF		17
#define CMD_HDR_PORT_MSK		0xe0000
#define CMD_HDR_PRIORITY_OFF		27
#define CMD_HDR_PRIORITY_MSK		0x8000000
#define CMD_HDR_MODE_OFF		28
#define CMD_HDR_MODE_MSK		0x10000000
#define CMD_HDR_CMD_OFF			29
#define CMD_HDR_CMD_MSK			0xe0000000
/* dw1 */
#define CMD_HDR_VERIFY_DTL_OFF		10
#define CMD_HDR_VERIFY_DTL_MSK		0x400
#define CMD_HDR_SSP_FRAME_TYPE_OFF	13
#define CMD_HDR_SSP_FRAME_TYPE_MSK	0xe000
#define CMD_HDR_DEVICE_ID_OFF		16
#define CMD_HDR_DEVICE_ID_MSK		0xffff0000
/* dw2 */
#define CMD_HDR_CFL_OFF			0
#define CMD_HDR_CFL_MSK			0x1ff
#define CMD_HDR_MRFL_OFF		15
#define CMD_HDR_MRFL_MSK		0xff8000
#define CMD_HDR_FIRST_BURST_OFF		25
#define CMD_HDR_FIRST_BURST_MSK		0x2000000
/* dw3 */
#define CMD_HDR_IPTT_OFF		0
#define CMD_HDR_IPTT_MSK		0xffff
/* dw6 */
#define CMD_HDR_DATA_SGL_LEN_OFF	16
#define CMD_HDR_DATA_SGL_LEN_MSK	0xffff0000

/* Completion header */
#define CMPLT_HDR_IPTT_OFF		0
#define CMPLT_HDR_IPTT_MSK		(0xffff << CMPLT_HDR_IPTT_OFF)
#define CMPLT_HDR_CMD_CMPLT_MSK		BIT17
#define CMPLT_HDR_ERR_RCRD_XFRD_MSK	BIT18
#define CMPLT_HDR_RSPNS_XFRD_MSK	BIT19
#define CMPLT_HDR_IO_CFG_ERR_MSK	BIT27

EFI_GUID mSasV1DevicePathGuid = EFI_CALLER_ID_GUID;

// The time between interrupt polls, in units of 100 nanoseconds
// 10 Microseconds
#define SAS_INTERRUPT_POLL_PERIOD 10000

STATIC EFI_STATUS prepare_cmd (
  struct hisi_hba *hba, 
  EFI_EXT_SCSI_PASS_THRU_SCSI_REQUEST_PACKET    *Packet
  )
{
	struct hisi_sas_slot *slot;
	struct hisi_sas_cmd_hdr *hdr;
	struct hisi_sas_sge	*sge;
	struct hisi_sas_sts	*sts;
	struct hisi_sas_cmd	*cmd;
	VOID   *Buffer = NULL;
	UINT32 BufferSize = 0;
	int queue = hba->queue;
	UINT32 r, w = 0, slot_idx = 0;
	
	while (1) {
		w = READ_REG32(DLVRY_Q_0_WR_PTR + (queue * 0x14));
		r = READ_REG32(DLVRY_Q_0_RD_PTR + (queue * 0x14));
		slot_idx = queue * QUEUE_SLOTS + w;
		slot = &hba->slots[slot_idx];
		if (slot->used || (r == (w+1) % QUEUE_SLOTS)) {
			queue = (queue + 1) % QUEUE_CNT;
			if (queue == hba->queue) {
				DEBUG ((EFI_D_ERROR, "could not find free slot\n"));
				return EFI_NOT_READY;
			}
			continue;
		}
		break;
	}

	//DEBUG ((EFI_D_ERROR, "prepare_cmd queue=%d slot_idx=%d\n", queue, slot_idx));
	hdr = &hba->cmd_hdr[queue][w];
	cmd = &hba->command_table[queue][w];
	sts = &hba->status_buf[queue][w];
	sge = &hba->sge[queue][w];

	ZeroMem (cmd, sizeof (struct hisi_sas_cmd));
	ZeroMem (sts, sizeof (struct hisi_sas_sts));

	slot->used = TRUE;
	slot->sts = sts;
	hba->queue = (queue + 1) % QUEUE_CNT;

	//only consider ssp
	//prep_ssp_v1_hw
	/* create header */
	hdr->dw0 = (1 << CMD_HDR_RESP_REPORT_OFF) |
		   (0x2 << CMD_HDR_TLR_CTRL_OFF) |
		   (hba->port_id << CMD_HDR_PORT_OFF) |
		   (1 << CMD_HDR_MODE_OFF) | /* ini mode */
		   (1 << CMD_HDR_CMD_OFF); /* ssp */
	hdr->dw1 = 1 << CMD_HDR_VERIFY_DTL_OFF;
	/* device_id = 0 */
	hdr->dw1 |= 0 << CMD_HDR_DEVICE_ID_OFF;
	hdr->dw2 = 0x83000d;
	hdr->transfer_tags = slot_idx << CMD_HDR_IPTT_OFF;

	if (Packet->DataDirection == EFI_EXT_SCSI_DATA_DIRECTION_READ) {
		Buffer = Packet->InDataBuffer;
		BufferSize = Packet->InTransferLength;
		if (Buffer) {
			hdr->dw1 |= 1 << CMD_HDR_SSP_FRAME_TYPE_OFF;
			InvalidateDataCacheRange (Buffer, BufferSize);
		}
	} else if (Packet->DataDirection == EFI_EXT_SCSI_DATA_DIRECTION_WRITE) {
		Buffer = Packet->OutDataBuffer;
		BufferSize = Packet->OutTransferLength;
		if (Buffer)
			hdr->dw1 |= 2 << CMD_HDR_SSP_FRAME_TYPE_OFF;
	} else {
		hdr->dw1 |= 0 << CMD_HDR_SSP_FRAME_TYPE_OFF;
	}

	//DEBUG ((EFI_D_ERROR, "hdr->dw0=0x%x hdr->dw1=0x%x, hdr->dw2=0x%x\n", hdr->dw0, hdr->dw1, hdr->dw2));
	//DEBUG ((EFI_D_ERROR, "Packet->DataDirection=%d\n", Packet->DataDirection));
	//DEBUG ((EFI_D_ERROR, "Packet->SenseDataLength=%d\n", Packet->SenseDataLength));
	//DEBUG ((EFI_D_ERROR, "Buffer=0x%x, BufferSize=%d\n", Buffer, BufferSize));

	if (Buffer != NULL) {
		//only 1 entry
		sge->addr = (UINT64)Buffer;
		sge->page_ctrl_0 = sge->page_ctrl_1 = 0;
		sge->data_len = BufferSize;
		sge->data_off = 0;
		hdr->prd_table_addr = (UINT64)sge;
		hdr->sg_len = 1 << CMD_HDR_DATA_SGL_LEN_OFF;
	}

	hdr->data_transfer_len = BufferSize;
	hdr->cmd_table_addr = (UINT64)cmd;
	hdr->sts_buffer_addr = (UINT64)sts;

	CopyMem (&cmd->cmd[36], Packet->Cdb, Packet->CdbLength);
#if 1	
	//DEBUG ((EFI_D_ERROR, "cmd->cmd[36]=0x%x Packet->Cdb[0]=0x%x Packet->CdbLength=%d\n", cmd->cmd[36], ((UINT8 *) Packet->Cdb)[0], Packet->CdbLength));
	//DEBUG ((EFI_D_ERROR, "Packet->CdbLength=%d Packet->Cdb[0]=0x%x\n", Packet->CdbLength, ((UINT8 *) Packet->Cdb)[0]));
	{
		int i;
		/*
		for (i=0; i<76; i++)
			if (cmd->cmd[i] != 0)
				DEBUG ((EFI_D_ERROR, "cmd->cmd[%d]=0x%x\n", i, cmd->cmd[i]));
		*/
		DEBUG ((EFI_D_ERROR, "["));
		for (i=0; i<Packet->CdbLength; i++)
			DEBUG ((EFI_D_ERROR, "0x%x ", ((UINT8 *) Packet->Cdb)[i]));
		DEBUG ((EFI_D_ERROR, "]\n"));

	}
#endif	
	if (Packet->DataDirection == EFI_EXT_SCSI_DATA_DIRECTION_WRITE)
		WriteBackDataCacheRange (Buffer, BufferSize);

	asm("dsb  sy");
	asm("isb  sy");
	
	//start_delivery_v1_hw
	WRITE_REG32(DLVRY_Q_0_WR_PTR + queue * 0x14, ++w % QUEUE_SLOTS);
	
	//waiting for slot free, dma completed
	while (slot->used) {
		if (READ_REG32(OQ_INT_SRC) & BIT(queue)) {
			//DEBUG ((EFI_D_ERROR, "slot->used=%d\n", slot->used));
			break;
		}
		NanoSecondDelay (100);
	}

/*
	{
		int i;
		for (i = 0; i < BufferSize; i++)
			if (((UINT8 *)Buffer)[i] != 0)
			DEBUG ((EFI_D_ERROR, "Buffer[%d]=0x%x\n", i, ((UINT8 *)Buffer)[i]));
	}
*/
	
#if 1	
	{
		UINT8 *p = (UINT8 *)&slot->sts->status[0];
		UINT32 *pp = &slot->sts->status[0];
		if (p[26]) {
			/* hack for spin up */
			EFI_SCSI_SENSE_DATA *SensePtr = Packet->SenseData; 
			DEBUG ((EFI_D_ERROR, "p[26]=0x%x pp[6]=0x%x\n", p[26], pp[6]));
			SensePtr->Sense_Key = EFI_SCSI_SK_NOT_READY;
			SensePtr->Addnl_Sense_Code = EFI_SCSI_ASC_NOT_READY;
			SensePtr->Addnl_Sense_Code_Qualifier = EFI_SCSI_ASCQ_IN_PROGRESS;
			MicroSecondDelay(1000000);
		}

	}
#endif
	return EFI_SUCCESS;
}

// Instead of actually registering interrupt handlers, we poll the controller's
//  interrupt source register in this function.
STATIC
VOID
CheckInterrupts (
  IN EFI_EVENT  Event,
  IN VOID      *Context
  )
{
	SAS_V1_INFO *SasV1Info = (SAS_V1_INFO *) Context;;
	struct hisi_hba *hba = SasV1Info->hba;
	struct hisi_sas_slot *slot;
	int i, val, queue = 0;
	

	switch (hba->state) {
	case PHY_DOWN:
	//check phyup
	for (i = 0; i < PHY_CNT; i++) {
		val = PHY_READ_REG32(CHL_INT2, i);

		if (val & CHL_INT2_SL_PHY_ENA) {
			struct hisi_sas_itct *itct = &hba->itct[0]; //device_id = 0

			//check sata or sas
			val = READ_REG32(PHY_CONTEXT);
			if (val & BIT(i)) {
				DEBUG ((EFI_D_ERROR, "phyup: phy%d SATA attached equipment\n", i));
				goto phyup_exit;
			}

			hba->state = PHY_UP;
			hba->port_id = (READ_REG32(PHY_PORT_NUM_MA) >> (4 * i)) & 0xf;
			//setup itct
			itct->qw0 = 0x355;
			itct->sas_addr = PHY_READ_REG32(RX_IDAF_DWORD3, i);
			itct->sas_addr = itct->sas_addr << 32 | PHY_READ_REG32(RX_IDAF_DWORD4, i);
			itct->qw2 = 0;

phyup_exit:
			//clear int
			PHY_WRITE_REG32(CHL_INT2, i, CHL_INT2_SL_PHY_ENA);
			val = PHY_READ_REG32(CHL_INT0, i);
			val &= ~CHL_INT0_PHYCTRL_NOTRDY;
			PHY_WRITE_REG32(CHL_INT0, i, val);
			PHY_WRITE_REG32(CHL_INT0_MSK, i, 0x3ce3ee);
			return;
		}
	}
	break;
	
	//Do not consider hotplug
	
	case PHY_UP:
	//check data irq
	val = READ_REG32(OQ_INT_SRC);
	if (val) {
		UINT32 rd, wr;

		for (i = 0; i < QUEUE_CNT; i++) {
			if (val & BIT(i)) {
				queue = i;
				break;
			}
		}

		rd = READ_REG32(COMPL_Q_0_RD_PTR + (0x14 * queue));
		wr = READ_REG32(COMPL_Q_0_WR_PTR + (0x14 * queue));

		while (rd != wr) {
			struct hisi_sas_complete_hdr *complete_hdr;
			int idx;
			UINT32 data;

			complete_hdr = &hba->complete_hdr[queue][rd];
			data = complete_hdr->data;
			idx = (data & CMPLT_HDR_IPTT_MSK) >> CMPLT_HDR_IPTT_OFF;
			slot = &hba->slots[idx];
			slot->used = FALSE;
			if (++rd >= QUEUE_SLOTS)
				rd = 0;

			if (data & CMPLT_HDR_ERR_RCRD_XFRD_MSK) {
				DEBUG ((EFI_D_ERROR, "CMPLT_HDR_ERR_RCRD_XFRD_MSK data=0x%x\n", data));
				DEBUG ((EFI_D_ERROR, "slot->sts[0]=0x%x\n", slot->sts->status[0]));
				DEBUG ((EFI_D_ERROR, "slot->sts[1]=0x%x\n", slot->sts->status[1]));
				DEBUG ((EFI_D_ERROR, "slot->sts[2]=0x%x\n", slot->sts->status[2]));
			}
			/*
			{
				UINT8 *p = (UINT8 *)&slot->sts->status[0];
				DEBUG ((EFI_D_ERROR, "irq p[26]=0x%x\n", p[26]));
				DEBUG ((EFI_D_ERROR, "slot->sts[6]=0x%x\n", slot->sts->status[6]));
			}
			*/
		}
		/* update rd */
		WRITE_REG32(COMPL_Q_0_RD_PTR + (0x14 * queue), rd);
		/* clear int */
		WRITE_REG32(OQ_INT_SRC, val);
		
		//update slot->used
		asm("dsb  sy");
		asm("isb  sy");
	}
	break;
	default:
	DEBUG ((EFI_D_ERROR, "error state\n"));
	return;
	}

}

STATIC VOID hisi_sas_v1_init(struct hisi_hba *hba)
{
	int i, j;
	UINT32 val;

	//reset
	for (i = 0; i < PHY_CNT; i++) {
		UINT32 phy_ctrl = PHY_READ_REG32(PHY_CTRL, i);

		phy_ctrl |= PHY_CTRL_RESET;
		PHY_WRITE_REG32(PHY_CTRL, i, phy_ctrl);
	}
	MicroSecondDelay(1000);

	/* Ensure DMA tx & rx idle */
	for (i = 0; i < PHY_CNT; i++) {
		UINT32 dma_tx_status, dma_rx_status;

		for (j = 0; j < 100; j++) {
			dma_tx_status = PHY_READ_REG32(DMA_TX_STATUS, i);
			dma_rx_status = PHY_READ_REG32(DMA_RX_STATUS, i);

			if (!(dma_tx_status & DMA_TX_STATUS_BUSY) &&
				!(dma_rx_status & DMA_RX_STATUS_BUSY))
				break;
		}
	}

	/* Ensure axi bus idle */
	for (j = 0; j < 100; j++) {
		UINT32 axi_status = READ_REG32(AXI_CFG);
		if (axi_status == 0)
			break;
	}

	/* Apply reset and disable clock */
	CTRL_WRITE_REG32(CTRL_RESET, RESET_VALUE);
	CTRL_WRITE_REG32(CTRL_CLK_DIS, RESET_VALUE);
	MicroSecondDelay(1000);

	/* De-reset and enable clock */
	CTRL_WRITE_REG32(CTRL_DRESET, RESET_VALUE);
	CTRL_WRITE_REG32(CTRL_CLK_ENA, RESET_VALUE);
	MicroSecondDelay(1000);

	//init_reg_v1_hw
	WRITE_REG32(DLVRY_QUEUE_ENABLE, 0xffffffff);
	WRITE_REG32(HGC_TRANS_TASK_CNT_LIMIT, 0x11);
	WRITE_REG32(DEVICE_MSG_WORK_MODE, 0x1);
	WRITE_REG32(HGC_SAS_TXFAIL_RETRY_CTRL, 0x1ff);
	WRITE_REG32(HGC_ERR_STAT_EN, 0x401);
	WRITE_REG32(CFG_1US_TIMER_TRSH, 0x64);
	WRITE_REG32(HGC_GET_ITV_TIME, 0x1);
	WRITE_REG32(I_T_NEXUS_LOSS_TIME, 0x64);
	WRITE_REG32(BUS_INACTIVE_LIMIT_TIME, 0x2710);
	WRITE_REG32(REJECT_TO_OPEN_LIMIT_TIME, 0x1);
	WRITE_REG32(CFG_AGING_TIME, 0x7a12);
	WRITE_REG32(HGC_DFX_CFG2, 0x9c40);
	WRITE_REG32(FIS_LIST_BADDR_L, 0x2);
	WRITE_REG32(INT_COAL_EN, 0xc);
	WRITE_REG32(OQ_INT_COAL_TIME, 0x186a0);
	WRITE_REG32(OQ_INT_COAL_CNT, 1);
	WRITE_REG32(ENT_INT_COAL_TIME, 0x1);
	WRITE_REG32(ENT_INT_COAL_CNT, 0x1);
	WRITE_REG32(OQ_INT_SRC, 0xffffffff);
	//mask OQ_INT_SRC?
	//WRITE_REG32(OQ_INT_SRC_MSK, 0);
	WRITE_REG32(ENT_INT_SRC1, 0xffffffff);
	WRITE_REG32(ENT_INT_SRC_MSK1, 0);
	WRITE_REG32(ENT_INT_SRC2, 0xffffffff);
	WRITE_REG32(ENT_INT_SRC_MSK2, 0);
	WRITE_REG32(SAS_ECC_INTR_MSK, 0);
	WRITE_REG32(AXI_AHB_CLK_CFG, 0x2);
	WRITE_REG32(CFG_SAS_CONFIG, 0x22000000);

	for (i = 0; i < PHY_CNT; i++) {
		PHY_WRITE_REG32(PROG_PHY_LINK_RATE, i, 0x88a);
		PHY_WRITE_REG32(PHY_CONFIG2, i, 0x7c080);
		PHY_WRITE_REG32(PHY_RATE_NEGO, i, 0x415ee00);
		PHY_WRITE_REG32(PHY_PCN, i, 0x80a80000);
		PHY_WRITE_REG32(SL_TOUT_CFG, i, 0x7d7d7d7d);
		PHY_WRITE_REG32(DONE_RECEIVED_TIME, i, 0x0);
		PHY_WRITE_REG32(RXOP_CHECK_CFG_H, i, 0x1000);
		PHY_WRITE_REG32(DONE_RECEIVED_TIME, i, 0);
		PHY_WRITE_REG32(CON_CFG_DRIVER, i, 0x13f0a);
		PHY_WRITE_REG32(CHL_INT_COAL_EN, i, 3);
		PHY_WRITE_REG32(DONE_RECEIVED_TIME, i, 8);
	}

	for (i = 0; i < QUEUE_CNT; i++) {
		/* Delivery queue */
		WRITE_REG32(DLVRY_Q_0_BASE_ADDR_HI + (i * 0x14), upper_32_bits((UINT64)(hba->cmd_hdr[i])));
		WRITE_REG32(DLVRY_Q_0_BASE_ADDR_LO + (i * 0x14), lower_32_bits((UINT64)(hba->cmd_hdr[i])));
		WRITE_REG32(DLVRY_Q_0_DEPTH + (i * 0x14), QUEUE_SLOTS);

		/* Completion queue */
		WRITE_REG32(COMPL_Q_0_BASE_ADDR_HI + (i * 0x14), upper_32_bits((UINT64)(hba->complete_hdr[i])));
		WRITE_REG32(COMPL_Q_0_BASE_ADDR_LO + (i * 0x14), lower_32_bits((UINT64)(hba->complete_hdr[i])));
		WRITE_REG32(COMPL_Q_0_DEPTH + (i * 0x14), QUEUE_SLOTS);
	}

	/* itct */
	WRITE_REG32(ITCT_BASE_ADDR_LO, lower_32_bits((UINT64)(hba->itct)));
	WRITE_REG32(ITCT_BASE_ADDR_HI, upper_32_bits((UINT64)(hba->itct)));

	/* iost */
	WRITE_REG32(IOST_BASE_ADDR_LO, lower_32_bits((UINT64)(hba->iost)));
	WRITE_REG32(IOST_BASE_ADDR_HI, upper_32_bits((UINT64)(hba->iost)));

	/* breakpoint */
	WRITE_REG32(BROKEN_MSG_ADDR_LO, lower_32_bits((UINT64)(hba->breakpoint)));
	WRITE_REG32(BROKEN_MSG_ADDR_HI, upper_32_bits((UINT64)(hba->breakpoint)));


	//interrupt_openall_v1_hw
	for (i = 0; i < PHY_CNT; i++) {
		/* Clear interrupt status */
		val = PHY_READ_REG32(CHL_INT0, i);
		PHY_WRITE_REG32(CHL_INT0, i, val);
		val = PHY_READ_REG32(CHL_INT1, i);
		PHY_WRITE_REG32(CHL_INT1, i, val);
		val = PHY_READ_REG32(CHL_INT2, i);
		PHY_WRITE_REG32(CHL_INT2, i, val);

		/* bypass chip bug mask abnormal intr */
		PHY_WRITE_REG32(CHL_INT0_MSK, i, 0x3fffff & ~CHL_INT0_MSK_PHYCTRL_NOTRDY);
	}

	//phys_init_v1_hw
	for (i = 0; i < PHY_CNT; i++) {
		PHY_WRITE_REG32(TX_ID_DWORD0, i, 0x10010e00);
		PHY_WRITE_REG32(TX_ID_DWORD1, i, 0x16);
		PHY_WRITE_REG32(TX_ID_DWORD2, i, 0x20880150);
		PHY_WRITE_REG32(TX_ID_DWORD3, i, 0x16);
		PHY_WRITE_REG32(TX_ID_DWORD4, i, 0x20880150);
		PHY_WRITE_REG32(TX_ID_DWORD5, i, 0x0);
		
		val = PHY_READ_REG32(PHY_CFG, i);
		val &= ~PHY_CFG_DC_OPT_MSK;
		val |= 1 << PHY_CFG_DC_OPT_OFF;
		PHY_WRITE_REG32(PHY_CFG, i, val);

		val = PHY_READ_REG32(PHY_CONFIG2, i);
		val &= ~PHY_CONFIG2_FORCE_TXDEEMPH_MSK;
		PHY_WRITE_REG32(PHY_CONFIG2, i, val);
		
		val = PHY_READ_REG32(PHY_CFG, i);
		val |= PHY_CFG_ENA_MSK;
		PHY_WRITE_REG32(PHY_CFG, i, val);
	}
}

STATIC VOID sas_init(SAS_V1_INFO *SasV1Info)
{
  /* alloc memory */
  struct hisi_hba *hba = SasV1Info->hba;
  int i, s;
#if 0
  DEBUG ((EFI_D_ERROR, "sizeof(struct hisi_sas_cmd_hdr)=0x%x\n", sizeof(struct hisi_sas_cmd_hdr)));
  DEBUG ((EFI_D_ERROR, "sizeof(struct hisi_sas_complete_hdr)=0x%x\n", sizeof(struct hisi_sas_complete_hdr)));
  DEBUG ((EFI_D_ERROR, "sizeof(struct hisi_sas_sts)=0x%x\n", sizeof(struct hisi_sas_sts)));
  DEBUG ((EFI_D_ERROR, "sizeof(struct hisi_sas_cmd)=0x%x\n", sizeof(struct hisi_sas_cmd)));
  DEBUG ((EFI_D_ERROR, "sizeof(struct hisi_sas_sge)=0x%x\n", sizeof(struct hisi_sas_sge)));
  DEBUG ((EFI_D_ERROR, "sizeof(struct hisi_sas_iost)=0x%x\n", sizeof(struct hisi_sas_iost)));
  DEBUG ((EFI_D_ERROR, "sizeof(struct hisi_sas_breakpoint)=0x%x\n", sizeof(struct hisi_sas_breakpoint)));
  DEBUG ((EFI_D_ERROR, "sizeof(struct hisi_sas_itct)=0x%x\n", sizeof(struct hisi_sas_itct)));
  DEBUG ((EFI_D_ERROR, "sizeof(struct hisi_sas_slot)=0x%x\n", sizeof(struct hisi_sas_slot)));
#endif

  for (i = 0; i < QUEUE_CNT; i++) {
	  /* Delivery queue */
	  s = sizeof(struct hisi_sas_cmd_hdr) * QUEUE_SLOTS;
	  hba->cmd_hdr[i] = UncachedAllocateZeroPool(s);

	  /* Completion queue */
	  s = sizeof(struct hisi_sas_complete_hdr) * QUEUE_SLOTS;
	  hba->complete_hdr[i] = UncachedAllocateZeroPool(s);

	  s = sizeof(struct hisi_sas_sts) * QUEUE_SLOTS;
	  hba->status_buf[i] = UncachedAllocateZeroPool(s);

	  s = sizeof(struct hisi_sas_cmd) * QUEUE_SLOTS;
	  hba->command_table[i] = UncachedAllocateZeroPool(s);

	  s = sizeof(struct hisi_sas_sge) * QUEUE_SLOTS;
	  hba->sge[i] = UncachedAllocateZeroPool(s);
  }

#if 0
  {
	  int j;
	  for (j = 0; j < QUEUE_CNT; j++) {
		  for (i = 0; i < QUEUE_SLOTS; i++) {
			  DEBUG ((EFI_D_ERROR, "&hba->sge[%d][%d]=0x%x\n", j, i, &hba->sge[j][i]));
			  DEBUG ((EFI_D_ERROR, "&hba->cmd_hdr[%d][%d]=0x%x\n", j, i, &hba->cmd_hdr[j][i]));
			  DEBUG ((EFI_D_ERROR, "&hba->complete_hdr[%d][%d]=0x%x\n", j, i, &hba->complete_hdr[j][i]));
			  DEBUG ((EFI_D_ERROR, "&hba->status_buf[%d][%d]=0x%x\n", j, i, &hba->status_buf[j][i]));
			  DEBUG ((EFI_D_ERROR, "&hba->command_table[%d][%d]=0x%x\n", j, i, &hba->command_table[j][i]));
		  }
	  }
  }
#endif

  s = SLOT_ENTRIES * sizeof(struct hisi_sas_iost);
  hba->iost = UncachedAllocateZeroPool(s);

  s = SLOT_ENTRIES * sizeof(struct hisi_sas_breakpoint);
  hba->breakpoint = UncachedAllocateZeroPool(s);

 #if 0 
  for (i = 0; i < SLOT_ENTRIES; i++) {
	  DEBUG ((EFI_D_ERROR, "&hba->iost[%d]=0x%x\n", i, &hba->iost[i]));
	  DEBUG ((EFI_D_ERROR, "&hba->breakpoint[%d]=0x%x\n", i, &hba->breakpoint[i]));
  }
#endif
  
  s = MAX_ITCT_ENTRIES * sizeof(struct hisi_sas_itct);
  hba->itct = UncachedAllocateZeroPool(s);

  hba->slots = AllocateZeroPool (SLOT_ENTRIES * sizeof(struct hisi_sas_slot));
  ASSERT (hba->slots != NULL);   
 
  /* init hardware */
  hisi_sas_v1_init(hba);
}

STATIC
EFI_STATUS
EFIAPI
SasV1ExtScsiPassThruFunction (
  IN EFI_EXT_SCSI_PASS_THRU_PROTOCOL                    *This,
  IN UINT8                                              *Target,
  IN UINT64                                             Lun,
  IN OUT EFI_EXT_SCSI_PASS_THRU_SCSI_REQUEST_PACKET     *Packet,
  IN EFI_EVENT                                          Event OPTIONAL
  )
{
  SAS_V1_INFO *SasV1Info = SAS_FROM_PASS_THRU(This);
  struct hisi_hba *hba = SasV1Info->hba;

  return prepare_cmd(hba, Packet);
}

STATIC
EFI_STATUS
EFIAPI
SasV1ExtScsiPassThruGetNextTargetLun (
  IN  EFI_EXT_SCSI_PASS_THRU_PROTOCOL    *This,
  IN OUT UINT8                           **Target,
  IN OUT UINT64                          *Lun
  )
{
  SAS_V1_INFO *SasV1Info = SAS_FROM_PASS_THRU(This);
  struct hisi_hba *hba = SasV1Info->hba;
  UINT8 ScsiId[TARGET_MAX_BYTES];
  UINT8 TargetId;

  if (*Target == NULL || Lun == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  
  SetMem (ScsiId, TARGET_MAX_BYTES, 0xFF);

  TargetId = (*Target)[0];
  
  if (TargetId == MAX_TARGET_ID) {
    return EFI_NOT_FOUND;
  }
  
  if (CompareMem(*Target, ScsiId, TARGET_MAX_BYTES) == 0) {
    SetMem (*Target, TARGET_MAX_BYTES,0);
  } else {
    (*Target)[0] = (UINT8) (hba->LatestTargetId + 1);
  }

  *Lun = 0;
  
  //
  // Update the LatestTargetId.
  //
  hba->LatestTargetId  = (*Target)[0];
  hba->LatestLun       = *Lun;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
SasV1ExtScsiPassThruBuildDevicePath (
  IN     EFI_EXT_SCSI_PASS_THRU_PROTOCOL    *This,
  IN     UINT8                              *Target,
  IN     UINT64                             Lun,
  IN OUT EFI_DEVICE_PATH_PROTOCOL           **DevicePath
  )
{
  EFI_DEVICE_PATH_PROTOCOL *NewDevicePathNode;

  NewDevicePathNode = CreateDeviceNode (HARDWARE_DEVICE_PATH, HW_VENDOR_DP, sizeof (VENDOR_DEVICE_PATH));
  CopyGuid (& ((VENDOR_DEVICE_PATH*)NewDevicePathNode)->Guid, &mSasV1DevicePathGuid);
  *DevicePath = NewDevicePathNode;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
SasV1ExtScsiPassThruGetTargetLun (
  IN  EFI_EXT_SCSI_PASS_THRU_PROTOCOL    *This,
  IN  EFI_DEVICE_PATH_PROTOCOL           *DevicePath,
  OUT UINT8                              **Target,
  OUT UINT64                             *Lun
  )
{

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
SasV1ExtScsiPassThruResetChannel (
  IN  EFI_EXT_SCSI_PASS_THRU_PROTOCOL   *This
  )
{

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
SasV1ExtScsiPassThruResetTarget (
  IN EFI_EXT_SCSI_PASS_THRU_PROTOCOL    *This,
  IN UINT8                              *Target,
  IN UINT64                             Lun
  )
{

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
SasV1ExtScsiPassThruGetNextTarget (
  IN  EFI_EXT_SCSI_PASS_THRU_PROTOCOL    *This,
  IN OUT UINT8                           **Target
  )
{

  return EFI_SUCCESS;
}

STATIC EFI_EXT_SCSI_PASS_THRU_PROTOCOL SasV1ExtScsiPassThruProtocolTemplate = {
  NULL,
  SasV1ExtScsiPassThruFunction,
  SasV1ExtScsiPassThruGetNextTargetLun,
  SasV1ExtScsiPassThruBuildDevicePath,
  SasV1ExtScsiPassThruGetTargetLun,
  SasV1ExtScsiPassThruResetChannel,
  SasV1ExtScsiPassThruResetTarget,
  SasV1ExtScsiPassThruGetNextTarget
};

EFI_STATUS
SasV1Initialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS  Status;
  SAS_V1_INFO *SasV1Info = NULL;
  EFI_EVENT TimerEvent;
  EFI_DEVICE_PATH_PROTOCOL  *DevicePath;

  SasV1Info = AllocateZeroPool (sizeof (SAS_V1_INFO));
  ASSERT (SasV1Info);
  SasV1Info->Signature = SAS_DEVICE_SIGNATURE;
 
  SasV1Info->hba = AllocateZeroPool (sizeof(struct hisi_hba));
  ASSERT (SasV1Info->hba);

  sas_init(SasV1Info);

  CopyMem (&SasV1Info->ExtScsiPassThru, &SasV1ExtScsiPassThruProtocolTemplate, sizeof (EFI_EXT_SCSI_PASS_THRU_PROTOCOL));
  SasV1Info->ExtScsiPassThruMode.AdapterId = 2;
  SasV1Info->ExtScsiPassThruMode.Attributes = EFI_EXT_SCSI_PASS_THRU_ATTRIBUTES_PHYSICAL | EFI_EXT_SCSI_PASS_THRU_ATTRIBUTES_LOGICAL;
  SasV1Info->ExtScsiPassThruMode.IoAlign  = 4;
  SasV1Info->ExtScsiPassThru.Mode    = &SasV1Info->ExtScsiPassThruMode;
  
  SasV1Info->DevicePath = CreateDeviceNode (HARDWARE_DEVICE_PATH, HW_VENDOR_DP, sizeof (VENDOR_DEVICE_PATH));
  ASSERT (SasV1Info->DevicePath); 
  
  CopyGuid (& ((VENDOR_DEVICE_PATH*)SasV1Info->DevicePath)->Guid, &mSasV1DevicePathGuid);
  
  DevicePath = AppendDevicePathNode (
                 SasV1Info->DevicePath,
                 NULL 
                 );
  SasV1Info->DevicePath = DevicePath;
  
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &ImageHandle,
                  &gEfiExtScsiPassThruProtocolGuid, &SasV1Info->ExtScsiPassThru,
                 // &gEfiDevicePathProtocolGuid, SasV1Info->DevicePath,
                  NULL
                  );
  ASSERT_EFI_ERROR (Status);

  // Register a timer event so CheckInterupts gets called periodically
  Status = gBS->CreateEvent (
                  EVT_TIMER | EVT_NOTIFY_SIGNAL,
                  TPL_CALLBACK,
                  CheckInterrupts,
                  SasV1Info,
                  &TimerEvent
                  );
  ASSERT_EFI_ERROR (Status);

  Status = gBS->SetTimer (
                  TimerEvent,
                  TimerPeriodic,
                  SAS_INTERRUPT_POLL_PERIOD
                  );
  ASSERT_EFI_ERROR (Status);

  return Status;
}
