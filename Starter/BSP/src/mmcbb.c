
/* Common include file for FatFs and disk I/O layer */
#include "diskio.h"


/*-------------------------------------------------------------------------*/
/* Platform dependent macros and functions needed to be modified           */
/*-------------------------------------------------------------------------*/
/* Include device specific declareation file here */
#include "stm32f10x.h"
#include "bsp.h"
#include <stdio.h>

/* Initialize MMC control port (CS/CLK/DI:output, DO/WP/INS:input) */
#define	INIT_PORT()	{ init_port(); }

/* Set MMC CS "high" */
#define	CS_H()		  GPIO_SetBits(GPIOB,GPIO_Pin_12)
/* Set MMC CS "low" */
#define CS_L()		  GPIO_ResetBits(GPIOB,GPIO_Pin_12)

/* Delay n microseconds */
#define DLY_US(n)	{ dly_us(n); }

/* Socket: Card is inserted (yes:true, no:false, default:true) */
#define	INS			  (1)
/* Socket: Card is write protected (yes:true, no:false, default:false) */
#define	WP			  (0)

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

/* MMC/SD command (SPI mode) */
/* MMC/SD command (SPI mode) */
#define CMD0	(0)			        /* GO_IDLE_STATE */
#define CMD1	(1)			        /* SEND_OP_COND */
#define	ACMD41	(0x80+41)	    /* SEND_OP_COND (SDC) */
#define CMD8	(8)			        /* SEND_IF_COND */
#define CMD9	(9)			        /* SEND_CSD */
#define CMD10	(10)		        /* SEND_CID */
#define CMD12	(12)		        /* STOP_TRANSMISSION */
#define ACMD13	(0x80+13)	    /* SD_STATUS (SDC) */
#define CMD16	(16)		        /* SET_BLOCKLEN */
#define CMD17	(17)		        /* READ_SINGLE_BLOCK */
#define CMD18	(18)		        /* READ_MULTIPLE_BLOCK */
#define CMD23	(23)		        /* SET_BLOCK_COUNT */
#define	ACMD23	(0x80+23)	    /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(24)		        /* WRITE_BLOCK */
#define CMD25	(25)		        /* WRITE_MULTIPLE_BLOCK */
#define CMD41	(41)		        /* SEND_OP_COND (ACMD) */
#define CMD55	(55)		        /* APP_CMD */
#define CMD58	(58)		        /* READ_OCR */

/* Card type flags (CardType) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */


/* Disk status */
static DSTATUS Stat = STA_NOINIT;

/* b0:MMC, b1:SDv1, b2:SDv2, b3:Block addressing */
static
BYTE CardType;

//初始化端口
void init_port()
{
  //初始化时钟 GPIOE
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE ,ENABLE );
  //配置GPIOE.7
  //定义一个GPIO结构体
  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

}

//软件演示函数
void dly_us(uint16_t n)
{
  for( ; n > 0 ; n--)
    for(uint8_t i = 100 ; i > 0 ; i--);
}

/*-----------------------------------------------------------------------*/
/* Transmit bytes to the card (bitbanging)                               */
/*-----------------------------------------------------------------------*/

static void xmit_mmc (const BYTE* buff,	UINT bc)
{
	BYTE d;

	do {
    /* Get a byte to be sent */
		d = *buff++;
    //通过SPI发送
    SPI2_SendByte(d);

	} while (--bc);
}



/*-----------------------------------------------------------------------*/
/* Receive bytes from the card (bitbanging)                              */
/*-----------------------------------------------------------------------*/

static void rcvr_mmc ( BYTE *buff,	UINT bc	)
{
	BYTE r;

	do {
    //重新赋值
		r = 0;
    //通过SPI获得数据
    r = SPI2_ReceiveByte();
    /* Store a received byte */
		*buff++ = r;
	} while (--bc);
}



/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/
/* 1:OK, 0:Timeout */
static int wait_ready (void)
{
	BYTE d;
	UINT tmr;

  /* Wait for ready in timeout of 500ms */
	for (tmr = 5000; tmr; tmr--)
  {
		rcvr_mmc(&d, 1);
		if (d == 0xFF)
    {
      break;
    }
    //没能读到FF，超时 问题所在 通过设置MISO为上拉输入解决
		DLY_US(100);
	}

	return tmr ? 1 : 0;
}



/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/

static void deselect (void)
{
	BYTE d;

	CS_H();
  /* Dummy clock (force DO hi-z for multiple slave SPI) */
	rcvr_mmc(&d, 1);
}



/*-----------------------------------------------------------------------*/
/* Select the card and wait for ready                                    */
/*-----------------------------------------------------------------------*/

static int select (void)	/* 1:OK, 0:Timeout */
{
	BYTE d;

	CS_L();
  /* Dummy clock (force DO enabled) */
	rcvr_mmc(&d, 1);
  /* OK */
	if (wait_ready()) return 1;
	deselect();
	return 0;	/* Timeout */
}



/*-----------------------------------------------------------------------*/
/* Receive a data packet from the card                                   */
/*-----------------------------------------------------------------------*/
/* 1:OK, 0:Failed */
static int rcvr_datablock (	BYTE *buff, UINT btr)
{
	BYTE d[2];
	UINT tmr;

  /* Wait for data packet in timeout of 100ms */
	for (tmr = 1000; tmr; tmr--)
  {
		rcvr_mmc(d, 1);
		if (d[0] != 0xFF) break;
		DLY_US(100);
	}
  /* If not valid data token, return with error */
	if (d[0] != 0xFE) return 0;
  /* Receive the data block into buffer */
	rcvr_mmc(buff, btr);
  /* Discard CRC */
	rcvr_mmc(d, 2);
  /* Return with success */
	return 1;
}



/*-----------------------------------------------------------------------*/
/* Send a data packet to the card                                        */
/*-----------------------------------------------------------------------*/
/* 1:OK, 0:Failed */
static int xmit_datablock (	const BYTE *buff,	BYTE token)
{
	BYTE d[2];

	if (!wait_ready()) return 0;

	d[0] = token;
  /* Xmit a token */
	xmit_mmc(d, 1);
  /* Is it data token? */
	if (token != 0xFD)
  {
		/* Xmit the 512 byte data block to MMC */
    xmit_mmc(buff, 512);
    /* Xmit dummy CRC (0xFF,0xFF) */
		rcvr_mmc(d, 2);
		/* Receive data response */
		rcvr_mmc(d, 1);
    /* If not accepted, return with error */
		if ((d[0] & 0x1F) != 0x05)
			return 0;
	}

	return 1;
}



/*-----------------------------------------------------------------------*/
/* Send a command packet to the card                                     */
/*-----------------------------------------------------------------------*/
/* Returns command response (bit7==1:Send failed)*/
static BYTE send_cmd (BYTE cmd,	DWORD arg	)
{
	BYTE n, d, buf[6];

  /* ACMD<n> is the command sequense of CMD55-CMD<n> */
	if (cmd & 0x80) {
		cmd &= 0x7F;
		n = send_cmd(CMD55, 0);
		if (n > 1) return n;
	}

	/* Select the card and wait for ready */
	deselect();

	if (!select()) return 0xFF;

	/* Send a command packet */
	buf[0] = 0x40 | cmd;			    /* Start + Command index */
	buf[1] = (BYTE)(arg >> 24);		/* Argument[31..24] */
	buf[2] = (BYTE)(arg >> 16);		/* Argument[23..16] */
	buf[3] = (BYTE)(arg >> 8);		/* Argument[15..8] */
	buf[4] = (BYTE)arg;				    /* Argument[7..0] */
	n = 0x01;						          /* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;		/* (valid CRC for CMD0(0)) */
	if (cmd == CMD8) n = 0x87;		/* (valid CRC for CMD8(0x1AA)) */
	buf[5] = n;
	xmit_mmc(buf, 6);

	/* Receive command response */
  /* Skip a stuff byte when stop reading */
	if (cmd == CMD12) rcvr_mmc(&d, 1);
  /* Wait for a valid response in timeout of 10 attempts */

	n = 10;
	do
  {
		rcvr_mmc(&d, 1);
  }
	while ((d & 0x80) && --n);

	return d;			/* Return with the response value */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE drv			/* Drive number (always 0) */
)
{
	DSTATUS s = Stat;
	BYTE ocr[4];


	if (drv || !INS) {
		s = STA_NODISK | STA_NOINIT;
	} else {
		s &= ~STA_NODISK;
		if (WP)						/* Check card write protection */
			s |= STA_PROTECT;
		else
			s &= ~STA_PROTECT;
		if (!(s & STA_NOINIT)) {
			if (send_cmd(CMD58, 0))	/* Check if the card is kept initialized */
				s |= STA_NOINIT;
			rcvr_mmc(ocr, 4);
			CS_H();
		}
	}
	Stat = s;

  //printf("DSTATUS %d\n",s);
	return s;
}



/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE drv		/* Physical drive nmuber (0) */
)
{
	BYTE n, ty, cmd, buf[4];
	UINT tmr;
	DSTATUS s;

  /* Initialize control port */
	INIT_PORT();

	s = disk_status(drv);		/* Check if card is in the socket */
	if (s & STA_NODISK) return s;

	CS_H();
	for (n = 10; n; n--) rcvr_mmc(buf, 1);	/* 80 dummy clocks */

	ty = 0;

	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2? */
			rcvr_mmc(buf, 4);							/* Get trailing return value of R7 resp */
			if (buf[2] == 0x01 && buf[3] == 0xAA) {		/* The card can work at vdd range of 2.7-3.6V */
				for (tmr = 1000; tmr; tmr--) {			/* Wait for leaving idle state (ACMD41 with HCS bit) */
					if (send_cmd(ACMD41, 1UL << 30) == 0) break;
					DLY_US(1000);
				}
				if (tmr && send_cmd(CMD58, 0) == 0) {	/* Check CCS bit in the OCR */
					rcvr_mmc(buf, 4);
					ty = (buf[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 */
				}
			}
		} else {							/* SDv1 or MMCv3 */
			if (send_cmd(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
			}
			for (tmr = 1000; tmr; tmr--) {			/* Wait for leaving idle state */
				if (send_cmd(ACMD41, 0) == 0) break;
				DLY_US(1000);
			}
			if (!tmr || send_cmd(CMD16, 512) != 0)	/* Set R/W block length to 512 */
				ty = 0;
		}
	}
	CardType = ty;

  /* Initialization succeded */
	if (ty)
  {
		s &= ~STA_NOINIT;
    //printf("Initialization succeded!\n");
  }
  /* Initialization failed */
	else
  {
		s |= STA_NOINIT;
    //printf("nitialization failed!\n");
  }

	Stat = s;

	deselect();

	return s;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE drv,			/* Physical drive nmuber (0) */
	BYTE *buff,			/* Pointer to the data buffer to store read data */
	DWORD sector,		/* Start sector number (LBA) */
	BYTE count			/* Sector count (1..128) */
)
{
	DSTATUS s;


	s = disk_status(drv);
	if (s & STA_NOINIT) return RES_NOTRDY;
	if (!count) return RES_PARERR;
	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert LBA to byte address if needed */
	if (count == 1) {	/* Single block read */
		if ((send_cmd(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
			&& rcvr_datablock(buff, 512))
			count = 0;
	}
	else {				/* Multiple block read */
		if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512)) break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0) */
	const BYTE *buff,	/* Pointer to the data to be written */
	DWORD sector,		/* Start sector number (LBA) */
	BYTE count			/* Sector count (1..128) */
)
{
	DSTATUS s;


	s = disk_status(drv);
	if (s & STA_NOINIT) return RES_NOTRDY;
	if (s & STA_PROTECT) return RES_WRPRT;
	if (!count) return RES_PARERR;
	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert LBA to byte address if needed */

	if (count == 1) {	/* Single block write */
		if ((send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple block write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}
	deselect();

	return count ? RES_ERROR : RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	BYTE n, csd[16];
	WORD cs;


	if (disk_status(drv) & STA_NOINIT)					/* Check if card is in the socket */
		return RES_NOTRDY;

	res = RES_ERROR;
	switch (ctrl) {
		case CTRL_SYNC :		/* Make sure that no pending write process */
			if (select()) {
				deselect();
				res = RES_OK;
			}
			break;

		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
				if ((csd[0] >> 6) == 1) {	/* SDC ver 2.00 */
					cs= csd[9] + ((WORD)csd[8] << 8) + 1;
					*(DWORD*)buff = (DWORD)cs << 10;
				} else {					/* SDC ver 1.XX or MMC */
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					cs = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
					*(DWORD*)buff = (DWORD)cs << (n - 9);
				}
				res = RES_OK;
			}
			break;

		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */
			*(DWORD*)buff = 128;
			res = RES_OK;
			break;

		default:
			res = RES_PARERR;
	}

	deselect();

	return res;
}

