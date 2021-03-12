/*
 * drivers/mtd/serial-nand.h
 *
 * Copyright(C) TOSHIBA MEMORY CORPORATION 2017 All rights reserved
 *
 * Overview:
 *	Contains standard defines and IDs for NAND flash devices
 *  Based on linux/include/linux/mtd/nand_base.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LINUX_MTD_S_NAND_H
#define __LINUX_MTD_S_NAND_H


#define LP_OPTIONS NAND_SAMSUNG_LP_OPTIONS
#define LP_OPTIONS16 (LP_OPTIONS | NAND_BUSWIDTH_16)

#define SP_OPTIONS NAND_NEED_READRDY
#define SP_OPTIONS16 (SP_OPTIONS | NAND_BUSWIDTH_16)

/*Length of SPI buffer */
#define S_NAND_SPI_BUF_SIZ	256

/* Start: Add by gale 2018-05-14 */
/*pagesize*/
#define NAND_MAX_PAGESIZE       744
/*oobsize*/
#define NAND_MAX_OOBSIZE        8192

/* Search good/bad pattern through all pages of a block */
#define NAND_BBT_SCANALLPAGES   0x00000400
/* Scan block empty during good / bad block scan */
#define NAND_BBT_SCANEMPTY      0x00000800
/*End: Add by gale 2018-05-14 */

//Keep serial_nand_hw_control happy
struct serial_nand_chip;

struct serial_nand_hw_control {
	spinlock_t lock;
	struct serial_nand_chip *active;
	wait_queue_head_t wq;
};


/**
 * struct serial_nand_buffers - buffer structure for read/write
 * @databuf:	buffer for data - dynamically sized
 *
 * Do not change the order of buffers. databuf and oobrbuf must be in
 * consecutive order.
 */
struct serial_nand_buffers {
	uint8_t databuf[NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE];
};


/**
 * struct nand_ecc_ctrl - Control structure for ECC
 * @mode:	ECC mode
 * @steps:	number of ECC steps per page
 * @size:	data bytes per ECC step
 * @bytes:	ECC bytes per step
 * @total:	total number of ECC bytes per page
 * @strength:	max number of correctible bits per ECC step
 * @prepad:	padding information for syndrome based ECC generators
 * @postpad:	padding information for syndrome based ECC generators
 * @layout:	ECC layout control struct pointer
 * @priv:	pointer to private ECC control data
 * @hwctl:	function to control hardware ECC generator. Must only
 *		be provided if an hardware ECC is available
 * @calculate:	function for ECC calculation or readback from ECC hardware
 * @correct:	function for ECC correction, matching to ECC generator (sw/hw)
 * @read_page_raw:	function to read a raw page without ECC
 * @write_page_raw:	function to write a raw page without ECC
 * @read_page:	function to read a page according to the ECC generator
 *		requirements; returns maximum number of bitflips corrected in
 *		any single ECC step, 0 if bitflips uncorrectable, -EIO hw error
 * @read_subpage:	function to read parts of the page covered by ECC;
 *			returns same as read_page()
 * @write_subpage:	function to write parts of the page covered by ECC.
 * @write_page:	function to write a page according to the ECC generator
 *		requirements.
 * @write_oob_raw:	function to write chip OOB data without ECC
 * @read_oob_raw:	function to read chip OOB data without ECC
 * @read_oob:	function to read chip OOB data
 * @write_oob:	function to write chip OOB data
 */
struct serial_nand_ecc_ctrl {
	nand_ecc_modes_t mode;
	int steps;
	int size;
	int bytes;
	int total;
	int strength;
	int prepad;
	int postpad;
	struct nand_ecclayout	*layout;
	void *priv;
	void (*hwctl)(struct mtd_info *mtd, int mode);
	int (*calculate)(struct mtd_info *mtd, const uint8_t *dat,
			uint8_t *ecc_code);
	int (*correct)(struct mtd_info *mtd, uint8_t *dat, uint8_t *read_ecc,
			uint8_t *calc_ecc);
	int (*read_page_raw)(struct mtd_info *mtd, struct serial_nand_chip *chip,
			uint8_t *buf, int oob_required, int page);
	int (*write_page_raw)(struct mtd_info *mtd, struct serial_nand_chip *chip,
			const uint8_t *buf, int oob_required);
	int (*read_page)(struct mtd_info *mtd, struct serial_nand_chip *chip,
			uint8_t *buf, int oob_required, int page);
	int (*read_subpage)(struct mtd_info *mtd, struct serial_nand_chip *chip,
			uint32_t offs, uint32_t len, uint8_t *buf);
	int (*write_subpage)(struct mtd_info *mtd, struct serial_nand_chip *chip,
			uint32_t offset, uint32_t data_len,
			const uint8_t *data_buf, int oob_required);
	int (*write_page)(struct mtd_info *mtd, struct serial_nand_chip *chip,
			const uint8_t *buf, int oob_required);
	int (*write_oob_raw)(struct mtd_info *mtd, struct serial_nand_chip *chip,
			int page);
	int (*read_oob_raw)(struct mtd_info *mtd, struct serial_nand_chip *chip,
			int page);
	int (*read_oob)(struct mtd_info *mtd, struct serial_nand_chip *chip, int page);
	int (*write_oob)(struct mtd_info *mtd, struct serial_nand_chip *chip,
			int page);
};


/**
 * struct serial_nand_chip - NAND Private Flash Chip Data
 * const char			*name;
 * struct spi_device	*spi;
 * struct mtd_info		mtd;
 * @select_chip:	[REPLACEABLE] select chip nr
 * @block_bad:		[REPLACEABLE] check, if the block is bad
 * @block_markbad:	[REPLACEABLE] mark the block bad
 * @dev_ready:		[BOARDSPECIFIC] hardwarespecific function for accessing
 *			device ready/busy line. If set to NULL no access to
 *			ready/busy is available and the ready/busy information
 *			is read from the chip status register.
 * @ecc:		[BOARDSPECIFIC] ECC control structure
 * @buffers:		buffer structure for read/write
 * @hwcontrol:		platform-specific hardware control structure
 * @scan_bbt:		[REPLACEABLE] function to scan bad block table
 * @state:		[INTERN] the current state of the NAND device
 * @oob_poi:		"poison value buffer," used for laying out OOB data
 *			before writing
 * @page_shift:		[INTERN] number of address bits in a page (column
 *			address bits).
 * @phys_erase_shift:	[INTERN] number of address bits in a physical eraseblock
 * @bbt_erase_shift:	[INTERN] number of address bits in a bbt entry
 * @chip_shift:		[INTERN] number of address bits in one chip
 * @options:		[BOARDSPECIFIC] various chip options. They can partly
 *			be set to inform nand_scan about special functionality.
 *			See the defines for further explanation.
 * @bbt_options:	[INTERN] bad block specific options. All options used
 *			here must come from bbm.h. By default, these options
 *			will be copied to the appropriate nand_bbt_descr's.
 * @badblockpos:	[INTERN] position of the bad block marker in the oob
 *			area.
 * @badblockbits:	[INTERN] minimum number of set bits in a good block's
 *			bad block marker position; i.e., BBM == 11110111b is
 *			not bad when badblockbits == 7
 * @numchips:		[INTERN] number of physical chips
 * @chipsize:		[INTERN] the size of one chip for multichip arrays
 * @pagemask:		[INTERN] page number mask = number of (pages / chip) - 1
 * @pagebuf:		[INTERN] holds the pagenumber which is currently in
 *			data_buf.
 * @pagebuf_bitflips:	[INTERN] holds the bitflip count for the page which is
 *			currently in data_buf.
 * @subpagesize:	[INTERN] holds the subpagesize
 * @bbt:		[INTERN] bad block table pointer
 * @bbt_td:		[REPLACEABLE] bad block table descriptor for flash
 *			lookup.
 * @bbt_md:		[REPLACEABLE] bad block table mirror descriptor
 * @badblock_pattern:	[REPLACEABLE] bad block scan pattern used for initial
 *			bad block scan.
 * @controller:		[REPLACEABLE] a pointer to a hardware controller
 *			structure which is shared among multiple independent
 *			devices.
 * @write_page:		[REPLACEABLE] High-level page write function
 */
struct serial_nand_chip {
	const char			*name;
	struct spi_device	*spi;
	struct mtd_info		mtd;

	void (*select_chip)(struct mtd_info *mtd, int chip);
	int (*block_bad)(struct mtd_info *mtd, loff_t ofs, int getchip);
	int (*block_markbad)(struct mtd_info *mtd, loff_t ofs);

	int (*scan_bbt)(struct mtd_info *mtd);
	int (*write_page)(struct mtd_info *mtd, struct serial_nand_chip *chip,
			uint32_t offset, int data_len, const uint8_t *buf,
			int oob_required, int page, int cached, int raw);
	unsigned int options;
	unsigned int bbt_options;
	int page_shift;
	int phys_erase_shift;
	int bbt_erase_shift;
	int chip_shift;
	int numchips;
	uint64_t chipsize;
	int pagemask;
	int pagebuf;
	unsigned int pagebuf_bitflips;
	int subpagesize;
	int badblockpos;
	int badblockbits;
	flstate_t state;
	uint8_t *oob_poi;
	struct serial_nand_hw_control *controller;
	struct serial_nand_ecc_ctrl ecc;
	struct serial_nand_buffers *buffers;
	struct serial_nand_hw_control hwcontrol;
	uint8_t *bbt;
	struct nand_bbt_descr *bbt_td;
	struct nand_bbt_descr *bbt_md;
	struct nand_bbt_descr *badblock_pattern;
};


#define S_NAND_STATUS_P_FAIL		(1 << 3)
#define S_NAND_STATUS_E_FAIL		(1 << 2)
#define S_NAND_STATUS_OIP			(1 << 0)

#define S_NAND_LAST_ROW_INVALID		0xFFFFFFFF
#define	S_NAND_BAD_BLOCK_MARK_OFFSET	0x08

/*
 * Serial NAND commands
 */
#define S_NAND_CMD_READ					0x13
#define S_NAND_CMD_READ_BUFFM			0x03
#define S_NAND_CMD_READ_CACHE_X2		0x3b
#define S_NAND_CMD_READ_CACHE_X4		0x6b

#define S_NAND_CMD_PROG_PAGE_CLRCACHE	0x02
#define S_NAND_CMD_PROG_PAGE			0x84
#define S_NAND_CMD_PROG_PAGE_EXC		0x10
#define S_NAND_CMD_ERASE_BLK			0xd8
#define S_NAND_CMD_WR_ENABLE			0x06
#define S_NAND_CMD_WR_DISABLE			0x04
#define S_NAND_CMD_READ_ID				0x9f
#define S_NAND_CMD_RESET				0xff
#define S_NAND_CMD_READ_REG				0x0f
#define S_NAND_CMD_WRITE_REG			0x1f

struct spi_nand_cmd {
	u8		cmd;
	u32		n_addr;		/* Number of address */
	u32		addr;		/* Reg Offset */
	u32		n_dmy;		/* Number of dmy bytes */
	u32		n_tx;		/* Number of tx bytes */
	u8		*tx_buf;	/* Tx buf */
	u8		tx_nbits;
	u32		n_rx;		/* Number of rx bytes */
	u8		*rx_buf;	/* Rx buf */
	u8		rx_nbits;
};

extern int serial_nand_scan_bbt(struct mtd_info *mtd, struct nand_bbt_descr *bd);
extern int serial_nand_update_bbt(struct mtd_info *mtd, loff_t offs);
extern int serial_nand_default_bbt(struct mtd_info *mtd);
extern int serial_nand_isbad_bbt(struct mtd_info *mtd, loff_t offs, int allowbbt);
extern int serial_nand_erase_nand(struct mtd_info *mtd, struct erase_info *instr,
			   int allowbbt);

#endif /* __LINUX_MTD_S_NAND_H */
