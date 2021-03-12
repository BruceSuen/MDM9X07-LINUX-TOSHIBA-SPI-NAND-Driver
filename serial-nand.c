/*
 * drivers/mtd/serisal-nand.c
 *
 * Copyright(C) TOSHIBA MEMORY CORPORATION 2017 All rights reserved
 *
 * Overview:
 *  This program supports Serial NAND.
 *  Based on drivers/mtd/nand/nand_base.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/leds.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
//matthew.ma-2018/06/05: add spi flash driver as a kernel module
#include <linux/of.h>
#include <linux/of_device.h>


#if IS_ENABLED(CONFIG_SPI_APPLITE_SPIB)
#include <linux/spi/applite-spib.h>
#include <asm/cacheflush.h>
#endif

#include "serial-nand.h"

static int serial_nand_read_buffer_cmd(struct spi_device *spi, int col,
                 int len, const u_char *data);
static int serial_nand_program_load_cmd(struct spi_device *spi, int col,
                 int len, const u_char *data, int clr);
static int serial_nand_get_feature(struct spi_device *spi, int reg,
                 uint8_t *data);
static int serial_nand_set_feature(struct spi_device *spi, int reg,
                 uint8_t *data);
static int serial_nand_wait_ready(struct mtd_info *mtd, uint8_t *data);
static int serial_nand_erase_block_cmd(struct spi_device *spi, uint32_t page);
static int serial_nand_write_enable(struct spi_device *spi);
static int serial_nand_read_cell(struct mtd_info *mtd, uint32_t page);

static int serial_nand_get_device(struct mtd_info *mtd, int new_state);
static int serial_nand_do_write_oob(struct mtd_info *mtd, loff_t to,
                 struct mtd_oob_ops *ops);
static int serial_nand_read_cell_array_cmd(struct spi_device *spi,uint32_t page);


/* Define default oob placement schemes for large and small page devices */
static struct nand_ecclayout serial_nand_oob_64 = {
    .eccbytes = 0,
    .eccpos = { },
    .oobfree = {
        {.offset = 4,
         .length = 60} }
};

static struct nand_ecclayout serial_nand_oob_128 = {
    .eccbytes = 0,
    .eccpos = { },
    .oobfree = {
        {.offset = 4,
         .length = 124} }
};


static int on_flash_bbt = 1;
module_param(on_flash_bbt, int, 0);

/*
 * The chip ID list:
 *    name, device ID, page size, chip size in MiB, eraseblock size, options
 *
 * If page size and eraseblock size are 0, the sizes are taken from the
 * extended chip ID.
 */
struct nand_flash_dev serial_nand_flash_ids[] = {
    /*
     * Some incompatible NAND chips share device ID's and so must be
     * listed by full ID. We list them first so that we can easily identify
     * the most specific match.
     */
    { .name = "TSBWS1G3.3V",
     {.mfr_id = 0x98,
      .dev_id = 0xC2, },
      .pagesize = SZ_2K,
      .chipsize = SZ_128,
      .erasesize = SZ_128K,
      .options = 0,
      .id_len =  2,
      .oobsize = 64},
    { .name = "TSBWS2G3.3V",
     {.mfr_id = 0x98,
      .dev_id = 0xCB, },
      .pagesize = SZ_2K,
      .chipsize = SZ_256,
      .erasesize = SZ_128K,
      .options = 0,
      .id_len =  2,
      .oobsize = 64},
    { .name = "TSBWS4G3.3V",
     {.mfr_id = 0x98,
      .dev_id = 0xCD, },
      .pagesize = SZ_4K,
      .chipsize = SZ_512,
      .erasesize = SZ_256K,
      .options = 0,
      .id_len =  2,
      .oobsize = 128},
    { .name = "TSBWS1G1.8V",
     {.mfr_id = 0x98,
      .dev_id = 0xB2, },
      .pagesize = SZ_2K,
      .chipsize = SZ_128,
      .erasesize = SZ_128K,
      .options = 0,
      .id_len =  2,
      .oobsize = 64},
    { .name = "TSBWS2G1.8V",
     {.mfr_id = 0x98,
      .dev_id = 0xBB, },
      .pagesize = SZ_2K,
      .chipsize = SZ_256,
      .erasesize = SZ_128K,
      .options = 0,
      .id_len =  2,
      .oobsize = 64},
    { .name = "TSBWS4G1.8V",
     {.mfr_id = 0x98,
      .dev_id = 0xBD, },
      .pagesize = SZ_4K,
      .chipsize = SZ_512,
      .erasesize = SZ_256K,
      .options = NAND_NO_SUBPAGE_WRITE,
          .ecc = NAND_ECC_INFO(8,1024),
      .id_len =  2,
      .oobsize = 128},
    { .name = "TC58CYG2S0HRAIJ",
     {.mfr_id = 0x98,
      .dev_id = 0xDD, },
      .pagesize = SZ_4K,
      .chipsize = SZ_512,
      .erasesize = SZ_256K,
      .options = NAND_NO_SUBPAGE_WRITE,
          .ecc = NAND_ECC_INFO(8,1024),
      .id_len =  2,
      .oobsize = 128},
    {NULL}
};


/*
 * For devices which display every fart in the system on a separate LED. Is
 * compiled away when LED support is disabled.
 */
//DEFINE_LED_TRIGGER(serial_nand_led_trigger);

static int check_offs_len(struct mtd_info *mtd,
                    loff_t ofs, uint64_t len)
{
    struct serial_nand_chip *chip = mtd->priv;
    int ret = 0;

    /* Start address must align on block boundary */
    if (ofs & ((1 << chip->phys_erase_shift) - 1)) {
        pr_debug("%s: unaligned address\n", __func__);
        ret = -EINVAL;
    }

    /* Length must align on block boundary */
    if (len & ((1 << chip->phys_erase_shift) - 1)) {
        pr_debug("%s: length not block aligned\n", __func__);
        ret = -EINVAL;
    }

    return ret;
}

/**
 * serial_nand_release_device - [GENERIC] release chip
 * @mtd: MTD device structure
 *
 * Release chip lock and wake up anyone waiting on the device.
 */
static void serial_nand_release_device(struct mtd_info *mtd)
{
    struct serial_nand_chip *chip = mtd->priv;

    /* Release the controller and the chip */
    spin_lock(&chip->controller->lock);
    chip->controller->active = NULL;
    chip->state = FL_READY;
    wake_up(&chip->controller->wq);
    spin_unlock(&chip->controller->lock);
}


//Internal fumction
static int serial_nand_command(struct spi_device *spi, struct spi_nand_cmd *cmd)
{
    struct spi_message message;
    struct spi_transfer x[3];
    uint8_t cmdbuf[5];
    int        i, index = 0;

    spi_message_init(&message);
    memset(x, 0, sizeof(x));

    cmdbuf[index++] = cmd->cmd;
    if (cmd->n_addr) {
        for(i = cmd->n_addr; i > 0; i--) {
            cmdbuf[index++] = (uint8_t)(cmd->addr >> 8*(i - 1));
        }
    }
    if (cmd->n_dmy) {
        for(i = cmd->n_dmy; i > 0; i--) {
            cmdbuf[index++] = (uint8_t)0x00;
        }
    }
    x[0].len = index;
    x[0].tx_buf = cmdbuf;
    spi_message_add_tail(&x[0], &message);

    if (cmd->n_tx) {
        x[1].len = cmd->n_tx;
        x[1].tx_nbits = cmd->tx_nbits;
        x[1].tx_buf = cmd->tx_buf;
        spi_message_add_tail(&x[1], &message);
    }

    if (cmd->n_rx) {
        x[2].len = cmd->n_rx;
        x[2].rx_nbits = cmd->rx_nbits;
        x[2].rx_buf = cmd->rx_buf;
        spi_message_add_tail(&x[2], &message);
    }
    return spi_sync(spi, &message);
}

static int serial_nand_reset(struct spi_device *spi)
{
    struct spi_nand_cmd cmd = {0};
    cmd.cmd = S_NAND_CMD_RESET;
    return serial_nand_command(spi, &cmd);
}

static int serial_nand_raed_id(struct spi_device *spi,  uint8_t *data)
{
    struct spi_nand_cmd cmd = {0};
    cmd.cmd = S_NAND_CMD_READ_ID;
    cmd.n_dmy = 1;                /* Number of dummy */
    cmd.rx_buf = data;            /* Rx buf */
    cmd.n_rx = 2;                /* Number of rx bytes */
    return  serial_nand_command(spi, &cmd);
}

/*
 * nand_spi_er_get_feature
 *    Get Feature register
 */
static int serial_nand_get_feature(struct spi_device *spi, int reg, uint8_t *data)
{
    struct spi_nand_cmd cmd = {0};
    cmd.cmd = S_NAND_CMD_READ_REG;
    cmd.addr = reg;    
    cmd.n_addr = 1;                /* Number of address */
    cmd.rx_buf = data;            /* Rx buf */
    cmd.n_rx = 1;                /* Number of rx bytes */
    return serial_nand_command(spi, &cmd);
}

static int serial_nand_set_feature(struct spi_device *spi, int reg, uint8_t *data)
{
    struct spi_nand_cmd cmd = {0};
    cmd.cmd = S_NAND_CMD_WRITE_REG;
    cmd.addr = reg;    
    cmd.n_addr = 1;                /* Number of address */
    cmd.tx_buf = data;            /* Tx buf */
    cmd.n_tx = 1;                /* Number of rx bytes */
    return serial_nand_command(spi, &cmd);
}

static int serial_nand_erase_block_cmd(struct spi_device *spi, uint32_t page)
{
    struct spi_nand_cmd cmd = {0};
    cmd.cmd = S_NAND_CMD_ERASE_BLK;
    cmd.addr = page;    
    cmd.n_addr = 3;                /* Number of address */
    return serial_nand_command(spi, &cmd);
}

static int serial_nand_program_load_cmd(struct spi_device *spi, int col, int len, const u_char *data, int clr)
{
    struct spi_nand_cmd cmd = {0};
    if(clr) {
        cmd.cmd = S_NAND_CMD_PROG_PAGE_CLRCACHE;
    } else {
        cmd.cmd = S_NAND_CMD_PROG_PAGE;
    }
    cmd.addr = col;    
    cmd.n_addr = 2;                /* Number of address */
    cmd.tx_buf = (u8 *)data;    /* Rx buf */
    cmd.n_tx = len;                /* Number of rx bytes */
    return serial_nand_command(spi, &cmd);
}

static int serial_nand_write_page_load(struct spi_device *spi, int col, int len, const u_char *buf, int clear)
{
    int    wl, i, res;
    for(i = 0; len > 0; len -= S_NAND_SPI_BUF_SIZ, col += S_NAND_SPI_BUF_SIZ, i++)
    {
        if(len > S_NAND_SPI_BUF_SIZ) {
            wl = S_NAND_SPI_BUF_SIZ;
        } else {
            wl = len;
        }
        res = serial_nand_program_load_cmd(spi, col, wl, &buf[i*S_NAND_SPI_BUF_SIZ], clear);
        clear = 0;
    }
    return res;
}

static int serial_nand_program_execute_cmd(struct spi_device *spi, int page)
{
    struct spi_nand_cmd cmd = {0};
    cmd.cmd = S_NAND_CMD_PROG_PAGE_EXC;
    cmd.addr = page;    
    cmd.n_addr = 3;                /* Number of address */
    return serial_nand_command(spi, &cmd);
}


static int serial_nand_read_buffer_cmd(struct spi_device *spi, int col, int len, const u_char *data)
{
    struct spi_nand_cmd cmd = {0};
    if(spi->mode & SPI_RX_QUAD) {
        cmd.rx_nbits = 4;
        cmd.cmd = S_NAND_CMD_READ_CACHE_X4;
    } else if(spi->mode & SPI_RX_DUAL) {
        cmd.rx_nbits = 2;
        cmd.cmd = S_NAND_CMD_READ_CACHE_X2;
    } else {
        cmd.rx_nbits = 1;
        cmd.cmd = S_NAND_CMD_READ_BUFFM;
    }
    cmd.addr = col;                /* For col 2Byte */
    cmd.n_addr = 2;                /* Number of address */
    cmd.n_dmy = 1;                /* Number of dummy */
    cmd.rx_buf = (u8 *)data;    /* Rx buf */
    cmd.n_rx = len;                /* Number of rx bytes */
    return serial_nand_command(spi, &cmd);
}

static int serial_nand_read_page(struct spi_device *spi, int col, int len, const u_char *buf)
{
    int    wl, i, res;

    for(i = 0; len > 0; len -= S_NAND_SPI_BUF_SIZ, col += S_NAND_SPI_BUF_SIZ, i++)
    {
        if(len > S_NAND_SPI_BUF_SIZ) {
            wl = S_NAND_SPI_BUF_SIZ;
        } else {
            wl = len;
        }
        res = serial_nand_read_buffer_cmd(spi, col, wl, &buf[i*S_NAND_SPI_BUF_SIZ]);
        if (res) {
            return res;
        }
    }
    return res;
}

static int serial_nand_read_cell_array_cmd(struct spi_device *spi,uint32_t page)
{
    struct spi_nand_cmd cmd = {0};
    cmd.cmd = S_NAND_CMD_READ;
    cmd.addr = page;    
    cmd.n_addr = 3;                /* Number of address */
    return serial_nand_command(spi, &cmd);
}
/*
 * serial_nand_write_enable
 *    Enable serial NAND access
 */
static int serial_nand_write_enable(struct spi_device *spi)
{
    struct spi_nand_cmd cmd = {0};
    cmd.cmd = S_NAND_CMD_WR_ENABLE;
    return serial_nand_command(spi, &cmd);
}


/*
 * nand_spi_er_busywait
 *    Wait until the chip is not busy
 */
/* Wait for the ready pin, after a command. The timeout is caught later. */
static int serial_nand_wait_ready(struct mtd_info *mtd, uint8_t *data)
{
    struct serial_nand_chip *chip = mtd->priv;
    struct spi_device *spi = chip->spi;
    int res;
    unsigned long timeo;

    switch(chip->state) {
    case FL_ERASING:
        timeo = 400;        // Timeout valu is 400mSec
        break;
    case FL_WRITING:
        timeo = 200;        // Timeout valu is 200mSec
        break;
    default:
        timeo = 40;           
        break;
    }

//modify by [francis],20181116,make sure get status from spi-flash
        timeo = jiffies + msecs_to_jiffies(timeo);
        while (time_before(jiffies, timeo)) {
            res = serial_nand_get_feature(spi, 0xC0, data);
            if (res) {
                return res;
            }
            if (!(*data & S_NAND_STATUS_OIP))
                break;
        }
        
        if(!time_before(jiffies, timeo)) {
            res = serial_nand_get_feature(spi, 0xC0, data);
            if (res) {
                return res;
            }
        }
        
    return 0;
}


/*
 * serial_nand_read_cell
 *    Read page data from cell to internal buffer
 */
static int serial_nand_read_cell(struct mtd_info *mtd, uint32_t page)
{
    struct serial_nand_chip *chip = mtd->priv;
    struct spi_device *spi = chip->spi;
    uint8_t stat;
    int ret;
    unsigned int max_bitflips = 0;

    /*
     * Load the appropriate page
     */
    ret = serial_nand_read_cell_array_cmd(spi, page);
    if (ret) {
        pr_err("%s: failed page load ret=%d\n", chip->name, ret);
        return ret;
    }

    /*
     * Wait
     */
    ret = serial_nand_wait_ready(mtd, &stat);
    if (ret || (stat & S_NAND_STATUS_OIP)) {
        if (ret) 
            return ret;

        /*
         * Chip is stuck?
         */
        return -EIO;
    }

    /*
     * Check the ECC bits
     */
    stat = (stat >> 4) & 0x03;
    if ((stat == 0x03) || (stat == 0x1)) {
        u8 cmd;
        uint8_t stat2, corrected;
        int i, eccsteps;

        cmd = 0x40;
        eccsteps = chip->ecc.steps;
                //eccsteps=4;
        pr_info("%s: ECC recovered, page=%x eccsteps=%d ecc.size=%d ecc.bytes\n", chip->name, page,eccsteps,chip->ecc.size,chip->ecc.bytes);

        for (i = 0 ; eccsteps ; eccsteps -= 1, cmd += 0x10) {
            ret = serial_nand_get_feature(spi, cmd, &stat2);
            if (ret) {
                mtd->ecc_stats.failed++;
                pr_err("%s: time out (serial_nand_get_feature) res %d\n", __func__, ret);
                continue;
            }
            if(!stat2)
            {
                //pr_err("%s: stat2=%02X eccsteps=%d\n",__func__,stat2,eccsteps);
                continue;
            }
            corrected = stat2 & 0x0f;
             if(corrected != 0xf)
            {
                mtd->ecc_stats.corrected += corrected;
            }
            max_bitflips = max_t(unsigned int, max_bitflips, corrected);
            corrected = stat2 >> 4;
            if(corrected != 0xf)
            {
                mtd->ecc_stats.corrected += corrected;
            }
            max_bitflips = max_t(unsigned int, max_bitflips, corrected);
        }
    } else if (stat == 0x02) {
        pr_err("%s: failed ECC, page=%d\n", chip->name, page);
        mtd->ecc_stats.failed++;
    }
    return max_bitflips;
}

/*
 * serial_nand_program_execute
 *    Write page data from internal buffer to cell
 */
static int serial_nand_program_execute(struct mtd_info *mtd, uint32_t page, uint8_t *stat)
{
    struct serial_nand_chip *chip = mtd->priv;
    struct spi_device *spi = chip->spi;
    int status;

    /*
     * Program execute
     */
    status = serial_nand_program_execute_cmd(spi, page);
    if (status) {
        pr_err("%s: failed prog execute status=%d\n", chip->name, status);
        return -EIO;
    }

    /*
     * Wait
     */
    status = serial_nand_wait_ready(mtd, stat);
    if (status || (*stat & S_NAND_STATUS_OIP)) {
        if (status) {
            return -EIO;
        }

        /*
         * Chip is stuck?
         */
        return -EIO;
    }
    if (*stat & (1 << 3)) {
        pr_err("%s: Program fasiler status=%d stat=%02x\n", chip->name, status, *stat);
            return -EIO;
    }
    return 0;
}

/**
 * serial_nand_select_chip - [DEFAULT] control CE line
 * @mtd: MTD device structure
 * @chipnr: chipnumber to select, -1 for deselect
 *
 * Default select function for 1 chip devices.
 */
static void serial_nand_select_chip(struct mtd_info *mtd, int chipnr)
{

    switch (chipnr) {
    case -1:
        break;
    case 0:
        break;

    default:
        BUG();
    }
}


/**
 * serial_nand_block_bad - [DEFAULT] Read bad block marker from the chip
 * @mtd: MTD device structure
 * @ofs: offset from device start
 * @getchip: 0, if the chip is already selected
 *
 * Check, if the block is bad.
 */
static int serial_nand_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
    int page, chipnr, res = 0, i = 0;
    struct serial_nand_chip *chip = mtd->priv;
    struct spi_device *spi = chip->spi;
    unsigned char bad;

    if (chip->bbt_options & NAND_BBT_SCANLASTPAGE)
        ofs += mtd->erasesize - mtd->writesize;

    page = (int)(ofs >> chip->page_shift) & chip->pagemask;

    if (getchip) {
        chipnr = (int)(ofs >> chip->chip_shift);

        serial_nand_get_device(mtd, FL_READING);

        /* Select the NAND device */
        chip->select_chip(mtd, chipnr);
    }

    do {
        res = serial_nand_read_cell(mtd, page);
        if (res < 0) {
            break;
        }
        res = serial_nand_read_page(spi, mtd->writesize, 1, &bad);
        if (res < 0) 
            break;

        if (likely(chip->badblockbits == 8))
            res = bad != 0xFF;
        else
            res = hweight8(bad) < chip->badblockbits;
        ofs += mtd->writesize;
        page = (int)(ofs >> chip->page_shift) & chip->pagemask;
        i++;
    } while (!res && i < 2 && (chip->bbt_options & NAND_BBT_SCAN2NDPAGE));

    if (getchip) {
        chip->select_chip(mtd, -1);
        serial_nand_release_device(mtd);
    }

    return res;
}

/**
 * serial_nand_default_block_markbad - [DEFAULT] mark a block bad
 * @mtd: MTD device structure
 * @ofs: offset from device start
 *
 * This is the default implementation, which can be overridden by a hardware
 * specific driver. We try operations in the following order, according to our
 * bbt_options (NAND_BBT_NO_OOB_BBM and NAND_BBT_USE_FLASH):
 *  (1) erase the affected block, to allow OOB marker to be written cleanly
 *  (2) update in-memory BBT
 *  (3) write bad block marker to OOB area of affected block
 *  (4) update flash-based BBT
 * Note that we retain the first error encountered in (3) or (4), finish the
 * procedures, and dump the error in the end.
*/
static int serial_nand_default_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
    struct serial_nand_chip *chip = mtd->priv;
    uint8_t buf[2] = { 0, 0 };
    int block, res, ret = 0, i = 0;
    int write_oob = !(chip->bbt_options & NAND_BBT_NO_OOB_BBM);

    if (write_oob) {
        struct erase_info einfo;

        /* Attempt erase before marking OOB */
        memset(&einfo, 0, sizeof(einfo));
        einfo.mtd = mtd;
        einfo.addr = ofs;
        einfo.len = 1 << chip->phys_erase_shift;
        serial_nand_erase_nand(mtd, &einfo, 0);
    }

    /* Get block number */
    block = (int)(ofs >> chip->bbt_erase_shift);
    /* Mark block bad in memory-based BBT */
    if (chip->bbt)
        chip->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);

//modify by [francis],20181116,update bbt first ,then try to write oob
    /* Update flash-based bad block table */
    if (chip->bbt_options & NAND_BBT_USE_FLASH) {
        res = serial_nand_update_bbt(mtd, ofs);
        if (!ret)
            ret = res;
    }

    /* Write bad block marker to OOB */
    if (write_oob) {
        struct mtd_oob_ops ops;
        loff_t wr_ofs = ofs;

        serial_nand_get_device(mtd, FL_WRITING);

        ops.datbuf = NULL;
        ops.oobbuf = buf;
        ops.ooboffs = chip->badblockpos;
        ops.len = ops.ooblen = 1;
        ops.mode = MTD_OPS_PLACE_OOB;

        /* Write to first/last page(s) if necessary */
        if (chip->bbt_options & NAND_BBT_SCANLASTPAGE)
            wr_ofs += mtd->erasesize - mtd->writesize;
        do {
            res = serial_nand_do_write_oob(mtd, wr_ofs, &ops);
            i++;
            wr_ofs += mtd->writesize;
			pr_info("chip->bbt_options=%d i=%d  \n ",chip->bbt_options,i);
        } while ((chip->bbt_options & NAND_BBT_SCAN2NDPAGE) && i < 2);

        serial_nand_release_device(mtd);
    }


    if (!ret)
        mtd->ecc_stats.badblocks++;

    return ret;
}

/**
 * serial_nand_check_wp - [GENERIC] check if the chip is write protected
 * @mtd: MTD device structure
 *
 * Check, if the device is write protected. The function expects, that the
 * device is already selected.
 */
static int serial_nand_check_wp(struct mtd_info *mtd)
{
    return 0;    // Always not protection
}

/**
 * serial_nand_block_checkbad - [GENERIC] Check if a block is marked bad
 * @mtd: MTD device structure
 * @ofs: offset from device start
 * @getchip: 0, if the chip is already selected
 * @allowbbt: 1, if its allowed to access the bbt area
 *
 * Check, if the block is bad. Either by reading the bad block table or
 * calling of the scan function.
 */
static int serial_nand_block_checkbad(struct mtd_info *mtd, loff_t ofs, int getchip,
                   int allowbbt)
{
    struct serial_nand_chip *chip = mtd->priv;

    if (!chip->bbt)
        return chip->block_bad(mtd, ofs, getchip);
            //chip->block_bad = serial_nand_block_bad; set in serial_nand_set_defaults( )
    /* Return info from the table */
    return serial_nand_isbad_bbt(mtd, ofs, allowbbt);
}


/**
 * serial_nand_get_device - [GENERIC] Get chip for selected access
 * @mtd: MTD device structure
 * @new_state: the state which is requested
 *
 * Get the device and lock it for exclusive access
 */
static int
serial_nand_get_device(struct mtd_info *mtd, int new_state)
{
    struct serial_nand_chip *chip = mtd->priv;
    spinlock_t *lock = &chip->controller->lock;
    wait_queue_head_t *wq = &chip->controller->wq;
    DECLARE_WAITQUEUE(wait, current);
retry:
    spin_lock(lock);

    /* Hardware controller shared among independent devices */
    if (!chip->controller->active)
        chip->controller->active = chip;

    if (chip->controller->active == chip && chip->state == FL_READY) {
        chip->state = new_state;
        spin_unlock(lock);
        return 0;
    }
    if (new_state == FL_PM_SUSPENDED) {
        if (chip->controller->active->state == FL_PM_SUSPENDED) {
            chip->state = FL_PM_SUSPENDED;
            spin_unlock(lock);
            return 0;
        }
    }
    set_current_state(TASK_UNINTERRUPTIBLE);
    add_wait_queue(wq, &wait);
    spin_unlock(lock);
    schedule();
    remove_wait_queue(wq, &wait);
    goto retry;
}


/**
 * serial_nand_read_page_raw - [INTERN] read raw page data without ecc
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: buffer to store read data
 * @oob_required: caller requires OOB data read to chip->oob_poi
 * @page: page number to read
 *
 * Not for syndrome calculating ECC controllers, which use a special oob layout.
 */
static int serial_nand_read_page_raw(struct mtd_info *mtd, struct serial_nand_chip *chip,
                  uint8_t *buf, int oob_required, int page)
{
    struct spi_device *spi = chip->spi;
    int ret = 0;
    ret = serial_nand_read_page(spi, 0, mtd->writesize, buf);
    if(ret) 
        return ret;
    if (oob_required)
        ret = serial_nand_read_page(spi, mtd->writesize, mtd->oobsize, chip->oob_poi);
    return ret;
}

/**
 * serial_nand_transfer_oob - [INTERN] Transfer oob to client buffer
 * @chip: nand chip structure
 * @oob: oob destination address
 * @ops: oob ops structure
 * @len: size of oob to transfer
 */
static uint8_t *serial_nand_transfer_oob(struct serial_nand_chip *chip, uint8_t *oob,
                  struct mtd_oob_ops *ops, size_t len)
{
    switch (ops->mode) {

    case MTD_OPS_PLACE_OOB:
    case MTD_OPS_RAW:
        memcpy(oob, chip->oob_poi + ops->ooboffs, len);
        return oob + len;

    case MTD_OPS_AUTO_OOB: {
        struct nand_oobfree *free = chip->ecc.layout->oobfree;
        uint32_t boffs = 0, roffs = ops->ooboffs;
        size_t bytes = 0;

        for (; free->length && len; free++, len -= bytes) {
            /* Read request not from offset 0? */
            if (unlikely(roffs)) {
                if (roffs >= free->length) {
                    roffs -= free->length;
                    continue;
                }
                boffs = free->offset + roffs;
                bytes = min_t(size_t, len,
                          (free->length - roffs));
                roffs = 0;
            } else {
                bytes = min_t(size_t, len, free->length);
                boffs = free->offset;
            }
            memcpy(oob, chip->oob_poi + boffs, bytes);
            oob += bytes;
        }
        return oob;
    }
    default:
        BUG();
    }
    return NULL;
}

/**
 * serial_nand_do_read_ops - [INTERN] Read data with ECC
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob ops structure
 *
 * Internal function. Called with chip held.
 */
static int serial_nand_do_read_ops(struct mtd_info *mtd, loff_t from,
                struct mtd_oob_ops *ops)
{
    int chipnr, page, realpage, col, bytes, aligned, oob_required;
    struct serial_nand_chip *chip = mtd->priv;
    struct mtd_ecc_stats stats;
    int ret = 0;
    uint32_t readlen = ops->len;
    uint32_t oobreadlen = ops->ooblen;
    uint32_t max_oobsize = ops->mode == MTD_OPS_AUTO_OOB ?
        mtd->oobavail : mtd->oobsize;

    uint8_t *bufpoi, *oob, *buf;
    unsigned int max_bitflips = 0;

    stats = mtd->ecc_stats;

    chipnr = (int)(from >> chip->chip_shift);
    chip->select_chip(mtd, chipnr);

    realpage = (int)(from >> chip->page_shift);
    page = realpage & chip->pagemask;

    col = (int)(from & (mtd->writesize - 1));

    buf = ops->datbuf;
    oob = ops->oobbuf;
    oob_required = oob ? 1 : 0;


    while (1) {
        bytes = min(mtd->writesize - col, readlen);
        aligned = (bytes == mtd->writesize);

        /* Is the current page in the buffer? */
        if (realpage != chip->pagebuf || oob) {
            bufpoi = aligned ? buf : chip->buffers->databuf;
            /*
             * Page read operation
             */
            ret = serial_nand_read_cell(mtd, page);
            if (ret < 0) {
                if (!aligned)
                    /* Invalidate page cache */
                    chip->pagebuf = -1;
                break;
            }
            /*
             * Now read the page into the buffer.  Absent an error,
             * the read methods return max bitflips per ecc step.
             */
            ret = chip->ecc.read_page_raw(mtd, chip, bufpoi, oob_required, page);
            if (ret < 0) {
                if (!aligned)
                    /* Invalidate page cache */
                    chip->pagebuf = -1;
                break;
            }

            max_bitflips = max_t(unsigned int, max_bitflips, ret);

            /* Transfer not aligned data */
            if (!aligned) {
                if (!NAND_HAS_SUBPAGE_READ(chip) && !oob &&
                    !(mtd->ecc_stats.failed - stats.failed) &&
                    (ops->mode != MTD_OPS_RAW)) {
                    chip->pagebuf = realpage;
                    chip->pagebuf_bitflips = ret;
                } else {
                    /* Invalidate page cache */
                    chip->pagebuf = -1;
                }
                memcpy(buf, chip->buffers->databuf + col, bytes);
            }

            buf += bytes;

            if (unlikely(oob)) {
                int toread = min(oobreadlen, max_oobsize);

                if (toread) {
                    oob = serial_nand_transfer_oob(chip,
                        oob, ops, toread);
                    oobreadlen -= toread;
                }
            }
        } else {
            memcpy(buf, chip->buffers->databuf + col, bytes);
            buf += bytes;
            max_bitflips = max_t(unsigned int, max_bitflips,
                         chip->pagebuf_bitflips);
        }

        readlen -= bytes;

        if (!readlen)
            break;

        /* For subsequent reads align to page boundary */
        col = 0;
        /* Increment page address */
        realpage++;

        page = realpage & chip->pagemask;
        /* Check, if we cross a chip boundary */
        if (!page) {
            chipnr++;
            chip->select_chip(mtd, -1);
            chip->select_chip(mtd, chipnr);
        }
    }
    chip->select_chip(mtd, -1);

    ops->retlen = ops->len - (size_t) readlen;
    if (oob)
        ops->oobretlen = ops->ooblen - oobreadlen;

    if (ret < 0)
        return ret;

    if (mtd->ecc_stats.failed - stats.failed)
        return -EBADMSG;

    return max_bitflips;
}

/**
 * serial_nand_read - [MTD Interface] MTD compatibility function for serial_nand_do_read_ecc
 * @mtd: MTD device structure
 * @from: offset to read from
 * @len: number of bytes to read
 * @retlen: pointer to variable to store the number of read bytes
 * @buf: the databuffer to put data
 *
 * Get hold of the chip and call serial_nand_do_read.
 */
static int serial_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
             size_t *retlen, uint8_t *buf)
{
    struct mtd_oob_ops ops;
    int ret;

    unsigned char *bounce_buf = NULL;
    bounce_buf = kmalloc(len, GFP_KERNEL);

    serial_nand_get_device(mtd, FL_READING);
    ops.len = len;
    ops.datbuf = bounce_buf;
    ops.oobbuf = NULL;
    ops.mode = MTD_OPS_PLACE_OOB;
    ret = serial_nand_do_read_ops(mtd, from, &ops);
    *retlen = ops.retlen;
    memcpy(buf, bounce_buf, ops.retlen);
    kfree(bounce_buf);
    serial_nand_release_device(mtd);
    return ret;
}


/**
 * serial_nand_read_oob_std - [REPLACEABLE] the most common OOB data read function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @page: page number to read
 */
static int serial_nand_read_oob_std(struct mtd_info *mtd, struct serial_nand_chip *chip,
                 int page)
{
    struct spi_device *spi = chip->spi;
    int ret = 0;

    /*
     * Page read operation
     */
    ret = serial_nand_read_cell(mtd, page);
    if (ret) {
        pr_err("%s: %s: failed data read page=%x ret=%d\n", __func__, chip->name, page, ret);
        return ret;
    }

    ret = serial_nand_read_page(spi, mtd->writesize, mtd->oobsize, chip->oob_poi);
    if (ret) {
        pr_err("%s: %s: failed data read page=%x ret=%d\n", __func__, chip->name, page, ret);
        return ret;
    }
    return 0;
}


/**
 * serial_nand_write_oob_std - [REPLACEABLE] the most common OOB data write function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @page: page number to write
 */
static int serial_nand_write_oob_std(struct mtd_info *mtd, struct serial_nand_chip *chip,
                  int page)
{
//    int status, subpage;
    int status;
    uint8_t stat;
    struct spi_device *spi = chip->spi;

    /*
     * Write enable
     */
    status = serial_nand_write_enable(spi);
    if (status) {
        pr_err("%s: failed write enable status=%d\n", chip->name, status);
        return status;
    }

    status = serial_nand_write_page_load(spi, mtd->writesize, mtd->oobsize, chip->oob_poi, 1);

    if (status < 0)
    {
        pr_err("%s: fail page load ",__func__);
        return status;
    }

    status = serial_nand_program_execute(mtd, page, &stat);
    if (status) {
        pr_err("%s: fail excute ",__func__);
        return -EIO;
    }

    return status;
}

/**
 * serial_nand_do_read_oob - [INTERN] NAND read out-of-band
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob operations description structure
 *
 * NAND read out-of-band data from the spare area.
 */
static int serial_nand_do_read_oob(struct mtd_info *mtd, loff_t from,
                struct mtd_oob_ops *ops)
{
    int page, realpage, chipnr;
    struct serial_nand_chip *chip = mtd->priv;
    struct mtd_ecc_stats stats;
    int readlen = ops->ooblen;
    int len;
    uint8_t *buf = ops->oobbuf;
    int ret = 0;

    pr_debug("%s: from = 0x%08Lx, len = %i\n",
            __func__, (unsigned long long)from, readlen);

    stats = mtd->ecc_stats;

    if (ops->mode == MTD_OPS_AUTO_OOB)
        len = chip->ecc.layout->oobavail;
    else
        len = mtd->oobsize;

    if (unlikely(ops->ooboffs >= len)) {
        pr_debug("%s: attempt to start read outside oob\n",
                __func__);
        return -EINVAL;
    }

    /* Do not allow reads past end of device */
    if (unlikely(from >= mtd->size ||
             ops->ooboffs + readlen > ((mtd->size >> chip->page_shift) -
                    (from >> chip->page_shift)) * len)) {
        pr_debug("%s: attempt to read beyond end of device\n",
                __func__);
        return -EINVAL;
    }

    chipnr = (int)(from >> chip->chip_shift);
    chip->select_chip(mtd, chipnr);

    /* Shift to get page */
    realpage = (int)(from >> chip->page_shift);
    page = realpage & chip->pagemask;

    while (1) {
        if (ops->mode == MTD_OPS_RAW)
            ret = chip->ecc.read_oob_raw(mtd, chip, page);
        else
            ret = chip->ecc.read_oob(mtd, chip, page);

        if (ret < 0)
            break;

        len = min(len, readlen);
        buf = serial_nand_transfer_oob(chip, buf, ops, len);

        readlen -= len;
        if (!readlen)
            break;

        /* Increment page address */
        realpage++;

        page = realpage & chip->pagemask;
        /* Check, if we cross a chip boundary */
        if (!page) {
            chipnr++;
            chip->select_chip(mtd, -1);
            chip->select_chip(mtd, chipnr);
        }
    }
    chip->select_chip(mtd, -1);

    ops->oobretlen = ops->ooblen - readlen;

    if (ret < 0)
        return ret;

    if (mtd->ecc_stats.failed - stats.failed)
        return -EBADMSG;

    return mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0;
}

/**
 * serial_nand_read_oob - [MTD Interface] NAND read data and/or out-of-band
 * @mtd: MTD device structure
 * @from: offset to read from
 * @ops: oob operation description structure
 *
 * NAND read data and/or out-of-band data.
 */
static int serial_nand_read_oob(struct mtd_info *mtd, loff_t from,
             struct mtd_oob_ops *ops)
{
    int ret = -ENOTSUPP;

    ops->retlen = 0;

    /* Do not allow reads past end of device */
    if (ops->datbuf && (from + ops->len) > mtd->size) {
        pr_debug("%s: attempt to read beyond end of device\n",
                __func__);
        return -EINVAL;
    }

    serial_nand_get_device(mtd, FL_READING);

    switch (ops->mode) {
    case MTD_OPS_PLACE_OOB:
    case MTD_OPS_AUTO_OOB:
    case MTD_OPS_RAW:
        break;

    default:
        goto out;
    }

    if (!ops->datbuf)
        ret = serial_nand_do_read_oob(mtd, from, ops);
    else
        ret = serial_nand_do_read_ops(mtd, from, ops);

out:
    serial_nand_release_device(mtd);
    return ret;
}

/**
 * serial_nand_write_page_raw - [INTERN] raw page write function
 * @mtd: mtd info structure
 * @chip: nand chip info structure
 * @buf: data buffer
 * @oob_required: must write chip->oob_poi to OOB
 *
 * Not for syndrome calculating ECC controllers, which use a special oob layout.
 */
static int serial_nand_write_page_raw(struct mtd_info *mtd, struct serial_nand_chip *chip,
                const uint8_t *buf, int oob_required)
{
    struct spi_device *spi = chip->spi;
    int    res;

    res = serial_nand_write_page_load(spi, 0, mtd->writesize, buf, 1);
    if(res)
	{
		pr_info("%s: end\n",__func__);
        return res;
	}

    if (oob_required)
        res = serial_nand_write_page_load(spi, mtd->writesize, mtd->oobsize, chip->oob_poi, 0);

    return res;
}

/**
 * serial_nand_write_page - [REPLACEABLE] write one page
 * @mtd: MTD device structure
 * @chip: NAND chip descriptor
 * @offset: address offset within the page
 * @data_len: length of actual data to be written
 * @buf: the data to write
 * @oob_required: must write chip->oob_poi to OOB
 * @page: page number to write
 * @cached: cached programming
 * @raw: use _raw version of write_page
 */
static int serial_nand_write_page(struct mtd_info *mtd, struct serial_nand_chip *chip,
        uint32_t offset, int data_len, const uint8_t *buf,
        int oob_required, int page, int cached, int raw)
{
//    int status, subpage;
    int status;
    uint8_t stat;
    struct spi_device *spi = chip->spi;

    status = serial_nand_write_enable(spi);
    if (status) {
        pr_err("%s: failed write enable status=%d\n", chip->name, status);
        return status;
    }

    status = chip->ecc.write_page_raw(mtd, chip, buf, oob_required);
    if (status < 0)
		{
		
		pr_err("%s: page write failed,status=%d\n",__func__,status);
        return status;
		}

    status = serial_nand_program_execute(mtd, page, &stat);
    if (status) {
        return -EIO;
    }

    return 0;
}


/**
 * serial_nand_fill_oob - [INTERN] Transfer client buffer to oob
 * @mtd: MTD device structure
 * @oob: oob data buffer
 * @len: oob data write length
 * @ops: oob ops structure
 */
static uint8_t *serial_nand_fill_oob(struct mtd_info *mtd, uint8_t *oob, size_t len,
                  struct mtd_oob_ops *ops)
{
    struct serial_nand_chip *chip = mtd->priv;

    /*
     * Initialise to all 0xFF, to avoid the possibility of left over OOB
     * data from a previous OOB read.
     */
    memset(chip->oob_poi, 0xff, mtd->oobsize);

    switch (ops->mode) {
    case MTD_OPS_PLACE_OOB:
    case MTD_OPS_RAW:
        memcpy(chip->oob_poi + ops->ooboffs, oob, len);
        return oob + len;

    case MTD_OPS_AUTO_OOB: {
        struct nand_oobfree *free = chip->ecc.layout->oobfree;
        uint32_t boffs = 0, woffs = ops->ooboffs;
        size_t bytes = 0;

        for (; free->length && len; free++, len -= bytes) {
            /* Write request not from offset 0? */
            if (unlikely(woffs)) {
                if (woffs >= free->length) {
                    woffs -= free->length;
                    continue;
                }
                boffs = free->offset + woffs;
                bytes = min_t(size_t, len,
                          (free->length - woffs));
                woffs = 0;
            } else {
                bytes = min_t(size_t, len, free->length);
                boffs = free->offset;
            }
            memcpy(chip->oob_poi + boffs, oob, bytes);
            oob += bytes;
        }
        return oob;
    }
    default:
        BUG();
    }
    return NULL;
}

#define NOTALIGNED(x)    ((x & (chip->subpagesize - 1)) != 0)

/**
 * serial_nand_do_write_ops - [INTERN] NAND write with ECC
 * @mtd: MTD device structure
 * @to: offset to write to
 * @ops: oob operations description structure
 *
 * NAND write with ECC.
 */
static int serial_nand_do_write_ops(struct mtd_info *mtd, loff_t to,
                 struct mtd_oob_ops *ops)
{
    int chipnr, realpage, page, blockmask, column;
    struct serial_nand_chip *chip = mtd->priv;
    uint32_t writelen = ops->len;

    uint32_t oobwritelen = ops->ooblen;
    uint32_t oobmaxlen = ops->mode == MTD_OPS_AUTO_OOB ?
                mtd->oobavail : mtd->oobsize;

    uint8_t *oob = ops->oobbuf;
    uint8_t *buf = ops->datbuf;
    int ret;
    int oob_required = oob ? 1 : 0;

    ops->retlen = 0;
    if (!writelen)
        return 0;

    /* Reject writes, which are not page aligned */
    if (NOTALIGNED(to) || NOTALIGNED(ops->len)) {
        pr_debug("%s: attempt to write non page aligned data\n",
               __func__);
        return -EINVAL;
    }

    column = to & (mtd->writesize - 1);

    chipnr = (int)(to >> chip->chip_shift);
    chip->select_chip(mtd, chipnr);

    /* Check, if it is write protected */
    if (serial_nand_check_wp(mtd)) {
        ret = -EIO;
        goto err_out;
    }

    realpage = (int)(to >> chip->page_shift);
    page = realpage & chip->pagemask;
    blockmask = (1 << (chip->phys_erase_shift - chip->page_shift)) - 1;

    /* Invalidate the page cache, when we write to the cached page */
    if (to <= (chip->pagebuf << chip->page_shift) &&
        (chip->pagebuf << chip->page_shift) < (to + ops->len))
        chip->pagebuf = -1;

    /* Don't allow multipage oob writes with offset */
    if (oob && ops->ooboffs && (ops->ooboffs + ops->ooblen > oobmaxlen)) {
        ret = -EINVAL;
        goto err_out;
    }

    while (1) {
        int bytes = mtd->writesize;
        int cached = writelen > bytes && page != blockmask;
        uint8_t *wbuf = buf;

        /* Partial page write? */
        if (unlikely(column || writelen < (mtd->writesize - 1))) {
            cached = 0;
            bytes = min_t(int, bytes - column, (int) writelen);
            chip->pagebuf = -1;
            memset(chip->buffers->databuf, 0xff, mtd->writesize);
            memcpy(&chip->buffers->databuf[column], buf, bytes);
            wbuf = chip->buffers->databuf;
        }

        if (unlikely(oob)) {
            size_t len = min(oobwritelen, oobmaxlen);
            oob = serial_nand_fill_oob(mtd, oob, len, ops);
            oobwritelen -= len;
        } else {
            /* We still need to erase leftover OOB data */
            memset(chip->oob_poi, 0xff, mtd->oobsize);
        }
        ret = chip->write_page(mtd, chip, column, bytes, wbuf,
                    oob_required, page, cached,
                    (ops->mode == MTD_OPS_RAW));
        if (ret)
            break;

        writelen -= bytes;
        if (!writelen)
            break;

        column = 0;
        buf += bytes;
        realpage++;

        page = realpage & chip->pagemask;
        /* Check, if we cross a chip boundary */
        if (!page) {
            chipnr++;
            chip->select_chip(mtd, -1);
            chip->select_chip(mtd, chipnr);
        }
    }

    ops->retlen = ops->len - writelen;
    if (unlikely(oob))
        ops->oobretlen = ops->ooblen;

err_out:
    chip->select_chip(mtd, -1);
    return ret;
}

/**
 * serial_nand_write - [MTD Interface] NAND write with ECC
 * @mtd: MTD device structure
 * @to: offset to write to
 * @len: number of bytes to write
 * @retlen: pointer to variable to store the number of written bytes
 * @buf: the data to write
 *
 * NAND write with ECC.
 */
static int serial_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
              size_t *retlen, const uint8_t *buf)
{
    struct mtd_oob_ops ops;
    int ret;

    unsigned char *bounce_buf = NULL;
    bounce_buf = kmalloc(len, GFP_KERNEL);
    memcpy(bounce_buf, buf, len);

    serial_nand_get_device(mtd, FL_WRITING);
    ops.len = len;
    ops.datbuf = (uint8_t *)bounce_buf;
    ops.oobbuf = NULL;
    ops.mode = MTD_OPS_PLACE_OOB;
    ret = serial_nand_do_write_ops(mtd, to, &ops);
    *retlen = ops.retlen;
    kfree(bounce_buf);
    serial_nand_release_device(mtd);
    return ret;
}

/**
 * serial_nand_do_write_oob - [MTD Interface] NAND write out-of-band
 * @mtd: MTD device structure
 * @to: offset to write to
 * @ops: oob operation description structure
 *
 * NAND write out-of-band.
 */
static int serial_nand_do_write_oob(struct mtd_info *mtd, loff_t to,
                 struct mtd_oob_ops *ops)
{
    int chipnr, page, status, len;
    struct serial_nand_chip *chip = mtd->priv;

    pr_debug("%s: to = 0x%08x, len = %d\n",
             __func__, (unsigned int)to, (int)ops->ooblen);

    if (ops->mode == MTD_OPS_AUTO_OOB)
        len = chip->ecc.layout->oobavail;
    else
        len = mtd->oobsize;

    /* Do not allow write past end of page */
    if ((ops->ooboffs + ops->ooblen) > len) {
        pr_debug("%s: attempt to write past end of page\n",
                __func__);
        return -EINVAL;
    }

    if (unlikely(ops->ooboffs >= len)) {
        pr_debug("%s: attempt to start write outside oob\n",
                __func__);
        return -EINVAL;
    }

    /* Do not allow write past end of device */
    if (unlikely(to >= mtd->size ||
             ops->ooboffs + ops->ooblen >
            ((mtd->size >> chip->page_shift) -
             (to >> chip->page_shift)) * len)) {
        pr_debug("%s: attempt to write beyond end of device\n",
                __func__);
        return -EINVAL;
    }

    chipnr = (int)(to >> chip->chip_shift);
    chip->select_chip(mtd, chipnr);

    /* Shift to get page */
    page = (int)(to >> chip->page_shift);

    /* Check, if it is write protected */
    if (serial_nand_check_wp(mtd)) {
        chip->select_chip(mtd, -1);
        return -EROFS;
    }

    /* Invalidate the page cache, if we write to the cached page */
    if (page == chip->pagebuf)
        chip->pagebuf = -1;

    serial_nand_fill_oob(mtd, ops->oobbuf, ops->ooblen, ops);

    if (ops->mode == MTD_OPS_RAW)
        status = chip->ecc.write_oob_raw(mtd, chip, page & chip->pagemask);
    else
        status = chip->ecc.write_oob(mtd, chip, page & chip->pagemask);

    chip->select_chip(mtd, -1);

    if (status)
        return status;

    ops->oobretlen = ops->ooblen;

    return 0;
}

/**
 * serial_nand_write_oob - [MTD Interface] NAND write data and/or out-of-band
 * @mtd: MTD device structure
 * @to: offset to write to
 * @ops: oob operation description structure
 */
static int serial_nand_write_oob(struct mtd_info *mtd, loff_t to,
              struct mtd_oob_ops *ops)
{
    int ret = -ENOTSUPP;

    ops->retlen = 0;

    /* Do not allow writes past end of device */
    if (ops->datbuf && (to + ops->len) > mtd->size) {
        pr_debug("%s: attempt to write beyond end of device\n",
                __func__);
        return -EINVAL;
    }

    serial_nand_get_device(mtd, FL_WRITING);

    switch (ops->mode) {
    case MTD_OPS_PLACE_OOB:
    case MTD_OPS_AUTO_OOB:
    case MTD_OPS_RAW:
        break;

    default:
        goto out;
    }

    if (!ops->datbuf)
        ret = serial_nand_do_write_oob(mtd, to, ops);
    else
        ret = serial_nand_do_write_ops(mtd, to, ops);

out:
    serial_nand_release_device(mtd);
    return ret;
}


/**
 * serial_nand_erase - [MTD Interface] erase block(s)
 * @mtd: MTD device structure
 * @instr: erase instruction
 *
 * Erase one ore more blocks.
 */
static int serial_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
    return serial_nand_erase_nand(mtd, instr, 0);
}

/**
 * serial_nand_erase_nand - [INTERN] erase block(s)
 * @mtd: MTD device structure
 * @instr: erase instruction
 * @allowbbt: allow erasing the bbt area
 *
 * Erase one ore more blocks.
 */
int serial_nand_erase_nand(struct mtd_info *mtd, struct erase_info *instr,
            int allowbbt)
{
    int page, status, pages_per_block, ret, chipnr;
    struct serial_nand_chip *chip = mtd->priv;
    struct spi_device *spi = chip->spi;
    loff_t len;
    uint8_t stat;


    pr_debug("%s: start = 0x%012llx, len = %llu\n",
            __func__, (unsigned long long)instr->addr,
            (unsigned long long)instr->len);

    if (check_offs_len(mtd, instr->addr, instr->len))
        return -EINVAL;

    /* Grab the lock and see if the device is available */
    serial_nand_get_device(mtd, FL_ERASING);

    /* Shift to get first page */
    page = (int)(instr->addr >> chip->page_shift);
    chipnr = (int)(instr->addr >> chip->chip_shift);

    /* Calculate pages in each block */
    pages_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);

    /* Select the NAND device */
    chip->select_chip(mtd, chipnr);

    /* Check, if it is write protected */
    if (serial_nand_check_wp(mtd)) {
        pr_debug("%s: device is write protected!\n",
                __func__);
        instr->state = MTD_ERASE_FAILED;
        goto erase_exit;
    }

    /* Loop through the pages */
    len = instr->len;

    instr->state = MTD_ERASING;

    while (len) {
        /* Check if we have a bad block, we do not erase bad blocks! */
        uint32_t block;

        if (serial_nand_block_checkbad(mtd, ((loff_t) page) <<
                    chip->page_shift, 0, allowbbt)) {
            pr_warn("%s: attempt to erase a bad block at page 0x%08x\n",
                    __func__, page);
            instr->state = MTD_ERASE_FAILED;
            goto erase_exit;
        }

        /*
         * Invalidate the page cache, if we erase the block which
         * contains the current cached page.
         */
        if (page <= chip->pagebuf && chip->pagebuf <
            (page + pages_per_block))
            chip->pagebuf = -1;

        block = page / 64;
        /*
         * Write enable
         */
        status = serial_nand_write_enable(spi);
        if (status) {
            pr_err("%s: failed write enable status=%d\n", chip->name, status);
            return status;
        }

        /*
         * Block erase
         */
        status = serial_nand_erase_block_cmd(spi, page);
        if (status) {
            pr_err("%s: failed block erase status=%d\n", chip->name, status);
                instr->state = MTD_ERASE_FAILED;
                instr->fail_addr = ((loff_t)page << chip->page_shift);
            goto erase_exit;
        }

        /*
         * Wait
         */
        status = serial_nand_wait_ready(mtd, &stat);

        /*
         * See if operation failed and additional status checks are
         * available
         */
        if (status || (stat & S_NAND_STATUS_OIP)) {
            instr->state = MTD_ERASE_FAILED;
            instr->fail_addr = ((loff_t)page << chip->page_shift);
            pr_err("%s: chip is busy or nonresponsive status=%d stat=%02x\n", chip->name, status, stat);
            if (status) {
                goto erase_exit;
            }

            /*
             * Chip is stuck?
             */
                instr->state = MTD_ERASE_FAILED;
                instr->fail_addr =
                    ((loff_t)page << chip->page_shift);
                goto erase_exit;
        }

        /*
         * Check the status register
         */
        if (stat & S_NAND_STATUS_E_FAIL) {
            pr_err("%s: block %d E_FAIL signalled (%02x)\n", chip->name, block, stat);
            instr->state = MTD_ERASE_FAILED;
            instr->fail_addr =
                ((loff_t)page << chip->page_shift);
            goto erase_exit;
        }

        /* Increment page address and decrement length */
        len -= (1 << chip->phys_erase_shift);
        page += pages_per_block;

        /* Check, if we cross a chip boundary */
        if (len && !(page & chip->pagemask)) {
            chipnr++;
            chip->select_chip(mtd, -1);
            chip->select_chip(mtd, chipnr);
        }
    }
    instr->state = MTD_ERASE_DONE;

erase_exit:

    ret = instr->state == MTD_ERASE_DONE ? 0 : -EIO;

    /* Deselect and wake up anyone waiting on the device */
    chip->select_chip(mtd, -1);
    serial_nand_release_device(mtd);

    /* Do call back function */
    if (!ret)
        mtd_erase_callback(instr);

    /* Return more or less happy */
    return ret;
}

/**
 * serial_nand_sync - [MTD Interface] sync
 * @mtd: MTD device structure
 *
 * Sync is actually a wait for chip ready function.
 */
static void serial_nand_sync(struct mtd_info *mtd)
{
    pr_debug("%s: called\n", __func__);

    /* Grab the lock and see if the device is available */
    serial_nand_get_device(mtd, FL_SYNCING);
    /* Release it and go back */
    serial_nand_release_device(mtd);
}


/**
 * serial_nand_block_isbad - [MTD Interface] Check if block at offset is bad
 * @mtd: MTD device structure
 * @offs: offset relative to mtd start
 */
static int serial_nand_block_isbad(struct mtd_info *mtd, loff_t offs)
{
    return serial_nand_block_checkbad(mtd, offs, 1, 0);
}

/**
 * serial_nand_block_markbad - [MTD Interface] Mark block at the given offset as bad
 * @mtd: MTD device structure
 * @ofs: offset relative to mtd start
 */
static int serial_nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
    struct serial_nand_chip *chip = mtd->priv;
    int ret;

    ret = serial_nand_block_isbad(mtd, ofs);
    if (ret) {
        /* If it was bad already, return success and do nothing */
        if (ret > 0)
            return 0;
        return ret;
    }

    return chip->block_markbad(mtd, ofs);
        // chip->block_markbad = serial_nand_default_block_markbad; in serial_nand_set_defaults( )
}

/* Set default functions */
static void serial_nand_set_defaults(struct serial_nand_chip *chip)
{
    if (!chip->select_chip)
        chip->select_chip = serial_nand_select_chip;
    if (!chip->block_bad)
        chip->block_bad = serial_nand_block_bad;
    if (!chip->block_markbad)
        chip->block_markbad = serial_nand_default_block_markbad;
    if (!chip->scan_bbt)
        chip->scan_bbt = serial_nand_default_bbt;
    if (!chip->controller) {
        chip->controller = &chip->hwcontrol;
        spin_lock_init(&chip->controller->lock);
        init_waitqueue_head(&chip->controller->wq);
    }
}

/*
 * Set the bad block marker/indicator (BBM/BBI) patterns according to some
 * heuristic patterns using various detected parameters (e.g., manufacturer,
 * page size, cell-type information).
 */
static void serial_nand_decode_bbm_options(struct mtd_info *mtd,
                    struct serial_nand_chip *chip, u8 id_data[8])
{
    chip->badblockpos = NAND_LARGE_BADBLOCK_POS;
    chip->bbt_options |= NAND_BBT_SCAN2NDPAGE;
}

static inline bool is_full_id_nand(struct nand_flash_dev *type)
{
    return type->id_len;
}


static bool find_full_id_nand(struct mtd_info *mtd, struct serial_nand_chip *chip,
           struct nand_flash_dev *type, u8 *id_data)
{
    if (!strncmp(type->id, id_data, type->id_len)) {
        mtd->name = type->name;
        mtd->writesize = type->pagesize;
        mtd->erasesize = type->erasesize;
        mtd->oobsize = type->oobsize;
        chip->name = type->name;
        chip->chipsize = (uint64_t)type->chipsize << 20;
        chip->options |= type->options;
        return true;
    }
    return false;
}

/*
 * Get the flash and manufacturer id and lookup if the type is supported.
 */
static struct nand_flash_dev *serial_nand_get_flash_type(struct mtd_info *mtd,
                          struct serial_nand_chip *chip,
                          uint8_t *id_data,
                          struct nand_flash_dev *type)
{
    /* Select the device */
    chip->select_chip(mtd, 0);
    if (!type)
        type = serial_nand_flash_ids;

    for (; type->name != NULL; type++) {
        if (is_full_id_nand(type)) {
            if (find_full_id_nand(mtd, chip, type, id_data))
                break;
        }
    }
    if (!type->name)
        return ERR_PTR(-ENODEV);

    serial_nand_decode_bbm_options(mtd, chip, id_data);

    /* Calculate the address shift from the page size */
    chip->page_shift = ffs(mtd->writesize) - 1;
    /* Convert chipsize to number of pages per chip -1 */
    chip->pagemask = (chip->chipsize >> chip->page_shift) - 1;

    chip->bbt_erase_shift = chip->phys_erase_shift =
        ffs(mtd->erasesize) - 1;
    if (chip->chipsize & 0xffffffff)
        chip->chip_shift = ffs((unsigned)chip->chipsize) - 1;
    else {
        chip->chip_shift = ffs((unsigned)(chip->chipsize >> 32));
        chip->chip_shift += 32 - 1;
    }
    chip->badblockbits = 8;

    pr_info("NAND device: Manufacturer ID: 0x%02x, Chip ID: 0x%02x (%s),"
        " %dMiB, page size: %d, OOB size: %d\n",
        id_data[0], id_data[1], 
        type->name,
        (int)(chip->chipsize >> 20), mtd->writesize, mtd->oobsize);

    return type;
}

/**
 * serial_nand_scan_ident - [NAND Interface] Scan for the NAND device
 * @mtd: MTD device structure
 * @maxchips: number of chips to scan for
 * @table: alternative NAND ID table
 *
 * This is the first phase of the normal serial_nand_scan() function. It reads the
 * flash ID and sets up MTD fields accordingly.
 *
 * The mtd->owner field must be set to the module of the caller.
 */
int serial_nand_scan_ident(struct mtd_info *mtd, int maxchips,
            struct nand_flash_dev *table, uint8_t *idbuf)
{
    struct serial_nand_chip *chip = mtd->priv;
    struct nand_flash_dev *type;

    serial_nand_set_defaults(chip);

    /* Read the flash type */
    type = serial_nand_get_flash_type(mtd, chip, idbuf, table);
    if (IS_ERR(type)) {
        if (!(chip->options & NAND_SCAN_SILENT_NODEV))
            pr_warn("No NAND device found\n");
        chip->select_chip(mtd, -1);
        return PTR_ERR(type);
    }

    chip->ecc.size = type->ecc.step_ds;
    pr_info("%s ecc.size=%d ecc.bytes=%d\n",__func__,chip->ecc.size,chip->ecc.bytes);

    chip->select_chip(mtd, -1);

    /* Store the number of chips and calc total size for mtd */
    chip->numchips = 1;

    mtd->size = chip->numchips  * chip->chipsize;

    return 0;
}
EXPORT_SYMBOL(serial_nand_scan_ident);


/**
 * serial_nand_scan_tail - [NAND Interface] Scan for the NAND device
 * @mtd: MTD device structure
 *
 * This is the second phase of the normal serial_nand_scan() function. It fills out
 * all the uninitialized function pointers with the defaults and scans for a
 * bad block table if appropriate.
 */
int serial_nand_scan_tail(struct mtd_info *mtd)
{
    int i;
    struct serial_nand_chip *chip = mtd->priv;

    /* New bad blocks should be marked in OOB, flash-based BBT, or both */
    BUG_ON((chip->bbt_options & NAND_BBT_NO_OOB_BBM) &&
            !(chip->bbt_options & NAND_BBT_USE_FLASH));

    if (!(chip->options & NAND_OWN_BUFFERS))
        chip->buffers = kmalloc(sizeof(*chip->buffers), GFP_KERNEL);
    if (!chip->buffers)
        return -ENOMEM;

    /* Set the internal oob buffer location, just after the page data */
    chip->oob_poi = chip->buffers->databuf + mtd->writesize;

    /*
     * If no default placement scheme is given, select an appropriate one.
     */
    if (!chip->ecc.layout) {
        switch (mtd->oobsize) {
        case 64:
            chip->ecc.layout = &serial_nand_oob_64;
            break;
        case 128:
            chip->ecc.layout = &serial_nand_oob_128;
            break;
        default:
            pr_warn("No oob scheme defined for oobsize %d\n",
                   mtd->oobsize);
            BUG();
        }
    }

    if (!chip->write_page)
        chip->write_page = serial_nand_write_page;

    chip->ecc.read_page_raw     = serial_nand_read_page_raw;
    chip->ecc.read_oob             = serial_nand_read_oob_std;
    chip->ecc.write_page_raw     = serial_nand_write_page_raw;
    chip->ecc.write_oob         = serial_nand_write_oob_std;
    pr_err("%s mtd.writesize=%d ecc.size=%d\n",__func__,mtd->writesize,chip->ecc.size);
	//chip->ecc.size = mtd->writesize;

	chip->ecc.bytes = 2;
    chip->ecc.strength = 8;
    pr_info("%s ecc.size=%d ecc.bytes=%d\n",__func__,chip->ecc.size,chip->ecc.bytes);

    /* For many systems, the standard OOB write also works for raw */
    if (!chip->ecc.read_oob_raw)
        chip->ecc.read_oob_raw = chip->ecc.read_oob;
    if (!chip->ecc.write_oob_raw)
        chip->ecc.write_oob_raw = chip->ecc.write_oob;

    /*
     * The number of bytes available for a client to place data into
     * the out of band area.
     */
    chip->ecc.layout->oobavail = 0;
    for (i = 0; chip->ecc.layout->oobfree[i].length
            && i < ARRAY_SIZE(chip->ecc.layout->oobfree); i++)
        chip->ecc.layout->oobavail +=
            chip->ecc.layout->oobfree[i].length;
    mtd->oobavail = chip->ecc.layout->oobavail;

    /*
     * Set the number of read / write steps for one page depending on ECC
     * mode.
     */
    chip->ecc.steps = mtd->writesize / chip->ecc.size;
    if (chip->ecc.steps * chip->ecc.size != mtd->writesize) {
        pr_warn("Invalid ECC parameters\n");
        BUG();
    }
    chip->ecc.total = chip->ecc.steps * chip->ecc.bytes;

    /* Allow subpage writes up to ecc.steps. Not possible for MLC flash */
    if (!(chip->options & NAND_NO_SUBPAGE_WRITE)) {
        switch (chip->ecc.steps) {
        case 2:
            mtd->subpage_sft = 1;
            break;
        case 4:
        case 8:
        case 16:
            mtd->subpage_sft = 2;
            break;
        }
    }
    pr_info("%s options=0x%08x ecc.steps=%d subpage_sft=%d\n",__func__,chip->options,chip->ecc.steps,mtd->subpage_sft);
    chip->subpagesize = mtd->writesize >> mtd->subpage_sft;

    /* Initialize state */
    chip->state = FL_READY;

    /* Invalidate the pagebuffer reference */
    chip->pagebuf = -1;

    /* Fill in remaining MTD driver data */
    mtd->type = MTD_NANDFLASH;
    mtd->flags = (chip->options & NAND_ROM) ? MTD_CAP_ROM :
                        MTD_CAP_NANDFLASH;
    mtd->_erase = serial_nand_erase;
    mtd->_point = NULL;
    mtd->_unpoint = NULL;
    mtd->_read = serial_nand_read;
    mtd->_write = serial_nand_write;
    mtd->_read_oob = serial_nand_read_oob;
    mtd->_write_oob = serial_nand_write_oob;
    mtd->_sync = serial_nand_sync;
    mtd->_lock = NULL;
    mtd->_unlock = NULL;
    mtd->_suspend = NULL;
    mtd->_resume = NULL;
    mtd->_block_isbad = serial_nand_block_isbad;
    mtd->_block_markbad = serial_nand_block_markbad;
    mtd->writebufsize = mtd->writesize;

    /* propagate ecc info to mtd_info */
    mtd->ecclayout = chip->ecc.layout;
    mtd->ecc_strength = chip->ecc.strength;
    /*
     * Initialize bitflip_threshold to its default prior scan_bbt() call.
     * scan_bbt() might invoke mtd_read(), thus bitflip_threshold must be
     * properly set.
     */
    if (!mtd->bitflip_threshold)
        mtd->bitflip_threshold = mtd->ecc_strength;

	pr_info("%s:mtd->bitflip_threshold=%d mtd->ecc_strength=%d",__func__,mtd->bitflip_threshold,mtd->ecc_strength);
    /* Check, if we should skip the bad block table scan */
    if (chip->options & NAND_SKIP_BBTSCAN)
        return 0;

    /* Build bad block table */
    return chip->scan_bbt(mtd);
}
EXPORT_SYMBOL(serial_nand_scan_tail);

/*
 * is_module_text_address() isn't exported, and it's mostly a pointless
 * test if this is a module _anyway_ -- they'd have to try _really_ hard
 * to call us from in-kernel code if the core NAND support is modular.
 */
#ifdef MODULE
#define caller_is_module() (1)
#else
#define caller_is_module() \
    is_module_text_address((unsigned long)__builtin_return_address(0))
#endif

/**
 * serial_nand_scan - [NAND Interface] Scan for the NAND device
 * @mtd: MTD device structure
 * @maxchips: number of chips to scan for
 *
 * This fills out all the uninitialized function pointers with the defaults.
 * The flash ID is read and the mtd/chip structures are filled with the
 * appropriate values. The mtd->owner field must be set to the module of the
 * caller.
 */
int serial_nand_scan(struct mtd_info *mtd, int maxchips, uint8_t *idbuf)
{
    struct serial_nand_chip *chip = mtd->priv;
    int ret;

    /* Many callers got this wrong, so check for it for a while... */
    if (!mtd->owner && caller_is_module()) {
        pr_crit("%s called with NULL mtd->owner!\n", __func__);
        BUG();
    }

    ret = serial_nand_scan_ident(mtd, maxchips, NULL, idbuf);
    if (ret)
        return ret;

    if (on_flash_bbt) {
        pr_info("%s Use On Flash BBT\n", __func__);
        chip->bbt_options |= NAND_BBT_USE_FLASH;
    }

    ret = serial_nand_scan_tail(mtd);
    return ret;
}
EXPORT_SYMBOL(serial_nand_scan);

/**
 * serial_nand_release - [NAND Interface] Free resources held by the NAND device
 * @mtd: MTD device structure
 */
void serial_nand_release(struct mtd_info *mtd)
{
    struct serial_nand_chip *chip = mtd->priv;

    mtd_device_unregister(mtd);

    /* Free bad block table memory */
    kfree(chip->bbt);
    if (!(chip->options & NAND_OWN_BUFFERS))
        kfree(chip->buffers);

    /* Free bad block descriptor memory */
    if (chip->badblock_pattern && chip->badblock_pattern->options
            & NAND_BBT_DYNAMICSTRUCT)
        kfree(chip->badblock_pattern);
}
EXPORT_SYMBOL_GPL(serial_nand_release);

static int read_only = 0;
module_param(read_only, int, 0);
MODULE_PARM_DESC(read_only, "Leave device locked");

#ifndef MODULE
struct flash_info {
    /* JEDEC id zero means "no ID" (most older chips); otherwise it has
     * a high byte of zero plus three data bytes: the manufacturer id,
     * then a two byte device id.
     */
    u32        jedec_id;
    u16        ext_id;

    /* The size listed here is what works with OPCODE_SE, which isn't
     * necessarily called a "sector" by the vendor.
     */
    unsigned    sector_size;
    u16        n_sectors;

    u16        page_size;
    u16        addr_width;

    u16        flags;
};

#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)    \
    ((kernel_ulong_t)&(struct flash_info) {                \
        .jedec_id = (_jedec_id),                \
        .ext_id = (_ext_id),                    \
        .sector_size = (_sector_size),                \
        .n_sectors = (_n_sectors),                \
        .page_size = 256,                    \
        .flags = (_flags),                    \
    })

/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */

static const struct spi_device_id serial_nand_ids[] = {
    { "TSBserial-nand",  INFO(0x201234, 0, 64 * 1024, 128, 0) },
    { },
};

/*
 * Called at boot time:
 *
 * nand_spi_er=read_only
 *    if read_only specified then do not unlock device
 */
static int __init serial_nand_setup(char *str)
{
    if (str && (strncasecmp(str, "read_only", 9) == 0)) {
        read_only = 1;
    }
    return 0;
}
__setup("nandspi=", serial_nand_setup);
//matthew.ma-2018/06/05: add spi flash driver as a kernel module
#else
struct flash_info {
    /* JEDEC id zero means "no ID" (most older chips); otherwise it has
     * a high byte of zero plus three data bytes: the manufacturer id,
     * then a two byte device id.
     */
    u32        jedec_id;
    u16        ext_id;

    /* The size listed here is what works with OPCODE_SE, which isn't
     * necessarily called a "sector" by the vendor.
     */
    unsigned    sector_size;
    u16        n_sectors;

    u16        page_size;
    u16        addr_width;

    u16        flags;
};

#define INFO(_jedec_id, _ext_id, _sector_size, _n_sectors, _flags)    \
    ((kernel_ulong_t)&(struct flash_info) {                \
        .jedec_id = (_jedec_id),                \
        .ext_id = (_ext_id),                    \
        .sector_size = (_sector_size),                \
        .n_sectors = (_n_sectors),                \
        .page_size = 256,                    \
        .flags = (_flags),                    \
    })

/* NOTE: double check command sets and memory organization when you add
 * more flash chips.  This current list focusses on newer chips, which
 * have been converging on command sets which including JEDEC ID.
 */

static const struct spi_device_id serial_nand_ids[] = {
    { "TSBserial-nand",  INFO(0x201234, 0, 64 * 1024, 128, 0) },
    { },
};

#endif  // #ifndef MODULE

/*
 * nand_spi_er_probe
 *    Detect and initialize nand_spi_er device.
 */
static int serial_nand_prove(struct spi_device *spi)
{
    uint8_t txbuf[1];
    uint8_t rxbuf[2];
    int i;
    int res;
    struct serial_nand_chip *chip;
    struct flash_platform_data    *data;

    res = spi_setup(spi);
    if (res) {
        pr_warn("%s spi_setup error %d\n", __func__, res);
        return res;
    }
    
    /*
     * Reset
     */
    for (i = 0; i < 2; i++) {
        res = serial_nand_reset(spi);
        if (res) 
            return res;
        udelay(250);
    }
    udelay(1000);

    /*
     * Read ID
     */
    res = serial_nand_raed_id(spi, rxbuf);
    if (res) {
        pr_warn("%s serial_nand_raed_id error %d\n", __func__, res);
        return res;
    }
    pr_info("%s read ID id[0] %2x  id[1] %2x\n", __func__, rxbuf[0], rxbuf[1]);

    /*
     * Initialize our chip structure
     */

    chip = kzalloc(sizeof(struct serial_nand_chip), GFP_KERNEL);
    if (!chip) {
        return -ENOMEM;
    }
    chip->spi = spi;
    chip->mtd.owner = THIS_MODULE;
    chip->mtd.priv = chip;

    /*
     * Unlock serial NAND device
     */
    txbuf[0] = 0x00;
    res = serial_nand_set_feature(spi, 0xA0, txbuf);        // Unlock device
    if (res) {
        pr_err(">%s<: failed lock operation res=%d\n", chip->name, res);
        return res;
    }

    /*
     * Scan NAND defnition table
     */
    res = serial_nand_scan(&chip->mtd, 1, rxbuf);
    if (res) {
        pr_warn("%s serial_nand_scan error %d\n", __func__, res);
        return res;
    }

	pr_info("%s:%d MTD device size %llu, eraseblock size %u, "
           "page size %u, OOB size %u\n",
			 __func__,__LINE__,
           (unsigned long long)chip->mtd.size, chip->mtd.erasesize,
           chip->mtd.writesize, chip->mtd.oobsize);

    spi_set_drvdata(spi, chip);

    data = spi->dev.platform_data;

    if(NULL == data)
    {
        pr_info("%s data is null\n",__func__);
    }
    else
    {
        pr_info("%s:%d MTD device data->parts=%d data->nr_parts=%d\n",
            __func__,__LINE__,data->parts,data->nr_parts);
    }
    return mtd_device_parse_register(&chip->mtd, NULL, NULL,
            data ? data->parts : NULL,
            data ? data->nr_parts : 0);
}

/*
 * nand_spi_er_remove
 */
static int serial_nand_remove(struct spi_device *spi)        // 
{
    struct serial_nand_chip *chip = spi_get_drvdata(spi);
    int status;

    pr_info("%s chip->buffers %p\n", __func__, chip->buffers);
    if (!(chip->options & NAND_OWN_BUFFERS) && (chip->buffers))
        kfree(chip->buffers);

    status = mtd_device_unregister(&chip->mtd);
    if (status == 0)
        kfree(chip);
    return status;
}

static struct spi_driver serial_nand_driver = {

    .driver = {
        .name        = "serial_nand_official",
        .owner        = THIS_MODULE,
    },
    .id_table    = serial_nand_ids,
    .probe        = serial_nand_prove,
    .remove        = serial_nand_remove,

};
module_spi_driver(serial_nand_driver);

//matthew.ma-2018/06/05: add spi flash driver as a kernel module
/*static int __init serial_nand_base_init(void)
{
    led_trigger_register_simple("nand-disk", &serial_nand_led_trigger);
    return 0;
}

static void __exit serial_nand_base_exit(void)
{
    led_trigger_unregister_simple(serial_nand_led_trigger);
}

module_init(serial_nand_base_init);
module_exit(serial_nand_base_exit);*/

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("TOSHIBA MEMORY CORPORATION");
MODULE_DESCRIPTION("Serial NAND (Built-in ECC NAND) support");
