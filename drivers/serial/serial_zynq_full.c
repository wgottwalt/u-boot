// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2023 Wilken Gottwalt
 */

#include <clk.h>
#include <common.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <linux/err.h>
#include <asm/io.h>
#include <serial.h>

/* ctrl */
#define UART_CR_STOPBRK	BIT(8)	/* TX stop break */
#define UART_CR_STARTBR	BIT(7)	/* TX start break */
#define UART_CR_TORST	BIT(6)	/* RX timeout timer reset */
#define UART_CR_TX_DIS	BIT(5)	/* TX disable */
#define UART_CR_TX_EN	BIT(4)	/* TX enabled */
#define UART_CR_RX_DIS	BIT(3)	/* RX disable */
#define UART_CR_RX_EN	BIT(2)	/* RX enabled */
#define UART_CR_TXRST	BIT(1)	/* TX logic reset */
#define UART_CR_RXRST	BIT(0)	/* RX logic reset */

/* mode */
#define UART_MR_CHMODE_RLOOP	0x0300	/* remote loopback */
#define UART_MR_CHMODE_LLOOP	0x0200	/* local loopback */
#define UART_MR_CHMODE_ECHO	0x0100	/* automatic echo */
#define UART_MR_NBSTOP_2_BIT	0x0080	/* 2 stop bits */
#define UART_MR_NBSTOP_1_5_BIT	0x0040	/* 1.5 stop bits */
#define UART_MR_PAR_NONE	0x0020	/* no parity */
#define UART_MR_PAR_MARK	0x0018	/* forced 1 parity (mark) */
#define UART_MR_PAR_SPACE	0x0010	/* forced 0 parity (space) */
#define UART_MR_PAR_ODD		0x0008	/* odd parity */
#define UART_MR_CHRL_6_BIT	0x0006	/* 6 bits */
#define UART_MR_CHRL_7_BIT	0x0004	/* 7 bits */
#define UART_MR_CLKSEL		0x0001	/* clock / 8 mode */

#define UART_MR_CHMODE_NORM	0x0000	/* unset chanmodes - normal mode */
#define UART_MR_NBSTOP_1_BIT	0x0000	/* unset stopbits - 1 stop bits */
#define UART_MR_CHRL_8_BIT	0x0000	/* unset bits - 8 bits */
#define UART_MR_PAR_EVEN	0x0000	/* unset parities - even parity */

/* chanstat */
#define UART_SR_TNFUL		BIT(14) /* TX FIFO only one byte free */
#define UART_SR_TTRIG		BIT(13)	/* TX FIFO fill level TTRIG reached */
#define UART_SR_FLOWDEL		BIT(12) /* RX FIFO fill level FDEL reached - delays flow */
#define UART_SR_TXACTIVE	BIT(11)	/* TX active */
#define UART_SR_RXACTIVE	BIT(10)	/* RX active */
#define UART_SR_TXFULL		BIT(4)	/* TX FIFO full */
#define UART_SR_TXEMPTY		BIT(3)	/* TX FIFO empty */
#define UART_SR_RXFULL		BIT(2)	/* RX FIFO full */
#define UART_SR_RXEMPTY		BIT(1)	/* RX FIFO empty */
#define UART_SR_RXOVR		BIT(0)	/* RX FIFO fill level RTRIG reached */

struct serial_zynq_full_priv {
	u32 ctrl;		/* 0x00 - UART Control Register - rw def 128 */
	u32 mode;		/* 0x04 - UART Mode Register - rw def 0 */
	u32 intr_en;		/* 0x08 - Interrupt Enable Register - rw def 0 */
	u32 intr_dis;		/* 0x0C - Interrupt Disable Register - rw def 0 */
	u32 intr_mask;		/* 0x10 - Interrupt Mask Register - ro def 0 */
	u32 intr_chanstat;	/* 0x14 - Channel Interrupt Status Register - wtc def 0 */
	u32 baudrate_gen;	/* 0x18 - Baud Rate Generator Register - rw def 651 */
	u32 recv_timeout;	/* 0x1C - Receiver Timeout Register - rw def 0 */
	u32 recv_fifo_level;	/* 0x20 - Receiver FIFO Trigger Level Register - rw def 32 */
	u32 modem_ctrl;		/* 0x24 - Modem Control Register - rw def 0 */
	u32 modem_stat;		/* 0x28 - Modem Status Register - rw def none */
	u32 chanstat;		/* 0x2C - Channel Status Register - ro def 0 */
	u32 tx_rx_fifo;		/* 0x30 - Transmit and Receive FIFO - rw def 0 */
	u32 baudrate_div;	/* 0x34 - Baud Rate Divider Register - rw def 15 */
	u32 flow_ctrl_delay;	/* 0x38 - Flow Control Delay Register - rw def 0 */
	u32 dummy[3];
	u32 trans_fifo_level;	/* 0x44 - Transmitter FIFO Trigger Level Register - rw def 32 */
};

struct serial_zynq_full_plat {
	struct serial_zynq_full_priv *regs;
	ulong clockrate;
	u32 baudrate;
	u32 precission;
	u32 mode;
	bool handle_crlf;
};

static int _serial_zync_full_set_charframe(struct serial_zynq_full_priv *regs, u32 mode)
{
	u32 val = 0;

	/* TODO: possible clock / 8 mode */

	switch (SERIAL_GET_BITS(mode)) {
	case SERIAL_6_BITS:
		val |= UART_MR_CHRL_6_BIT;
		break;
	case SERIAL_7_BITS:
		val |= UART_MR_CHRL_7_BIT;
		break;
	case SERIAL_8_BITS:
		val |= UART_MR_CHRL_8_BIT;
		break;
	default:
		return -ENOTSUPP;
	}

	switch (SERIAL_GET_PARITY(mode)) {
	case SERIAL_PAR_NONE:
		val |= UART_MR_PAR_NONE;
		break;
	case SERIAL_PAR_ODD:
		val |= UART_MR_PAR_ODD;
		break;
	case SERIAL_PAR_EVEN:
		val |= UART_MR_PAR_EVEN;
		break;
	default:
		return -ENOTSUPP;
	}

	switch (SERIAL_GET_STOP(mode)) {
	case SERIAL_ONE_STOP:
		val |= UART_MR_NBSTOP_1_BIT;
		break;
	case SERIAL_ONE_HALF_STOP:
		val |= UART_MR_NBSTOP_1_5_BIT;
		break;
	case SERIAL_TWO_STOP:
		val |= UART_MR_NBSTOP_2_BIT;
		break;
	default:
		return -ENOTSUPP;
	}

	/* TODO: channel mode setup */

	writel(val, &regs->mode);

	return 0;
}

static int _serial_zync_full_set_baudrate(struct serial_zynq_full_priv *regs, ulong clockrate,
					  u32 baudrate, u32 precission)
{
	ulong calc, error;
	u32 div, gen;

	/* use known gen/div values */
	if (baudrate == 115200) {
		gen = 124;
		div = 6;
	} else if (baudrate == 500000) {
		gen = 40;
		div = 4;
	} else {
		if (clockrate < 1000000 && baudrate > 4800)
			return -ENOTSUPP;

		for (div = 4; div < 255; ++div) {
			gen = clockrate / (baudrate * (div + 1));
			if (gen < 2 || gen > 65535)
				continue;

			calc = clockrate / (gen * (div + 1));

			if (baudrate > calc)
				error = baudrate - calc;
			else
				error = calc - baudrate;
			if (((error * 100) / baudrate) < precission)
				break;
		}
	}

	writel(div, &regs->baudrate_div);
	writel(gen, &regs->baudrate_gen);

	return 0;
}

static void _serial_zync_full_set_fifo(struct serial_zynq_full_priv *regs)
{
	/* TODO: set trigger levels, currently stick with defaults */
}

static void _serial_zync_full_set_ctrl(struct serial_zynq_full_priv *regs, bool on)
{
	if (on) {
		writel(UART_CR_TX_EN | UART_CR_RX_EN | UART_CR_TXRST | UART_CR_RXRST,
		       &regs->ctrl);
	} else {
		writel(UART_CR_TX_DIS | UART_CR_RX_DIS | UART_CR_TXRST | UART_CR_RXRST,
		       &regs->ctrl);
	}
}

static void _serial_zync_full_set_timeouts(struct serial_zynq_full_priv *regs)
{
	/* TODO: set timeouts, currently stick with defaults */
}

static int _serial_zynq_full_putc(struct serial_zynq_full_priv *regs, const char chr,
				  const bool handle_crlf)
{
	if (readl(&regs->chanstat) & UART_SR_TXFULL)
		return -EAGAIN;

	if (handle_crlf && chr == '\n')
		writel('\r', &regs->tx_rx_fifo);
	writel(chr, &regs->tx_rx_fifo);

	return 0;
}

static int serial_zynq_full_setbrg(struct udevice *dev, int baudrate)
{
	struct serial_zynq_full_plat *plat = dev_get_plat(dev);

	return _serial_zync_full_set_baudrate(plat->regs, plat->clockrate, plat->baudrate,
					      plat->precission);
}

static int serial_zynq_full_getc(struct udevice *dev)
{
	struct serial_zynq_full_plat *plat = dev_get_plat(dev);
	struct serial_zynq_full_priv *regs = plat->regs;

	if (readl(&regs->chanstat) & UART_SR_RXEMPTY)
		return -EAGAIN;

	return readl(&regs->tx_rx_fifo);
}

static int serial_zynq_full_putc(struct udevice *dev, const char chr)
{
	struct serial_zynq_full_plat *plat = dev_get_plat(dev);

	return _serial_zynq_full_putc(plat->regs, chr, plat->handle_crlf);
}

static int serial_zynq_full_pending(struct udevice *dev, bool input)
{
	struct serial_zynq_full_plat *plat = dev_get_plat(dev);
	struct serial_zynq_full_priv *regs = plat->regs;

	if (input)
		return !(readl(&regs->chanstat) & UART_SR_RXEMPTY);
	else
		return !!(readl(&regs->chanstat) & UART_SR_TXACTIVE);
}

static int serial_zynq_full_clear(struct udevice *dev)
{
	struct serial_zynq_full_plat *plat = dev_get_plat(dev);

	/*
	 * it is possible to check FIFOs afterwards by reading UART_SR_TXEMPTY/UART_SR_RXEMPTY,
	 * but there is actually no need for it, because the resets guarantee emptied FIFOs
	 */
	writel(UART_CR_TXRST | UART_CR_RXRST, &plat->regs);

	return 0;
}

static int serial_zynq_full_getconfig(struct udevice *dev, u32 *serial_config)
{
	struct serial_zynq_full_plat *plat = dev_get_plat(dev);
	struct serial_zynq_full_priv *regs = plat->regs;
	u32 val = readl(&regs->mode);
	u32 mbits = 0;
	u32 mpar = 0;
	u32 mstop = 0;

	if (val & UART_MR_CHRL_6_BIT)
		mbits = SERIAL_6_BITS;
	else if (val & UART_MR_CHRL_7_BIT)
		mbits = SERIAL_7_BITS;
	else
		mbits = SERIAL_8_BITS;

	if (val & UART_MR_PAR_NONE)
		mpar = SERIAL_PAR_NONE;
	else if (val & UART_MR_PAR_ODD)
		mpar = SERIAL_PAR_ODD;
	else if (val & UART_MR_PAR_MARK || val & UART_MR_PAR_SPACE)
		return -ENOTSUPP;
	else
		mpar = SERIAL_PAR_EVEN;

	if (val & UART_MR_NBSTOP_2_BIT)
		mstop = SERIAL_TWO_STOP;
	else if (val & UART_MR_NBSTOP_1_5_BIT)
		mstop = SERIAL_ONE_HALF_STOP;
	else
		mstop = SERIAL_ONE_STOP;

	*serial_config = SERIAL_CONFIG(mpar, mbits, mstop);

	return 0;
}

static int serial_zynq_full_setconfig(struct udevice *dev, u32 serial_config)
{
	struct serial_zynq_full_plat *plat = dev_get_plat(dev);

	return _serial_zync_full_set_charframe(plat->regs, serial_config);
}

static int serial_zynq_full_getinfo(struct udevice *dev, struct serial_device_info *info)
{
	struct serial_zynq_full_plat *plat = dev_get_plat(dev);
	struct serial_zynq_full_priv *regs = plat->regs;
	u32 div, gen;

	info->type = SERIAL_CHIP_UNKNOWN;
	info->addr_space = SERIAL_ADDRESS_SPACE_MEMORY;
	info->addr = (ulong)plat->regs;
	info->reg_width = 4;
	info->reg_offset = 0;
	info->reg_shift = 0;
	info->clock = plat->clockrate;

	gen = readl(&regs->baudrate_gen);
	div = readl(&regs->baudrate_div);

	info->baudrate = plat->clockrate / (gen * (div + 1));

	return 0;
}

static int serial_zynq_full_of_to_plat(struct udevice *dev)
{
	struct serial_zynq_full_plat *plat = dev_get_plat(dev);
	u32 val;

	plat->regs = (struct serial_zynq_full_priv *)dev_read_addr(dev);
	if (IS_ERR(plat->regs))
		return PTR_ERR(plat->regs);

	plat->baudrate = dev_read_u32_default(dev, "baudrate", CONFIG_BAUDRATE);
	plat->precission = dev_read_u32_default(dev, "precission", 3000);

	plat->mode = 0;
	val = dev_read_u32_default(dev, "mode-bits", 8);
	switch (val) {
	case 6:
		plat->mode |= SERIAL_SET_BITS(SERIAL_6_BITS);
		break;
	case 7:
		plat->mode |= SERIAL_SET_BITS(SERIAL_7_BITS);
		break;
	case 8:
	default:
		plat->mode |= SERIAL_SET_BITS(SERIAL_8_BITS);
	}
	val = dev_read_u32_default(dev, "mode-parity", 0);
	switch (val) {
	case 2:
		plat->mode |= SERIAL_SET_PARITY(SERIAL_PAR_EVEN);
		break;
	case 1:
		plat->mode |= SERIAL_SET_PARITY(SERIAL_PAR_ODD);
		break;
	case 0:
	default:
		plat->mode |= SERIAL_SET_PARITY(SERIAL_PAR_NONE);
	}
	val = dev_read_u32_default(dev, "mode-stopbits", 1);
	switch (val) {
	case 2:
		plat->mode |= SERIAL_SET_STOP(SERIAL_TWO_STOP);
		break;
	case 15:
		plat->mode |= SERIAL_SET_STOP(SERIAL_ONE_HALF_STOP);
		break;
	case 1:
	default:
		plat->mode |= SERIAL_SET_STOP(SERIAL_ONE_STOP);
	}

	plat->handle_crlf = dev_read_bool(dev, "handle-crlf");

	return 0;
}

static int serial_zynq_full_probe(struct udevice *dev)
{
	struct serial_zynq_full_plat *plat = dev_get_plat(dev);
	struct serial_zynq_full_priv *regs = plat->regs;
	struct clk clock;
	ulong clockrate;
	int ret;

	plat->clockrate = 0;

	ret = clk_get_by_index(dev, 0, &clock);
	if (ret < 0) {
		dev_err(dev, "failed to get clock\n");
		return ret;
	}

	clockrate = clk_get_rate(&clock);
	if (IS_ERR_VALUE(clockrate)) {
		dev_err(dev, "failed to get clockrate\n");
		return clockrate;
	}

	ret = clk_enable(&clock);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		return ret;
	}

	plat->clockrate = clockrate;

	_serial_zync_full_set_ctrl(regs, false);
	ret = _serial_zync_full_set_charframe(regs, plat->mode);
	if (ret < 0)
		return ret;
	ret = _serial_zync_full_set_baudrate(regs, plat->clockrate, plat->baudrate,
					     plat->precission);
	if (ret < 0)
		return ret;
	_serial_zync_full_set_fifo(regs);
	_serial_zync_full_set_ctrl(regs, true);
	_serial_zync_full_set_timeouts(regs);

	return 0;
}

static const struct dm_serial_ops serial_zynq_full_ops = {
	.setbrg		= serial_zynq_full_setbrg,
	.getc		= serial_zynq_full_getc,
	.putc		= serial_zynq_full_putc,
	.pending	= serial_zynq_full_pending,
	.clear		= serial_zynq_full_clear,
	/*.loop TODO: add loop mode */
	.getconfig	= serial_zynq_full_getconfig,
	.setconfig	= serial_zynq_full_setconfig,
	.getinfo	= serial_zynq_full_getinfo,
};

static const struct udevice_id serial_zynq_full_ids[] = {
	{ .compatible = "xlnx,xuartps" },
	{ .compatible = "cdns,uart-r1p8" },
	{ .compatible = "cdns,uart-r1p12" },
	{ },
};

U_BOOT_DRIVER(serial_zynq_full) = {
	.name		= "serial_zynq_full",
	.id		= UCLASS_SERIAL,
	.of_match	= serial_zynq_full_ids,
	.of_to_plat	= serial_zynq_full_of_to_plat,
	.plat_auto	= sizeof(struct serial_zynq_full_plat),
	.probe		= serial_zynq_full_probe,
	.ops		= &serial_zynq_full_ops,
};
