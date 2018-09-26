#include <stdlib.h>
#include <string.h>

#include <zephyr.h>
#include <misc/printk.h>
#include <shell/legacy_shell.h>
#include <flash.h>
#include <device.h>
#include <soc.h>
#include <i2c.h>

#define THIS_MODULE_NAME "flash"

/* ------------------------------ */
// General register set
#define THR 0x00
#define RHR 0x00
#define FCR 0x02
#define LCR 0x03
#define MCR 0x04
#define EFCR 0x0f

#define RXLVL 0x09

// Special register set
#define DLL 0x00
#define DLH 0x01

// Enhanced register set
#define EFR 0x02

#define I2C_ADDR 0x4c

/* ------------------------------ */

#define I2C_INIT \
	("<i2c init>\n\n" \
	 "Set flash device by name. If a flash device was not found,\n" \
	 "this command must be run first to bind a device to this module.")

/*
 * Canonicalize argc/argv.
 *
 * When run as "<command> <arguments>", i.e. after running "select
 * flash", the module name is not part of argv. When run as "flash
 * <command> <arguments>", it is. Normalize argc and argv to skip
 * "flash" regardless of the case.
 */
#define CANONICALIZE_ARGS(argc, argv) do {			\
		if (!strcmp(argv[0], THIS_MODULE_NAME)) {	\
			argc--;				\
			argv++;				\
		}						\
		/* Skip the command name as well. */		\
		argc--;					\
		argv++;					\
	} while (0)

#define ARGC_MAX 10 /* from shell.c */

//static struct device *flash_device;
struct device *i2c_device;

//static int check_flash_device(void)
//{
//	if (flash_device == NULL) {
//		printk("Flash device is unknown. Run set_device first.\n");
//		return -ENODEV;
//	}
//	return 0;
//}

//static void dump_buffer(u8_t *buf, size_t size)
//{
//	bool newline = false;
//	u8_t *p = buf;
//
//	while (size >= 8) {
//		printk("%02x %02x %02x %02x | %02x %02x %02x %02x\n",
//		       p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
//		p += 8;
//		size -= 8;
//	}
//	if (size > 4) {
//		printk("%02x %02x %02x %02x | ",
//		       p[0], p[1], p[2], p[3]);
//		p += 4;
//		size -= 4;
//		newline = true;
//	}
//	while (size--) {
//		printk("%02x ", *p++);
//		newline = true;
//	}
//	if (newline) {
//		printk("\n");
//	}
//}

//static int parse_ul(const char *str, unsigned long *result)
//{
//	char *end;
//	unsigned long val;
//
//	val = strtoul(str, &end, 0);
//
//	if (*str == '\0' || *end != '\0') {
//		return -EINVAL;
//	}
//
//	*result = val;
//	return 0;
//}

//static int parse_u8(const char *str, u8_t *result)
//{
//	unsigned long val;
//	if (parse_ul(str, &val) || val > 0xff) {
//		return -EINVAL;
//	}
//	*result = (u8_t)val;
//	return 0;
//}

///* Read bytes, dumping contents to console and printing on error. */
//static int do_read(off_t offset, size_t len)
//{
//	u8_t buf[64];
//	int ret;
//
//	while (len > sizeof(buf)) {
//		ret = flash_read(flash_device, offset, buf, sizeof(buf));
//		if (ret) {
//			goto err_read;
//		}
//		dump_buffer(buf, sizeof(buf));
//		len -= sizeof(buf);
//		offset += sizeof(buf);
//	}
//	ret = flash_read(flash_device, offset, buf, len);
//	if (ret) {
//		goto err_read;
//	}
//	dump_buffer(buf, len);
//	return 0;
//
// err_read:
//	printk("flash_read: %d\n", ret);
//	return 0;
//}

///* Erase area, handling write protection and printing on error. */
//static int do_erase(off_t offset, size_t size)
//{
//	int ret;
//
//	ret = flash_write_protection_set(flash_device, false);
//	if (ret) {
//		printk("flash_write_protection_set: %d (can't disable)\n", ret);
//		return ret;
//	}
//	ret = flash_erase(flash_device, offset, size);
//	if (ret) {
//		printk("flash_erase: %d\n", ret);
//		return ret;
//	}
//	ret = flash_write_protection_set(flash_device, true);
//	if (ret) {
//		printk("flash_write_protection_set: %d (can't enable)\n", ret);
//	}
//	return ret;
//}

///* Write bytes, handling write protection and printing on error. */
//static int do_write(off_t offset, u8_t *buf, size_t len, bool read_back)
//{
//	int ret;
//
//	ret = flash_write_protection_set(flash_device, false);
//	if (ret) {
//		printk("flash_write_protection_set: %d (can't disable)\n", ret);
//		return ret;
//	}
//	ret = flash_write(flash_device, offset, buf, len);
//	if (ret) {
//		printk("flash_write: %d\n", ret);
//		return ret;
//	}
//	ret = flash_write_protection_set(flash_device, true);
//	if (ret) {
//		printk("flash_write_protection_set: %d (can't enable)\n", ret);
//		return ret;
//	}
//	if (read_back) {
//		printk("Reading back written bytes:\n");
//		ret = do_read(offset, len);
//	}
//	return ret;
//}

//static int flash_shell_write_block_size(int argc, char *argv[])
//{
//	if (check_flash_device()) {
//		return 0;
//	}
//
//	CANONICALIZE_ARGS(argc, argv);
//	if (argc) {
//		printk("No arguments expected.\n");
//		return 0;
//	}
//
//	printk("%d\n", flash_get_write_block_size(flash_device));
//	return 0;
//}

//static int flash_shell_read(int argc, char *argv[])
//{
//	long unsigned int offset, len;
//
//	if (check_flash_device()) {
//		return 0;
//	}
//
//	CANONICALIZE_ARGS(argc, argv);
//	if (argc != 2 || parse_ul(argv[0], &offset) ||
//	    parse_ul(argv[1], &len)) {
//		goto bail;
//	}
//
//	do_read(offset, len);
//	return 0;
//
// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
//}

//static int flash_shell_erase(int argc, char *argv[])
//{
//	long unsigned int offset, size;
//
//	if (check_flash_device()) {
//		return 0;
//	}
//
//	CANONICALIZE_ARGS(argc, argv);
//	if (argc != 2 || parse_ul(argv[0], &offset) ||
//	    parse_ul(argv[1], &size)) {
//		goto bail;
//	}
//
//	do_erase((off_t)offset, (size_t)size);
//	return 0;
//
// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
//}

//static int flash_shell_write(int argc, char *argv[])
//{
//	u8_t buf[ARGC_MAX];
//	long unsigned int i, offset;
//
//	if (check_flash_device()) {
//		return 0;
//	}
//
//	CANONICALIZE_ARGS(argc, argv);
//	if (argc < 2 || parse_ul(argv[0], &offset)) {
//		goto bail;
//	} else if (argc - 1 > sizeof(buf)) {
//		/* Can only happen if Zephyr limit is increased. */
//		printk("At most %lu bytes can be written.\n",
//		       (unsigned long)sizeof(buf));
//		return -EINVAL;
//	}
//	argc--;
//	argv++;
//	for (i = 0; i < argc; i++) {
//		if (parse_u8(argv[i], &buf[i])) {
//			printk("Argument %lu (%s) is not a byte.\n",
//			       i + 1, argv[i]);
//			goto bail;
//		}
//	}
//
//	do_write(offset, buf, i, true);
//	return 0;
//
// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
//}

//#ifdef CONFIG_FLASH_PAGE_LAYOUT
//static int flash_shell_page_count(int argc, char *argv[])
//{
//	size_t page_count;
//
//	if (check_flash_device()) {
//		return 0;
//	}
//
//	CANONICALIZE_ARGS(argc, argv);
//	if (argc) {
//		goto bail;
//	}
//
//	page_count = flash_get_page_count(flash_device);
//	printk("Flash device contains %lu pages.\n",
//	       (long unsigned int)page_count);
//	return 0;
//
// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
//}

//struct page_layout_data {
//	long unsigned int start_page;
//	long unsigned int end_page;
//};

//static bool page_layout_cb(const struct flash_pages_info *info, void *datav)
//{
//	struct page_layout_data *data = datav;
//	long unsigned int sz;
//
//	if (info->index < data->start_page) {
//		return true;
//	} else if (info->index > data->end_page) {
//		return false;
//	}
//
//	sz = info->size;
//	printk("\tPage %u: start 0x%08x, length 0x%lx (%lu, %lu KB)\n",
//	       info->index, info->start_offset, sz, sz, sz / KB(1));
//	return true;
//}

//static int flash_shell_page_layout(int argc, char *argv[])
//{
//	long unsigned int start_page, end_page;
//	struct page_layout_data data;
//
//	if (check_flash_device()) {
//		return 0;
//	}
//
//	CANONICALIZE_ARGS(argc, argv);
//	switch (argc) {
//	case 0:
//		start_page = 0;
//		end_page = flash_get_page_count(flash_device) - 1;
//		break;
//	case 1:
//		if (parse_ul(argv[0], &start_page)) {
//			goto bail;
//		}
//		end_page = flash_get_page_count(flash_device) - 1;
//		break;
//	case 2:
//		if (parse_ul(argv[0], &start_page) ||
//		    parse_ul(argv[1], &end_page)) {
//			goto bail;
//		}
//		break;
//	default:
//		goto bail;
//	}
//
//	data.start_page = start_page;
//	data.end_page = end_page;
//	flash_page_foreach(flash_device, page_layout_cb, &data);
//	return 0;
//
// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
//}

//static int flash_shell_read_page(int argc, char *argv[])
//{
//	long unsigned int page, offset, len;
//	struct flash_pages_info info;
//	int ret;
//
//	if (check_flash_device()) {
//		return 0;
//	}
//
//	CANONICALIZE_ARGS(argc, argv);
//	if (argc != 2 && argc != 3) {
//		goto bail;
//	}
//	if (argc == 2) {
//		if (parse_ul(argv[0], &page) || parse_ul(argv[1], &len)) {
//			goto bail;
//		}
//		offset = 0;
//	} else if (parse_ul(argv[0], &page) || parse_ul(argv[1], &offset) ||
//		   parse_ul(argv[2], &len)) {
//			goto bail;
//	}
//
//	ret = flash_get_page_info_by_idx(flash_device, page, &info);
//	if (ret) {
//		printk("flash_page_info_by_idx: %d\n", ret);
//		return 0;
//	}
//	offset += info.start_offset;
//	do_read(offset, len);
//	return 0;
//
// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
//}

//static int flash_shell_erase_page(int argc, char *argv[])
//{
//	struct flash_pages_info info;
//	long unsigned int i, page, num;
//	int ret;
//
//	if (check_flash_device()) {
//		return 0;
//	}
//
//	CANONICALIZE_ARGS(argc, argv);
//	if (argc < 1 || parse_ul(argv[0], &page)) {
//		goto bail;
//	}
//	if (argc == 1) {
//		num = 1;
//	} else if (parse_ul(argv[1], &num)) {
//		goto bail;
//	}
//
//	for (i = 0; i < num; i++) {
//		ret = flash_get_page_info_by_idx(flash_device, page + i, &info);
//		if (ret) {
//			printk("flash_get_page_info_by_idx: %d\n", ret);
//			return 0;
//		}
//		printk("Erasing page %u (start offset 0x%x, size 0x%x)\n",
//		       info.index, info.start_offset, info.size);
//		ret = do_erase(info.start_offset, info.size);
//		if (ret) {
//			return 0;
//		}
//	}
//
//	return 0;
//
// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
//}

//static int flash_shell_write_page(int argc, char *argv[])
//{
//	struct flash_pages_info info;
//	long unsigned int page, off;
//	u8_t buf[ARGC_MAX];
//	size_t i;
//	int ret;
//
//	if (check_flash_device()) {
//		return 0;
//	}
//
//	CANONICALIZE_ARGS(argc, argv);
//	if (argc < 2 || parse_ul(argv[0], &page) || parse_ul(argv[1], &off)) {
//		goto bail;
//	}
//	argc -= 2;
//	argv += 2;
//	for (i = 0; i < argc; i++) {
//		if (parse_u8(argv[i], &buf[i])) {
//			printk("Argument %d (%s) is not a byte.\n",
//			       i + 2, argv[i]);
//			goto bail;
//		}
//	}
//
//	ret = flash_get_page_info_by_idx(flash_device, page, &info);
//	if (ret) {
//		printk("flash_get_page_info_by_idx: %d\n", ret);
//		return 0;
//	}
//	do_write(info.start_offset + off, buf, i, true);
//	return 0;
//
// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
//}
//#endif	/* CONFIG_FLASH_PAGE_LAYOUT */

//static int flash_shell_set_device(int argc, char *argv[])
//{
//	struct device *dev;
//	const char *name;
//
//	/* Parse arguments. */
//	CANONICALIZE_ARGS(argc, argv);
//	if (argc != 1) {
//		goto bail;
//	}
//	name = argv[0];
//
//	/* Run command. */
//	dev = device_get_binding(name);
//	if (!dev) {
//		printk("No device named %s.\n", name);
//		return -EINVAL;
//	}
//	if (flash_device) {
//		printk("Leaving behind device %s", flash_device->config->name);
//	}
//	flash_device = dev;
//	return 0;
//
// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
//}

static int i2c_init(int argc, char *argv[])
{
	struct device *dev;
	const char *name;

	/* Parse arguments. */
	CANONICALIZE_ARGS(argc, argv);

	/*
	if (argc != 1) {
		goto bail;
	}

	name = argv[0];
    */

	/* Run command. */
	/*
	dev = device_get_binding(name);

	if (!dev) {
		printk("No device named %s.\n", name);
		return -EINVAL;
	}

	if (flash_device) {
		printk("Leaving behind device %s", flash_device->config->name);
	}

	flash_device = dev;
	*/

    /* ------------------------------ */

//    struct device *i2c_device;
//    u8_t cmp_data[16];
//    u8_t data[16];

    printk("i2c_init!!!!!!!!!\n");

//    i2c_device = device_get_binding(CONFIG_I2C_0_NAME);

//    if (!i2c_device) {
//        printk("I2C: Device driver not found.\n");
//        return -EINVAL;
//    }

    // special register editing
    i2c_reg_write_byte(i2c_device, I2C_ADDR, LCR << 3, 0x80); // divisor latch enable
    i2c_reg_write_byte(i2c_device, I2C_ADDR, DLL << 3, 0xa6); // divisor latch LSB
    i2c_reg_write_byte(i2c_device, I2C_ADDR, DLH << 3, 0x0e); // divisor latch MSB
    i2c_reg_write_byte(i2c_device, I2C_ADDR, LCR << 3, 0x00); // divisor latch disable

    i2c_reg_write_byte(i2c_device, I2C_ADDR, FCR << 3, 0x97); // fifo

    i2c_reg_write_byte(i2c_device, I2C_ADDR, LCR << 3, 1 << 6); // enable break

    printk("out\n");

    /* ------------------------------ */

	return 0;

// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
}

static int i2c_send(int argc, char *argv[])
{
    struct device *dev;
    const char *name;

    /* Parse arguments. */
    CANONICALIZE_ARGS(argc, argv);

    /*
    if (argc != 1) {
        goto bail;
    }

    name = argv[0];
    */

    /* Run command. */
    /*
    dev = device_get_binding(name);

    if (!dev) {
        printk("No device named %s.\n", name);
        return -EINVAL;
    }

    if (flash_device) {
        printk("Leaving behind device %s", flash_device->config->name);
    }

    flash_device = dev;
    */

    /* ------------------------------ */

//    struct device *i2c_device;
    u8_t cmp_data[16];
    u8_t data[16];

//    printk("i2c_send!!!!!!!!!\n");

//    i2c_device = device_get_binding(CONFIG_I2C_0_NAME);

//    if (!i2c_device) {
//        printk("I2C: Device driver not found.\n");
//        return -EINVAL;
//    }

    u8_t data2[] = {
            THR << 3,
            //0x01, 0x02, 0x03, 0x04, 0x05
            //0xe2, 0xf9, 0xf6, 0xf2, 0xfd, 0xf2, 0xfd, 0xfa, 0xf5, 0xe3 // 検針データ要求
            0xe2, 0xfe, 0xf1, 0xf2, 0xfd, 0xf2, 0xfd, 0xfa, 0xf5, 0xe3 // 弁開要求

/*
            0x02, // STX

            0x19, // 制御: 検針データ要求
            0x16, // 制御: 検針データ要求 (反転)

            0x12, // 事業体: ガス
            0x1d, // 事業体: ガス (反転)

            0x10, // 器番: ノーチェック
            0x1f, // 器番: ノーチェック (反転)

            0x10, // 器番: ノーチェック
            0x1f, // 器番: ノーチェック (反転)

            0x03, // ETX
            */
    };

    i2c_reg_write_byte(i2c_device, I2C_ADDR, LCR << 3, 0x00); // disable break

    k_sleep(150);

    i2c_write(i2c_device, data2, sizeof(data2), I2C_ADDR);

    i2c_reg_write_byte(i2c_device, I2C_ADDR, LCR << 3, 1 << 6); // enable break

     k_sleep(348);

    i2c_reg_write_byte(i2c_device, I2C_ADDR, LCR << 3, 0x00); // disable break

    k_sleep(500);

    i2c_reg_write_byte(i2c_device, I2C_ADDR, LCR << 3, 1 << 6); // enable break

    printk("i2c_send ok !!!!!!!!!\n");

    /* ------------------------------ */

    return 0;

// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
}

static int i2c_receive_buffer(int argc, char *argv[])
{
    struct device *dev;
    const char *name;

    /* Parse arguments. */
    CANONICALIZE_ARGS(argc, argv);

    /*
    if (argc != 1) {
        goto bail;
    }

    name = argv[0];
    */

    /* Run command. */
    /*
    dev = device_get_binding(name);

    if (!dev) {
        printk("No device named %s.\n", name);
        return -EINVAL;
    }

    if (flash_device) {
        printk("Leaving behind device %s", flash_device->config->name);
    }

    flash_device = dev;
    */

    /* ------------------------------ */

//    struct device *i2c_device;
//    u8_t cmp_data[16];
    u8_t data[16];

//    printk("i2c_receive!!!!!!!!!\n");

//    i2c_device = device_get_binding(CONFIG_I2C_0_NAME);

//    if (!i2c_device) {
//        printk("I2C: Device driver not found.\n");
//        return -EINVAL;
//    }

//    printk("i2c_receive ok !!!!!!!!!\n");

//    read_bytes(i2c_device, RXLVL << 3, &data[0], 1);

    i2c_reg_read_byte(i2c_device, I2C_ADDR, RXLVL << 3, data);

    printk("receive buffer length:%d\n", data[0]);

    /* ------------------------------ */

    return 0;

// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
}

static int i2c_receive(int argc, char *argv[])
{
    struct device *dev;
    const char *name;

    /* Parse arguments. */
    CANONICALIZE_ARGS(argc, argv);

    /*
    if (argc != 1) {
        goto bail;
    }

    name = argv[0];
    */

    /* Run command. */
    /*
    dev = device_get_binding(name);

    if (!dev) {
        printk("No device named %s.\n", name);
        return -EINVAL;
    }

    if (flash_device) {
        printk("Leaving behind device %s", flash_device->config->name);
    }

    flash_device = dev;
    */

    /* ------------------------------ */

//    struct device *i2c_device;
//    u8_t cmp_data[16];
    u8_t data[64];

    printk("i2c_receive!!!!!!!!!\n");

//    i2c_device = device_get_binding(CONFIG_I2C_0_NAME);

//    if (!i2c_device) {
//        printk("I2C: Device driver not found.\n");
//        return -EINVAL;
//    }

    printk("i2c_receive ok !!!!!!!!!\n");

    i2c_reg_read_byte(i2c_device, I2C_ADDR, RXLVL << 3, data);

    u8_t fifo_length = data[0];

    printk("receive buffer length:%d\n", fifo_length);

    // FIXME FIFOサイズと受信バッファサイズの整合性をチェックする

    int out = i2c_burst_read(i2c_device, I2C_ADDR, RHR << 3, data, fifo_length);

    printk("out: %d\n", out);

    for (int i = 0; i < fifo_length; i++) {
        printk("0x%02x\n", data[i]);
    }

    printk("received\n");

    /* ------------------------------ */

    return 0;

// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
}

static int i2c_break(int argc, char *argv[])
{
//    struct device *dev;
//    const char *name;

    /* Parse arguments. */
    CANONICALIZE_ARGS(argc, argv);

    /*
    if (argc != 1) {
        goto bail;
    }

    name = argv[0];
    */

    /* Run command. */
    /*
    dev = device_get_binding(name);

    if (!dev) {
        printk("No device named %s.\n", name);
        return -EINVAL;
    }

    if (flash_device) {
        printk("Leaving behind device %s", flash_device->config->name);
    }

    flash_device = dev;
    */

    /* ------------------------------ */

//    struct device *i2c_device;
//    u8_t cmp_data[16];
//    u8_t data[64];

    printk("i2c_break!!!!!!!!!\n");

    i2c_reg_write_byte(i2c_device, I2C_ADDR, LCR << 3, 1 << 6); // enable break

    printk("break\n");

    /* ------------------------------ */

    return 0;

// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
}

static int i2c_unbreak(int argc, char *argv[])
{
//    struct device *dev;
//    const char *name;

    /* Parse arguments. */
    CANONICALIZE_ARGS(argc, argv);

    /*
    if (argc != 1) {
        goto bail;
    }

    name = argv[0];
    */

    /* Run command. */
    /*
    dev = device_get_binding(name);

    if (!dev) {
        printk("No device named %s.\n", name);
        return -EINVAL;
    }

    if (flash_device) {
        printk("Leaving behind device %s", flash_device->config->name);
    }

    flash_device = dev;
    */

    /* ------------------------------ */

//    struct device *i2c_device;
//    u8_t cmp_data[16];
//    u8_t data[64];

    printk("i2c_break!!!!!!!!!\n");

    i2c_reg_write_byte(i2c_device, I2C_ADDR, LCR << 3, 0x00); // disable break

    printk("break\n");

    /* ------------------------------ */

    return 0;

// bail:
//	printk("Invalid arguments.\n");
//	return -EINVAL;
}

static struct shell_cmd commands[] = {
	{ "init", i2c_init, I2C_INIT },
	{ "send", i2c_send, I2C_INIT },
	{ "length", i2c_receive_buffer, I2C_INIT },
	{ "receive", i2c_receive, I2C_INIT },
	{ "break", i2c_break, I2C_INIT },
	{ "unbreak", i2c_unbreak, I2C_INIT },
	{ NULL, NULL, NULL },
};

void main(void)
{
    printk("i2c start initialize\n");

    i2c_device = device_get_binding(CONFIG_I2C_0_NAME);

    if (!i2c_device) {
        printk("I2C: Device driver not found.\n");
//        return -EINVAL;
        return; // FIXME エラー処理を追加する
    }

    printk("i2c finish initialize\n");

    /*
	flash_device = device_get_binding(FLASH_DEV_NAME);

	if (flash_device) {
		printk("Found flash device %s.\n", FLASH_DEV_NAME);
		printk("Flash I/O commands can be run.\n");
	} else {
		printk("**No flash device found!**\n");
		printk("Run set_device <name> to specify one before using other commands.\n");
	}
    */

	SHELL_REGISTER(THIS_MODULE_NAME, commands);
}
