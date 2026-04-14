#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pcie/pcie.h>
#include <zephyr/drivers/pcie/msi.h>
#include <zephyr/drivers/pcie/cap.h>
#include <zephyr/dt-bindings/pcie/pcie.h>

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

#ifndef BIT
#define BIT(n) (1UL << (n))
#endif

#define MY_PCIE_ID_VENDOR(id)   ((uint16_t)((id) & 0xFFFFu))
#define MY_PCIE_ID_DEVICE(id)   ((uint16_t)(((id) >> 16) & 0xFFFFu))

/* MSI-X capability layout */
#define MY_PCIE_MSIX_MSGCTL_OFFSET_WORD   0
#define MY_PCIE_MSIX_TABLE_OFFSET_WORD    1
#define MY_PCIE_MSIX_PBA_OFFSET_WORD      2

/* MSI-X Message Control bits */
#define MY_PCIE_MSIX_MSGCTL_TSIZE_MASK    0x07FF0000u
#define MY_PCIE_MSIX_MSGCTL_TSIZE_SHIFT   16
#define MY_PCIE_MSIX_MSGCTL_FMASK         BIT(30)
#define MY_PCIE_MSIX_MSGCTL_EN            BIT(31)

/* MSI-X Table / PBA fields */
#define MY_PCIE_MSIX_BIR_MASK             0x00000007u
#define MY_PCIE_MSIX_OFFSET_MASK          0xFFFFFFF8u

static const char *cap_name(uint8_t cap_id)
{
	switch (cap_id) {
	case PCI_CAP_ID_PM:
		return "PM";
	case PCI_CAP_ID_MSI:
		return "MSI";
	case PCI_CAP_ID_MSIX:
		return "MSI-X";
	case PCI_CAP_ID_EXP:
		return "PCIe";
	default:
		return "OTHER";
	}
}

static void dump_standard_header(pcie_bdf_t bdf)
{
	uint32_t id, cmdstat, classrev, type, intr;
	uint32_t bar;
	int i;

	id = pcie_conf_read(bdf, PCIE_CONF_ID);
	cmdstat = pcie_conf_read(bdf, PCIE_CONF_CMDSTAT);
	classrev = pcie_conf_read(bdf, PCIE_CONF_CLASSREV);
	type = pcie_conf_read(bdf, PCIE_CONF_TYPE);
	intr = pcie_conf_read(bdf, PCIE_CONF_INTR);

	printk("  ID       = 0x%08" PRIx32 "\n", id);
	printk("    vendor = 0x%04x device = 0x%04x\n",
	       MY_PCIE_ID_VENDOR(id), MY_PCIE_ID_DEVICE(id));

	printk("  CMDSTAT  = 0x%08" PRIx32 "\n", cmdstat);
	printk("    IO=%u MEM=%u MASTER=%u CAPS=%u INT_STATUS=%u\n",
	       !!(cmdstat & PCIE_CONF_CMDSTAT_IO),
	       !!(cmdstat & PCIE_CONF_CMDSTAT_MEM),
	       !!(cmdstat & PCIE_CONF_CMDSTAT_MASTER),
	       !!(cmdstat & PCIE_CONF_CMDSTAT_CAPS),
	       !!(cmdstat & PCIE_CONF_CMDSTAT_INTERRUPT));

	printk("  CLASSREV = 0x%08" PRIx32 "\n", classrev);
	printk("    class=0x%02x subclass=0x%02x progif=0x%02x rev=0x%02x\n",
	       PCIE_CONF_CLASSREV_CLASS(classrev),
	       PCIE_CONF_CLASSREV_SUBCLASS(classrev),
	       PCIE_CONF_CLASSREV_PROGIF(classrev),
	       PCIE_CONF_CLASSREV_REV(classrev));

	printk("  TYPE     = 0x%08" PRIx32 "\n", type);
	printk("    header_type=0x%x multifunction=%u bridge=%u\n",
	       PCIE_CONF_TYPE_GET(type),
	       PCIE_CONF_MULTIFUNCTION(type),
	       PCIE_CONF_TYPE_BRIDGE(type));

	printk("  INTR     = 0x%08" PRIx32 " irq=%u\n",
	       intr, PCIE_CONF_INTR_IRQ(intr));

	for (i = 0; i < 6; i++) {
		bar = pcie_conf_read(bdf, PCIE_CONF_BAR0 + i);
		printk("  BAR%d    = 0x%08" PRIx32 "\n", i, bar);
	}
}

static void dump_cap_summary(pcie_bdf_t bdf)
{
	uint32_t off_msi = pcie_get_cap(bdf, PCI_CAP_ID_MSI);
	uint32_t off_msix = pcie_get_cap(bdf, PCI_CAP_ID_MSIX);
	uint32_t off_pcie = pcie_get_cap(bdf, PCI_CAP_ID_EXP);

	printk("  Summary: MSI=%s", off_msi ? "yes" : "no");
	if (off_msi) {
		printk(" (word=%" PRIu32 " byte=0x%02" PRIx32 ")", off_msi, off_msi << 2);
	}
	printk("\n");

	printk("  Summary: MSI-X=%s", off_msix ? "yes" : "no");
	if (off_msix) {
		printk(" (word=%" PRIu32 " byte=0x%02" PRIx32 ")", off_msix, off_msix << 2);
	}
	printk("\n");

	printk("  Summary: PCIe Cap=%s", off_pcie ? "yes" : "no");
	if (off_pcie) {
		printk(" (word=%" PRIu32 " byte=0x%02" PRIx32 ")", off_pcie, off_pcie << 2);
	}
	printk("\n");

	printk("  pcie_is_msi() = %d\n", pcie_is_msi(bdf));
}

static void dump_capability_chain(pcie_bdf_t bdf)
{
	uint32_t w;
	uint32_t cap_reg;
	int guard = 0;

	w = pcie_conf_read(bdf, PCIE_CONF_CAPPTR);
	if (w == 0xFFFFFFFFu) {
		printk("  CAP: cannot read capability pointer word\n");
		return;
	}

	cap_reg = PCIE_CONF_CAPPTR_FIRST(w);

	printk("  CAP_PTR word[%u] = 0x%08" PRIx32
	       ", first_reg=%" PRIu32 " first_byte=0x%02" PRIx32 "\n",
	       PCIE_CONF_CAPPTR, w, cap_reg, cap_reg << 2);

	if (cap_reg == 0U) {
		printk("  No conventional capability list\n");
		return;
	}

	while (cap_reg != 0U && guard < 32) {
		uint32_t capw = pcie_conf_read(bdf, cap_reg);
		uint8_t cap_id;
		uint32_t next_reg;

		if (capw == 0xFFFFFFFFu) {
			printk("  CAP reg[%" PRIu32 "] read failed\n", cap_reg);
			break;
		}

		cap_id = PCIE_CONF_CAP_ID(capw);
		next_reg = PCIE_CONF_CAP_NEXT(capw);

		printk("  CAP reg[%" PRIu32 "] byte=0x%02" PRIx32
		       " val=0x%08" PRIx32 " id=0x%02x (%s)"
		       " next_reg=%" PRIu32 " next_byte=0x%02" PRIx32 "\n",
		       cap_reg, cap_reg << 2, capw, cap_id, cap_name(cap_id),
		       next_reg, next_reg << 2);

		cap_reg = next_reg;
		guard++;
	}

	if (guard >= 32) {
		printk("  Capability walk stopped by guard\n");
	}
}

static void dump_msix_capability_detail(pcie_bdf_t bdf)
{
	uint32_t msix_cap_reg;
	uint32_t msgctl_word;
	uint32_t table_word;
	uint32_t pba_word;
	uint32_t table_size;
	uint32_t table_bir;
	uint32_t table_offset;
	uint32_t pba_bir;
	uint32_t pba_offset;
	uint32_t bar[6];
	int i;

	msix_cap_reg = pcie_get_cap(bdf, PCI_CAP_ID_MSIX);

	if (msix_cap_reg == 0U) {
		printk("  MSI-X detail: not present\n");
		return;
	}

	msgctl_word = pcie_conf_read(bdf, msix_cap_reg + MY_PCIE_MSIX_MSGCTL_OFFSET_WORD);
	table_word  = pcie_conf_read(bdf, msix_cap_reg + MY_PCIE_MSIX_TABLE_OFFSET_WORD);
	pba_word    = pcie_conf_read(bdf, msix_cap_reg + MY_PCIE_MSIX_PBA_OFFSET_WORD);

	if ((msgctl_word == 0xFFFFFFFFu) ||
	    (table_word == 0xFFFFFFFFu) ||
	    (pba_word == 0xFFFFFFFFu)) {
		printk("  MSI-X detail: config read failed\n");
		return;
	}

	table_size   = ((msgctl_word & MY_PCIE_MSIX_MSGCTL_TSIZE_MASK) >>
			MY_PCIE_MSIX_MSGCTL_TSIZE_SHIFT) + 1U;

	table_bir    = table_word & MY_PCIE_MSIX_BIR_MASK;
	table_offset = table_word & MY_PCIE_MSIX_OFFSET_MASK;

	pba_bir      = pba_word & MY_PCIE_MSIX_BIR_MASK;
	pba_offset   = pba_word & MY_PCIE_MSIX_OFFSET_MASK;

	printk("  MSI-X detail:\n");
	printk("    cap_reg      = %" PRIu32 " (byte=0x%02" PRIx32 ")\n",
	       msix_cap_reg, msix_cap_reg << 2);
	printk("    msgctl_word  = 0x%08" PRIx32 "\n", msgctl_word);
	printk("      enable     = %u\n", !!(msgctl_word & MY_PCIE_MSIX_MSGCTL_EN));
	printk("      func_mask  = %u\n", !!(msgctl_word & MY_PCIE_MSIX_MSGCTL_FMASK));
	printk("      table_size = %" PRIu32 "\n", table_size);

	printk("    table_word   = 0x%08" PRIx32 "\n", table_word);
	printk("      table_bir  = %" PRIu32 "\n", table_bir);
	printk("      table_off  = 0x%08" PRIx32 "\n", table_offset);

	printk("    pba_word     = 0x%08" PRIx32 "\n", pba_word);
	printk("      pba_bir    = %" PRIu32 "\n", pba_bir);
	printk("      pba_off    = 0x%08" PRIx32 "\n", pba_offset);

	for (i = 0; i < 6; i++) {
		bar[i] = pcie_conf_read(bdf, PCIE_CONF_BAR0 + i);
		printk("    BAR%d        = 0x%08" PRIx32 "\n", i, bar[i]);
	}

	if (table_bir < 6U) {
		uint32_t bar_base = bar[table_bir] & ~0xFu;

		printk("    table_addr   = BAR%" PRIu32 " + 0x%08" PRIx32
		       " => approx 0x%08" PRIx32 "\n",
		       table_bir, table_offset, bar_base + table_offset);
	}

	if (pba_bir < 6U) {
		uint32_t bar_base = bar[pba_bir] & ~0xFu;

		printk("    pba_addr     = BAR%" PRIu32 " + 0x%08" PRIx32
		       " => approx 0x%08" PRIx32 "\n",
		       pba_bir, pba_offset, bar_base + pba_offset);
	}

	printk("    estimated table bytes = %" PRIu32 "\n", table_size * 16U);
}

static bool scan_cb(pcie_bdf_t bdf, pcie_id_t id, void *cb_data)
{
	ARG_UNUSED(cb_data);

	printk("\n[PCIe] BDF %02x:%02x.%x\n",
	       PCIE_BDF_TO_BUS(bdf),
	       PCIE_BDF_TO_DEV(bdf),
	       PCIE_BDF_TO_FUNC(bdf));

	printk("  callback id: vendor=0x%04x device=0x%04x\n",
	       MY_PCIE_ID_VENDOR(id), MY_PCIE_ID_DEVICE(id));

	if (!PCIE_ID_IS_VALID(id)) {
		printk("  invalid endpoint id\n");
		return true;
	}

	dump_standard_header(bdf);
	dump_cap_summary(bdf);
	dump_capability_chain(bdf);
	dump_msix_capability_detail(bdf);

	return true;
}

void main(void)
{
	int ret;
	struct pcie_scan_opt opt = {
		.cb = scan_cb,
		.cb_data = NULL,
		.flags = PCIE_SCAN_RECURSIVE | PCIE_SCAN_CB_ALL,
	};

	printk("=== PCIe scan start ===\n");

	ret = pcie_scan(&opt);
	printk("pcie_scan() ret = %d\n", ret);
}