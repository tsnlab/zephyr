#include <stdint.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include "lan8651.h"
#include "spi.h"

/* FIXME: These are borrowed from our T1S demo */

uint8_t get_parity(uint32_t valueToCalculateParity)
{
	valueToCalculateParity &= 0xFFFFFFFE;
	valueToCalculateParity ^= valueToCalculateParity >> 1;
	valueToCalculateParity ^= valueToCalculateParity >> 2;
	valueToCalculateParity = ((valueToCalculateParity & 0x11111111) * 0x11111111);
	return !((valueToCalculateParity >> 28) & 1);
}

static bool t1s_hw_readreg(const struct spi_dt_spec *spi_dev, struct ctrl_cmd_reg *p_regInfoInput,
			   struct ctrl_cmd_reg *p_readRegData)
{
	const uint8_t ignored_echoedbytes = HEADER_SIZE;
	bool readstatus = false;
	uint8_t txbuffer[MAX_REG_DATA_ONECONTROLCMD + HEADER_SIZE + REG_SIZE] = {0};
	uint8_t rxbuffer[MAX_REG_DATA_ONECONTROLCMD + HEADER_SIZE + REG_SIZE] = {0};
	uint16_t numberof_bytestosend = 0;
	union ctrl_header commandheader;
	union ctrl_header commandheader_echoed;
	uint32_t converted_commandheader;

	commandheader.ctrl_frame_head = 0;
	commandheader_echoed.ctrl_frame_head = 0;
	commandheader.ctrl_head_bits.dnc = DNC_COMMANDTYPE_CONTROL;
	commandheader.ctrl_head_bits.hdrb = 0;
	commandheader.ctrl_head_bits.wnr = REG_COMMAND_TYPE_READ; // Read from register
	if (p_regInfoInput->length != 0) {
		commandheader.ctrl_head_bits.aid =
			REG_ADDR_INCREMENT_ENABLE; // Read register continously from given address
	} else {
		commandheader.ctrl_head_bits.aid =
			REG_ADDR_INCREMENT_DISABLE; // Read from same register
	}
	commandheader.ctrl_head_bits.mms = (uint32_t)p_regInfoInput->memorymap;
	commandheader.ctrl_head_bits.addr = (uint32_t)p_regInfoInput->address;
	commandheader.ctrl_head_bits.len = (uint32_t)(p_regInfoInput->length & 0x7F);
	commandheader.ctrl_head_bits.p = 0;
	commandheader.ctrl_head_bits.p = get_parity(commandheader.ctrl_frame_head);

	converted_commandheader = sys_cpu_to_be32(commandheader.ctrl_frame_head);
	memcpy(txbuffer, &converted_commandheader, HEADER_SIZE);

	numberof_bytestosend = HEADER_SIZE + ((commandheader.ctrl_head_bits.len + 1) * REG_SIZE) +
			       ignored_echoedbytes; // Added extra 4 bytes because first 4 bytes
						    // during reception shall be ignored

	transceive(txbuffer, rxbuffer, numberof_bytestosend);

	memcpy((uint8_t *)&commandheader_echoed.ctrl_frame_head, &rxbuffer[ignored_echoedbytes],
	       HEADER_SIZE);
	commandheader_echoed.ctrl_frame_head =
		sys_be32_to_cpu(commandheader_echoed.ctrl_frame_head);

	if (commandheader_echoed.ctrl_head_bits.hdrb !=
	    1) // if MACPHY received header with parity error then it will be 1
	{
		if (commandheader.ctrl_head_bits.len == 0) {
			memcpy((uint8_t *)&p_readRegData->databuffer[0],
			       &(rxbuffer[ignored_echoedbytes + HEADER_SIZE]), REG_SIZE);
			p_readRegData->databuffer[0] =
				sys_be32_to_cpu(p_readRegData->databuffer[0]);
		} else {
			for (int regCount = 0; regCount <= commandheader.ctrl_head_bits.len;
			     regCount++) {
				memcpy((uint8_t *)&p_readRegData->databuffer[regCount],
				       &rxbuffer[ignored_echoedbytes + HEADER_SIZE +
						 (REG_SIZE * regCount)],
				       REG_SIZE);
				p_readRegData->databuffer[regCount] =
					sys_be32_to_cpu(p_readRegData->databuffer[regCount]);
			}
		}
		readstatus = true;
	} else {
		// TODO: Error handling if MACPHY received with header parity error
		printk("Parity Error READMACPHYReg header value : 0x%08x\n",
		       commandheader_echoed.ctrl_frame_head);
	}

	return readstatus;
}

static bool t1s_hw_writereg(const struct spi_dt_spec *spi_dev, struct ctrl_cmd_reg *p_regData)
{
	uint8_t numberof_bytestosend = 0;
	uint8_t numberof_registerstosend = 0;
	bool writestatus = true;
	bool execution_status = false;
	const uint8_t ignored_echoedbytes = HEADER_SIZE;
	uint8_t txbuffer[MAX_PAYLOAD_BYTE + HEADER_SIZE] = {0};
	uint8_t rxbuffer[MAX_PAYLOAD_BYTE + HEADER_SIZE] = {0};
	union ctrl_header commandheader;
	union ctrl_header commandheader_echoed;
	uint32_t converted_commandheader;

	commandheader.ctrl_frame_head = commandheader_echoed.ctrl_frame_head = 0;

	commandheader.ctrl_head_bits.dnc = DNC_COMMANDTYPE_CONTROL;
	commandheader.ctrl_head_bits.hdrb = 0;
	commandheader.ctrl_head_bits.wnr = REG_COMMAND_TYPE_WRITE; // Write into register
	if (p_regData->length != 0) {
		commandheader.ctrl_head_bits.aid =
			REG_ADDR_INCREMENT_ENABLE; // Write register continously from given address
	} else {
		commandheader.ctrl_head_bits.aid =
			REG_ADDR_INCREMENT_DISABLE; // Write into same register
	}
	commandheader.ctrl_head_bits.mms = (uint32_t)(p_regData->memorymap & 0x0F);
	commandheader.ctrl_head_bits.addr = (uint32_t)p_regData->address;
	commandheader.ctrl_head_bits.len = (uint32_t)(p_regData->length & 0x7F);
	commandheader.ctrl_head_bits.p = 0;
	commandheader.ctrl_head_bits.p = get_parity(commandheader.ctrl_frame_head);

	converted_commandheader = sys_cpu_to_be32(commandheader.ctrl_frame_head);
	memcpy(txbuffer, &converted_commandheader, HEADER_SIZE);

	numberof_registerstosend = commandheader.ctrl_head_bits.len + 1;
	for (uint8_t controlRegIndex = 0; controlRegIndex < numberof_registerstosend;
	     controlRegIndex++) {
		uint32_t data = sys_cpu_to_be32(p_regData->databuffer[controlRegIndex]);
		memcpy(&txbuffer[HEADER_SIZE + controlRegIndex * REG_SIZE], &data,
		       sizeof(uint32_t));
	}

	numberof_bytestosend = HEADER_SIZE + ((commandheader.ctrl_head_bits.len + 1) * REG_SIZE) +
			       ignored_echoedbytes; // Added extra 4 bytes because last 4 bytes of
						    // payload will be ignored by MACPHY

	transceive(txbuffer, rxbuffer, numberof_bytestosend);

	memcpy((uint8_t *)&commandheader_echoed.ctrl_frame_head, &rxbuffer[ignored_echoedbytes],
	       HEADER_SIZE);
	commandheader_echoed.ctrl_frame_head =
		sys_be32_to_cpu(commandheader_echoed.ctrl_frame_head);
	// TODO: check this logic and modify it if needed
	if (commandheader.ctrl_head_bits.mms == 0) {
		struct ctrl_cmd_reg readreg_infoinput;
		struct ctrl_cmd_reg readreg_data;

		// Reads CONFIG0 register from MMS 0
		readreg_infoinput.memorymap = MMS0;
		readreg_infoinput.length = 0;
		readreg_infoinput.address = OA_CONFIG0;
		memset(&readreg_infoinput.databuffer[0], 0, MAX_REG_DATA_ONECHUNK);

		execution_status = t1s_hw_readreg(spi_dev, &readreg_infoinput, &readreg_data);
		if (execution_status == false) {
			printk("Reading CONFIG0 reg failed after writing (inside WriteReg)\n");
		} else {
			// printk("CONFIG0 reg value is 0x%08x in WriteReg function\n", readreg_data.databuffer[0]);
		}
	}

	return writestatus;
}

uint32_t write_register(const struct spi_dt_spec *spi_dev, uint8_t mms, uint16_t address,
			uint32_t data)
{
	struct ctrl_cmd_reg writereg_input;
	bool execution_status = false;
	writereg_input.memorymap = mms;
	writereg_input.length = 0;
	writereg_input.address = address;
	writereg_input.databuffer[0] = data;

	execution_status = t1s_hw_writereg(spi_dev, &writereg_input);
	if (execution_status == true) {
		return writereg_input.databuffer[0];
	} else {
		printk("ERROR: Register Write failed at MMS %d, Address %4x\n", mms, address);
		return 0;
	}
}

uint32_t read_register(const struct spi_dt_spec *spi_dev, uint8_t mms, uint16_t address)
{
	bool execution_status = false;
	struct ctrl_cmd_reg readreg_infoinput;
	struct ctrl_cmd_reg readreg_data;
	readreg_infoinput.memorymap = mms;
	readreg_infoinput.length = 0;
	readreg_infoinput.address = address;

	execution_status = t1s_hw_readreg(spi_dev, &readreg_infoinput, &readreg_data);
	if (execution_status == true) {
		return readreg_data.databuffer[0];
	} else {
		printk("ERROR: Register Read failed at MMS %d, Address %4x\n", mms, address);
		return 0;
	}
}

static void set_macphy_register(const struct spi_dt_spec *spi_dev)
{
	uint8_t value1 = read_register(spi_dev, 0x04, 0x1F);
	int8_t offset1;
	if ((value1 & 0x10) != 0) {
		offset1 = (int8_t)((uint8_t)value1 - 0x20);
	} else {
		offset1 = (int8_t)value1;
	}

	uint8_t value2 = read_register(spi_dev, 0x08, 0x1F);
	int8_t offset2;
	if ((value2 & 0x10) != 0) {
		offset2 = (int8_t)((uint8_t)value2 - 0x20);
	} else {
		offset2 = (int8_t)value2;
	}

	uint16_t cfgparam1 = (uint16_t)(((9 + offset1) & 0x3F) << 10) |
			     (uint16_t)(((14 + offset1) & 0x3F) << 4) | 0x03;
	uint16_t cfgparam2 = (uint16_t)(((40 + offset2) & 0x3F) << 10);

	write_register(spi_dev, MMS4, 0x00D0, 0x3F31);
	write_register(spi_dev, MMS4, 0x00E0, 0xC000);
	write_register(spi_dev, MMS4, 0x0084, cfgparam1);
	write_register(spi_dev, MMS4, 0x008A, cfgparam2);
	write_register(spi_dev, MMS4, 0x00E9, 0x9E50);
	write_register(spi_dev, MMS4, 0x00F5, 0x1CF8);
	write_register(spi_dev, MMS4, 0x00F4, 0xC020);
	write_register(spi_dev, MMS4, 0x00F8, 0xB900);
	write_register(spi_dev, MMS4, 0x00F9, 0x4E53);
	write_register(spi_dev, MMS4, 0x0081, 0x0080);
	write_register(spi_dev, MMS4, 0x0091, 0x9660);
	write_register(spi_dev, MMS4, 0x0077, 0x0028);
	write_register(spi_dev, MMS4, 0x0043, 0x00FF);
	write_register(spi_dev, MMS4, 0x0044, 0xFFFF);
	write_register(spi_dev, MMS4, 0x0045, 0x0000);
	write_register(spi_dev, MMS4, 0x0053, 0x00FF);
	write_register(spi_dev, MMS4, 0x0054, 0xFFFF);
	write_register(spi_dev, MMS4, 0x0055, 0x0000);
	write_register(spi_dev, MMS4, 0x0040, 0x0002);
	write_register(spi_dev, MMS4, 0x0050, 0x0002);
}

int set_register(const struct spi_dt_spec *spi_dev, uint8_t node_count, uint8_t node_id, const uint8_t* mac_address)
{
	uint32_t regval;

	/* Read OA_STATUS0 */
	regval = read_register(spi_dev, MMS0, OA_STATUS0);
	printk("OA_STATUS0: 0x%08x\n", regval);

	/* Write 1 to RESETC bit of OA_STATUS0 */
	regval |= (1 << 6);
	write_register(spi_dev, MMS0, OA_STATUS0, regval);
	printk("OA_STATUS0: 0x%08x\n", regval);

	regval = read_register(spi_dev, MMS4, CDCTL0);
	write_register(spi_dev, MMS4, CDCTL0,
		       regval | (1 << 15)); // Initial logic (disable collision detection)
	printk("CDCTL0: 0x%08x\n", regval);

	// PLCA Configuration based on mode
	// TODO: This process is temporary and assumes that there are only two nodes.
	// TODO: Should be changed to get node info. from the command line.
	
	write_register(spi_dev, MMS4, PLCA_CTRL1,
				((uint32_t)node_id) | ((uint32_t)node_count << 8));
	write_register(spi_dev, MMS1, MAC_SAB1,
				((uint32_t)*(mac_address + 0)) | ((uint32_t)*(mac_address + 1) << 8) |
				((uint32_t)*(mac_address + 2) << 16) | ((uint32_t)*(mac_address + 3) << 24));
	write_register(spi_dev, MMS1, MAC_SAT1,
				((uint32_t)*(mac_address + 4)) | ((uint32_t)*(mac_address + 5) << 8));
	printk("PLCA_CTRL1: 0x%08x\n", read_register(spi_dev, MMS4, PLCA_CTRL1));
	printk("MAC_SAB1: 0x%08x\n", read_register(spi_dev, MMS1, MAC_SAB1));
	printk("MAC_SAT1: 0x%08x\n", read_register(spi_dev, MMS1, MAC_SAT1));

	set_macphy_register(spi_dev); // AN_LAN865x-Configuration

	write_register(spi_dev, MMS4, PLCA_CTRL0, 0x00008000); // Enable PLCA
	write_register(spi_dev, MMS1, MAC_NCFGR, 0x000000C0);  // Enable unicast, multicast
	write_register(spi_dev, MMS1, MAC_NCR, 0x0000000C);    // Enable MACPHY TX, RX
	printk("PLCA_CTRL0: 0x%08x\n", read_register(spi_dev, MMS4, PLCA_CTRL0));
	printk("MAC_NCFGR: 0x%08x\n", read_register(spi_dev, MMS1, MAC_NCFGR));
	printk("MAC_NCR: 0x%08x\n", read_register(spi_dev, MMS1, MAC_NCR));

	/* Read OA_CONFIG0 */
	regval = read_register(spi_dev, MMS0, OA_CONFIG0);
	printk("OA_CONFIG0: 0x%08x\n", regval);

	/* Set SYNC bit of OA_CONFIG0 */
	regval |= (1 << 15);
	regval &= ~(0x7);
	regval |= (0x6);
	write_register(spi_dev, MMS0, OA_CONFIG0, regval);
	printk("OA_CONFIG0: 0x%08x\n", regval);

	/* Read OA_STATUS0 */
	regval = read_register(spi_dev, MMS0, OA_STATUS0);
	printk("OA_STATUS0: 0x%08x\n", regval);

	/* Clear RESETC bit of OA_STATUS0 */
	regval &= ~(1UL << 6);
	write_register(spi_dev, MMS0, OA_STATUS0, regval);
	printk("OA_STATUS0: 0x%08x\n", regval);

	return true;
}

int send_packet(const struct spi_dt_spec *spi_dev, const uint8_t *data, uint16_t length)
{
	if (length > MAX_PACKET_SIZE) {
		printk("Packet size is too large: %d\n", length);
		return -1;
	}

	uint8_t tx_buffer[HEADER_SIZE + MAX_PAYLOAD_BYTE] = {0};
	uint8_t rx_buffer[MAX_PAYLOAD_BYTE + FOOTER_SIZE] = {0};

	union data_header header;
	uint32_t be_header;

	uint32_t chunk_count = length / MAX_PAYLOAD_BYTE;
	if (length % 64) {
		chunk_count++;
	}

	uint32_t chunk_size = MAX_PAYLOAD_BYTE;

	header.tx_header_bits.dnc = DNC_COMMANDTYPE_DATA;
	header.tx_header_bits.norx = NORX_NO_RECEIVE;
	header.tx_header_bits.dv = DV_DATA_VALID;
	for (uint32_t i = 0; i < chunk_count; i++) {
		header.tx_header_bits.sv = SV_START_INVALID;
		header.tx_header_bits.ev = EV_END_INVALID;
		header.tx_header_bits.ebo = 0;

		if (i == 0) { // First chunk
			header.tx_header_bits.sv = SV_START_VALID;
		}
		if (i == chunk_count - 1) {
			chunk_size = length % MAX_PAYLOAD_BYTE;
			if (chunk_size == 0) {
				chunk_size = MAX_PAYLOAD_BYTE;
			}
			header.tx_header_bits.ev = EV_END_VALID;
			header.tx_header_bits.ebo = chunk_size - 1;
		}

		header.tx_header_bits.p = 0;
		header.tx_header_bits.p = get_parity(header.data_frame_head);

		be_header = sys_cpu_to_be32(header.data_frame_head);
		memcpy(tx_buffer, &be_header, HEADER_SIZE);
		memcpy(tx_buffer + HEADER_SIZE, data + i * MAX_PAYLOAD_BYTE, MAX_PAYLOAD_BYTE);

		struct spi_buf tx_buf = {.buf = tx_buffer, .len = HEADER_SIZE + MAX_PAYLOAD_BYTE};
		const struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
		struct spi_buf rx_buf = {.buf = rx_buffer, .len = MAX_PAYLOAD_BYTE + FOOTER_SIZE};
		const struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};
		if (spi_transceive_dt(spi_dev, &tx_buf_set, &rx_buf_set) != 0) {
			printk("SPI transceive failed while sending packet\n");
			return -1;
		}
	}

	return 0;
}

int receive_packet(const struct spi_dt_spec *spi_dev, uint8_t *data, uint16_t *length)
{
	uint8_t tx_buffer[HEADER_SIZE + MAX_PAYLOAD_BYTE] = {0};
	uint8_t rx_buffer[MAX_PAYLOAD_BYTE + FOOTER_SIZE] = {0};

	union data_header header;
	union data_footer footer;
	uint32_t be_header;
	uint32_t be_footer;
	size_t data_offset = 0;
	size_t chunk_size = MAX_PAYLOAD_BYTE;
	bool finished = false;

	*length = 0;

	header.tx_header_bits.dnc = DNC_COMMANDTYPE_DATA;
	header.tx_header_bits.norx = NORX_RECEIVE;
	header.tx_header_bits.dv = DV_DATA_INVALID;
	header.tx_header_bits.sv = SV_START_INVALID;
	header.tx_header_bits.ev = EV_END_INVALID;
	header.tx_header_bits.ebo = 0;
	header.tx_header_bits.p = get_parity(header.data_frame_head);
	while (*length < MAX_PACKET_SIZE && !finished) {
		be_header = sys_cpu_to_be32(header.data_frame_head);
		memcpy(tx_buffer, &be_header, HEADER_SIZE);

		struct spi_buf tx_buf = {.buf = tx_buffer, .len = HEADER_SIZE + MAX_PAYLOAD_BYTE};
		const struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
		struct spi_buf rx_buf = {.buf = rx_buffer, .len = MAX_PAYLOAD_BYTE + FOOTER_SIZE};
		const struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};
		if (spi_transceive_dt(spi_dev, &tx_buf_set, &rx_buf_set) != 0) {
			printk("SPI transceive failed while receiving packet\n");
			return -1;
		}

		be_footer = *((uint32_t *)(rx_buffer + MAX_PAYLOAD_BYTE));
		footer.data_frame_foot = sys_be32_to_cpu(be_footer);

		if (footer.rx_footer_bits.p != get_parity(footer.data_frame_foot)) {
			printk("Parity error while receiving packet\n");
			return -1;
		}

		if (*length == 0) { /* First chunk */
			if (footer.rx_footer_bits.dv == DV_DATA_INVALID) {
				/* No data to be received */
				return 0;
			}

			if (footer.rx_footer_bits.sv == SV_START_INVALID) {
				//printk("Invalid start of the packet\n");
				return -1;
			}

			data_offset = (footer.rx_footer_bits.swo * 4); /* 4 bytes per word */
		} else {
			if (footer.rx_footer_bits.sv == SV_START_VALID) {
				//printk("Unexpected start of the packet\n");
				return -1;
			}
		}

		if (footer.rx_footer_bits.ev == EV_END_VALID) { /* Last chunk */
			chunk_size = footer.rx_footer_bits.ebo + 1;
			finished = true;
		}

		memcpy(data + *length, rx_buffer + data_offset, chunk_size - data_offset);
		*length += chunk_size - data_offset;
		data_offset = 0;
	}

	return 0;
}
