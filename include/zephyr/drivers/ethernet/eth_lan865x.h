/*
 * Copyright (c) 2024 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ETH_LAN865X_H__
#define ZEPHYR_INCLUDE_DRIVERS_ETH_LAN865X_H__

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

/**
 * @brief      Read C22 registers using LAN865X MDIO Bus
 *
 * This routine provides an interface to perform a C22 register read on the
 * LAN865X MDIO bus.
 *
 * @param[in]  dev         Pointer to the device structure for the controller
 * @param[in]  prtad       Port address
 * @param[in]  regad       Register address
 * @param      data        Pointer to receive read data
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 * @retval -ETIMEDOUT If transaction timedout on the bus
 * @retval -ENOSYS if read is not supported
 */
int eth_lan865x_mdio_c22_read(const struct device *dev, uint8_t prtad, uint8_t regad,
			      uint16_t *data);

/**
 * @brief      Write C22 registers using LAN865X MDIO Bus
 *
 * This routine provides an interface to perform a C22 register write on the
 * LAN865X MDIO bus.
 *
 * @param[in]  dev         Pointer to the device structure for the controller
 * @param[in]  prtad       Port address
 * @param[in]  regad       Register address
 * @param[in]  data        Write data
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 * @retval -ETIMEDOUT If transaction timedout on the bus
 * @retval -ENOSYS if read is not supported
 */
int eth_lan865x_mdio_c22_write(const struct device *dev, uint8_t prtad, uint8_t regad,
			       uint16_t data);

/**
 * @brief      Read C45 registers using LAN865X MDIO Bus
 *
 * This routine provides an interface to perform a C45 register read on the
 * LAN865X MDIO bus.
 *
 * @param[in]  dev         Pointer to the device structure for the controller
 * @param[in]  prtad       Port address
 * @param[in]  devad       MMD device address
 * @param[in]  regad       Register address
 * @param      data        Pointer to receive read data
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 * @retval -ETIMEDOUT If transaction timedout on the bus
 * @retval -ENOSYS if read is not supported
 */
int eth_lan865x_mdio_c45_read(const struct device *dev, uint8_t prtad, uint8_t devad,
			      uint16_t regad, uint16_t *data);

/**
 * @brief      Write C45 registers using LAN865X MDIO Bus
 *
 * This routine provides an interface to perform a C45 register write on the
 * LAN865X MDIO bus.
 *
 * @param[in]  dev         Pointer to the device structure for the controller
 * @param[in]  prtad       Port address
 * @param[in]  devad       MMD device address
 * @param[in]  regad       Register address
 * @param[in]  data        Write data
 *
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 * @retval -ETIMEDOUT If transaction timedout on the bus
 * @retval -ENOSYS if read is not supported
 */
int eth_lan865x_mdio_c45_write(const struct device *dev, uint8_t prtad, uint8_t devad,
			       uint16_t regad, uint16_t data);

/**
 * @brief Transmit a raw Ethernet frame through the LAN865x device.
 *
 * This function provides an application-accessible interface to send a
 * complete Ethernet frame directly through the LAN865x driver without
 * relying on the Zephyr network stack transmission path.
 *
 * Internally, the function allocates a net_pkt, copies the provided frame
 * data into the packet buffer, and forwards it to the OA-TC6 transport
 * layer (oa_tc6_send_chunks) which transmits the frame over SPI to the
 * LAN865x PHY.
 *
 * This API is primarily intended for bring-up, debugging, and validation
 * of the SPI + LAN865x + 10BASE-T1S transmission path.
 *
 * @param dev  Pointer to the LAN865x device instance.
 * @param data Pointer to the raw Ethernet frame buffer to transmit.
 * @param len  Length of the Ethernet frame in bytes.
 *
 * @retval 0        Frame successfully queued for transmission.
 * @retval -ENOMEM  Packet allocation failed.
 * @retval -EIO     Device not ready or reset not completed.
 * @retval <0       Error returned from the underlying TC6 transmission.
 */
int lan865x_tx_frame(const struct device *dev, const uint8_t *data, size_t len);


/**
 * @brief LAN865x receive callback prototype
 *
 * The callback is called by the LAN865x driver when received Ethernet
 * frame data is read from the device. The driver may invoke the callback
 * multiple times for a single frame if the frame spans multiple network
 * buffer fragments.
 *
 * The callback runs in the driver RX thread context.
 *
 * @param data Pointer to received data fragment
 * @param len Length of the data fragment in bytes
 * @param user_data User-defined pointer supplied during callback registration
 */
typedef void (*lan865x_rx_cb_t)(const uint8_t *data,
                                size_t len,
                                void *user_data);

								/**
 * @brief Register an RX callback for LAN865x received data.
 *
 * This function allows an application to register a callback that will be
 * invoked whenever the LAN865x driver receives Ethernet frame data from the
 * device. The callback is executed in the context of the LAN865x RX handling
 * thread.
 *
 * The received packet may be delivered in multiple fragments depending on
 * how the packet buffers are structured internally. Therefore, the callback
 * may be invoked multiple times for a single Ethernet frame.
 *
 * The driver internally clones the received packet before invoking the
 * callback, ensuring that the original packet continues through the Zephyr
 * networking stack without being modified by the application.
 *
 * @param dev Pointer to the LAN865x device instance.
 * @param cb Callback function to be invoked when RX data is available.
 *           If NULL, any previously registered callback will be disabled.
 * @param user_data User-defined pointer that will be passed to the callback
 *                  when it is invoked.
 *
 * @retval 0 If the callback was successfully registered.
 * @retval -ENODEV If the device context is invalid.
 */
int lan865x_register_rx_callback(const struct device *dev,
				 lan865x_rx_cb_t cb,
				 void *user_data);

#endif /* ZEPHYR_INCLUDE_DRIVERS_ETH_LAN865X_H__ */
