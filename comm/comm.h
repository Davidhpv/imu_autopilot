#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMM_HW_H
#define COMM_HW_H

#include "conf.h"
#include "mavlink_types.h"

// MAVLink Protocol settings
mavlink_system_t mavlink_system;
/*
 * Put this here or in main.c (where it currently is)
 *
mavlink_settings.sysid = 100 // System ID, 1-255
mavlink_settings.compid = 50 // Component/Subsystem ID, 1-255
 */

/** @addtogroup COMM */
//@{
/** @name Communication functions
 */
//@{

/**
 * @brief Initialize the communication channel
 */
extern void comm_init ( mavlink_channel_t chan );

extern uint8_t comm_ch_available ( mavlink_channel_t chan );

extern void comm_send_message_ch ( mavlink_channel_t chan, uint8_t c );

/**
 * @brief Send one char over the comm channel
 *
 * This stub has to be implemented by each individual platform
 *
 * @param chan the comm channel
 * @param c char to send
 */
extern void comm_send_ch ( mavlink_channel_t chan, uint8_t c );

/**
 * @brief Get one char from the comm channel
 *
 * This stub has to be implemented by each individual platform
 *
 * @param chan The channel to get one char from
 * @return The next char
 */
extern uint8_t comm_get_ch( mavlink_channel_t chan );

/**
 * @brief Check free space to send on a channel
 *
 * This stub has to be implemented by each individual platform
 *
 * @param chan The channel write to
 * @return 1 if space is available, 0 else
 */
extern uint8_t comm_check_free_space ( mavlink_channel_t chan, uint8_t len );

//@}}

#endif /* COMM_HW_H */

#ifdef __cplusplus
}
#endif
