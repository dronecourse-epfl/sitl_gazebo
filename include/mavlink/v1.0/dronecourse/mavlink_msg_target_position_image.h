#pragma once
// MESSAGE TARGET_POSITION_IMAGE PACKING

#define MAVLINK_MSG_ID_TARGET_POSITION_IMAGE 152

MAVPACKED(
typedef struct __mavlink_target_position_image_t {
 uint64_t time_usec; /*< Timestamp (micros since boot or Unix epoch)*/
 uint32_t x; /*< X Position in pixels frame in meters*/
 uint32_t y; /*< Y Position in pixels frame in meters*/
 float dist; /*<  distance to target meters*/
 float pitch; /*<  Pitch from body to camera frame*/
 float yaw; /*<  Yaw from body to camera frame*/
 uint8_t target_num; /*< ID of the target object.*/
}) mavlink_target_position_image_t;

#define MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN 29
#define MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_MIN_LEN 29
#define MAVLINK_MSG_ID_152_LEN 29
#define MAVLINK_MSG_ID_152_MIN_LEN 29

#define MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_CRC 225
#define MAVLINK_MSG_ID_152_CRC 225



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TARGET_POSITION_IMAGE { \
    152, \
    "TARGET_POSITION_IMAGE", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_target_position_image_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_target_position_image_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_target_position_image_t, y) }, \
         { "dist", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_position_image_t, dist) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_position_image_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_target_position_image_t, yaw) }, \
         { "target_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_target_position_image_t, target_num) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TARGET_POSITION_IMAGE { \
    "TARGET_POSITION_IMAGE", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_target_position_image_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_target_position_image_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_target_position_image_t, y) }, \
         { "dist", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_target_position_image_t, dist) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_target_position_image_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_target_position_image_t, yaw) }, \
         { "target_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_target_position_image_t, target_num) }, \
         } \
}
#endif

/**
 * @brief Pack a target_position_image message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param x X Position in pixels frame in meters
 * @param y Y Position in pixels frame in meters
 * @param dist  distance to target meters
 * @param pitch  Pitch from body to camera frame
 * @param yaw  Yaw from body to camera frame
 * @param target_num ID of the target object.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_position_image_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint32_t x, uint32_t y, float dist, float pitch, float yaw, uint8_t target_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, x);
    _mav_put_uint32_t(buf, 12, y);
    _mav_put_float(buf, 16, dist);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, target_num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN);
#else
    mavlink_target_position_image_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.dist = dist;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.target_num = target_num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_POSITION_IMAGE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_CRC);
}

/**
 * @brief Pack a target_position_image message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param x X Position in pixels frame in meters
 * @param y Y Position in pixels frame in meters
 * @param dist  distance to target meters
 * @param pitch  Pitch from body to camera frame
 * @param yaw  Yaw from body to camera frame
 * @param target_num ID of the target object.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_target_position_image_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint32_t x,uint32_t y,float dist,float pitch,float yaw,uint8_t target_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, x);
    _mav_put_uint32_t(buf, 12, y);
    _mav_put_float(buf, 16, dist);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, target_num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN);
#else
    mavlink_target_position_image_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.dist = dist;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.target_num = target_num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TARGET_POSITION_IMAGE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_CRC);
}

/**
 * @brief Encode a target_position_image struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param target_position_image C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_position_image_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_target_position_image_t* target_position_image)
{
    return mavlink_msg_target_position_image_pack(system_id, component_id, msg, target_position_image->time_usec, target_position_image->x, target_position_image->y, target_position_image->dist, target_position_image->pitch, target_position_image->yaw, target_position_image->target_num);
}

/**
 * @brief Encode a target_position_image struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_position_image C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_target_position_image_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_target_position_image_t* target_position_image)
{
    return mavlink_msg_target_position_image_pack_chan(system_id, component_id, chan, msg, target_position_image->time_usec, target_position_image->x, target_position_image->y, target_position_image->dist, target_position_image->pitch, target_position_image->yaw, target_position_image->target_num);
}

/**
 * @brief Send a target_position_image message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp (micros since boot or Unix epoch)
 * @param x X Position in pixels frame in meters
 * @param y Y Position in pixels frame in meters
 * @param dist  distance to target meters
 * @param pitch  Pitch from body to camera frame
 * @param yaw  Yaw from body to camera frame
 * @param target_num ID of the target object.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_target_position_image_send(mavlink_channel_t chan, uint64_t time_usec, uint32_t x, uint32_t y, float dist, float pitch, float yaw, uint8_t target_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, x);
    _mav_put_uint32_t(buf, 12, y);
    _mav_put_float(buf, 16, dist);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, target_num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE, buf, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_CRC);
#else
    mavlink_target_position_image_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.dist = dist;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.target_num = target_num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE, (const char *)&packet, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_CRC);
#endif
}

/**
 * @brief Send a target_position_image message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_target_position_image_send_struct(mavlink_channel_t chan, const mavlink_target_position_image_t* target_position_image)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_target_position_image_send(chan, target_position_image->time_usec, target_position_image->x, target_position_image->y, target_position_image->dist, target_position_image->pitch, target_position_image->yaw, target_position_image->target_num);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE, (const char *)target_position_image, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_CRC);
#endif
}

#if MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_target_position_image_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint32_t x, uint32_t y, float dist, float pitch, float yaw, uint8_t target_num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint32_t(buf, 8, x);
    _mav_put_uint32_t(buf, 12, y);
    _mav_put_float(buf, 16, dist);
    _mav_put_float(buf, 20, pitch);
    _mav_put_float(buf, 24, yaw);
    _mav_put_uint8_t(buf, 28, target_num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE, buf, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_CRC);
#else
    mavlink_target_position_image_t *packet = (mavlink_target_position_image_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->x = x;
    packet->y = y;
    packet->dist = dist;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->target_num = target_num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE, (const char *)packet, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_MIN_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_CRC);
#endif
}
#endif

#endif

// MESSAGE TARGET_POSITION_IMAGE UNPACKING


/**
 * @brief Get field time_usec from target_position_image message
 *
 * @return Timestamp (micros since boot or Unix epoch)
 */
static inline uint64_t mavlink_msg_target_position_image_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field x from target_position_image message
 *
 * @return X Position in pixels frame in meters
 */
static inline uint32_t mavlink_msg_target_position_image_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field y from target_position_image message
 *
 * @return Y Position in pixels frame in meters
 */
static inline uint32_t mavlink_msg_target_position_image_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field dist from target_position_image message
 *
 * @return  distance to target meters
 */
static inline float mavlink_msg_target_position_image_get_dist(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitch from target_position_image message
 *
 * @return  Pitch from body to camera frame
 */
static inline float mavlink_msg_target_position_image_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yaw from target_position_image message
 *
 * @return  Yaw from body to camera frame
 */
static inline float mavlink_msg_target_position_image_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field target_num from target_position_image message
 *
 * @return ID of the target object.
 */
static inline uint8_t mavlink_msg_target_position_image_get_target_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Decode a target_position_image message into a struct
 *
 * @param msg The message to decode
 * @param target_position_image C-struct to decode the message contents into
 */
static inline void mavlink_msg_target_position_image_decode(const mavlink_message_t* msg, mavlink_target_position_image_t* target_position_image)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    target_position_image->time_usec = mavlink_msg_target_position_image_get_time_usec(msg);
    target_position_image->x = mavlink_msg_target_position_image_get_x(msg);
    target_position_image->y = mavlink_msg_target_position_image_get_y(msg);
    target_position_image->dist = mavlink_msg_target_position_image_get_dist(msg);
    target_position_image->pitch = mavlink_msg_target_position_image_get_pitch(msg);
    target_position_image->yaw = mavlink_msg_target_position_image_get_yaw(msg);
    target_position_image->target_num = mavlink_msg_target_position_image_get_target_num(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN? msg->len : MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN;
        memset(target_position_image, 0, MAVLINK_MSG_ID_TARGET_POSITION_IMAGE_LEN);
    memcpy(target_position_image, _MAV_PAYLOAD(msg), len);
#endif
}
