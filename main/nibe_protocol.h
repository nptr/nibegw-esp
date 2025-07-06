#include <cstdint>

namespace nibe_protocol {

/*
 * Master Request Format:
 * +----+------+------+-----+-----+----+----+-----+
 * | 5C | ADDR | ADDR | CMD | LEN |  DATA   | CHK |
 * +----+------+------+-----+-----+----+----+-----+
 *
 * Checksum is XOR sum from [ADDR..DATA]
 */

constexpr size_t MASTER_HDR_LEN = 5; // [5C..LEN]
constexpr size_t MASTER_FRAME_POS_START = 0;
constexpr size_t MASTER_FRAME_POS_ADDR = 1;
constexpr size_t MASTER_FRAME_POS_CMD = 3;
constexpr size_t MASTER_FRAME_POS_LEN = 4;
constexpr size_t MASTER_FRAME_POS_DATA = 5;

/*
 * Slave Response Format:
 * +----+-----+-----+----+----+-----+
 * | C0 | CMD | LEN |  DATA   | CHK |
 * +----+-----+-----+----+----+-----+
 *
 * Checksum is XOR sum from [C0 .. DATA]
 */
constexpr size_t SLAVE_HDR_LEN = 3; // [C0..LEN]
constexpr size_t SLAVE_FRAME_POS_START = 0;
constexpr size_t SLAVE_FRAME_POS_CMD = 1;
constexpr size_t SLAVE_FRAME_POS_LEN = 2;
constexpr size_t SLAVE_FRAME_POS_DATA = 3;

enum control_bytes : uint8_t {
    MASTER_START = 0x5C,
    SLAVE_START = 0xC0,
    ACK = 0x06, // could be from either
    NACK = 0x15 // could be from either
};


enum command_bytes : uint8_t {
    /*  Heatpump periodically sends configured registers.
        PAYLOAD: (multiple of)
            u16 LSB address
            u16 LSB value
        RESPONSE:
            ACK
        EXAMPLE:
            M > S: 5C 00 20 68 50 C9 AF 00 00 EC 9F 00 00 ED 9F 00 00 93 9C 14 00 94 9C 00 00 91 9C
       16 00 92 9C 00 00 8F 9C 01 00 90 9C 00 00 48 9C 22 01 47 9C F4 00 C1 9C F0 00 25 AC 41 00 26
       AC 00 00 A9 AD 01 00 4C 9C 08 01 4D 9C D7 01 4E 9C B1 01 98 9F 00 80 61 9C 1A 01 S > M: 06
    */
    MODBUS_DATA_MSG = 0x68,


    /*  Master sends MODBUS_READ_REQ message to ask if we want to read anything.
        PAYLOAD: None
        RESPONSE (nothing to read):
            ACK
        RESPONSE (something to read):
            Response frame with
            u16 LSB register address
        EXAMPLE:
            M > S: 5C 00 20 69 00 49
            S > M: C0 69 02 24 AC 23
            M > S: 06
    */
    MODBUS_READ_REQ = 0x69,


    /*  Master sends MODBUS_READ_RESP message to give us the data requested
        PAYLOAD:
            u16 LSB register address
            u32 LSB register value
        RESPONSE:
            ACK / NACK
        EXAMPLE 1:
            M > S: 5C 00 20 6A 06 4D 9C F3 01 B3 01 DD
            S > M: 06

            In this example, register=40013, value=499=0x1F3.
            The other word "0x1B3" is already register 40014!
        EXAMPLE 2:
            M > S: 5C 00 20 6A 07 73 B0 5C 5C 00 00 00 8E
            S > M: 06

            Payload contains 0x5C. Heatpump escapes that by duplicating it.
            CRC cover the stream as-sent, so dedup. must happend after that.
    */
    MODBUS_READ_RESP = 0x6A,


    /*  Master sends MODBUS_WRITE_REQ message to ask if we want to write anything.
        PAYLOAD: None
        RESPONSE (nothing to write):
            ACK
        RESPONSE (something to write):
            u16 LSB register address
            u32 LSB register value
        EXAMPLE:
            M > S: 5C 00 20 6B 00 4B
            S > M: C0 6B 06 A3 B7 02 00 00 00 BB
            M > S: 06

            In this example, register=47011 (Heat Offset S1), value=0x0002.
            Contrary to "MODBUS_READ_RESP", the second word (here 0x0000)
            is NOT the value for the subsequent register. This would have
            meant that it's not possible to to set individual registers,
            but is luckily not the case.
    */
    MODBUS_WRITE_REQ = 0x6B,


    /*  Master sends MODBUS_WRITE_RESP message with the write result.
        PAYLOAD:
            u8 result, where 1 = written, 0 ?= not written
        RESPONSE:
            ACK/NACK
        EXAMPLE:
            M > S: 5C 00 20 6C 01 01 4C
            S > M: 06
    */
    MODBUS_WRITE_RESP = 0x6C,

    /* Heatpump asks if we want to write some datapoint. */
    RMU_WRITE_REQ = 0x60,
    /* Heatpump periodically sends data for RMU? */
    RMU_DATA_MSG = 0x62,
    /* Unknown setup command. */
    RMU_UNK_REQ = 0x63,
    /* Heatpump asks for device info/version. */
    ACCESSORY_INFO_REQ = 0xEE,
};


enum device_addresses : uint8_t { SMS40 = 0x16, RMU40 = 0x19, MODBUS40 = 0x20 };

/* Calculate the XOR checksum.
   The checksum shall include address, command, length and payload.
*/
inline uint8_t xorsum(const uint8_t* data, size_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum ^= data[i];
    }

    // 0x5C must not occur in the checksum.
    if (sum == MASTER_START)
        return 0xC5;
    else
        return sum;
}


}