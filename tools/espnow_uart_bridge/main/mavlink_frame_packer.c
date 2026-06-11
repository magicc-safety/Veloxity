#include "mavlink_frame_packer.h"

#include <string.h>

#define MAVLINK_V1_STX 0xfeu
#define MAVLINK_V1_HEADER_LEN 6u
#define MAVLINK_V1_CHECKSUM_LEN 2u
#define MAVLINK_V1_MAX_FRAME_LEN 263u

static void compact_rx(mavlink_frame_packer_t *packer)
{
    size_t start = 0;
    while (start < packer->rx_len && packer->rx[start] != MAVLINK_V1_STX) {
        start++;
    }
    if (start == 0) {
        return;
    }
    if (start < packer->rx_len) {
        memmove(packer->rx, packer->rx + start, packer->rx_len - start);
        packer->rx_len -= start;
    } else {
        packer->rx_len = 0;
    }
}

void mavlink_frame_packer_init(mavlink_frame_packer_t *packer, size_t max_packet_len)
{
    memset(packer, 0, sizeof(*packer));
    if (max_packet_len > MAVLINK_FRAME_PACKER_MAX_PACKET) {
        max_packet_len = MAVLINK_FRAME_PACKER_MAX_PACKET;
    }
    packer->max_packet_len = max_packet_len;
}

void mavlink_frame_packer_flush(mavlink_frame_packer_t *packer,
                                mavlink_frame_packer_emit_fn emit,
                                void *emit_ctx)
{
    if (packer->packet_len == 0) {
        return;
    }
    if (!emit(emit_ctx, packer->packet, packer->packet_len)) {
        packer->drops++;
    }
    packer->packet_len = 0;
}

void mavlink_frame_packer_push(mavlink_frame_packer_t *packer,
                               const uint8_t *bytes,
                               size_t len,
                               mavlink_frame_packer_emit_fn emit,
                               void *emit_ctx)
{
    size_t offset = 0;
    while (offset < len) {
        size_t room = sizeof(packer->rx) - packer->rx_len;
        if (room == 0) {
            packer->drops++;
            packer->rx_len = 0;
            room = sizeof(packer->rx);
        }

        size_t copy_len = len - offset;
        if (copy_len > room) {
            copy_len = room;
        }
        memcpy(packer->rx + packer->rx_len, bytes + offset, copy_len);
        packer->rx_len += copy_len;
        offset += copy_len;

        compact_rx(packer);
        while (packer->rx_len >= MAVLINK_V1_HEADER_LEN) {
            if (packer->rx[0] != MAVLINK_V1_STX) {
                compact_rx(packer);
                continue;
            }

            const size_t frame_len =
                MAVLINK_V1_HEADER_LEN + (size_t)packer->rx[1] + MAVLINK_V1_CHECKSUM_LEN;
            if (frame_len > MAVLINK_V1_MAX_FRAME_LEN || frame_len > packer->max_packet_len) {
                memmove(packer->rx, packer->rx + 1, packer->rx_len - 1);
                packer->rx_len--;
                packer->drops++;
                compact_rx(packer);
                continue;
            }
            if (packer->rx_len < frame_len) {
                break;
            }

            if (packer->packet_len > 0 &&
                packer->packet_len + frame_len > packer->max_packet_len) {
                mavlink_frame_packer_flush(packer, emit, emit_ctx);
            }
            memcpy(packer->packet + packer->packet_len, packer->rx, frame_len);
            packer->packet_len += frame_len;
            packer->frames++;
            memmove(packer->rx, packer->rx + frame_len, packer->rx_len - frame_len);
            packer->rx_len -= frame_len;
            compact_rx(packer);
        }
    }
}
