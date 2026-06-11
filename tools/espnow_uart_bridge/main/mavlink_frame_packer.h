#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAVLINK_FRAME_PACKER_MAX_PACKET 237u

typedef bool (*mavlink_frame_packer_emit_fn)(void *ctx, const uint8_t *bytes, size_t len);

typedef struct {
    uint8_t rx[526];
    uint8_t packet[MAVLINK_FRAME_PACKER_MAX_PACKET];
    size_t rx_len;
    size_t packet_len;
    size_t max_packet_len;
    uint32_t frames;
    uint32_t drops;
} mavlink_frame_packer_t;

void mavlink_frame_packer_init(mavlink_frame_packer_t *packer, size_t max_packet_len);
void mavlink_frame_packer_push(mavlink_frame_packer_t *packer,
                               const uint8_t *bytes,
                               size_t len,
                               mavlink_frame_packer_emit_fn emit,
                               void *emit_ctx);
void mavlink_frame_packer_flush(mavlink_frame_packer_t *packer,
                                mavlink_frame_packer_emit_fn emit,
                                void *emit_ctx);
