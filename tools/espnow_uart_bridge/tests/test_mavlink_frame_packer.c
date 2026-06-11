#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../main/mavlink_frame_packer.h"

typedef struct {
    uint8_t packets[8][MAVLINK_FRAME_PACKER_MAX_PACKET];
    size_t lens[8];
    size_t count;
} capture_t;

static bool capture_emit(void *ctx, const uint8_t *bytes, size_t len)
{
    capture_t *capture = (capture_t *)ctx;
    assert(capture->count < 8);
    memcpy(capture->packets[capture->count], bytes, len);
    capture->lens[capture->count] = len;
    capture->count++;
    return true;
}

static void make_frame(uint8_t seq, uint8_t payload_len, uint8_t *out, size_t *out_len)
{
    out[0] = 0xfe;
    out[1] = payload_len;
    out[2] = seq;
    out[3] = 1;
    out[4] = 250;
    out[5] = 181;
    for (uint8_t i = 0; i < payload_len; i++) {
        out[6 + i] = (uint8_t)(seq + i);
    }
    out[6 + payload_len] = 0xaa;
    out[7 + payload_len] = 0x55;
    *out_len = 8u + payload_len;
}

static void fragmented_frames_pack_into_one_payload(void)
{
    uint8_t frame_a[32];
    uint8_t frame_b[32];
    size_t len_a;
    size_t len_b;
    make_frame(1, 4, frame_a, &len_a);
    make_frame(2, 5, frame_b, &len_b);

    mavlink_frame_packer_t packer;
    capture_t capture = { 0 };
    mavlink_frame_packer_init(&packer, 64);
    mavlink_frame_packer_push(&packer, frame_a, 3, capture_emit, &capture);
    mavlink_frame_packer_push(&packer, frame_a + 3, len_a - 3, capture_emit, &capture);
    mavlink_frame_packer_push(&packer, frame_b, len_b, capture_emit, &capture);
    assert(capture.count == 0);

    mavlink_frame_packer_flush(&packer, capture_emit, &capture);
    assert(capture.count == 1);
    assert(capture.lens[0] == len_a + len_b);
    assert(memcmp(capture.packets[0], frame_a, len_a) == 0);
    assert(memcmp(capture.packets[0] + len_a, frame_b, len_b) == 0);
    assert(packer.frames == 2);
    assert(packer.drops == 0);
}

static void packet_limit_splits_between_complete_frames(void)
{
    uint8_t frame_a[32];
    uint8_t frame_b[32];
    size_t len_a;
    size_t len_b;
    make_frame(3, 10, frame_a, &len_a);
    make_frame(4, 10, frame_b, &len_b);

    mavlink_frame_packer_t packer;
    capture_t capture = { 0 };
    mavlink_frame_packer_init(&packer, len_a + 1);
    mavlink_frame_packer_push(&packer, frame_a, len_a, capture_emit, &capture);
    mavlink_frame_packer_push(&packer, frame_b, len_b, capture_emit, &capture);
    assert(capture.count == 1);
    assert(capture.lens[0] == len_a);
    assert(memcmp(capture.packets[0], frame_a, len_a) == 0);

    mavlink_frame_packer_flush(&packer, capture_emit, &capture);
    assert(capture.count == 2);
    assert(capture.lens[1] == len_b);
    assert(memcmp(capture.packets[1], frame_b, len_b) == 0);
}

static void junk_resyncs_to_next_frame(void)
{
    uint8_t junk_and_frame[40] = { 0x00, 0x11, 0xfe, 0xff, 0x22 };
    uint8_t frame[32];
    size_t frame_len;
    make_frame(5, 3, frame, &frame_len);
    memcpy(junk_and_frame + 5, frame, frame_len);

    mavlink_frame_packer_t packer;
    capture_t capture = { 0 };
    mavlink_frame_packer_init(&packer, 64);
    mavlink_frame_packer_push(&packer, junk_and_frame, 5 + frame_len, capture_emit, &capture);
    mavlink_frame_packer_flush(&packer, capture_emit, &capture);

    assert(capture.count == 1);
    assert(capture.lens[0] == frame_len);
    assert(memcmp(capture.packets[0], frame, frame_len) == 0);
    assert(packer.drops == 1);
}

static void incomplete_frame_waits_for_more_bytes(void)
{
    uint8_t frame[32];
    size_t frame_len;
    make_frame(6, 8, frame, &frame_len);

    mavlink_frame_packer_t packer;
    capture_t capture = { 0 };
    mavlink_frame_packer_init(&packer, 64);
    mavlink_frame_packer_push(&packer, frame, frame_len - 2, capture_emit, &capture);
    mavlink_frame_packer_flush(&packer, capture_emit, &capture);
    assert(capture.count == 0);

    mavlink_frame_packer_push(&packer, frame + frame_len - 2, 2, capture_emit, &capture);
    mavlink_frame_packer_flush(&packer, capture_emit, &capture);
    assert(capture.count == 1);
    assert(capture.lens[0] == frame_len);
    assert(memcmp(capture.packets[0], frame, frame_len) == 0);
}

int main(void)
{
    fragmented_frames_pack_into_one_payload();
    packet_limit_splits_between_complete_frames();
    junk_resyncs_to_next_frame();
    incomplete_frame_waits_for_more_bytes();
    return 0;
}
