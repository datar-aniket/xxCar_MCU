#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "rc.h"

static int fails = 0;
#define CHECK(cond, msg, ...) do { if (!(cond)) { printf("  FAIL: " msg "\n", ##__VA_ARGS__); fails++; } } while (0)

/* Pack 16 x 11-bit little-endian channels into 22 bytes (the format both use) */
static void pack11(const uint16_t *raw, uint8_t *d)
{
  unsigned bit = 0, i;
  memset(d, 0, 22);
  for (i = 0; i < 16; i++)
    {
      uint32_t v = raw[i] & 0x7ff;
      unsigned byte = bit >> 3, sh = bit & 7;
      d[byte]     |= (uint8_t)(v << sh);
      d[byte + 1] |= (uint8_t)(v >> (8 - sh));
      if (sh > 5) d[byte + 2] |= (uint8_t)(v >> (16 - sh));
      bit += 11;
    }
}

static uint8_t crc8_dvb(const uint8_t *p, int n)
{
  uint8_t c = 0;
  for (int i = 0; i < n; i++) { c ^= p[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xd5) : (uint8_t)(c << 1); }
  return c;
}

int main(void)
{
  struct rc_decoder_s d;
  struct rc_frame_s f;
  struct rc_link_quality_s quality;
  uint16_t raw[16];
  uint8_t sbus[25], crsf[26];
  int i;

  /* ---------------- SBUS ---------------- */
  for (i = 0; i < 16; i++) raw[i] = 1024;      /* centre */
  raw[0] = 200; raw[1] = 1800;                 /* the documented endpoints */
  sbus[0] = 0x0f;
  pack11(raw, &sbus[1]);
  sbus[23] = 0x00;
  sbus[24] = 0x00;

  rc_decoder_reset(&d, RC_PROTO_SBUS);
  CHECK(rc_decode(&d, sbus, 25, &f), "SBUS frame not decoded");
  CHECK(f.count == 16, "SBUS count %u != 16", f.count);
  CHECK(f.channel[0] == 999,  "SBUS raw 200 -> %u us, expected 999",  f.channel[0]);
  CHECK(f.channel[1] == 1999, "SBUS raw 1800 -> %u us, expected 1999", f.channel[1]);
  CHECK(f.channel[2] == 1514, "SBUS raw 1024 -> %u us, expected 1514", f.channel[2]);
  CHECK(!f.failsafe && !f.frame_lost, "SBUS flags set when they should not be");
  printf("  SBUS  200->%u  1024->%u  1800->%u\n", f.channel[0], f.channel[2], f.channel[1]);

  /* failsafe + frame-lost flags */
  sbus[23] = (1 << 3) | (1 << 2);
  rc_decoder_reset(&d, RC_PROTO_SBUS);
  CHECK(rc_decode(&d, sbus, 25, &f), "SBUS (flags) not decoded");
  CHECK(f.failsafe, "SBUS failsafe bit not seen");
  CHECK(f.frame_lost, "SBUS frame-lost bit not seen");
  sbus[23] = 0x00;

  /* resync: garbage in front of a good frame */
  {
    uint8_t noisy[40];
    memset(noisy, 0xa5, 15);
    memcpy(noisy + 15, sbus, 25);
    rc_decoder_reset(&d, RC_PROTO_SBUS);
    CHECK(rc_decode(&d, noisy, 40, &f), "SBUS did not resync after garbage");
  }

  /* Link qualification: isolated bad packets hold the last good sample, but
   * the tenth consecutive rejection declares loss. Any valid packet resets
   * the streak completely.
   */

  rc_link_quality_reset(&quality);
  CHECK(!rc_link_note_invalid(&quality, 1),
        "invalid packet before first good packet reported OK");
  CHECK(rc_link_note_valid(&quality), "valid packet did not establish link");
  for (i = 1; i < RC_INVALID_LIMIT; i++)
    {
      CHECK(rc_link_note_invalid(&quality, 1),
            "link lost after only %d consecutive invalid packets", i);
    }
  CHECK(!rc_link_note_invalid(&quality, 1),
        "link remained OK at invalid packet %d", RC_INVALID_LIMIT);
  CHECK(rc_link_note_valid(&quality), "valid packet did not recover link");
  CHECK(quality.invalid_streak == 0, "valid packet did not reset streak");
  CHECK(quality.lost_frames == RC_INVALID_LIMIT + 1,
        "lost-frame total is %u", quality.lost_frames);

  /* A held command retains its last-good age, even if bad traffic continues.
   * Explicit receiver failsafe is immediate and only GOOD data clears it.
   */
  rc_link_quality_reset(&quality);
  memset(&f, 0, sizeof(f));
  CHECK(rc_link_frame(&quality, &f, 1000), "first good frame rejected");
  f.frame_lost = true;
  CHECK(rc_link_frame(&quality, &f, 2000), "isolated lost frame not held");
  CHECK(quality.last_valid_us == 1000, "bad frame refreshed freshness");
  CHECK(!rc_link_ok(&quality, 101000), "held data did not expire");
  f.failsafe = true;
  CHECK(!rc_link_frame(&quality, &f, 3000), "failsafe was delayed");
  f.failsafe = false;
  CHECK(!rc_link_frame(&quality, &f, 4000), "frame-lost cleared failsafe");
  CHECK(!rc_link_note_invalid(&quality, 1), "garbage cleared failsafe");
  f.frame_lost = false;
  CHECK(rc_link_frame(&quality, &f, 5000), "good frame did not recover");
  CHECK(!quality.failsafe && quality.last_valid_us == 5000, "bad recovery");
  CHECK(!rc_link_ok(&quality, 4999), "future timestamp accepted");

  rc_decoder_reset(&d, RC_PROTO_SBUS);
  CHECK(!rc_decode(&d, sbus, 12, &f), "partial frame accepted");
  CHECK(rc_decoder_idle(&d), "idle failed to discard partial frame");
  CHECK(d.errors == 1 && d.nbuf == 0, "idle accounting incorrect");
  CHECK(!rc_decoder_idle(&d), "idle counted nonexistent partial frame");
  CHECK(rc_decode(&d, sbus, 25, &f), "idle recovery failed");
  {
    uint8_t overlap[30];
    memset(overlap, 0x55, 5);
    overlap[0] = SBUS_START_BYTE;
    memcpy(overlap + 5, sbus, 25);
    rc_decoder_reset(&d, RC_PROTO_SBUS);
    CHECK(rc_decode(&d, overlap, sizeof(overlap), &f),
          "SBUS discarded real header inside a rejected candidate");
    CHECK(d.errors >= 1, "SBUS overlap did not reject false candidate");
    sbus[23] = 0xf0;
    rc_decoder_reset(&d, RC_PROTO_SBUS);
    CHECK(!rc_decode(&d, sbus, sizeof(sbus), &f), "reserved SBUS flags accepted");
    sbus[23] = 0;
  }

  /* ---------------- CRSF ---------------- */
  for (i = 0; i < 16; i++) raw[i] = 992;       /* centre */
  raw[0] = 172; raw[1] = 1811;                 /* the documented endpoints */
  crsf[0] = 0xc8;
  crsf[1] = 24;                                /* type + 22 payload + crc */
  crsf[2] = 0x16;
  pack11(raw, &crsf[3]);
  crsf[25] = crc8_dvb(&crsf[2], 23);

  rc_decoder_reset(&d, RC_PROTO_CRSF);
  CHECK(rc_decode(&d, crsf, 26, &f), "CRSF frame not decoded");
  CHECK(f.count == 16, "CRSF count %u != 16", f.count);
  CHECK(f.channel[0] == 988,  "CRSF raw 172 -> %u us, expected 988",  f.channel[0]);
  CHECK(f.channel[1] == 2012, "CRSF raw 1811 -> %u us, expected 2012", f.channel[1]);
  CHECK(f.channel[2] == 1500, "CRSF raw 992 -> %u us, expected 1500", f.channel[2]);
  printf("  CRSF  172->%u  992->%u  1811->%u\n", f.channel[0], f.channel[2], f.channel[1]);

  /* a corrupted CRC must be rejected, and counted */
  crsf[25] ^= 0xff;
  rc_decoder_reset(&d, RC_PROTO_CRSF);
  CHECK(!rc_decode(&d, crsf, 26, &f), "CRSF accepted a bad CRC");
  CHECK(d.errors == 1, "CRSF bad CRC not counted (errors=%u)", d.errors);
  crsf[25] ^= 0xff;

  /* An impossible length is a rejected packet too, not silent garbage. */

  rc_decoder_reset(&d, RC_PROTO_CRSF);
  crsf[0] = 0xc8;
  crsf[1] = 1;
  CHECK(!rc_decode(&d, crsf, 2, &f), "CRSF accepted a bad length");
  CHECK(d.errors == 1, "CRSF bad length not counted (errors=%u)", d.errors);
  crsf[1] = 24;

  /* ---- the one that makes autodetect work: each must REJECT the other ---- */
  {
    int j;
    rc_decoder_reset(&d, RC_PROTO_SBUS);
    for (j = 0; j < 50; j++) CHECK(!rc_decode(&d, crsf, 26, &f), "SBUS decoder accepted a CRSF stream!");
    rc_decoder_reset(&d, RC_PROTO_CRSF);
    for (j = 0; j < 50; j++) CHECK(!rc_decode(&d, sbus, 25, &f), "CRSF decoder accepted an SBUS stream!");
    printf("  cross-reject OK (SBUS<->CRSF): this is what makes probing work\n");
  }

  for (unsigned proto = RC_PROTO_SBUS; proto <= RC_PROTO_CRSF; proto++)
    {
      uint32_t random = 0x1523aaff;
      rc_decoder_reset(&d, proto);
      for (unsigned byte = 0; byte < 200000; byte++)
        {
          random = random * 1664525u + 1013904223u;
          uint8_t value = random >> 24;
          rc_decode(&d, &value, 1, &f);
          CHECK(d.nbuf < sizeof(d.buf), "decoder buffer bound exceeded");
          if (byte % 1000 == 0) rc_decoder_idle(&d);
        }
    }
  printf(fails ? "\n%d FAILURE(S)\n" : "\nall decoder tests passed\n", fails);
  return fails != 0;
}
