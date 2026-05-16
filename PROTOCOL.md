# Raex / Neo Smart Blinds — RF protocol (summary)

The concise, implementation-relevant protocol. It is what the `raex_blind`
component actually decodes/encodes — reverse-engineered from RTL-SDR captures,
round-trip-verified, and independently re-confirmed on-device. Full derivation
and captures live in the author's workspace; this builds on nickw444's
**blindkit** (see Credits in `README.md`).

433.92 MHz, ASK/OOK, Manchester, **static code** (no rolling code → both RX
live-sync and TX replay work).

## Physical layer

- Carrier **433.92 MHz**, ASK / OOK (carrier on = `1`, off = `0`).
- Line code: **Manchester, inverted** — `decoded_bit = first_half_level XOR 1`
  (bit `0` → HIGH,LOW; bit `1` → LOW,HIGH; HIGH = carrier on for one half-bit).
- **Half-bit ≈ 640 µs** (measured 624–656; one bit = 2 half-bits ≈ 1280 µs).
  Consecutive equal half-bits merge into ~1280 µs runs (expected).
- **AGC preamble:** a long ~320 µs on / ~320 µs off run (real remote ≈ 500
  cycles; ~150–250 is enough to TX). Varies per remote — treated as a sync
  marker, not data.
- **Long SYNC:** ~2640 µs HIGH + ~2640 µs LOW immediately before the payload —
  universal and unambiguous (no payload pulse exceeds ~1370 µs); the RX anchor.
- Frame is sent **~3× per press**, ~25 ms between repeats; AGC preamble once
  before the burst.

## Frame format

One leading framing bit (observed `0`), then **5 payload bytes, LSB-first** on
the wire:

```
[channel] [id_lo] [id_hi] [command] [checksum]

checksum  = (channel + id_lo + id_hi + command + 3) & 0xFF
remote_id = id_lo | (id_hi << 8)          # little-endian
```

- `channel` = the remote's **raw RF channel byte** (e.g. 223/224) — *not* the
  Neo controller's logical 1–15 channel. This is the `channel_id:` config value.
- `remote_id` = little-endian integer of the two ID bytes (the `remote_id:`
  config value).
- The 8-bit checksum is weak: **always validate a decoded frame against a known
  (channel, remote_id)** before acting — lone-checksum false positives are real.

## Command byte

| Command | Value |
|---|---|
| UP / full-open (100%) | `0xFE` (254) |
| STOP | `0xFD` (253) |
| DOWN / full-close (0%) | `0xFC` (252) |
| micro-up / favorite | `0xDC` (220) |
| micro-down | `0xDB` (219) |
| PAIR | `0x7F` (127) |
| go to position P% (1–99) | `P + 11` (`0x0C`–`0x6E`) |

Cover → RF mapping: 0 % → DOWN, 100 % → UP, 1–99 % → `pct + 11`, stop → STOP.
(Some remote models emit positional codes for their own buttons rather than the
plain UP/DOWN/STOP bytes — both decode through the same table.)

## RX (live-sync) decoding

The huge AGC preamble overflows naive receivers, so the component uses a
preamble-skip ISR state machine:

1. Idle; ignore the ~320 µs AGC preamble / header runs (sync, not data).
2. Anchor on the ~2640 µs **long SYNC**; buffer the data edges after it.
3. End of frame at the inter-repeat gap (~25 ms) or the next sync.
4. Quantize to 640 µs half-bits, pair them, `bit = first_half ^ 1`, drop the 1
   leading framing bit, take 5 LSB-first bytes.
5. Validate the checksum, then require `(channel, remote_id)` to resolve to a
   registered blind or alias; otherwise log it as an unregistered *discovery*
   frame. Never act on a lone-checksum match.

## Credits

Builds on nickw444's **blindkit**
(`https://github.com/nickw444/home/tree/master/blindkit`).
