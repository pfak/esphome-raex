# raex_blind — ESPHome component for Raex / Neo Smart Blinds

A custom [ESPHome](https://esphome.io) external component that controls **and
live-tracks** Raex / Neo Smart roller blinds over 433.92 MHz RF using a
**CC1101** transceiver.

It does two jobs at once:

- **TX** — sends open / close / stop / go-to-position commands to the blinds.
- **RX** — listens for the *physical handheld remotes* and mirrors their
  presses back into Home Assistant, so the HA cover stays in sync no matter how
  the blind was operated.

Position is **time-integrated** (the motors give no feedback): each cover
estimates its position from per-direction travel times, animates
OPENING/CLOSING, freezes correctly on STOP, and self-resyncs at the hard limits.

> Personal project for my own house. Shared as-is in case it's useful — the
> protocol notes ([`PROTOCOL.md`](PROTOCOL.md)) are the interesting part.

## Hardware

| | |
|---|---|
| MCU | ESP32 (`esp32dev`, ESP-IDF framework) |
| Radio | CC1101 (SPI), 433.92 MHz, OOK |
| Wiring | SPI: SCK 14 · MOSI 15 · MISO 16 · CS 13 · CC1101 GDO0 (TX data) 4 · CC1101 GDO2 (RX data) 36 |

The blinds use a **static (non-rolling) code**, so RX sync and replay both work.

## How it works

- One **hub** (`raex_blind:`) owns the CC1101 (TX queue + RX decoder + position
  engine). It is a pure radio/engine hub.
- Each blind is a **self-contained `cover:` entry**: its `remote_id` /
  `channel_id`, optional extra remotes (`aliases:`), and its own per-direction
  travel times.
- A **group** is just the same alias `(remote_id, channel_id)` listed under
  more than one cover — an overheard group/ALL-button frame fans out to every
  cover that lists it (one-to-many; no separate "group" config).
- `set_position` is an **absolute** motor command (the blind drives to its own
  calibrated %), so it self-corrects; the time-integration is only for the
  in-flight animation and STOP/relative tracking.

## Install

```yaml
external_components:
  - source: github://pfak/esphome-raex@main
    components: [raex_blind]
```

A complete, flashable starting point is [`example.yaml`](example.yaml) — generic
placeholders, `name_add_mac_suffix: true` (flash the same file to any number of
boards; each self-assigns a unique `raex-xxxxxx.local`), secrets via `!secret`.

**Repository layout** (consumed as an ESPHome external component):

```
esphome-raex/
├── components/raex_blind/   # the component: __init__.py, *.h, *.cpp, cover/
├── example.yaml             # copy-me device config (placeholders, no secrets)
├── README.md   PROTOCOL.md   LICENSE
```

(Your real device YAML and `secrets.yaml` live in your own HA config, never in
this repo.)

## Configuration

Minimal example:

```yaml
external_components:
  - source: github://pfak/esphome-raex@main
    components: [raex_blind]

spi:
  clk_pin: GPIO14
  mosi_pin: GPIO15
  miso_pin: GPIO16

raex_blind:
  id: raex_blind_transmit
  cs_pin: GPIO13
  gdo0_pin: GPIO4
  rx_pin: GPIO36            # optional; omit for TX-only (no live-sync)
  # frequency: 433.92MHz    # optional override

cover:
  - platform: raex_blind
    transmitter_id: raex_blind_transmit   # optional; auto-binds if only one hub
    # PLACEHOLDER values — get the real remote_id/channel_id from the
    # "Discovering a remote" log (see below). remote_id is the little-endian
    # integer of the on-air ID bytes.
    remote_id: 12345
    channel_id: 200
    name: "Office Blind"
    device_class: blind
    open_time: 17s                        # full close -> open (measure it)
    close_time: 17s                       # full open -> close
    micro_step: 0.5%                      # micro-up/down nudge per press
    aliases:                              # extra remotes for THIS blind
      - remote_id: 54321
        channel_id: 201
```

Two blinds sharing a remote's group button → list that alias under both:

```yaml
  - platform: raex_blind
    remote_id: 23456
    channel_id: 11
    name: "Left Blind"
    aliases:
      - { remote_id: 34567, channel_id: 99 }   # group/ALL -> also Right Blind
  - platform: raex_blind
    remote_id: 23456
    channel_id: 12
    name: "Right Blind"
    aliases:
      - { remote_id: 34567, channel_id: 99 }   # same alias = group fan-out
```

### Cover options

| Option | Default | Notes |
|---|---|---|
| `remote_id` | — | The blind's RF identity (little-endian integer of the on-air ID bytes). |
| `channel_id` | — | RF channel byte (0–255). |
| `aliases` | — | List of extra `{remote_id, channel_id}` that drive this blind. Same alias under multiple covers = a group. |
| `open_time` / `close_time` | `17s` | Full-travel time per direction — measure each blind. |
| `micro_step` | `0.5%` | Position change per micro-up/down press. |
| `transmitter_id` | auto | The `raex_blind:` hub to attach to (auto-binds when there's one). |

### Discovering a remote

Press a physical remote near the device and watch the log:

```
RX (unregistered) channel=99 id=7.135 remoteID=34567 (0x8707) action=254 cksum=OK
```

Add that `remote_id` / `channel_id` as a new cover (its own blind) or as an
`aliases:` entry (extra remote for an existing blind).

## Credits / attribution

Builds on **[blindkit](https://github.com/nickw444/home/tree/master/blindkit)**
by nickw444 (Nick Whyte) — the public Raex / Neo protocol work this project
builds on.

## License

[MIT](LICENSE) © 2026 Peter Kieser &lt;peter@kieser.ca&gt;.
