import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover

from .. import RaexBlindTransmitComponent, raex_blind_ns

CONF_TRANSMITTER_ID = "transmitter_id"
CONF_REMOTE_ID = "remote_id"
CONF_CHANNEL = "channel_id"
CONF_ALIASES = "aliases"
CONF_OPEN_TIME = "open_time"
CONF_CLOSE_TIME = "close_time"
CONF_MICRO_STEP = "micro_step"

RaexCover = raex_blind_ns.class_("RaexCover", cover.Cover, cg.Component)

# An alias is an extra physical remote paired to THIS blind: a frame from
# (remote_id, channel) is attributed to this cover's primary identity.
ALIAS_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_REMOTE_ID): cv.int_range(min=0, max=65535),
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=255),
    }
)

# Self-contained per-blind config: identity (the RX trust gate), its own
# aliases, per-direction travel times, micro-step. The component is a pure hub.
CONFIG_SCHEMA = (
    cover.cover_schema(RaexCover)
    .extend(
        {
            cv.GenerateID(CONF_TRANSMITTER_ID): cv.use_id(RaexBlindTransmitComponent),
            cv.Required(CONF_REMOTE_ID): cv.int_range(min=0, max=65535),
            cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=255),
            cv.Optional(CONF_ALIASES): cv.ensure_list(ALIAS_SCHEMA),
            cv.Optional(
                CONF_OPEN_TIME, default="17s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_CLOSE_TIME, default="17s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MICRO_STEP, default="0.5%"): cv.percentage,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)  # RaexCover is a Component (loop())
    parent = await cg.get_variable(config[CONF_TRANSMITTER_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_remote_id(config[CONF_REMOTE_ID]))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    # Register this blind with the hub: primary identity (trust gate),
    # per-direction travel times, micro-step, own persistence slot.
    cg.add(
        parent.register_blind(
            config[CONF_REMOTE_ID],
            config[CONF_CHANNEL],
            config[CONF_OPEN_TIME],
            config[CONF_CLOSE_TIME],
            config[CONF_MICRO_STEP],
        )
    )
    # Fold each extra remote onto this blind's primary identity.
    for a in config.get(CONF_ALIASES, []):
        cg.add(
            parent.add_alias(
                a[CONF_REMOTE_ID],
                a[CONF_CHANNEL],
                config[CONF_REMOTE_ID],
                config[CONF_CHANNEL],
            )
        )
