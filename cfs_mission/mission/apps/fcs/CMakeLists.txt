project(CFE_FCS C)

# The orrery flight controller (P3c). No external deps: its message
# structs come from orrery_defs/inc/orrery_bus_msg.h, whose include
# directory the cFE build already puts on every app's path, and the
# _Static_assert in that header is this app's contract with the EDS XML.
add_cfe_app(fcs
    fsw/src/fcs_app.c
)
