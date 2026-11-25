project(CFE_FCS C)

# The orrery flight controller. No external deps: its message
# structs come from orrery_defs/inc/orrery_bus_msg.h, whose include
# directory the cFE build already puts on every app's path, and the
# _Static_assert in that header is this app's contract with the EDS XML.
add_cfe_app(fcs
    fsw/src/fcs_app.c
)
# The CFDP downlink relay speaks CF's published command MID (cf_msgids.h);
# like cf_tcp, that header's expansion reaches into CF's fsw/inc (same
# pattern, same reason). Message layouts stay on the mission mirror in
# orrery_bus_msg.h.
target_include_directories(fcs PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../cFS/apps/cf/fsw/inc
)
