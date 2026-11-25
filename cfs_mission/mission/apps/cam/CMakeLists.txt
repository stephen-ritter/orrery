project(CFE_CAM C)

# The orrery camera payload command surface. No external deps:
# its message structs come from orrery_defs/inc/orrery_bus_msg.h, whose
# include directory the cFE build already puts on every app's path, and
# the _Static_assert in that header is this app's contract with the EDS XML.
add_cfe_app(cam
    fsw/src/cam_app.c
)
# The transferred-file witness subscribes CF's EOT MID (cf_msgids.h); like
# cf_tcp, that header's expansion reaches into CF's fsw/inc (same pattern,
# same reason). Message layout stays on the mission mirror in
# orrery_bus_msg.h.
target_include_directories(cam PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../cFS/apps/cf/fsw/inc
)
