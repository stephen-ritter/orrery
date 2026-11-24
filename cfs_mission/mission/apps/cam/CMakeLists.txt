project(CFE_CAM C)

# The orrery camera payload command surface (CAM v2a). No external deps:
# its message structs come from orrery_defs/inc/orrery_bus_msg.h, whose
# include directory the cFE build already puts on every app's path, and
# the _Static_assert in that header is this app's contract with the EDS XML.
add_cfe_app(cam
    fsw/src/cam_app.c
)
