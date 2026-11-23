project(CFE_OBC_HK C)

# The orrery OBC housekeeping app (P4b-3). Passive observer: message
# structs come from orrery_defs/inc/orrery_bus_msg.h (whose
# _Static_assert is the contract with obc.xml); no external deps.
add_cfe_app(obc_hk
    fsw/src/obc_hk_app.c
)

# |B| wants sqrt, and sqrt is the first libm symbol any orrery app has
# ever needed at load time (fcs's fabs compiles inline). Without this the
# .so loads with "undefined symbol: sqrt" and ES rejects the module. Same
# DT_NEEDED-from-the-system-loader idiom as bsk_link's zmq; plain signature
# because add_cfe_app links without keywords.
target_link_libraries(obc_hk m)
