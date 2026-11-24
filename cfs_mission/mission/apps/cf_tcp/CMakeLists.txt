project(CFE_CF_TCP C)

# The CFDP transport bridge: CF's channel PDUs (SB messages) to the one
# TCP conversation the ground CFDP engine dials into. The CF message ids
# come from cf_msgids.h, the same generated header CF's own default config
# table names - one source of truth for the two ends of the bridge.
add_cfe_app(cf_tcp
    fsw/src/cf_tcp.c
)
# cf_msgids.h's expansion reaches into CF's fsw/inc for cf_topicids.h; the
# generated msgid header itself is already on the mission-wide path.
target_include_directories(cf_tcp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../cFS/apps/cf/fsw/inc
)
