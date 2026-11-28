# For the native builds, include a wrapper script to start cFS on the container.
if (${SIMULATION} MATCHES "^native")
    install(PROGRAMS ${CMAKE_CURRENT_LIST_DIR}/container-start DESTINATION cpu2)
endif()

# The schedule table names the CF engine's MsgIds (registry/apps.toml: the
# timer speaks to the core, cf, fcs, obc_hk, sps). The 2026-11-27 prebaked
# purge retired the other twelve include feeds - their apps left the build,
# so their msgid headers left the search path with them - along with the
# to_lab subscription table, the SBN define, and the alternate/fixture
# table files (sch_lab_table.c is the only mission-authored table left).
# 2026-11-28: sc returns (sequence arc), and with it its msgid feed.
set(APP_INCLUDE_TABLE_LIST
    $<TARGET_PROPERTY:cf,INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:sc,INCLUDE_DIRECTORIES>
)
target_include_directories(sch_lab.table INTERFACE ${APP_INCLUDE_TABLE_LIST})
target_compile_definitions(sch_lab.table INTERFACE)

# The factory ships no sequences, here too: see cfs_mission/build.sh install().

# This executes the startup script generator
install(SCRIPT ${MISSION_DEFS}/generate_startup.cmake)
install(CODE "generate_cfs_startup_script(\"${TGTNAME}/${INSTALL_SUBDIR}\" ${${TGTNAME}_APPLIST})")
