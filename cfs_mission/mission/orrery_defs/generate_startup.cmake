# Fallback wake-up table: codegen/emit_startup.py derives a startup table
# per vehicle from the scenario and the registry, and the yard door lands
# it (docs/what_the_scenario_rules.md). This build-time table is what a
# hand-run compose yard boots - and it names only apps THIS build built:
# the build list itself is the scenario's wake list (codegen/emit_applist
# lands mission_applist.cmake), so a variant that never grew the eye has
# no cam object to boot, and this table cannot promise one.
# tests/test_startup_table.py pins the two sources of this grammar to each
# other - edit registry/apps.toml, and this file's list must follow.
function (generate_cfs_startup_script CFS_INSTALL_DIR)
    set (STARTUP_FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CFS_INSTALL_DIR}/cfe_es_startup.scr")

    # One record per app, boot order; the grammar is ES's (eight fields, ';'
    # ends each, no comments). The core's own lib answers first - nothing
    # decides whether it exists. Every mission app then rides the build
    # list: its line enters the table only if the app was compiled in.
    set (LINE_cfe_assert   "CFE_LIB, cfe_assert,  CFE_Assert_LibInit, ASSERT_LIB,    0,   0,     0x0, 0;\n")
    set (LINE_sch_lab      "CFE_APP, sch_lab,     SCH_LAB_AppMain,    SCH_LAB,      35,   32768, 0x0, 0;\n")
    set (LINE_bsk_link     "CFE_APP, bsk_link,     BSK_LINK_AppMain,   BSK_LINK,     55,   131072, 0x0, 0;\n")
    set (LINE_fcs          "CFE_APP, fcs,          FCS_AppMain,        FCS,          65,   65536,  0x0, 0;\n")
    set (LINE_obc_hk       "CFE_APP, obc_hk,       OBC_HK_AppMain,     OBC_HK,       66,   65536,  0x0, 0;\n")
    set (LINE_sps          "CFE_APP, sps,          SPS_AppMain,        SPS,          67,   65536,  0x0, 0;\n")
    set (LINE_cam          "CFE_APP, cam,          CAM_AppMain,        CAM,          68,   65536,  0x0, 0;\n")
    set (LINE_cf_tcp       "CFE_APP, cf_tcp,       CF_TCP_AppMain,     CF_TCP,       69,   65536,  0x0, 0;\n")
    set (LINE_gs_tcp       "CFE_APP, gs_tcp,       GS_TCP_AppMain,     GS_TCP,       60,   65536,  0x0, 0;\n")
    set (LINE_cf           "CFE_APP, cf,           CF_AppMain,         CF,           70,   131072, 0x0, 0;\n")
    set (LINE_sc           "CFE_APP, sc,           SC_AppMain,         SC,           70,   131072, 0x0, 0;\n")

    file (WRITE ${STARTUP_FILE} "${LINE_cfe_assert}")
    # sc last, the way the prebaked era booted it: the ATS executor comes up
    # only after every organ it will command is listening (2026-11-28 revival,
    # registry/apps.toml).
    foreach (APP sch_lab bsk_link fcs obc_hk sps cam cf_tcp gs_tcp cf sc)
        # the tail's idiom: IN_LIST wants policy CMP0057, which this install
        # scope never promised
        list (FIND ARGN ${APP} SHOULD_START)
        if (SHOULD_START GREATER -1)
            file (APPEND ${STARTUP_FILE} "${LINE_${APP}}")
        endif ()
    endforeach ()

endfunction(generate_cfs_startup_script)
