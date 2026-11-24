function (generate_cfs_startup_script CFS_INSTALL_DIR)
    set (STARTUP_FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${CFS_INSTALL_DIR}/cfe_es_startup.scr")

    # These apps should always be present
    file (WRITE ${STARTUP_FILE}
        "CFE_LIB, cfe_assert,  CFE_Assert_LibInit, ASSERT_LIB,    0,   0,     0x0, 0;\n"
        "CFE_LIB, sample_lib,  SAMPLE_LIB_Init,    SAMPLE_LIB,    0,   0,     0x0, 0;\n"
        "CFE_APP, sch_lab,     SCH_LAB_AppMain,    SCH_LAB,      35,   32768, 0x0, 0;\n"
        "CFE_APP, ci_lab,      CI_LAB_AppMain,     CI_LAB,       40,   32768, 0x0, 0;\n"
        "CFE_APP, to_lab,      TO_LAB_AppMain,     TO_LAB,       45,   32768, 0x0, 0;\n"
        "CFE_APP, sample_app,  SAMPLE_APP_Main,    SAMPLE_APP,   50,   32768, 0x0, 0;\n"
        "CFE_APP, bsk_link,     BSK_LINK_AppMain,   BSK_LINK,     55,   131072, 0x0, 0;\n"
        "CFE_APP, fcs,          FCS_AppMain,        FCS,          65,   65536,  0x0, 0;\n"
        "CFE_APP, obc_hk,       OBC_HK_AppMain,     OBC_HK,       66,   65536,  0x0, 0;\n"
        "CFE_APP, sps,          SPS_AppMain,        SPS,          67,   65536,  0x0, 0;\n"
        "CFE_APP, cam,          CAM_AppMain,        CAM,          68,   65536,  0x0, 0;\n"
        "CFE_APP, cf_tcp,       CF_TCP_AppMain,     CF_TCP,       69,   65536,  0x0, 0;\n"
        "CFE_APP, gs_tcp,       GS_TCP_AppMain,     GS_TCP,       60,   65536,  0x0, 0;\n"
    )

    # the rest of the apps can vary by config
    set(CFS_APP_STARTUP_ORDER lc cf ds fm hk hs mm sc md cs sbn)
    foreach(APP ${CFS_APP_STARTUP_ORDER})
        list(FIND ARGN ${APP} SHOULD_START)
        if (SHOULD_START GREATER -1)
            string(TOUPPER "${APP}" APP_UPPER)
            file (APPEND ${STARTUP_FILE}
                "CFE_APP, ${APP},  ${APP_UPPER}_AppMain, ${APP_UPPER}, 70,  131072, 0x0, 0;\n"
            )
        endif()
    endforeach()

endfunction(generate_cfs_startup_script)
