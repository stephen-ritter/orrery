project(CFE_BSK_LINK C)

# --- the mission EDS schema -------------------------------------------------
#
# This app decodes the Basilisk link with the mission's own EDS XML, compiled into a
# runtime database by edslink/.  Both artifacts are prebuilt (sedstool needs Lua and a
# host compiler, which the mission cross-toolchain is not supposed to care about):
#
#   <edslink build>/obj/orrery_eds_db.a        the compiled database
#   <edslink build>/eds/edslib/fsw/libedslib_...  the EdsLib runtime
#   <edslink build>/inc/*_eds_defines.h           EDS identities (field/type constants)
#
# Build them first:  ./edslink/build.sh   (cfs_mission/build.sh does this in `all`).
if (NOT DEFINED ORRERY_EDSLINK_BUILD)
    get_filename_component(ORRERY_EDSLINK_BUILD
        "${CMAKE_CURRENT_LIST_DIR}/../../../../edslink/build" ABSOLUTE)
endif()
if (NOT DEFINED ORRERY_EDSLIB_ROOT)
    get_filename_component(ORRERY_EDSLIB_ROOT
        "${CMAKE_CURRENT_LIST_DIR}/../../../cFS/tools/eds/edslib" ABSOLUTE)
endif()

set(ORRERY_EDS_DB "${ORRERY_EDSLINK_BUILD}/obj/orrery_eds_db.a")
set(ORRERY_EDS_RUNTIME "${ORRERY_EDSLINK_BUILD}/eds/edslib/fsw/libedslib_runtime_static.a")
# EdsLib splits its base primitives (pack/unpack, load/store, minimal lookups) into a
# second archive; the runtime archive declares it PUBLIC, which an imported raw archive
# does not carry over. Missing it fails at module LOAD time, not link time, because the
# app .so is loaded by CFE long after CMake is done.
set(ORRERY_EDS_MINIMAL "${ORRERY_EDSLINK_BUILD}/eds/edslib/fsw/libedslib_minimal.a")

if (NOT EXISTS "${ORRERY_EDS_DB}" OR NOT EXISTS "${ORRERY_EDS_RUNTIME}"
    OR NOT EXISTS "${ORRERY_EDS_MINIMAL}")
    message(FATAL_ERROR
        "EDS artifacts missing under ${ORRERY_EDSLINK_BUILD}\n"
        "  run: ./edslink/build.sh   (or pass -DORRERY_EDSLINK_BUILD=<dir>)")
endif()

# --- the application ---------------------------------------------------------
# libzmq is a host library provided by the mission build image (native PSP only);
# it arrives as a DT_NEEDED dep of the app .so and resolves from the system loader
# path at module load time.  EdsLib is linked statically from the archives above,
# so the flight image carries the schema it speaks - no runtime file lookup.
add_cfe_app(bsk_link
    fsw/src/bsk_link.c
)

target_include_directories(bsk_link PRIVATE
    "${ORRERY_EDSLINK_BUILD}/inc"
    "${ORRERY_EDSLIB_ROOT}/fsw/inc"
)

# plain (non-keyword) signature: add_cfe_app itself links the app target
# without keywords, and CMake forbids mixing the two on one target.
#
# whole-archive on the database is not optional: it is one object file whose only
# symbol (ORRERY_DATABASE) nothing in here references directly - the app looks it
# up as an extern - so a normal archive link would silently drop the entire schema
# and every frame would fail to decode.
target_link_libraries(bsk_link
    zmq
    "-Wl,--whole-archive,--no-as-needed"
    "${ORRERY_EDS_DB}"
    "-Wl,--no-whole-archive"
    "${ORRERY_EDS_RUNTIME}"
    "${ORRERY_EDS_MINIMAL}"
)
