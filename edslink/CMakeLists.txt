# EdsLib runtime database + CPython binding for the orrery EDS tree.
#
# Builds from the same XML that codegen/eds2cosmos turns into COSMOS definitions
# (the shared list lives in codegen/eds_sources.py and is mirrored here by
# eds_dirs.cmake), so the simulator side and the ground side cannot drift apart.
#
# Normal entry point is ./build.sh - it resolves the Python headers for the project
# venv first. Requirements: cmake, a C compiler, make, and Lua >= 5.3 development
# headers (liblua5.4-dev), which is what the cFS build image already installs.
cmake_minimum_required(VERSION 3.12)
project(ORRERY_EDSLINK C)

get_filename_component(ORRERY_REPO "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(CFS_DIR "${ORRERY_REPO}/cfs_mission/cFS")
set(EDS_ROOT "${CFS_DIR}/tools/eds")

if(NOT IS_DIRECTORY "${EDS_ROOT}/edslib")
    message(FATAL_ERROR
        "No EdsLib at ${EDS_ROOT} - run 'git submodule update --init' in cfs_mission/cFS")
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/eds_dirs.cmake")
    message(FATAL_ERROR
        "eds_dirs.cmake missing - run 'uv run python -m codegen.eds_sources --write'")
endif()

include("${EDS_ROOT}/tool/cmake/edstool-execute-target.cmake")
add_subdirectory("${EDS_ROOT}" "${CMAKE_BINARY_DIR}/eds")
include("${CMAKE_CURRENT_SOURCE_DIR}/eds_dirs.cmake")

# The binding dlopen()s "<name>_eds_db.so" by bare name, so give it a runpath that
# points next to itself: dist/ holds the module and the database together and no
# caller has to export LD_LIBRARY_PATH. ($ORIGIN must survive to the linker.)
# Guarded: EDSLINK_NO_PYTHON builds configure EdsLib without a CPython, in which
# case the binding target does not exist and the flight archives need no rpath.
if (TARGET edslib_python_module)
    set_target_properties(edslib_python_module PROPERTIES BUILD_RPATH "\$ORIGIN"
                                                          INSTALL_RPATH "\$ORIGIN")
endif()

# One database holding every package this mission is built from: cFE + apps +
# cFS message headers + mission defines + the BSK_LINK link containers.
#
# ${EDS_ROOT}/edslib contributes no XML: it carries the numbered Lua scripts
# (40-seds_write_headers ... 59-load_edsdb) that actually emit the runtime database.
# Without them sedstool parses the tree and writes nothing at all.
edstool_find_eds_sources(orrery ${CMAKE_BINARY_DIR}
    ${EDS_ROOT}/edslib
    ${ORRERY_EDS_DIRS})
edstool_setup_toplevel(edstool-execute-orrery orrery ${CMAKE_BINARY_DIR})
