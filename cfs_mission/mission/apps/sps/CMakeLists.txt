project(CFE_SPS C)

# The orrery time-tagged command executor. Its only extra input is the
# generated command dictionary (sps_cmd_table.h, codegen/eds2cosmos); the
# message structs and MIDs come from orrery_defs like every other mission app.
add_cfe_app(sps
    fsw/src/sps_app.c
)
