/*
 * cam: orrery camera payload command surface (CAM v2a).
 *
 * The smallest honest flight software for a commanded eye: the ground points
 * the camera (shutter, render rate), this app validates, remembers, and
 * forwards each accepted command to the sim through bsk_link. It is a
 * forwarder, not a supervisor - the app that commands does not grade its own
 * delivery. The sim's CONFIRMED state (shutter in use, rate in use, images
 * captured) rides the BSK_LINK/CamState registry row, and the ground screens
 * the pair: CAM_HK.RenderRateCmd against CamState.RenderRateS. Divergence is
 * the defect class, not an ack this lockstep would have to wait on - image
 * traffic never gates the physics (v1 law, unchanged).
 *
 * Commands:
 *   CAM_CMD         FC 0 NOOP
 *                   FC 1 MANIFEST - write /cf/manifest.txt, the store's
 *                   requestable inventory (name,size per file). The ground
 *                   then requests the manifest through FCS_CFDP_DOWN like
 *                   any other file: inventory is a file too, so the file
 *                   path stays the only data plane. No arguments, so it
 *                   rides CAM_CMD (own-interface law binds payload commands
 *                   only - the SetGain law is about losing arguments).
 *   CAM_SET_CONFIG  upload shutter + camera id + render rate (own packet,
 *                   payload command per the SetGain law: arguments need a
 *                   real payload or codegen loses them)
 *   CAM_SEND_HK     housekeeping on the scheduler tick
 *
 * Why the validation here is strict (ledger: Phase-0 spike + B1): the sim
 * door writes the module's live fields and vizInterface gates image requests
 * on a renderRate modulo - zero divides by zero inside Basilisk, and the
 * Phase-0 flights proved every OTHER value is live, reversible, and digest-
 * neutral. A knob that can crash the vehicle is not a knob, so a refused
 * command is a feature, reported on EVS and in CommandErrorCounter.
 *
 * The sensor frame subscription is READ-ONLY stamping: it gives every
 * forwarded config the sim time and frame it rode, so the sim can attribute
 * a device-state change to the command instant. cam answers nothing and
 * joins no barrier.
 */

#ifndef CAM_APP_H
#define CAM_APP_H

#include "cfe.h"
#include "orrery_bus_msg.h"
#include "orrery_msgids.h"

/** Command/data pipe depth (1 Hz sensor frames + a handful of commands). */
#define CAM_PIPE_DEPTH 8

/** SB receive timeout in ms. */
#define CAM_SB_TIMEOUT 500

/**
 * Render-rate bounds for SET_CONFIG uploads, seconds. The lower bound is
 * the B1 lesson (zero is a divide-by-zero in the render modulo, not a
 * config); the upper bound only rejects nonsense - one image per hour is a
 * dead payload, not a payload with opinions.
 */
#define CAM_RENDER_RATE_MAX 3600.0

/** The store's directory (the image store's home) and the inventory CAM_MANIFEST
 *  writes into. The ground requests /cf/manifest.txt through FCS_CFDP_DOWN
 *  by this exact name; one name, one law, both sides. */
#define CAM_IMG_DIR       "/cf/img"
#define CAM_MANIFEST_PATH "/cf/manifest.txt"
#define CAM_MANIFEST_TMP  "/cf/manifest.new"

typedef struct
{
    CFE_SB_PipeId_t Pipe;

    /* The commanded view (the confirmed view is CamState's job). Until the
     * first accepted SET_CONFIG, CameraId is 0 and Shutter/RenderRateCmd
     * carry NO meaning about the device - the app cannot know what TOML
     * boot-loaded, and guessing either way would be a lie with a label on
     * it. The sentinel, not the zeros, is the statement. */
    uint8  Shutter;          /* last commanded (1 open)                     */
    uint8  CameraId;         /* 0 = never configured this boot              */
    double RenderRateCmd;    /* last commanded image interval [s]           */

    uint32 CommandCounter;
    uint32 CommandErrorCounter;
    uint32 ConfigFramesForwarded;  /* accepted configs egressed to the sim  */
    uint32 ConfigFwdFailCount;     /* publishes that never reached bsk_link */

    /* CAM v2b pass-through witnesses (read-only): RECORDED is what the sim's
     * store outlet sent (CamState), STORED is what bsk_link reassembled into
     * /cf/img (CamStorage). Neither is this app's own doing - the four-number
     * report exists so their divergence, not its prose, names the defect. */
    uint64 FramesRecorded;
    uint64 FilesStored;
    uint64 FilesTransferred; /* CF EOT count: files CF closed clean, TX */

    /* Read-only stamping from the sensor feed (never a law input). */
    double LastSimTimeS;
    uint64 LastFrame;

    CamHkTlm_t HkTlm;
} CAM_AppData_t;

extern CAM_AppData_t CAM_AppData;

void CAM_AppMain(void);

#endif /* CAM_APP_H */
