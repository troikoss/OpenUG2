/* settings.h — OpenUG2 persistent settings backend.
 *
 * Implements docs/cleanroom-settings/SPEC.md (section numbers in comments refer to it):
 *   - machine-wide "registry" values (install facts, language, graphics) behind a small
 *     store interface: native registry on Windows, an INI file elsewhere, plus an
 *     in-memory store for tests;
 *   - the profile save container ("20CM", checksums) and the 54 930-byte profile payload,
 *     parsed into typed fields while every byte this module does not understand is kept
 *     and written back unchanged;
 *   - graphics presets, capability clamp and performance-level auto-detect.
 * Plain C99, no GUI, no engine dependencies. Design notes: docs/cleanroom-settings/IMPL_DESIGN.md. */
#ifndef OPENUG2_SETTINGS_H
#define OPENUG2_SETTINGS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- status codes ------------------------------------------------------------ */
enum {
    ST_OK = 0,
    ST_E_NOTFOUND = -1,      /* file/value cannot be opened (SPEC 3.5 step 1) */
    ST_E_IO = -2,            /* short read/write, unreadable length (3.5 steps 2, 4) */
    ST_E_CORRUPT = -3,       /* size, magic or checksum mismatch (3.5 steps 3, 5) */
    ST_E_INCOMPATIBLE = -4,  /* version / payload size refused at game level (3.5 step 6) */
    ST_E_ARG = -5,           /* invalid argument (e.g. bad profile name, buffer too small) */
    ST_E_NOMEM = -6
};

/* ---- 3.4 checksum and 3.2 container ------------------------------------------ */
uint32_t st_checksum(const uint8_t *data, size_t n);

#define ST_CONTAINER_HEADER 28
typedef struct StContainerView {
    const uint8_t *a, *b;     /* point into the parsed buffer */
    uint32_t size_a, size_b;
} StContainerView;

/* Validates a whole container held in memory (steps 2..5 of 3.5). */
int st_container_parse(const uint8_t *buf, size_t len, StContainerView *view);
/* Builds a container; returns total size, or 0 for insufficient capacity or
 * a nonempty block with a NULL data pointer. */
size_t st_container_build(uint8_t *out, size_t cap, const uint8_t *a, uint32_t size_a,
                          const uint8_t *b, uint32_t size_b);

/* ---- 5 profile payload ------------------------------------------------------- */
#define ST_PROFILE_VERSION   0x10Du
#define ST_PAYLOAD_SIZE      54930u
#define ST_PROFILE_FILE_SIZE (ST_CONTAINER_HEADER + 8u + ST_PAYLOAD_SIZE)  /* 54 966 */
#define ST_TRAX_COUNT        27
#define ST_INPUT_SLOTS       6
#define ST_INPUT_RECORD_SIZE 181
#define ST_NAME_MAX          15     /* profile name bytes (5.9 / 3.1) */

typedef struct StTraxEntry {        /* 5.6: one playlist slot */
    uint32_t track;                 /* 0..26 */
    uint8_t mode;                   /* 0 off, 1 front end, 2 in game, 3 both */
} StTraxEntry;

typedef struct StAudio {            /* 5.2 */
    float master, speech_movie, menu_music, game_music, sfx, engine, tire;
    float unk28, unk32, unk36;      /* unidentified */
    uint32_t mode;                  /* 0 mono, 1 stereo */
    uint32_t unk44;
    uint32_t trax_order;            /* 0 sequential, 1 random */
    uint32_t unk52, unk56;
} StAudio;

typedef struct StGameplay {         /* 5.3 */
    uint8_t rearview_mirror;
    uint8_t unk1;
    uint8_t speedo_units;           /* 0 MPH, 1 KM/H */
    uint8_t race_map_mode;          /* 0 off, 1 static, 2 rotate */
    uint8_t explore_map_mode;
    float jump_camera;              /* 0.0 = off, anything else on */
    float unk12;
} StGameplay;

typedef struct StPlayer {           /* 5.4 */
    uint8_t gauges, race_position, lap_info, score, feedback, leaderboard, ask_transmission;
    uint32_t controller_config;     /* 0..8 [inferred] */
    uint8_t unk12;
    uint32_t camera;                /* 0 bumper, 1 hood, 2 close, 3 far, 4 super far */
    uint8_t split_time;             /* 0..4 */
    uint8_t transmission;           /* 0 auto, 1 manual */
    uint8_t stability_control;      /* 0 = ON, 1 = OFF */
    uint8_t unk23, unk24, unk25;
    float wheel_range;              /* 0.1..1.0 */
    float road_feedback;            /* stored (inverted) form, 2.0..20.0 */
    float force_strength;           /* 0.0..1.0 */
} StPlayer;

typedef struct StOnline {           /* 5.5 */
    uint8_t show_player_names;
    uint8_t bad_connection_warning;
    uint8_t unk10;
    uint32_t prefs_a[5];            /* byte-packed at offsets 11 and 31 */
    uint32_t prefs_b[5];
} StOnline;                         /* offsets 0 and 4 are always written as 0 */

typedef struct StInputDevice {      /* 6 */
    uint8_t guid[16];
    int32_t axis[4];                /* steering, throttle, brake, combined */
    int32_t dir[4];
    uint8_t assigned[4];
    uint8_t combined_mode;
    int32_t binding[32];
} StInputDevice;

typedef struct StProfile {
    StTraxEntry trax[ST_TRAX_COUNT];
    StAudio audio;
    StGameplay gameplay;
    StPlayer player[2];
    float unkblk_f;                 /* 5.9 unknown block, default 0.5 */
    uint32_t unkblk_a, unkblk_b;
    uint8_t unkblk_c;
    uint32_t career_difficulty;     /* 5.7: 0 easy, 1 medium, 2 hard */
    uint8_t autosave, save_prompt;  /* 5.8 */
    char name[33];                  /* NUL-terminated copy of the 32-byte name field */
    StOnline online;
    StInputDevice input[ST_INPUT_SLOTS];
    /* The payload this state was unpacked from. Everything not covered by a typed field
     * (career region, opaque regions, padding bytes) lives only here and is written back
     * as is. For a fresh profile it is all zero. */
    uint8_t raw[ST_PAYLOAD_SIZE];
} StProfile;                        /* ~60 KB: allocate statically or on the heap */

/* Fresh-profile state (5.x defaults). language_index selects the speedometer-unit default
 * (2.4). Unknown defaults (SPEC 11) use the synthetic-vector placeholders; see IMPL_DESIGN. */
void st_profile_defaults(StProfile *p, int language_index);
void st_profile_pack_payload(const StProfile *p, uint8_t out[ST_PAYLOAD_SIZE]);
void st_profile_unpack_payload(StProfile *p, const uint8_t in[ST_PAYLOAD_SIZE]);

/* Whole file in memory <-> StProfile. Parse applies 3.5 steps 2..6. */
int st_profile_parse(const uint8_t *file, size_t len, StProfile *out);
/* Returns file size (ST_PROFILE_FILE_SIZE) or 0 if cap is too small. */
size_t st_profile_serialize(const StProfile *p, uint8_t *out, size_t cap);

/* "Unsaved changes" check (3.6): snapshot = packed payload taken at load/save. */
void st_profile_snapshot(const StProfile *p, uint8_t snapshot[ST_PAYLOAD_SIZE]);
int st_profile_modified(const StProfile *p, const uint8_t snapshot[ST_PAYLOAD_SIZE]);

/* Option helpers (value rules of 5.x). */
float st_jump_camera_toggle(float v);                    /* 5.3: 0.0 -> 127.0, else 0.0 */
float st_road_feedback_from_slider(float s);             /* 5.4: v = 20 - 18*clamp(s,0,1) */
float st_road_feedback_to_slider(float v);               /* s = 1 - (clamp(v,2,20)-2)/18 */
uint32_t st_career_difficulty_step(uint32_t v, int dir); /* wraps within 0..2 */
uint8_t st_trax_mode_step(uint8_t mode, int dir);        /* cycles 0..3 */
void st_trax_masks(const StProfile *p, uint32_t *front_end, uint32_t *in_game); /* bit = track id */
int st_input_find(const StProfile *p, const uint8_t guid[16]);  /* record index or -1 (zero GUID never matches) */

enum { ST_SAVE_OFF = 0, ST_SAVE_AUTOSAVE = 1, ST_SAVE_PROMPT = 2 };
int st_save_mode(const StProfile *p);                    /* 5.8 table */
void st_save_set_mode(StProfile *p, int mode);
/* dir > 0 right, dir < 0 left. Returns the new mode. Autosave is skipped if !autosave_ok. */
int st_save_cycle(StProfile *p, int dir, int autosave_ok);

/* ---- 3.1 profile files ------------------------------------------------------- */
int st_profile_name_valid(const char *name);             /* 1..15 bytes, valid file name */
int st_profile_path(const char *root, const char *name, char *out, size_t cap);
/* Profile directory/file entries must be real directories/regular files, not
 * links or special files. Writes reject files with multiple hard links. */
int st_profile_save(const char *root, const StProfile *p);     /* uses p->name; not atomic (3.3) */
int st_profile_load(const char *root, const char *name, StProfile *out);
int st_profile_exists(const char *root, const char *name);     /* directory holds anything */
int st_profile_presence(const char *root, const char *name);   /* OK present, NOTFOUND confirmed absent, IO unknown */
int st_profile_delete(const char *root, const char *name);
int st_profiles_exist(const char *root);                       /* root holds anything besides . .. */

int st_default_profile_root(char *out, size_t cap);   /* <local app data>/[OpenUG2/]NFS Underground 2 */
int st_default_config_path(char *out, size_t cap);    /* INI file used by the non-Windows store */

/* ---- 2 machine-wide value store (registry abstraction) ----------------------- */
typedef struct StRegOps {
    int (*exists)(void *impl, const char *name);                                  /* 1 / 0 */
    int (*get_dword)(void *impl, const char *name, uint32_t *out);                /* 1 ok, 0 absent/unusable */
    int (*get_string)(void *impl, const char *name, char *buf, size_t cap);       /* 1 ok, 0 absent/unusable */
    int (*set_dword)(void *impl, const char *name, uint32_t v);                   /* ST_OK or error */
    int (*set_string)(void *impl, const char *name, const char *v);
    void (*close)(void *impl);
} StRegOps;

typedef struct StRegStore {
    const StRegOps *ops;
    void *impl;
} StRegStore;

int st_reg_open_memory(StRegStore *s);                      /* volatile, for tests */
/* Missing file opens an empty store; malformed recognized values, read errors
 * and allocation failures return an error without publishing a partial store. */
int st_reg_open_ini(StRegStore *s, const char *path);       /* file store, any platform */
int st_reg_open_default(StRegStore *s);                     /* Windows: HKLM registry; else INI */
void st_reg_close(StRegStore *s);
int st_reg_exists(StRegStore *s, const char *name);
int st_reg_get_dword(StRegStore *s, const char *name, uint32_t *out);
int st_reg_get_string(StRegStore *s, const char *name, char *buf, size_t cap);
int st_reg_set_dword(StRegStore *s, const char *name, uint32_t v);
int st_reg_set_string(StRegStore *s, const char *name, const char *v);

/* 2.2 / 2.4 */
typedef struct StInstall {
    char install_dir[512];
    char cd_drive[512];
    int not_first_time;             /* value existed */
    long cache_size, swap_size;     /* atol semantics */
    int language_index;             /* -1 = value present but unrecognised */
    int streaming_install;          /* 0 only if the value equals "off" (any case) */
} StInstall;
int st_language_index(const char *name);                    /* case-insensitive; -1 if no match */
void st_install_read(StRegStore *s, StInstall *out);
int st_install_mark_started(StRegStore *s);                 /* writes NotFirstTime = 1 */

/* ---- 2.3 / 4 graphics --------------------------------------------------------- */
enum {  /* slot = "struct index" column of SPEC 2.3 */
    ST_GFX_VERSION = 0, ST_GFX_SIZE, ST_GFX_CAR_ENV_MAP, ST_GFX_CAR_ENV_MAP_UPDATE,
    ST_GFX_CAR_SHADOW, ST_GFX_CAR_HEADLIGHT, ST_GFX_CAR_LIGHTING, ST_GFX_CAR_DAMAGE,
    ST_GFX_CROWD, ST_GFX_ROAD_REFLECTION, ST_GFX_FOG, ST_GFX_MOTION_BLUR,
    ST_GFX_LIGHT_STREAKS, ST_GFX_LIGHT_GLOW, ST_GFX_ANIMATED_TEXTURE, ST_GFX_PARTICLES,
    ST_GFX_DEPTH_OF_FIELD, ST_GFX_WORLD_LOD, ST_GFX_CAR_LOD, ST_GFX_OVER_BRIGHT,
    ST_GFX_BLEACH_BYPASS, ST_GFX_TINTING, ST_GFX_FSAA, ST_GFX_HORIZON_FOG, ST_GFX_RAIN,
    ST_GFX_SLOT25,                  /* not persisted */
    ST_GFX_TEXTURE_FILTERING, ST_GFX_RESOLUTION,
    ST_GFX_SLOT28,                  /* unused here */
    ST_GFX_VSYNC,
    ST_GFX_PERF_LEVEL,              /* g_PerformanceLevel: no struct index in the original */
    ST_GFX_NSLOTS
};
#define ST_NO_CAP 0x7FFFFFFF

typedef struct StGraphics { int32_t v[ST_GFX_NSLOTS]; } StGraphics;
typedef struct StGfxCaps {
    int32_t cap[ST_GFX_NSLOTS];     /* ST_NO_CAP where nothing limits the value */
    int resolution_ok[6];           /* adapter supports resolution index at 32-bit colour */
} StGfxCaps;

const char *st_gfx_value_name(int slot);                    /* registry name, NULL if not persisted */
void st_gfx_defaults(StGraphics *g);                        /* static defaults */
void st_gfx_caps_full(StGfxCaps *c, int fsaa_levels);       /* "fully capable modern adapter" (4.3) */
void st_gfx_load(StRegStore *s, StGraphics *g);             /* keeps current value where absent */
int st_gfx_save(StRegStore *s, const StGraphics *g);        /* 2.3 write order */
void st_gfx_clamp(StGraphics *g, const StGfxCaps *c);       /* 4.3 */
void st_gfx_apply_preset(StGraphics *g, const StGfxCaps *c, int level, double gpu_score); /* 4.2 table, no clamp */
int st_perf_level(int cpu_mhz, double gpu_score);           /* 4.5 */
/* 4.2 auto-detect: resolution 0, preset for `level`, clamp; records level in PERF_LEVEL. */
void st_gfx_autodetect(StGraphics *g, const StGfxCaps *c, int level, double gpu_score);
/* Level Of Detail slider: records level, applies preset + clamp (resolution untouched). */
void st_gfx_set_level(StGraphics *g, const StGfxCaps *c, int level, double gpu_score);
/* 4.1 steps 3, 6, 7. Returns 1 when this was a first run, 0 otherwise. Safe mode also saves. */
int st_gfx_startup(StRegStore *s, StGraphics *g, const StGfxCaps *c, int safe_mode,
                   int cpu_mhz, double gpu_score);
int st_resolution_step(StGraphics *g, const StGfxCaps *c, int dir);  /* 4.4; 1 if changed */
int st_resolution_size(int index, int *w, int *h);          /* 2.3; 0 if index out of range */
int st_audio_rate_hz(int perf_level);                       /* 2.3: <=1 -> 22050 else 44100 */
int st_parse_safe_mode(int argc, char **argv);              /* 4.6: first argument "/s" */

#ifdef __cplusplus
}
#endif
#endif
