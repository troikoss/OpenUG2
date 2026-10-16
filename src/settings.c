/* settings.c — OpenUG2 persistent settings backend (see settings.h, docs/cleanroom-settings). */
#include "settings.h"

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#include <sys/stat.h>
#include <windows.h>
#define ST_SEP '\\'
#else
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#define ST_SEP '/'
#endif

/* ======================================================================= bytes */
static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static void wr32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static float rdf(const uint8_t *p) { uint32_t u = rd32(p); float f; memcpy(&f, &u, 4); return f; }
static void wrf(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); wr32(p, u); }

/* ================================================================== 3.4 checksum */
static uint32_t g_crc_tab[256];
static int g_crc_ready;
static void crc_init(void) {
    if (g_crc_ready) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t r = i << 24;
        for (int k = 0; k < 8; k++) r = (r & 0x80000000u) ? (r << 1) ^ 0x04C11DB7u : r << 1;
        g_crc_tab[i] = r;
    }
    g_crc_ready = 1;
}

uint32_t st_checksum(const uint8_t *d, size_t n) {
    if (n < 4) return 0;
    crc_init();
    uint32_t r = ~((uint32_t)d[0] << 24 | (uint32_t)d[1] << 16 | (uint32_t)d[2] << 8 | d[3]);
    for (size_t i = 4; i < n; i++) r = ((r << 8) | d[i]) ^ g_crc_tab[r >> 24];
    return ~r;
}

/* ================================================================ 3.2 container */
int st_container_parse(const uint8_t *buf, size_t len, StContainerView *v) {
    if (!buf || !v) return ST_E_ARG;
    if (len < ST_CONTAINER_HEADER) return ST_E_IO;
    uint32_t total = rd32(buf + 4), sa = rd32(buf + 8), sb = rd32(buf + 12);
    if ((uint64_t)total != (uint64_t)len || memcmp(buf, "20CM", 4) != 0 ||
        rd32(buf + 24) != st_checksum(buf, 24))
        return ST_E_CORRUPT;
    if ((uint64_t)sa + (uint64_t)sb > (uint64_t)len - ST_CONTAINER_HEADER) return ST_E_IO;
    const uint8_t *a = buf + ST_CONTAINER_HEADER, *b = a + sa;
    if (st_checksum(a, sa) != rd32(buf + 16) || st_checksum(b, sb) != rd32(buf + 20))
        return ST_E_CORRUPT;
    v->a = a; v->b = b; v->size_a = sa; v->size_b = sb;
    return ST_OK;
}

size_t st_container_build(uint8_t *out, size_t cap, const uint8_t *a, uint32_t sa,
                          const uint8_t *b, uint32_t sb) {
    uint64_t total = (uint64_t)ST_CONTAINER_HEADER + sa + sb;
    if (!out || (sa && !a) || (sb && !b) || total > 0xFFFFFFFFu || total > cap) return 0;
    memcpy(out, "20CM", 4);
    wr32(out + 4, (uint32_t)total);
    wr32(out + 8, sa);
    wr32(out + 12, sb);
    wr32(out + 16, st_checksum(a, sa));
    wr32(out + 20, st_checksum(b, sb));
    wr32(out + 24, st_checksum(out, 24));
    if (sa) memcpy(out + ST_CONTAINER_HEADER, a, sa);
    if (sb) memcpy(out + ST_CONTAINER_HEADER + sa, b, sb);
    return (size_t)total;
}

/* ============================================================== 5 profile payload */
enum {
    OFF_TRAX = 50211, OFF_AUDIO = 50427, OFF_GAMEPLAY = 50487, OFF_PLAYER = 50503,
    OFF_UNKBLK = 50583, OFF_DIFFICULTY = 50979, OFF_AUTOSAVE = 50987, OFF_PROMPT = 50988,
    OFF_NAME = 53761, OFF_ONLINE = 53793, OFF_INPUT = 53844,
    PLAYER_SIZE = 40, TRAX_SIZE = 8
};

void st_profile_defaults(StProfile *p, int language_index) {
    memset(p, 0, sizeof *p);
    /* EA TRAX default is unknown (SPEC 11.1): identity order, every mode "both". */
    for (int i = 0; i < ST_TRAX_COUNT; i++) { p->trax[i].track = (uint32_t)i; p->trax[i].mode = 3; }
    StAudio *a = &p->audio;
    a->master = a->speech_movie = a->game_music = a->sfx = a->engine = a->tire = 1.0f;
    a->menu_music = 0.7f;
    a->unk28 = a->unk32 = a->unk36 = 1.0f;
    a->mode = 1;
    a->trax_order = 1;
    StGameplay *g = &p->gameplay;
    g->rearview_mirror = 1;
    g->speedo_units = (uint8_t)(language_index != 0);
    g->race_map_mode = g->explore_map_mode = 2;
    g->jump_camera = g->unk12 = 127.5f;
    for (int i = 0; i < 2; i++) {
        StPlayer *q = &p->player[i];
        q->gauges = q->race_position = q->lap_info = q->score = q->feedback = 1;
        q->leaderboard = q->ask_transmission = 1;
        q->camera = 3;
        q->stability_control = 1;
        q->unk24 = 1;
        q->wheel_range = 1.0f;
        q->road_feedback = 12.4f;
        q->force_strength = 1.0f;
    }
    p->unkblk_f = 0.5f;
    p->autosave = 1;                     /* unknown default (SPEC 11): placeholder */
    p->online.show_player_names = 1;     /* unknown default (SPEC 5.5): placeholder */
    p->online.bad_connection_warning = 1;
}

void st_profile_pack_payload(const StProfile *p, uint8_t out[ST_PAYLOAD_SIZE]) {
    memcpy(out, p->raw, ST_PAYLOAD_SIZE);
    for (int i = 0; i < ST_TRAX_COUNT; i++) {
        uint8_t *e = out + OFF_TRAX + i * TRAX_SIZE;
        wr32(e, p->trax[i].track);
        e[4] = p->trax[i].mode;
    }
    const StAudio *a = &p->audio;
    uint8_t *w = out + OFF_AUDIO;
    wrf(w + 0, a->master); wrf(w + 4, a->speech_movie); wrf(w + 8, a->menu_music);
    wrf(w + 12, a->game_music); wrf(w + 16, a->sfx); wrf(w + 20, a->engine); wrf(w + 24, a->tire);
    wrf(w + 28, a->unk28); wrf(w + 32, a->unk32); wrf(w + 36, a->unk36);
    wr32(w + 40, a->mode); wr32(w + 44, a->unk44); wr32(w + 48, a->trax_order);
    wr32(w + 52, a->unk52); wr32(w + 56, a->unk56);
    const StGameplay *g = &p->gameplay;
    w = out + OFF_GAMEPLAY;
    w[0] = g->rearview_mirror; w[1] = g->unk1; w[2] = g->speedo_units;
    w[3] = g->race_map_mode; w[4] = g->explore_map_mode;
    wrf(w + 8, g->jump_camera); wrf(w + 12, g->unk12);
    for (int i = 0; i < 2; i++) {
        const StPlayer *q = &p->player[i];
        w = out + OFF_PLAYER + i * PLAYER_SIZE;
        w[0] = q->gauges; w[1] = q->race_position; w[2] = q->lap_info; w[3] = q->score;
        w[4] = q->feedback; w[5] = q->leaderboard; w[6] = q->ask_transmission;
        wr32(w + 8, q->controller_config); w[12] = q->unk12;
        wr32(w + 16, q->camera);
        w[20] = q->split_time; w[21] = q->transmission; w[22] = q->stability_control;
        w[23] = q->unk23; w[24] = q->unk24; w[25] = q->unk25;
        wrf(w + 28, q->wheel_range); wrf(w + 32, q->road_feedback); wrf(w + 36, q->force_strength);
    }
    w = out + OFF_UNKBLK;
    wrf(w, p->unkblk_f); wr32(w + 4, p->unkblk_a); wr32(w + 8, p->unkblk_b); w[12] = p->unkblk_c;
    wr32(out + OFF_DIFFICULTY, p->career_difficulty);
    out[OFF_AUTOSAVE] = p->autosave;
    out[OFF_PROMPT] = p->save_prompt;
    w = out + OFF_NAME;
    memset(w, 0, 32);
    size_t nl = 0;
    while (nl < 32 && p->name[nl]) nl++;
    memcpy(w, p->name, nl > 32 ? 32 : nl);
    w = out + OFF_ONLINE;
    wr32(w, 0); wr32(w + 4, 0);                 /* always written 0 (5.5) */
    w[8] = p->online.show_player_names; w[9] = p->online.bad_connection_warning;
    w[10] = p->online.unk10;
    for (int i = 0; i < 5; i++) {
        wr32(w + 11 + 4 * i, p->online.prefs_a[i]);
        wr32(w + 31 + 4 * i, p->online.prefs_b[i]);
    }
    for (int i = 0; i < ST_INPUT_SLOTS; i++) {
        const StInputDevice *d = &p->input[i];
        w = out + OFF_INPUT + i * ST_INPUT_RECORD_SIZE;
        memcpy(w, d->guid, 16);
        for (int k = 0; k < 4; k++) {
            wr32(w + 16 + 4 * k, (uint32_t)d->axis[k]);
            wr32(w + 32 + 4 * k, (uint32_t)d->dir[k]);
            w[48 + k] = d->assigned[k];
        }
        w[52] = d->combined_mode;
        for (int k = 0; k < 32; k++) wr32(w + 53 + 4 * k, (uint32_t)d->binding[k]);
    }
}

void st_profile_unpack_payload(StProfile *p, const uint8_t in[ST_PAYLOAD_SIZE]) {
    memcpy(p->raw, in, ST_PAYLOAD_SIZE);
    for (int i = 0; i < ST_TRAX_COUNT; i++) {
        const uint8_t *e = in + OFF_TRAX + i * TRAX_SIZE;
        p->trax[i].track = rd32(e);
        p->trax[i].mode = e[4];
    }
    StAudio *a = &p->audio;
    const uint8_t *r = in + OFF_AUDIO;
    a->master = rdf(r + 0); a->speech_movie = rdf(r + 4); a->menu_music = rdf(r + 8);
    a->game_music = rdf(r + 12); a->sfx = rdf(r + 16); a->engine = rdf(r + 20); a->tire = rdf(r + 24);
    a->unk28 = rdf(r + 28); a->unk32 = rdf(r + 32); a->unk36 = rdf(r + 36);
    a->mode = rd32(r + 40); a->unk44 = rd32(r + 44); a->trax_order = rd32(r + 48);
    a->unk52 = rd32(r + 52); a->unk56 = rd32(r + 56);
    StGameplay *g = &p->gameplay;
    r = in + OFF_GAMEPLAY;
    g->rearview_mirror = r[0]; g->unk1 = r[1]; g->speedo_units = r[2];
    g->race_map_mode = r[3]; g->explore_map_mode = r[4];
    g->jump_camera = rdf(r + 8); g->unk12 = rdf(r + 12);
    for (int i = 0; i < 2; i++) {
        StPlayer *q = &p->player[i];
        r = in + OFF_PLAYER + i * PLAYER_SIZE;
        q->gauges = r[0]; q->race_position = r[1]; q->lap_info = r[2]; q->score = r[3];
        q->feedback = r[4]; q->leaderboard = r[5]; q->ask_transmission = r[6];
        q->controller_config = rd32(r + 8); q->unk12 = r[12];
        q->camera = rd32(r + 16);
        q->split_time = r[20]; q->transmission = r[21]; q->stability_control = r[22];
        q->unk23 = r[23]; q->unk24 = r[24]; q->unk25 = r[25];
        q->wheel_range = rdf(r + 28); q->road_feedback = rdf(r + 32); q->force_strength = rdf(r + 36);
    }
    r = in + OFF_UNKBLK;
    p->unkblk_f = rdf(r); p->unkblk_a = rd32(r + 4); p->unkblk_b = rd32(r + 8); p->unkblk_c = r[12];
    p->career_difficulty = rd32(in + OFF_DIFFICULTY);
    p->autosave = in[OFF_AUTOSAVE];
    p->save_prompt = in[OFF_PROMPT];
    memcpy(p->name, in + OFF_NAME, 32);
    p->name[32] = 0;
    r = in + OFF_ONLINE;
    p->online.show_player_names = r[8]; p->online.bad_connection_warning = r[9];
    p->online.unk10 = r[10];
    for (int i = 0; i < 5; i++) {
        p->online.prefs_a[i] = rd32(r + 11 + 4 * i);
        p->online.prefs_b[i] = rd32(r + 31 + 4 * i);
    }
    for (int i = 0; i < ST_INPUT_SLOTS; i++) {
        StInputDevice *d = &p->input[i];
        r = in + OFF_INPUT + i * ST_INPUT_RECORD_SIZE;
        memcpy(d->guid, r, 16);
        for (int k = 0; k < 4; k++) {
            d->axis[k] = (int32_t)rd32(r + 16 + 4 * k);
            d->dir[k] = (int32_t)rd32(r + 32 + 4 * k);
            d->assigned[k] = r[48 + k];
        }
        d->combined_mode = r[52];
        for (int k = 0; k < 32; k++) d->binding[k] = (int32_t)rd32(r + 53 + 4 * k);
    }
}

int st_profile_parse(const uint8_t *file, size_t len, StProfile *out) {
    StContainerView v;
    int rc = st_container_parse(file, len, &v);
    if (rc != ST_OK) return rc;
    if (!out) return ST_E_ARG;
    /* 3.5 step 6; block B must also actually hold the payload we are going to read. */
    if (v.size_a < 8 || rd32(v.a) != ST_PROFILE_VERSION || rd32(v.a + 4) != ST_PAYLOAD_SIZE ||
        v.size_b != ST_PAYLOAD_SIZE)
        return ST_E_INCOMPATIBLE;
    st_profile_unpack_payload(out, v.b);
    return ST_OK;
}

size_t st_profile_serialize(const StProfile *p, uint8_t *out, size_t cap) {
    if (!p || !out || cap < ST_PROFILE_FILE_SIZE) return 0;
    uint8_t a[8];
    uint8_t *payload = (uint8_t *)malloc(ST_PAYLOAD_SIZE);
    if (!payload) return 0;
    wr32(a, ST_PROFILE_VERSION);
    wr32(a + 4, ST_PAYLOAD_SIZE);
    st_profile_pack_payload(p, payload);
    size_t n = st_container_build(out, cap, a, 8, payload, ST_PAYLOAD_SIZE);
    free(payload);
    return n;
}

void st_profile_snapshot(const StProfile *p, uint8_t snapshot[ST_PAYLOAD_SIZE]) {
    st_profile_pack_payload(p, snapshot);
}

int st_profile_modified(const StProfile *p, const uint8_t snapshot[ST_PAYLOAD_SIZE]) {
    uint8_t *cur = (uint8_t *)malloc(ST_PAYLOAD_SIZE);
    if (!cur) return 1;
    st_profile_pack_payload(p, cur);
    int diff = memcmp(cur, snapshot, ST_PAYLOAD_SIZE) != 0;
    free(cur);
    return diff;
}

/* ---- option helpers ---- */
float st_jump_camera_toggle(float v) { return v == 0.0f ? 127.0f : 0.0f; }

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float st_road_feedback_from_slider(float s) { return 20.0f - 18.0f * clampf(s, 0.0f, 1.0f); }
float st_road_feedback_to_slider(float v) { return 1.0f - (clampf(v, 2.0f, 20.0f) - 2.0f) / 18.0f; }

uint32_t st_career_difficulty_step(uint32_t v, int dir) {
    if (v > 2) return v;                          /* out of range: leave alone */
    return dir < 0 ? (v == 0 ? 2 : v - 1) : (v == 2 ? 0 : v + 1);
}

uint8_t st_trax_mode_step(uint8_t mode, int dir) {
    if (mode > 3) return mode;
    return (uint8_t)(dir < 0 ? (mode + 3) & 3 : (mode + 1) & 3);
}

void st_trax_masks(const StProfile *p, uint32_t *fe, uint32_t *ig) {
    uint32_t f = 0, g = 0;
    for (int i = 0; i < ST_TRAX_COUNT; i++) {
        uint32_t t = p->trax[i].track;
        if (t >= ST_TRAX_COUNT) continue;
        if (p->trax[i].mode & 1) f |= 1u << t;
        if (p->trax[i].mode & 2) g |= 1u << t;
    }
    if (fe) *fe = f;
    if (ig) *ig = g;
}

int st_input_find(const StProfile *p, const uint8_t guid[16]) {
    static const uint8_t zero[16];
    if (!memcmp(guid, zero, 16)) return -1;
    for (int i = 0; i < ST_INPUT_SLOTS; i++)
        if (!memcmp(p->input[i].guid, guid, 16)) return i;
    return -1;
}

int st_save_mode(const StProfile *p) {
    if (p->autosave) return ST_SAVE_AUTOSAVE;
    return p->save_prompt ? ST_SAVE_PROMPT : ST_SAVE_OFF;
}

void st_save_set_mode(StProfile *p, int mode) {
    p->autosave = (uint8_t)(mode == ST_SAVE_AUTOSAVE);
    p->save_prompt = (uint8_t)(mode == ST_SAVE_AUTOSAVE || mode == ST_SAVE_PROMPT);
}

int st_save_cycle(StProfile *p, int dir, int autosave_ok) {
    static const int right[3] = { ST_SAVE_OFF, ST_SAVE_AUTOSAVE, ST_SAVE_PROMPT };
    static const int left[3] = { ST_SAVE_OFF, ST_SAVE_PROMPT, ST_SAVE_AUTOSAVE };
    const int *seq = dir < 0 ? left : right;
    int cur = st_save_mode(p), i = 0;
    while (seq[i] != cur) i++;
    do { i = (i + 1) % 3; } while (seq[i] == ST_SAVE_AUTOSAVE && !autosave_ok);
    st_save_set_mode(p, seq[i]);
    return seq[i];
}

/* ================================================================= file helpers */
static int dir_iter(const char *path, int (*cb)(const char *name, void *ctx), void *ctx) {
    /* Calls cb for every entry except "." and ".."; stops if cb returns nonzero.
     * Returns -1 if the directory cannot be opened, else 0. */
#ifdef _WIN32
    char pat[1024];
    WIN32_FIND_DATAA fd;
    if (snprintf(pat, sizeof pat, "%s\\*", path) >= (int)sizeof pat) return -1;
    HANDLE h = FindFirstFileA(pat, &fd);
    if (h == INVALID_HANDLE_VALUE) return -1;
    do {
        if (!strcmp(fd.cFileName, ".") || !strcmp(fd.cFileName, "..")) continue;
        if (cb(fd.cFileName, ctx)) break;
    } while (FindNextFileA(h, &fd));
    FindClose(h);
    return 0;
#else
    DIR *d = opendir(path);
    if (!d) return -1;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
        if (cb(e->d_name, ctx)) break;
    }
    closedir(d);
    return 0;
#endif
}

static int make_dir(const char *path) {      /* ST_OK also if it already exists */
#ifdef _WIN32
    if (_mkdir(path) == 0 || errno == EEXIST) return ST_OK;
#else
    if (mkdir(path, 0777) == 0 || errno == EEXIST) return ST_OK;
#endif
    return ST_E_IO;
}

static int make_dirs(const char *path) {     /* mkdir -p */
    char tmp[1024];
    size_t n = strlen(path);
    if (n == 0 || n >= sizeof tmp) return ST_E_ARG;
    memcpy(tmp, path, n + 1);
    for (size_t i = 1; i < n; i++) {
        if (tmp[i] == '/' || tmp[i] == '\\') {
            char c = tmp[i];
            tmp[i] = 0;
            if (i > 0 && tmp[i - 1] != ':' && tmp[i - 1] != '/' && tmp[i - 1] != '\\')
                make_dir(tmp);                /* intermediate failures surface at the leaf */
            tmp[i] = c;
        }
    }
    return make_dir(tmp);
}

static int count_cb(const char *name, void *ctx) { (void)name; (*(int *)ctx)++; return 1; }
static int dir_nonempty(const char *path) {
    int n = 0;
    return dir_iter(path, count_cb, &n) == 0 && n > 0;
}

/* Profile components are untrusted directory entries. Do not follow links or
 * open special files; POSIX operations stay relative to the opened directory. */
static FILE *profile_open(const char *dir, const char *name, int write) {
#ifdef _WIN32
    /* shortcut: path-based directory checks assume no concurrent replacement;
     * anchor a directory handle if multiple writers must be supported. */
    DWORD attrs = GetFileAttributesA(dir);
    if (attrs == INVALID_FILE_ATTRIBUTES || !(attrs & FILE_ATTRIBUTE_DIRECTORY) ||
        (attrs & FILE_ATTRIBUTE_REPARSE_POINT)) return NULL;
    char path[1024];
    if (snprintf(path, sizeof path, "%s/%s", dir, name) >= (int)sizeof path) return NULL;
    HANDLE h = CreateFileA(path, write ? GENERIC_WRITE : GENERIC_READ, FILE_SHARE_READ,
                          NULL, write ? OPEN_ALWAYS : OPEN_EXISTING,
                          FILE_FLAG_OPEN_REPARSE_POINT, NULL);
    if (h == INVALID_HANDLE_VALUE) return NULL;
    BY_HANDLE_FILE_INFORMATION info;
    if (!GetFileInformationByHandle(h, &info) ||
        (info.dwFileAttributes & (FILE_ATTRIBUTE_REPARSE_POINT | FILE_ATTRIBUTE_DIRECTORY)) ||
        (write && info.nNumberOfLinks != 1)) { CloseHandle(h); return NULL; }
    int fd = _open_osfhandle((intptr_t)h, _O_BINARY | (write ? _O_WRONLY : _O_RDONLY));
    if (fd < 0) { CloseHandle(h); return NULL; }
    if (write && _chsize(fd, 0) != 0) { _close(fd); return NULL; }
    FILE *f = _fdopen(fd, write ? "wb" : "rb");
    if (!f) _close(fd);
#else
    int d = open(dir, O_RDONLY | O_DIRECTORY | O_NOFOLLOW);
    if (d < 0) return NULL;
    int fd = openat(d, name, O_NOFOLLOW | O_NONBLOCK |
                   (write ? O_WRONLY | O_CREAT : O_RDONLY), 0666);
    close(d);
    if (fd < 0) return NULL;
    struct stat info;
    if (fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) || (write && info.st_nlink != 1) ||
        (write && ftruncate(fd, 0) != 0)) { close(fd); return NULL; }
    FILE *f = fdopen(fd, write ? "wb" : "rb");
    if (!f) close(fd);
#endif
    return f;
}

/* ================================================================ 3.1 profile files */
int st_profile_name_valid(const char *name) {
    if (!name) return 0;
    size_t n = 0;
    while (n <= ST_NAME_MAX && name[n]) n++;
    if (n == 0 || n > ST_NAME_MAX) return 0;
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)name[i];
        if (c < 0x20 || c == 0x7F || strchr("/\\:*?\"<>|", c)) return 0;
    }
    if (name[n - 1] == ' ' || name[n - 1] == '.') return 0;   /* invalid on Windows */
    return 1;
}

static int dir_path(const char *root, const char *name, char *out, size_t cap) {
    if (!root || !st_profile_name_valid(name)) return ST_E_ARG;
    if (snprintf(out, cap, "%s/%s", root, name) >= (int)cap) return ST_E_ARG;
    return ST_OK;
}

int st_profile_path(const char *root, const char *name, char *out, size_t cap) {
    if (!root || !st_profile_name_valid(name)) return ST_E_ARG;
    if (snprintf(out, cap, "%s/%s/%s", root, name, name) >= (int)cap) return ST_E_ARG;
    return ST_OK;
}

int st_profile_save(const char *root, const StProfile *p) {
    if (!p) return ST_E_ARG;
    char dir[1024], path[1024];
    int rc = dir_path(root, p->name, dir, sizeof dir);
    if (rc == ST_OK) rc = st_profile_path(root, p->name, path, sizeof path);
    if (rc != ST_OK) return rc;
    uint8_t *buf = (uint8_t *)malloc(ST_PROFILE_FILE_SIZE);
    if (!buf) return ST_E_NOMEM;
    size_t n = st_profile_serialize(p, buf, ST_PROFILE_FILE_SIZE);
    if (n != ST_PROFILE_FILE_SIZE) { free(buf); return ST_E_IO; }
    rc = make_dirs(root);
    if (rc == ST_OK) rc = make_dir(dir);
    if (rc == ST_OK) {
        FILE *f = profile_open(dir, p->name, 1); /* create/truncate; not atomic (3.3) */
        if (!f) rc = ST_E_IO;
        else {
            if (fwrite(buf, 1, n, f) != n) rc = ST_E_IO;
            if (fclose(f) != 0) rc = ST_E_IO;
        }
    }
    free(buf);
    return rc;
}

/* Validate a block while retaining only the bytes its caller understands. */
static int profile_read_block(FILE *f, uint32_t size, uint8_t *out, size_t keep, uint32_t *crc) {
    uint8_t chunk[4096];
    uint32_t done = 0, r = 0;
    crc_init();
    while (done < size) {
        size_t n = size - done;
        if (n > sizeof chunk) n = sizeof chunk;
        if (fread(chunk, 1, n, f) != n) return ST_E_IO;
        if (done < keep) {
            size_t copy = keep - done;
            if (copy > n) copy = n;
            memcpy(out + done, chunk, copy);
        }
        for (size_t i = 0; i < n; i++) {
            if (done + i < 4) {
                r = (r << 8) | chunk[i];
                if (done + i == 3) r = ~r;
            } else r = ((r << 8) | chunk[i]) ^ g_crc_tab[r >> 24];
        }
        done += (uint32_t)n;
    }
    *crc = size < 4 ? 0 : ~r;
    return ST_OK;
}

int st_profile_load(const char *root, const char *name, StProfile *out) {
    char dir[1024];
    int rc = dir_path(root, name, dir, sizeof dir);
    if (rc != ST_OK) return rc;
    FILE *f = profile_open(dir, name, 0);
    if (!f) return ST_E_NOTFOUND;
    long len = -1;
    if (fseek(f, 0, SEEK_END) == 0) len = ftell(f);
    if (len < ST_CONTAINER_HEADER || len > 0x7FFFFFFFL || fseek(f, 0, SEEK_SET) != 0) {
        fclose(f); return ST_E_IO;
    }
    uint8_t header[ST_CONTAINER_HEADER], a[8];
    if (fread(header, 1, sizeof header, f) != sizeof header) { fclose(f); return ST_E_IO; }
    uint32_t sa = rd32(header + 8), sb = rd32(header + 12);
    if (rd32(header + 4) != (uint32_t)len || memcmp(header, "20CM", 4) ||
        rd32(header + 24) != st_checksum(header, 24)) { fclose(f); return ST_E_CORRUPT; }
    if ((uint64_t)sa + sb > (uint64_t)len - ST_CONTAINER_HEADER) { fclose(f); return ST_E_IO; }
    uint8_t *payload = sb == ST_PAYLOAD_SIZE ? (uint8_t *)malloc(ST_PAYLOAD_SIZE) : NULL;
    if (sb == ST_PAYLOAD_SIZE && !payload) { fclose(f); return ST_E_NOMEM; }
    uint32_t ca = 0, cb = 0;
    rc = profile_read_block(f, sa, a, sizeof a, &ca);
    if (rc == ST_OK) rc = profile_read_block(f, sb, payload, payload ? ST_PAYLOAD_SIZE : 0, &cb);
    fclose(f);
    if (rc == ST_OK && (ca != rd32(header + 16) || cb != rd32(header + 20))) rc = ST_E_CORRUPT;
    if (rc == ST_OK && !out) rc = ST_E_ARG;
    if (rc == ST_OK && (sa < 8 || rd32(a) != ST_PROFILE_VERSION ||
        rd32(a + 4) != ST_PAYLOAD_SIZE || sb != ST_PAYLOAD_SIZE)) rc = ST_E_INCOMPATIBLE;
    if (rc == ST_OK) st_profile_unpack_payload(out, payload);
    free(payload);
    return rc;
}

int st_profile_exists(const char *root, const char *name) {
    char dir[1024];
    return dir_path(root, name, dir, sizeof dir) == ST_OK && dir_nonempty(dir);
}

/* Distinguishes a confirmed absence from a failure to look: only a missing directory, or one that enumerates
 * cleanly to nothing, is NOTFOUND. Open, permission and mid-enumeration errors are ST_E_IO. */
int st_profile_presence(const char *root, const char *name) {
    char dir[1024];
    int rc = dir_path(root, name, dir, sizeof dir);
    if (rc != ST_OK) return rc;
    int found = 0;
#ifdef _WIN32
    char pat[1024];
    WIN32_FIND_DATAA fd;
    if (snprintf(pat, sizeof pat, "%s\\*", dir) >= (int)sizeof pat) return ST_E_IO;
    HANDLE h = FindFirstFileA(pat, &fd);
    if (h == INVALID_HANDLE_VALUE) {
        DWORD e = GetLastError();
        return e == ERROR_FILE_NOT_FOUND || e == ERROR_PATH_NOT_FOUND ? ST_E_NOTFOUND : ST_E_IO;
    }
    do {
        if (strcmp(fd.cFileName, ".") && strcmp(fd.cFileName, "..")) found = 1;
    } while (!found && FindNextFileA(h, &fd));
    DWORD e = GetLastError();
    FindClose(h);
    if (!found && e != ERROR_NO_MORE_FILES) return ST_E_IO;
#else
    DIR *d = opendir(dir);
    if (!d) return errno == ENOENT ? ST_E_NOTFOUND : ST_E_IO;
    struct dirent *e;
    errno = 0;
    while (!found && (e = readdir(d)) != NULL)
        if (strcmp(e->d_name, ".") && strcmp(e->d_name, "..")) found = 1;
    int err = !found && errno != 0;
    closedir(d);
    if (err) return ST_E_IO;
#endif
    return found ? ST_OK : ST_E_NOTFOUND;
}

int st_profiles_exist(const char *root) { return root && dir_nonempty(root); }

typedef struct { const char *dir; int rc; } DelCtx;
static int del_cb(const char *name, void *ctx) {
    DelCtx *d = (DelCtx *)ctx;
    char p[1400];
    if (snprintf(p, sizeof p, "%s/%s", d->dir, name) >= (int)sizeof p || remove(p) != 0)
        d->rc = ST_E_IO;
    return 0;
}

int st_profile_delete(const char *root, const char *name) {
    char dir[1024];
    int rc = dir_path(root, name, dir, sizeof dir);
    if (rc != ST_OK) return rc;
#ifdef _WIN32
    /* shortcut: path-based directory checks assume no concurrent replacement;
     * anchor a directory handle if multiple writers must be supported. */
    DWORD attrs = GetFileAttributesA(dir);
    if (attrs == INVALID_FILE_ATTRIBUTES) return ST_E_NOTFOUND;
    if (!(attrs & FILE_ATTRIBUTE_DIRECTORY) || (attrs & FILE_ATTRIBUTE_REPARSE_POINT)) return ST_E_IO;
    DelCtx d = { dir, ST_OK };
    if (dir_iter(dir, del_cb, &d) != 0) return ST_E_NOTFOUND;
    if (d.rc != ST_OK) return d.rc;
    return _rmdir(dir) == 0 ? ST_OK : ST_E_IO;
#else
    int fd = open(dir, O_RDONLY | O_DIRECTORY | O_NOFOLLOW);
    if (fd < 0) return errno == ENOENT ? ST_E_NOTFOUND : ST_E_IO;
    DIR *d = fdopendir(fd);
    if (!d) { close(fd); return ST_E_IO; }
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
        struct stat info;
        if (fstatat(fd, e->d_name, &info, AT_SYMLINK_NOFOLLOW) != 0 ||
            unlinkat(fd, e->d_name, S_ISDIR(info.st_mode) ? AT_REMOVEDIR : 0) != 0) rc = ST_E_IO;
    }
    closedir(d);
    if (rc != ST_OK) return rc;
    return rmdir(dir) == 0 ? ST_OK : ST_E_IO;
#endif
}

/* ---- default locations ---- */
static int join(char *out, size_t cap, const char *base, const char *tail) {
    return snprintf(out, cap, "%s%s", base, tail) < (int)cap ? ST_OK : ST_E_ARG;
}

static int user_base(const char *kind, char *out, size_t cap) {
    /* kind "data" or "config". */
    char tmp[900];
    (void)kind; (void)tmp;
#ifdef _WIN32
    const char *e = getenv("LOCALAPPDATA");
    if (e && *e) return join(out, cap, e, "");
    e = getenv("USERPROFILE");
    if (!e || !*e) return ST_E_NOTFOUND;
    if (snprintf(tmp, sizeof tmp, "%s\\AppData\\Local", e) >= (int)sizeof tmp) return ST_E_ARG;
    return join(out, cap, tmp, "");
#else
    const char *home = getenv("HOME");
    int cfg = !strcmp(kind, "config");
    (void)cfg;
#ifdef __APPLE__
    if (!home || !*home) return ST_E_NOTFOUND;
    if (snprintf(tmp, sizeof tmp, "%s/Library/Application Support", home) >= (int)sizeof tmp) return ST_E_ARG;
    return join(out, cap, tmp, "");
#else
    const char *x = getenv(cfg ? "XDG_CONFIG_HOME" : "XDG_DATA_HOME");
    if (x && *x == '/') return join(out, cap, x, "");
    if (!home || !*home) return ST_E_NOTFOUND;
    if (snprintf(tmp, sizeof tmp, cfg ? "%s/.config" : "%s/.local/share", home) >= (int)sizeof tmp) return ST_E_ARG;
    return join(out, cap, tmp, "");
#endif
#endif
}

int st_default_profile_root(char *out, size_t cap) {
    char base[900];
    int rc = user_base("data", base, sizeof base);
    if (rc != ST_OK) return rc;
#ifdef _WIN32
    if (snprintf(out, cap, "%s\\NFS Underground 2", base) >= (int)cap) return ST_E_ARG;
#else
    if (snprintf(out, cap, "%s/OpenUG2/NFS Underground 2", base) >= (int)cap) return ST_E_ARG;
#endif
    return ST_OK;
}

int st_default_config_path(char *out, size_t cap) {
    char base[900];
    int rc = user_base("config", base, sizeof base);
    if (rc != ST_OK) return rc;
    if (snprintf(out, cap, "%s%cOpenUG2%csettings.ini", base, ST_SEP, ST_SEP) >= (int)cap)
        return ST_E_ARG;
    return ST_OK;
}

/* ============================================================ 2 value store layer */
int st_reg_exists(StRegStore *s, const char *n) { return s && s->ops && s->ops->exists(s->impl, n); }
int st_reg_get_dword(StRegStore *s, const char *n, uint32_t *o) {
    return s && s->ops && s->ops->get_dword(s->impl, n, o);
}
int st_reg_get_string(StRegStore *s, const char *n, char *b, size_t c) {
    return s && s->ops && s->ops->get_string(s->impl, n, b, c);
}
int st_reg_set_dword(StRegStore *s, const char *n, uint32_t v) {
    return (s && s->ops) ? s->ops->set_dword(s->impl, n, v) : ST_E_ARG;
}
int st_reg_set_string(StRegStore *s, const char *n, const char *v) {
    return (s && s->ops) ? s->ops->set_string(s->impl, n, v) : ST_E_ARG;
}
void st_reg_close(StRegStore *s) {
    if (s && s->ops && s->ops->close) s->ops->close(s->impl);
    if (s) { s->ops = NULL; s->impl = NULL; }
}

/* ---- in-memory store (also the cache of the INI store) ---- */
typedef struct MemEntry { char *name; int is_dword; uint32_t dword; char *str; int dirty; } MemEntry;  /* dirty: written through this store */
typedef struct MemStore {
    MemEntry *e; size_t n, cap;
    char *path;               /* non-NULL: INI store, flushed after every write */
} MemStore;

static char *dup_str(const char *s) {
    size_t n = strlen(s) + 1;
    char *d = (char *)malloc(n);
    if (d) memcpy(d, s, n);
    return d;
}
static int ieq(const char *a, const char *b) {
    for (; *a && *b; a++, b++)
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
    return *a == *b;
}
static MemEntry *mem_find(MemStore *m, const char *name) {
    for (size_t i = 0; i < m->n; i++) if (ieq(m->e[i].name, name)) return &m->e[i];  /* registry names are case-insensitive */
    return NULL;
}
static MemEntry *mem_slot(MemStore *m, const char *name) {
    MemEntry *e = mem_find(m, name);
    if (e) return e;
    if (m->n == m->cap) {
        if (m->cap > SIZE_MAX / 2) return NULL;
        size_t nc = m->cap ? m->cap * 2 : 16;
        if (nc > SIZE_MAX / sizeof *m->e) return NULL;
        MemEntry *ne = (MemEntry *)realloc(m->e, nc * sizeof *ne);
        if (!ne) return NULL;
        m->e = ne; m->cap = nc;
    }
    e = &m->e[m->n];
    memset(e, 0, sizeof *e);
    e->name = dup_str(name);
    if (!e->name) return NULL;
    m->n++;
    return e;
}

static int ini_flush(MemStore *m);
static int ini_name_valid(const char *name) {
    size_t n = strlen(name);
    if (!n || strchr(" #;[\t", name[0]) || name[n - 1] == ' ' || name[n - 1] == '\t') return 0;
    for (size_t i = 0; i < n; i++)
        if ((unsigned char)name[i] < 32 || name[i] == 127 || name[i] == '=') return 0;
    return 1;
}

static int mem_exists(void *impl, const char *name) { return mem_find((MemStore *)impl, name) != NULL; }
static int mem_get_dword(void *impl, const char *name, uint32_t *out) {
    MemEntry *e = mem_find((MemStore *)impl, name);
    if (!e || !e->is_dword) return 0;
    *out = e->dword;
    return 1;
}
static int mem_get_string(void *impl, const char *name, char *buf, size_t cap) {
    MemEntry *e = mem_find((MemStore *)impl, name);
    if (!e || e->is_dword || !e->str || strlen(e->str) >= cap) return 0;
    strcpy(buf, e->str);
    return 1;
}
static void ini_refresh(MemStore *m);
static int mem_set_dword(void *impl, const char *name, uint32_t v) {
    MemStore *m = (MemStore *)impl;
    if (m->path && !ini_name_valid(name)) return ST_E_ARG;
    if (m->path) ini_refresh(m);
    MemEntry *e = mem_slot(m, name);
    if (!e) return ST_E_NOMEM;
    free(e->str); e->str = NULL;
    e->is_dword = 1; e->dword = v;
    e->dirty = 1;
    return m->path ? ini_flush(m) : ST_OK;
}
static int mem_set_string(void *impl, const char *name, const char *v) {
    MemStore *m = (MemStore *)impl;
    if (m->path && !ini_name_valid(name)) return ST_E_ARG;
    char *copy = dup_str(v);
    if (!copy) return ST_E_NOMEM;
    if (m->path) ini_refresh(m);
    MemEntry *e = mem_slot(m, name);
    if (!e) { free(copy); return ST_E_NOMEM; }
    free(e->str);
    e->is_dword = 0; e->str = copy;
    e->dirty = 1;
    return m->path ? ini_flush(m) : ST_OK;
}
static void mem_close(void *impl) {
    MemStore *m = (MemStore *)impl;
    if (!m) return;
    for (size_t i = 0; i < m->n; i++) { free(m->e[i].name); free(m->e[i].str); }
    free(m->e); free(m->path); free(m);
}
static const StRegOps k_mem_ops = { mem_exists, mem_get_dword, mem_get_string,
                                    mem_set_dword, mem_set_string, mem_close };

int st_reg_open_memory(StRegStore *s) {
    MemStore *m = (MemStore *)calloc(1, sizeof *m);
    if (!m) return ST_E_NOMEM;
    s->ops = &k_mem_ops; s->impl = m;
    return ST_OK;
}

/* ---- INI file: one section named after the registry key; values in .reg notation ---- */
#define INI_SECTION "Software\\EA Games\\Need for Speed Underground 2"

static int ini_flush(MemStore *m) {
    {   /* create the parent directory on first write; merely opening never writes */
        char dir[1024];
        size_t n = strlen(m->path);
        if (n < sizeof dir) {
            memcpy(dir, m->path, n + 1);
            char *sl = strrchr(dir, '/');
#ifdef _WIN32
            char *bs = strrchr(dir, '\\');
            if (bs && (!sl || bs > sl)) sl = bs;
#endif
            if (sl && sl != dir) { *sl = 0; make_dirs(dir); }
        }
    }
    size_t plen = strlen(m->path);
    if (plen > SIZE_MAX - 48) return ST_E_NOMEM;
    char *tmp = (char *)malloc(plen + 48);
    if (!tmp) return ST_E_NOMEM;
    static unsigned serial;
    int fd = -1;
    for (int tries = 0; tries < 32; tries++) {
#ifdef _WIN32
        snprintf(tmp, plen + 48, "%s.tmp.%lu.%u", m->path, (unsigned long)GetCurrentProcessId(), serial++);
        fd = _open(tmp, _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY, _S_IREAD | _S_IWRITE);
#else
        snprintf(tmp, plen + 48, "%s.tmp.%lu.%u", m->path, (unsigned long)getpid(), serial++);
        fd = open(tmp, O_WRONLY | O_CREAT | O_EXCL, 0600);
#endif
        if (fd >= 0 || errno != EEXIST) break;
    }
    if (fd < 0) { free(tmp); return ST_E_IO; }
#ifdef _WIN32
    FILE *f = _fdopen(fd, "wb");
    if (!f) _close(fd);
#else
    FILE *f = fdopen(fd, "wb");
    if (!f) close(fd);
#endif
    if (!f) { remove(tmp); free(tmp); return ST_E_IO; }
    int ok = fprintf(f, "[%s]\n", INI_SECTION) > 0;
    for (size_t i = 0; ok && i < m->n; i++) {
        const MemEntry *e = &m->e[i];
        if (e->is_dword) ok = fprintf(f, "%s=dword:%08x\n", e->name, (unsigned)e->dword) > 0;
        else {
            ok = fprintf(f, "%s=\"", e->name) > 0;
            for (const char *c = e->str; ok && *c; c++) {
                const char *escape = *c == '\n' ? "\\n" : *c == '\r' ? "\\r" : *c == '\t' ? "\\t" : NULL;
                if (escape) ok = fputs(escape, f) >= 0;
                else {
                    if (*c == '\\' || *c == '"') ok = fputc('\\', f) != EOF;
                    if (ok) ok = fputc((unsigned char)*c, f) != EOF;
                }
            }
            if (ok) ok = fputs("\"\n", f) >= 0;
        }
    }
    if (fclose(f) != 0) ok = 0;
    if (ok) {
#ifdef _WIN32
        ok = MoveFileExA(tmp, m->path, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
        ok = rename(tmp, m->path) == 0;
#endif
    }
    if (!ok) remove(tmp);
    free(tmp);
    return ok ? ST_OK : ST_E_IO;
}

static char *trim(char *s) {
    while (*s == ' ' || *s == '\t') s++;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n')) *--e = 0;
    return s;
}

static int ini_parse_line(MemStore *m, char *line, int *in_section) {
    char *l = trim(line);
    if (!*l || *l == ';' || *l == '#') return ST_OK;
    if (*l == '[') {
        char *r = strchr(l, ']');
        if (r) { *r = 0; *in_section = ieq(l + 1, INI_SECTION); }
        return ST_OK;
    }
    if (!*in_section) return ST_OK;
    char *eq = strchr(l, '=');
    if (!eq) return ST_OK;
    *eq = 0;
    char *name = trim(l), *val = trim(eq + 1);
    if (!ini_name_valid(name)) return ST_E_CORRUPT;
    if (!strncmp(val, "dword:", 6)) {
        char *end;
        errno = 0;
        unsigned long u = strtoul(val + 6, &end, 16);
        if (errno || u > UINT32_MAX || val[6] == '-' || end == val + 6 || *end) return ST_E_CORRUPT;
        return mem_set_dword(m, name, (uint32_t)u);
    } else if (*val == '"') {
        char *w = val, *r = val + 1;
        while (*r && *r != '"') {
            if (*r == '\\') {
                if (!r[1]) return ST_E_CORRUPT;
                r++;
                *w++ = *r == 'n' ? '\n' : *r == 'r' ? '\r' : *r == 't' ? '\t' : *r;
                r++;
            } else *w++ = *r++;
        }
        if (*r != '"' || *trim(r + 1)) return ST_E_CORRUPT;
        *w = 0;
        return mem_set_string(m, name, val);
    }
    return ST_OK;
}

static int ini_read(FILE *f, MemStore *m) {
    size_t n = 0, cap = 256;
    char *line = (char *)malloc(cap);
    if (!line) return ST_E_NOMEM;
    int section = 0, rc = ST_OK, c;
    while ((c = fgetc(f)) != EOF) {
        if (!c) { rc = ST_E_CORRUPT; break; }
        if (n == cap - 1) {
            if (cap > SIZE_MAX / 2) { rc = ST_E_NOMEM; break; }
            size_t nc = cap * 2;
            char *next = (char *)realloc(line, nc);
            if (!next) { rc = ST_E_NOMEM; break; }
            line = next; cap = nc;
        }
        if (c == '\n') {
            line[n] = 0; rc = ini_parse_line(m, line, &section); n = 0;
            if (rc != ST_OK) break;
        } else line[n++] = (char)c;
    }
    if (rc == ST_OK && ferror(f)) rc = ST_E_IO;
    if (rc == ST_OK && n) { line[n] = 0; rc = ini_parse_line(m, line, &section); }
    free(line);
    return rc;
}

/* Another store on the same file may have written since this one was opened (the graphics store lives as long as
 * the pause menu, the controls store is opened per save). Before a write, adopt every value of the file that
 * this store did not write itself, so the whole-file rewrite cannot drop the other store's values. A file that
 * is missing or unreadable leaves the cache as it is. */
static void ini_refresh(MemStore *m) {
    FILE *f = fopen(m->path, "rb");
    if (!f) return;
    MemStore *disk = (MemStore *)calloc(1, sizeof *disk);
    if (!disk) { fclose(f); return; }
    int rc = ini_read(f, disk);
    fclose(f);
    if (rc == ST_OK) {
        for (size_t i = 0; i < disk->n; i++) {
            MemEntry *d = &disk->e[i], *e = mem_find(m, d->name);
            if (e && e->dirty) continue;
            if (!e) { e = mem_slot(m, d->name); if (!e) break; }
            char *copy = d->is_dword ? NULL : (d->str ? dup_str(d->str) : NULL);
            if (!d->is_dword && !copy) continue;
            free(e->str);
            e->is_dword = d->is_dword; e->dword = d->dword; e->str = copy;
        }
    }
    mem_close(disk);
}

int st_reg_open_ini(StRegStore *s, const char *path) {
    if (!path || !*path) return ST_E_ARG;
    MemStore *m = (MemStore *)calloc(1, sizeof *m);
    if (!m) return ST_E_NOMEM;
    m->path = dup_str(path);
    if (!m->path) { mem_close(m); return ST_E_NOMEM; }
    FILE *f = fopen(path, "rb");
    if (f) {
        char *path_keep = m->path;
        m->path = NULL;
        int rc = ini_read(f, m);
        m->path = path_keep;
        for (size_t i = 0; i < m->n; i++) m->e[i].dirty = 0;     /* loaded, not written */
        if (fclose(f) != 0 && rc == ST_OK) rc = ST_E_IO;
        if (rc != ST_OK) { mem_close(m); return rc; }
    } else if (errno != ENOENT) { mem_close(m); return ST_E_IO; }
    s->ops = &k_mem_ops; s->impl = m;
    return ST_OK;
}

/* ---- Windows registry store ---- */
#ifdef _WIN32
#define REG_KEY_PATH "Software\\EA Games\\Need for Speed Underground 2"
/* The original is a 32-bit process: always address the 32-bit view (WOW6432Node). */
static HKEY reg_open(int write) {
    HKEY k = NULL;
    REGSAM sam = (write ? KEY_SET_VALUE | KEY_QUERY_VALUE : KEY_QUERY_VALUE) | KEY_WOW64_32KEY;
    if (write) {
        DWORD disp;
        if (RegCreateKeyExA(HKEY_LOCAL_MACHINE, REG_KEY_PATH, 0, NULL, 0, sam, NULL, &k, &disp) != ERROR_SUCCESS)
            return NULL;
    } else if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, REG_KEY_PATH, 0, sam, &k) != ERROR_SUCCESS) {
        return NULL;
    }
    return k;
}
static int win_exists(void *impl, const char *name) {
    (void)impl;
    HKEY k = reg_open(0);
    if (!k) return 0;
    LONG r = RegQueryValueExA(k, name, NULL, NULL, NULL, NULL);
    RegCloseKey(k);
    return r == ERROR_SUCCESS;
}
static int win_get_dword(void *impl, const char *name, uint32_t *out) {
    (void)impl;
    HKEY k = reg_open(0);
    if (!k) return 0;
    DWORD v = 0, sz = 4;                       /* type is not checked (2.3) */
    LONG r = RegQueryValueExA(k, name, NULL, NULL, (LPBYTE)&v, &sz);
    RegCloseKey(k);
    if (r != ERROR_SUCCESS) return 0;
    *out = v;
    return 1;
}
static int win_get_string(void *impl, const char *name, char *buf, size_t cap) {
    (void)impl;
    HKEY k = reg_open(0);
    if (!k || cap == 0) { if (k) RegCloseKey(k); return 0; }
    DWORD sz = (DWORD)(cap - 1), type = 0;
    LONG r = RegQueryValueExA(k, name, NULL, &type, (LPBYTE)buf, &sz);
    RegCloseKey(k);
    if (r != ERROR_SUCCESS) return 0;
    buf[sz < cap ? sz : cap - 1] = 0;
    return 1;
}
static int win_set_dword(void *impl, const char *name, uint32_t v) {
    (void)impl;
    HKEY k = reg_open(1);
    if (!k) return ST_E_IO;
    DWORD d = v;
    LONG r = RegSetValueExA(k, name, 0, REG_DWORD, (const BYTE *)&d, 4);
    RegCloseKey(k);
    return r == ERROR_SUCCESS ? ST_OK : ST_E_IO;
}
static int win_set_string(void *impl, const char *name, const char *v) {
    (void)impl;
    HKEY k = reg_open(1);
    if (!k) return ST_E_IO;
    LONG r = RegSetValueExA(k, name, 0, REG_SZ, (const BYTE *)v, (DWORD)strlen(v) + 1);
    RegCloseKey(k);
    return r == ERROR_SUCCESS ? ST_OK : ST_E_IO;
}
static void win_close(void *impl) { (void)impl; }
static const StRegOps k_win_ops = { win_exists, win_get_dword, win_get_string,
                                    win_set_dword, win_set_string, win_close };
#endif

int st_reg_open_default(StRegStore *s) {
#ifdef _WIN32
    s->ops = &k_win_ops; s->impl = NULL;
    return ST_OK;
#else
    char path[1024];
    int rc = st_default_config_path(path, sizeof path);
    return rc == ST_OK ? st_reg_open_ini(s, path) : rc;
#endif
}

/* ============================================================ 2.2 / 2.4 install */
static const struct { const char *name; int index; } k_languages[] = {
    { "English US", 0 }, { "English UK", 0 }, { "French", 1 }, { "German", 2 },
    { "Italian", 3 }, { "Spanish", 4 }, { "Dutch", 5 }, { "Swedish", 6 }, { "Danish", 7 },
    { "Korean", 8 }, { "Chinese (Traditional)", 9 }, { "English China", 9 },
    { "Japanese", 10 }, { "Thai", 11 }
};

int st_language_index(const char *name) {
    for (size_t i = 0; i < sizeof k_languages / sizeof *k_languages; i++)
        if (ieq(name, k_languages[i].name)) return k_languages[i].index;
    return -1;
}

void st_install_read(StRegStore *s, StInstall *o) {
    char tmp[64];
    memset(o, 0, sizeof *o);
    o->streaming_install = 1;
    o->language_index = -1;                        /* built-in initial value (2.4) */
    st_reg_get_string(s, "Install Dir", o->install_dir, sizeof o->install_dir);
    st_reg_get_string(s, "CD Drive", o->cd_drive, sizeof o->cd_drive);
    o->not_first_time = st_reg_exists(s, "NotFirstTime");
    if (st_reg_get_string(s, "CacheSize", tmp, sizeof tmp)) o->cache_size = atol(tmp);
    if (st_reg_get_string(s, "SwapSize", tmp, sizeof tmp)) o->swap_size = atol(tmp);
    char lang[128];
    if (st_reg_get_string(s, "Language", lang, sizeof lang)) o->language_index = st_language_index(lang);
    else if (!st_reg_exists(s, "Language")) o->language_index = 0;   /* absent -> 0 */
    if (st_reg_get_string(s, "StreamingInstall", tmp, sizeof tmp) && ieq(tmp, "off"))
        o->streaming_install = 0;
}

int st_install_mark_started(StRegStore *s) { return st_reg_set_dword(s, "NotFirstTime", 1); }

/* ================================================================ 2.3 / 4 graphics */
static const char *const k_gfx_names[ST_GFX_NSLOTS] = {
    "VERSION", "SIZE", "g_CarEnvironmentMapEnable", "g_CarEnvironmentMapUpdateData",
    "g_CarShadowEnable", "g_CarHeadlightEnable", "g_CarLightingEnable", "g_CarDamageEnable",
    "g_CrowdEnable", "g_RoadReflectionEnable", "g_FogEnable", "g_MotionBlurEnable",
    "g_LightStreaksEnable", "g_LightGlowEnable", "g_AnimatedTextureEnable",
    "g_ParticleSystemEnable", "g_DepthOfFieldEnable", "g_WorldLodLevel", "g_CarLodLevel",
    "g_OverBrightEnable", "g_BleachByPassEnable", "g_TintingEnable", "g_FSAALevel",
    "g_HorizonFogEnable", "g_RainEnable", NULL, "g_TextureFiltering", "g_RacingResolution",
    NULL, "g_VSyncOn", "g_PerformanceLevel"
};
/* Write/read order of 2.3: idx 0..27, then g_PerformanceLevel, then g_VSyncOn. */
static const int k_gfx_order[] = {
    ST_GFX_VERSION, ST_GFX_SIZE, ST_GFX_CAR_ENV_MAP, ST_GFX_CAR_ENV_MAP_UPDATE, ST_GFX_CAR_SHADOW,
    ST_GFX_CAR_HEADLIGHT, ST_GFX_CAR_LIGHTING, ST_GFX_CAR_DAMAGE, ST_GFX_CROWD,
    ST_GFX_ROAD_REFLECTION, ST_GFX_FOG, ST_GFX_MOTION_BLUR, ST_GFX_LIGHT_STREAKS,
    ST_GFX_LIGHT_GLOW, ST_GFX_ANIMATED_TEXTURE, ST_GFX_PARTICLES, ST_GFX_DEPTH_OF_FIELD,
    ST_GFX_WORLD_LOD, ST_GFX_CAR_LOD, ST_GFX_OVER_BRIGHT, ST_GFX_BLEACH_BYPASS, ST_GFX_TINTING,
    ST_GFX_FSAA, ST_GFX_HORIZON_FOG, ST_GFX_RAIN, ST_GFX_TEXTURE_FILTERING, ST_GFX_RESOLUTION,
    ST_GFX_PERF_LEVEL, ST_GFX_VSYNC
};
#define GFX_NPERSIST ((int)(sizeof k_gfx_order / sizeof *k_gfx_order))

const char *st_gfx_value_name(int slot) {
    return (slot >= 0 && slot < ST_GFX_NSLOTS) ? k_gfx_names[slot] : NULL;
}

void st_gfx_defaults(StGraphics *g) {
    memset(g, 0, sizeof *g);
    g->v[ST_GFX_VERSION] = 1;
    g->v[ST_GFX_SIZE] = 120;
    g->v[ST_GFX_ANIMATED_TEXTURE] = 1;
    g->v[ST_GFX_SLOT25] = 1;
    g->v[ST_GFX_TEXTURE_FILTERING] = 1;
}

void st_gfx_caps_full(StGfxCaps *c, int fsaa_levels) {
    for (int i = 0; i < ST_GFX_NSLOTS; i++) c->cap[i] = ST_NO_CAP;
    c->cap[ST_GFX_CAR_ENV_MAP] = 3;       c->cap[ST_GFX_CAR_ENV_MAP_UPDATE] = 1;
    c->cap[ST_GFX_CAR_SHADOW] = 2;        c->cap[ST_GFX_CAR_HEADLIGHT] = 1;
    c->cap[ST_GFX_CAR_LIGHTING] = 0;      c->cap[ST_GFX_CAR_DAMAGE] = 0;
    c->cap[ST_GFX_CROWD] = 1;             c->cap[ST_GFX_ROAD_REFLECTION] = 3;
    c->cap[ST_GFX_FOG] = 1;               c->cap[ST_GFX_MOTION_BLUR] = 1;
    c->cap[ST_GFX_LIGHT_STREAKS] = 1;     c->cap[ST_GFX_LIGHT_GLOW] = 1;
    c->cap[ST_GFX_ANIMATED_TEXTURE] = 1;  c->cap[ST_GFX_PARTICLES] = 1;
    c->cap[ST_GFX_DEPTH_OF_FIELD] = 1;    c->cap[ST_GFX_WORLD_LOD] = 3;
    c->cap[ST_GFX_CAR_LOD] = 1;           c->cap[ST_GFX_OVER_BRIGHT] = 1;
    c->cap[ST_GFX_BLEACH_BYPASS] = 1;     c->cap[ST_GFX_TINTING] = 1;
    c->cap[ST_GFX_FSAA] = fsaa_levels > 0 ? fsaa_levels - 1 : 0;
    c->cap[ST_GFX_HORIZON_FOG] = 1;       c->cap[ST_GFX_RAIN] = 1;
    c->cap[ST_GFX_SLOT25] = 1;            c->cap[ST_GFX_TEXTURE_FILTERING] = 2;
    for (int i = 0; i < 6; i++) c->resolution_ok[i] = 1;
}

void st_gfx_load(StRegStore *s, StGraphics *g) {
    for (int i = 0; i < GFX_NPERSIST; i++) {
        uint32_t v;
        int slot = k_gfx_order[i];
        if (st_reg_get_dword(s, k_gfx_names[slot], &v)) g->v[slot] = (int32_t)v;
    }
}

int st_gfx_save(StRegStore *s, const StGraphics *g) {
    for (int i = 0; i < GFX_NPERSIST; i++) {
        int slot = k_gfx_order[i];
        uint32_t v = (uint32_t)g->v[slot];
        if (slot == ST_GFX_VERSION) v = 1;            /* constants, never validated (2.3) */
        else if (slot == ST_GFX_SIZE) v = 120;
        int rc = st_reg_set_dword(s, k_gfx_names[slot], v);
        if (rc != ST_OK) return rc;
    }
    return ST_OK;
}

void st_gfx_clamp(StGraphics *g, const StGfxCaps *c) {
    for (int i = 0; i < ST_GFX_NSLOTS; i++)
        if (g->v[i] > c->cap[i]) g->v[i] = c->cap[i];  /* never raised, no lower bound */
    int r = g->v[ST_GFX_RESOLUTION];
    if (r >= 0 && r < 6 && !c->resolution_ok[r]) {      /* out-of-range index left alone (SPEC 11.10) */
        for (int k = r - 1; k >= 0; k--)
            if (c->resolution_ok[k]) { g->v[ST_GFX_RESOLUTION] = k; break; }
    }
}

#define CAP (-1000)    /* "the cap" in the FSAA row */
static const struct { int slot; int v[5]; } k_preset[] = {
    { ST_GFX_CAR_SHADOW,       { 0, 2, 2, 2, 2 } },
    { ST_GFX_TINTING,          { 0, 1, 1, 1, 1 } },
    { ST_GFX_LIGHT_GLOW,       { 0, 1, 1, 1, 1 } },
    { ST_GFX_CROWD,            { 0, 1, 1, 1, 1 } },
    { ST_GFX_CAR_HEADLIGHT,    { 0, 1, 1, 1, 1 } },
    { ST_GFX_WORLD_LOD,        { 0, 1, 2, 2, 2 } },
    { ST_GFX_ROAD_REFLECTION,  { 0, 0, 2, 3, 3 } },
    { ST_GFX_HORIZON_FOG,      { 0, 0, 1, 1, 1 } },
    { ST_GFX_OVER_BRIGHT,      { 0, 0, 1, 1, 1 } },
    { ST_GFX_CAR_LOD,          { 0, 0, 1, 1, 1 } },
    { ST_GFX_DEPTH_OF_FIELD,   { 0, 0, 1, 1, 1 } },
    { ST_GFX_PARTICLES,        { 0, 0, 1, 1, 1 } },
    { ST_GFX_LIGHT_STREAKS,    { 0, 0, 1, 1, 1 } },
    { ST_GFX_MOTION_BLUR,      { 0, 0, 1, 1, 1 } },
    { ST_GFX_FOG,              { 0, 0, 1, 1, 1 } },
    { ST_GFX_CAR_ENV_MAP_UPDATE, { 0, 0, 1, 1, 1 } },
    { ST_GFX_CAR_ENV_MAP,      { 0, 0, 1, 2, 3 } },
    { ST_GFX_RAIN,             { 0, 0, 0, 1, 1 } },
    { ST_GFX_BLEACH_BYPASS,    { 0, 0, 0, 1, 1 } },
    { ST_GFX_FSAA,             { 0, 0, 0, 1, CAP } },
    { ST_GFX_TEXTURE_FILTERING, { 1, 1, 1, 1, 2 } },
    { ST_GFX_VSYNC,            { 0, 0, 0, 0, 0 } },
};

void st_gfx_apply_preset(StGraphics *g, const StGfxCaps *c, int level, double gpu_score) {
    int col = level <= 1 ? 0 : (level >= 5 ? 4 : level - 1);
    for (size_t i = 0; i < sizeof k_preset / sizeof *k_preset; i++) {
        int v = k_preset[i].v[col];
        if (v == CAP) v = c->cap[k_preset[i].slot] == ST_NO_CAP ? 0 : c->cap[k_preset[i].slot];
        g->v[k_preset[i].slot] = v;
    }
    if (level >= 4 && gpu_score > 4.0) g->v[ST_GFX_RESOLUTION] = 1;
}

int st_perf_level(int cpu_mhz, double gpu_score) {
    int cpu = cpu_mhz < 1400 ? 1 : (cpu_mhz < 1800 ? 2 : (cpu_mhz < 2400 ? 3 : 4));
    return (int)(0.2 * gpu_score + 0.8 * cpu);
}

void st_gfx_autodetect(StGraphics *g, const StGfxCaps *c, int level, double gpu_score) {
    g->v[ST_GFX_PERF_LEVEL] = level;
    g->v[ST_GFX_RESOLUTION] = 0;
    st_gfx_apply_preset(g, c, level, gpu_score);
    st_gfx_clamp(g, c);
}

void st_gfx_set_level(StGraphics *g, const StGfxCaps *c, int level, double gpu_score) {
    g->v[ST_GFX_PERF_LEVEL] = level;
    st_gfx_apply_preset(g, c, level, gpu_score);
    st_gfx_clamp(g, c);
}

int st_gfx_startup(StRegStore *s, StGraphics *g, const StGfxCaps *c, int safe_mode,
                   int cpu_mhz, double gpu_score) {
    int first = !st_reg_exists(s, "NotFirstTime");
    st_gfx_defaults(g);
    st_gfx_load(s, g);
    if (first || safe_mode) {
        int level = safe_mode ? 0 : st_perf_level(cpu_mhz, gpu_score);
        st_gfx_autodetect(g, c, level, gpu_score);
        if (safe_mode) st_gfx_save(s, g);
    }
    st_gfx_clamp(g, c);
    return first;
}

int st_resolution_step(StGraphics *g, const StGfxCaps *c, int dir) {
    int r = g->v[ST_GFX_RESOLUTION], step = dir < 0 ? -1 : 1;
    if (r < 0 || r >= 6) return 0;
    for (int k = r + step; k >= 0 && k < 6; k += step)
        if (c->resolution_ok[k]) { g->v[ST_GFX_RESOLUTION] = k; return 1; }
    return 0;
}

int st_resolution_size(int index, int *w, int *h) {
    static const int sz[6][2] = { {640, 480}, {800, 600}, {1024, 768}, {1280, 960},
                                  {1280, 1024}, {1600, 1200} };
    if (index < 0 || index > 5) return 0;
    if (w) *w = sz[index][0];
    if (h) *h = sz[index][1];
    return 1;
}

int st_audio_rate_hz(int perf_level) { return perf_level <= 1 ? 22050 : 44100; }

int st_parse_safe_mode(int argc, char **argv) {
    return argc > 1 && argv[1] && ieq(argv[1], "/s");   /* case rule unknown (SPEC 11.7) */
}
