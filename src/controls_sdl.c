/* controls_sdl.c -- see controls_sdl.h. */
#include "controls_sdl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "controls_store.h"

#define AXIS_DEADZONE 0.15f

/* ---- devices --------------------------------------------------------------------------------------- */

static int hat_centidegrees(Uint8 h) {
    switch (h) {
    case SDL_HAT_UP: return 0;
    case SDL_HAT_RIGHTUP: return 4500;
    case SDL_HAT_RIGHT: return 9000;
    case SDL_HAT_RIGHTDOWN: return 13500;
    case SDL_HAT_DOWN: return 18000;
    case SDL_HAT_LEFTDOWN: return 22500;
    case SDL_HAT_LEFT: return 27000;
    case SDL_HAT_LEFTUP: return 31500;
    default: return -1;
    }
}

static void open_pad(CxInput *c, int device_index, int slot) {
    if (!SDL_IsGameController(device_index)) {      /* HID oddities (system-control, media keys) enumerate as one-axis joysticks */
        SDL_Joystick *probe = SDL_JoystickOpen(device_index);
        int real = probe && SDL_JoystickNumAxes(probe) >= 2;
        if (probe) SDL_JoystickClose(probe);
        if (!real) return;
    }
    SDL_Joystick *j = SDL_JoystickOpen(device_index);
    if (!j) return;
    CbPad *p = &c->b.pad[slot];
    memset(p, 0, sizeof *p);
    c->joy[slot] = j;
    if (SDL_IsGameController(device_index)) c->gc[slot] = SDL_GameControllerOpen(device_index);
    SDL_JoystickGUID g = SDL_JoystickGetGUID(j);
    memcpy(p->guid, g.data, 16);
    p->info.vid = SDL_JoystickGetVendor(j);
    p->info.pid = SDL_JoystickGetProduct(j);
    p->info.buttons = SDL_JoystickNumButtons(j);
    int axes = SDL_JoystickNumAxes(j);
    for (int a = 0; a < axes && a < CB_AXIS_COUNT; a++) p->info.axes_mask |= 1u << a;   /* raw SDL indices (D3) */
    p->info.wheel = SDL_JoystickGetType(j) == SDL_JOYSTICK_TYPE_WHEEL;
    memcpy(p->dev.guid, p->guid, 16);
    cb_pad_defaults(&p->dev, &p->info);
    p->present = 1;
}

void cx_init(CxInput *c) {
    memset(c, 0, sizeof *c);
    cb_bindings_defaults(&c->b);
    int slot = 0;
    for (int i = 0, n = SDL_NumJoysticks(); i < n && slot < CB_MAX_PADS; i++) {
        open_pad(c, i, slot);
        if (c->b.pad[slot].present) slot++;
    }
    char path[1024];
    StRegStore s;
    if (st_default_config_path(path, sizeof path) == ST_OK && st_reg_open_ini(&s, path) == ST_OK) {
        cs_kb_load(&s, &c->b.kb);
        st_reg_close(&s);
    }
    c->profile = (StProfile *)calloc(1, sizeof *c->profile);
    c->have_root = st_default_profile_root(c->profile_root, sizeof c->profile_root) == ST_OK;
    if (c->profile) {
        int lrc = c->have_root ? st_profile_load(c->profile_root, CS_PROFILE_NAME, c->profile) : ST_E_ARG;
        c->profile_writable = lrc == ST_OK;
        if (lrc != ST_OK) {
            /* Defaults stand in for the session, but the file is only ever created when it is genuinely absent:
             * a corrupt, incompatible or unreadable profile is preserved (its pad bindings are not saved). */
            c->profile_writable = c->have_root && st_profile_presence(c->profile_root, CS_PROFILE_NAME) == ST_E_NOTFOUND;
            memset(c->profile, 0, sizeof *c->profile);
            st_profile_defaults(c->profile, 0);
            snprintf(c->profile->name, sizeof c->profile->name, "%s", CS_PROFILE_NAME);
        }
        cs_pads_load(c->profile, &c->b);
    }
    c->ready = 1;
}

void cx_shutdown(CxInput *c) {
    for (int i = 0; i < CB_MAX_PADS; i++) {
        if (c->gc[i]) SDL_GameControllerClose(c->gc[i]);
        if (c->joy[i]) SDL_JoystickClose(c->joy[i]);
    }
    free(c->profile);
    memset(c, 0, sizeof *c);
}

int cx_save(CxInput *c) {
    int rc = 0;
    char path[1024];
    StRegStore s;
    if (st_default_config_path(path, sizeof path) == ST_OK && st_reg_open_ini(&s, path) == ST_OK) {
        if (cs_kb_save(&s, &c->b.kb) != ST_OK) rc = -1;
        st_reg_close(&s);
    } else rc = -1;
    if (c->profile && c->have_root && c->profile_writable) {
        cs_pads_store(c->profile, &c->b);
        if (st_profile_save(c->profile_root, c->profile) != ST_OK) rc = -1;
    } else rc = -1;
    return rc;
}

/* ---- names (D2) ---------------------------------------------------------------------------------------- */

static const char *key_name(void *user, int scancode) {
    (void)user;
    return SDL_GetScancodeName((SDL_Scancode)scancode);
}

/* SDL has no object names for raw joystick axes and hats. Where the device has a game-controller mapping the
 * controller's own axis name is used; otherwise there is no platform name and the cell shows "---". */
static const char *ctl_name(void *user, int pad, CbCtl ctl) {
    CxInput *c = (CxInput *)user;
    if (pad < 0 || pad >= CB_MAX_PADS || !c->gc[pad] || ctl.kind != CB_CTL_AXIS) return NULL;
    int slot = ctl.index / 4;
    for (int a = 0; a < SDL_CONTROLLER_AXIS_MAX; a++) {
        SDL_GameControllerButtonBind b = SDL_GameControllerGetBindForAxis(c->gc[pad], (SDL_GameControllerAxis)a);
        if (b.bindType == SDL_CONTROLLER_BINDTYPE_AXIS && b.value.axis == slot)
            return SDL_GameControllerGetStringForAxis((SDL_GameControllerAxis)a);
    }
    return NULL;
}

void cx_platform(CxInput *c, CmPlatform *out) {
    out->user = c;
    out->key_name = key_name;
    out->ctl_name = ctl_name;
}

/* ---- sampling ------------------------------------------------------------------------------------------- */

void cx_sample(const CxInput *c, int i, CbPadSample *out) {
    memset(out, 0, sizeof *out);
    for (int p = 0; p < CB_BIND_POVS; p++) out->pov[p] = -1;
    if (i < 0 || i >= CB_MAX_PADS || !c->joy[i]) return;
    SDL_Joystick *j = c->joy[i];
    int nb = SDL_JoystickNumButtons(j), nh = SDL_JoystickNumHats(j), na = SDL_JoystickNumAxes(j);
    for (int b = 0; b < nb && b < CB_BIND_BUTTONS; b++) if (SDL_JoystickGetButton(j, b)) out->buttons |= 1u << b;
    for (int h = 0; h < nh && h < CB_BIND_POVS; h++) out->pov[h] = hat_centidegrees(SDL_JoystickGetHat(j, h));
    for (int a = 0; a < na && a < CB_AXIS_COUNT; a++) out->axis[a] = SDL_JoystickGetAxis(j, a);
}

/* ---- driving queries -------------------------------------------------------------------------------------- */

static int pad_control_down(const CxInput *c, int i, int control) {
    SDL_Joystick *j = c->joy[i];
    if (!j) return 0;
    if (control < CB_BIND_BUTTONS) return control < SDL_JoystickNumButtons(j) && SDL_JoystickGetButton(j, control);
    int p = (control - CB_BIND_BUTTONS) / 4, d = (control - CB_BIND_BUTTONS) % 4;
    if (p >= SDL_JoystickNumHats(j)) return 0;
    Uint8 h = SDL_JoystickGetHat(j, p);
    static const Uint8 mask[4] = { SDL_HAT_DOWN, SDL_HAT_UP, SDL_HAT_LEFT, SDL_HAT_RIGHT };
    return (h & mask[d]) != 0;
}

static int pad_action_down(const CxInput *c, int action) {
    for (int i = 0; i < CB_MAX_PADS; i++) {
        if (!c->b.pad[i].present) continue;
        for (int k = 0; k < 32; k++)
            if (c->b.pad[i].dev.binding[k] == action && pad_control_down(c, i, k)) return 1;
    }
    return 0;
}

int cx_action_down(const CxInput *c, const Uint8 *keys, int action) {
    int k[2];
    int n = cb_kb_keys(&c->b.kb, action, k);
    for (int i = 0; i < n; i++) if (keys && keys[k[i]]) return 1;
    return pad_action_down(c, action);
}

uint32_t cx_joy_edges(CxInput *c, const SDL_Event *e) {
    uint32_t out = 0;
    if (e->type == SDL_JOYBUTTONDOWN) {
        for (int i = 0; i < CB_MAX_PADS; i++) {
            if (!c->joy[i] || SDL_JoystickInstanceID(c->joy[i]) != e->jbutton.which) continue;
            int b = e->jbutton.button;
            if (b >= 0 && b < CB_BIND_BUTTONS) { int a = c->b.pad[i].dev.binding[b]; if (a > 0 && a < 32) out |= 1u << a; }
        }
    } else if (e->type == SDL_JOYHATMOTION) {
        static const Uint8 mask[4] = { SDL_HAT_DOWN, SDL_HAT_UP, SDL_HAT_LEFT, SDL_HAT_RIGHT };
        for (int i = 0; i < CB_MAX_PADS; i++) {
            if (!c->joy[i] || SDL_JoystickInstanceID(c->joy[i]) != e->jhat.which) continue;
            int h = e->jhat.hat;
            if (h < 0 || h >= CB_BIND_POVS) continue;
            Uint8 now = e->jhat.value, fresh = (Uint8)(now & ~c->hat_prev[i][h]);
            c->hat_prev[i][h] = now;
            for (int d = 0; d < 4; d++) {
                if (!(fresh & mask[d])) continue;
                int a = c->b.pad[i].dev.binding[CB_BIND_BUTTONS + 4 * h + d];
                if (a > 0 && a < 32) out |= 1u << a;
            }
        }
    }
    return out;
}

int cx_key_is(const CxInput *c, int scancode, int action) {
    int k[2];
    int n = cb_kb_keys(&c->b.kb, action, k);
    for (int i = 0; i < n; i++) if (k[i] == scancode) return 1;
    return 0;
}

int cx_key_bound(const CxInput *c, int scancode) {
    CbRow rows[CB_MAX_ROWS];
    int n = cb_rows(CB_CLASS_KEYBOARD, rows);
    for (int i = 0; i < n; i++) if (cx_key_is(c, scancode, rows[i].action)) return 1;
    return 0;
}

int cx_joy_button_is(const CxInput *c, SDL_JoystickID instance, int button, int action) {
    if (button < 0 || button >= CB_BIND_BUTTONS) return 0;
    for (int i = 0; i < CB_MAX_PADS; i++)
        if (c->joy[i] && SDL_JoystickInstanceID(c->joy[i]) == instance) return c->b.pad[i].dev.binding[button] == action;
    return 0;
}

static float axis_unit(const CxInput *c, int pad, int offset) {      /* -1..1 */
    int slot = offset / 4;
    if (!c->joy[pad] || slot >= SDL_JoystickNumAxes(c->joy[pad])) return 0.0f;
    float v = (float)SDL_JoystickGetAxis(c->joy[pad], slot) / 32768.0f;
    return v > 1.0f ? 1.0f : v < -1.0f ? -1.0f : v;
}
static float dead(float v) {
    float a = v < 0 ? -v : v;
    if (a <= AXIS_DEADZONE) return 0.0f;
    a = (a - AXIS_DEADZONE) / (1.0f - AXIS_DEADZONE);
    return v < 0 ? -a : a;
}

/* Pad axes. Steering: positive steering-axis output times its direction is right. Combined pedals: output
 * times the combined direction is brake when positive, throttle when negative (the capture stores the
 * direction that way, CONTROLS_SPEC 6.5). A separate pedal axis counts only past its rest side. */
static float pad_throttle(const CxInput *c) {
    float best = 0.0f;
    for (int i = 0; i < CB_MAX_PADS; i++) {
        const StInputDevice *d = &c->b.pad[i].dev;
        if (!c->b.pad[i].present) continue;
        float t = 0.0f;
        if (d->combined_mode && d->assigned[CB_ROLE_COMBINED]) t = -dead(axis_unit(c, i, d->axis[CB_ROLE_COMBINED]) * (float)d->dir[CB_ROLE_COMBINED]);
        else {
            if (d->assigned[CB_ROLE_THROTTLE]) { float v = dead(axis_unit(c, i, d->axis[CB_ROLE_THROTTLE]) * (float)d->dir[CB_ROLE_THROTTLE]); if (v > 0) t += v; }
            if (d->assigned[CB_ROLE_BRAKE]) { float v = dead(axis_unit(c, i, d->axis[CB_ROLE_BRAKE]) * (float)d->dir[CB_ROLE_BRAKE]); if (v > 0) t -= v; }
        }
        if ((t < 0 ? -t : t) > (best < 0 ? -best : best)) best = t;
    }
    return best;
}

float cx_throttle(const CxInput *c, const Uint8 *keys) {
    if (cx_action_down(c, keys, CB_ACT_THROTTLE)) return 1.0f;      /* throttle wins over brake */
    if (cx_action_down(c, keys, CB_ACT_BRAKE)) return -1.0f;
    return pad_throttle(c);
}

float cx_steer(const CxInput *c, const Uint8 *keys) {
    float s = (cx_action_down(c, keys, CB_ACT_STEER_LEFT) ? 1.0f : 0.0f) - (cx_action_down(c, keys, CB_ACT_STEER_RIGHT) ? 1.0f : 0.0f);
    if (s != 0.0f) return s;
    for (int i = 0; i < CB_MAX_PADS; i++) {
        const StInputDevice *d = &c->b.pad[i].dev;
        if (!c->b.pad[i].present || !d->assigned[CB_ROLE_STEER]) continue;
        float r = dead(axis_unit(c, i, d->axis[CB_ROLE_STEER]) * (float)d->dir[CB_ROLE_STEER]);
        if (r != 0.0f) return -r;                                    /* right is negative in the driving code */
    }
    return 0.0f;
}
