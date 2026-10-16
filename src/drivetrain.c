/* drivetrain.c — OpenUG2 drivetrain / longitudinal acceleration (Stage 1).
 * Implements docs/cleanroom-driving-physics/SPEC.md Stage 1; see drivetrain.h
 * and docs/cleanroom-driving-physics/IMPL_DESIGN.md. Section tags "S1-x.y"
 * refer to that spec. Persistent state is binary32, intermediates are double. */
#include "drivetrain.h"

#include <math.h>
#include <string.h>

#define MPS_TO_MPH 2.237        /* S1-7.4 / U-8 */

static double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

/* ===================================================================== */
/* Car record: derived tables (S1-1.3, S1-1.4, S1-4.2)                     */
/* ===================================================================== */
int drv_drive_class(float s) {
    if (s < 0.2f) return DRV_CLASS_A;
    if (s > 0.8f) return DRV_CLASS_C;
    return DRV_CLASS_B;
}
float drv_launch_scale(int cls) {
    return cls == DRV_CLASS_A ? 1.0f : (cls == DRV_CLASS_C ? 1.1f : 1.2f);
}
float drv_curve_knot_rpm(float a, float b, int q, int i) {
    if (q < 2) return a;
    return (float)((double)a + (double)i * ((double)b - (double)a) / (double)(q - 1));
}

int drv_car_finalize(DrvCar *car) {
    if (car->gear_count < 1 || car->gear_count > DRV_MAX_FORWARD) return -1;
    if (!(car->engine_inertia > 0.0f) || !(car->mass > 0.0f)) return -1;
    if (!isfinite(car->idle_rpm) || !isfinite(car->shift_rpm) || !isfinite(car->max_rpm)) return -1;
    if (!(car->max_rpm > car->idle_rpm)) return -1;
    for (int i = 0; i < 4; i++) if (!(car->wheel_radius[i] > 0.0f)) return -1;

    /* Derive into a scratch copy and commit only when every derived value is
     * finite, so a rejected record leaves the caller's car untouched. */
    DrvCar t = *car;
    for (int j = 0; j < DRV_TORQUE_SAMPLES; j++) {
        t.torque[j] = t.torque_primary[j] + t.torque_secondary[j];   /* binary32 sum */
        if (!isfinite(t.torque[j])) return -1;
    }
    t.drive_class = drv_drive_class(t.rear_share);
    t.launch_scale = drv_launch_scale(t.drive_class);

    const int n = t.gear_count;
    const double e0 = t.idle_rpm, e1 = t.shift_rpm, e2 = t.max_rpm;
    memset(t.up_rpm, 0, sizeof t.up_rpm);
    memset(t.down_rpm, 0, sizeof t.down_rpm);
    t.up_rpm[1] = (float)((e0 + (e1 - e0) * (double)t.launch_fraction) * (double)t.launch_scale);
    for (int j = 2; j <= n; j++) {
        t.up_rpm[j] = (float)(e1 - 100.0);
        const double rj = t.ratio[j];
        t.down_rpm[j + 1] = fabs(rj) <= 1e-5 ? 0.0f
            : t.up_rpm[j] * (t.ratio[j + 1] / t.ratio[j]);   /* Reference rounding convention only; see S1-4.2 / U-14. */
    }
    t.up_rpm[n + 1] = (float)(e2 + 500.0);
    for (int j = 0; j < DRV_TABLE_SLOTS; j++)
        if (!isfinite(t.up_rpm[j]) || !isfinite(t.down_rpm[j])) return -1;
    if (!isfinite(t.launch_scale)) return -1;
    *car = t;
    return 0;
}

void drv_tune_default(DrvTune *t) {
    memset(t, 0, sizeof *t);
    t->torque_scale = 1.0f;
    t->grip_q = 1.0f;
    t->rate_scale = 1.0f;
}

void drv_env_default(DrvEnv *e) {
    memset(e, 0, sizeof *e);
    e->rear_grip_multiplier = 1.0f; /* Explicit level/straight/no-handbrake test adapter. */
    for (int i = 0; i < 4; i++) {
        e->wheel[i].contact = 1;
        e->wheel[i].mu_peak_surface = 1.0f;
        e->wheel[i].mu_slide_surface = 1.0f;
        e->wheel[i].roll_drag = 1.0f;
    }
}

/* ===================================================================== */
/* Stateless helpers                                                       */
/* ===================================================================== */
float drv_quantize_frame_dt_test_adapter(const DrvTune *tune, double seconds) {
    double q = seconds;
    if (tune) {
        if (tune->frame_dt_min > 0.0f && q < tune->frame_dt_min) q = tune->frame_dt_min;
        if (tune->frame_dt_max > 0.0f && q > tune->frame_dt_max) q = tune->frame_dt_max;
    }
    return (float)(floor(q / DRV_FRAME_QUANTUM + 0.5) * DRV_FRAME_QUANTUM);
}

int drv_substep_count(float frame_dt, float *h) {
    double dt = (double)frame_dt;
    int n = dt > 0.0 ? (int)(dt / (double)DRV_SUBSTEP_MAX) + 1 : 1;   /* trunc, then +1 */
    if (h) *h = (float)(dt / (double)n);
    return n;
}

/* S1-3.1 */
double drv_curve(const float *s, int q, double a, double b, double x) {
    double p = (x - a) * (double)(q - 1) / (b - a);
    if (isnan(p) || p < 0.0) return s[0];        /* also guards the integer conversion */
    if (p >= (double)(q - 1)) return s[q - 1];
    int i = (int)p;                              /* truncation toward zero, 0 <= p < q-1 */
    double f = p - (double)i;
    return f * (double)s[i + 1] + (1.0 - f) * (double)s[i];
}

/* S1-3.3 (nitrous and over-rev adjustment are deferred, S1-9) */
double drv_engine_torque_at(const DrvCar *car, const DrvTune *tune, int gear, int limiter, double rpm, double u) {
    const double G = tune->torque_scale, B = tune->torque_bonus ? 1.05 : 1.0;
    if (limiter) u = 0.0;
    double ton = drv_curve(car->torque, DRV_TORQUE_SAMPLES, car->idle_rpm, car->max_rpm, rpm) * G * B * u;
    double toff = -(1.0 - u) * drv_curve(car->engine_brake, DRV_BRAKE_SAMPLES, car->idle_rpm, car->max_rpm, rpm);
    if (limiter) toff *= 2.0;
    double te = ton + toff;
    if (te < 0.0 && !limiter) te *= (gear == DRV_GEAR_N) ? 0.65 : 0.85;
    return te;
}

/* S1-7.3 */
double drv_clutch_torque(const DrvCar *car, double omega_e, double omega_w, double engage, float *delta_prev) {
    double delta_w = clampd(omega_e - omega_w, -300.0, 300.0);
    double delta_rpm = (double)(float)((omega_w - omega_e) * (double)DRV_KR);
    double k = car->clutch_k;
    double dp = (double)*delta_prev;
    if (dp != 0.0 && delta_rpm * dp < 0.0 && fabs(delta_rpm) > 2000.0 && fabs(dp) > 2000.0) k *= 0.5;
    *delta_prev = (float)delta_rpm;
    return k * engage * delta_w;
}

/* S1-7.2 step 3 */
void drv_axle_split(double wa, double wb, int both_contact, double *share_a, double *share_b) {
    double p = both_contact ? 0.35 : 0.5;
    double f = fabs(wa + wb) <= 1e-5 ? 0.5 : wb / (wa + wb);
    if (f < p) f = 1.0 - 0.175;
    else if (f > 1.0 - p) f = 0.175;      /* else-if: the vectors need it (see IMPL_DESIGN) */
    if (share_b) *share_b = f;
    if (share_a) *share_a = 1.0 - f;
}

/* ===================================================================== */
/* Body helpers (S1-8.2)                                                   */
/* ===================================================================== */
void drv_body_accel_straight(void *ud, const float F[4], const float Fy[4], float A[4]) {
    const DrvBodyStraight *b = (const DrvBodyStraight *)ud;
    (void)Fy;                                   /* symmetric case: lateral forces cancel */
    double sum = b->ex;
    for (int i = 0; i < 4; i++) sum += F[i];
    float a = (float)(sum / (double)b->mass);
    for (int i = 0; i < 4; i++) A[i] = a;
}

void drv_body_accel_planar(void *ud, const float F[4], const float Fy[4], float A[4]) {
    const DrvBodyPlanar *b = (const DrvBodyPlanar *)ud;
    double fx = b->ex, fy = b->ey, mom = 0.0;
    for (int i = 0; i < 4; i++) {
        double cx = (double)F[i] * b->head[i][0] + (double)Fy[i] * b->lat[i][0];
        double cy = (double)F[i] * b->head[i][1] + (double)Fy[i] * b->lat[i][1];
        fx += cx; fy += cy;
        mom += (double)b->pos[i][0] * cy - (double)b->pos[i][1] * cx;
    }
    double ax = fx / (double)b->mass, ay = fy / (double)b->mass, al = mom * (double)b->inv_iz;
    for (int i = 0; i < 4; i++)
        A[i] = (float)((ax - al * (double)b->pos[i][1]) * (double)b->head[i][0]
                     + (al * (double)b->pos[i][0] + ay) * (double)b->head[i][1]);
}

/* ===================================================================== */
/* Instance                                                                */
/* ===================================================================== */
static void redraw_jitter(Drivetrain *d) {     /* S1-4.3 */
    for (int k = 0; k < 3; k++) {
        if (d->tune.rand01) {
            double u = (double)d->tune.rand01(d->tune.rand_ud);
            d->jitter[k] = (float)(-100.0 + 100.0 * u);
        } else {
            d->jitter[k] = 0.0f;                /* suppressed */
        }
    }
}

static void engine_post(Drivetrain *d);
static void clutch_post(Drivetrain *d);

int drv_init(Drivetrain *d, const DrvCar *car, const DrvTune *tune) {
    DrvCar fc = *car;
    if (drv_car_finalize(&fc) != 0) return -1;     /* *d is untouched on rejection */
    memset(d, 0, sizeof *d);
    d->car = fc;
    if (tune) d->tune = *tune; else drv_tune_default(&d->tune);
    if (!(d->tune.rate_scale > 0.0f)) d->tune.rate_scale = 1.0f;

    for (int i = 0; i < 4; i++) {
        d->wheel[i].w_long = 1.0f;
        d->wheel[i].w_lat = 1.0f;
        d->wheel[i].dir = DRV_DIR_STOPPED;
    }
    drv_env_default(&d->env);

    float rpm0 = d->tune.initial_rpm > 0.0f ? d->tune.initial_rpm : d->car.idle_rpm;       /* U-15 */
    d->omega_e = (float)clampd((double)rpm0 * DRV_KW, (double)d->car.idle_rpm * DRV_KW, (double)d->car.max_rpm * DRV_KW);
    d->throttle_mult = 1.0f;                       /* S1-3.2 */
    d->gear = d->target_gear = DRV_GEAR_N;         /* S1-4.1 */
    d->mode = DRV_CLUTCH_ENGAGING;
    d->engage = 0.0f;
    d->sel_gear = -1;
    d->manual_last_gear = -1;  /* Unspecified until drv_assist_supply(). */
    redraw_jitter(d);
    /* S1-2.5: zero-time post-processing, without derivative evaluation.
     * Initial wheel angles/spins are zero, so their post-step has no effect. */
    engine_post(d);
    clutch_post(d);
    drv_couple(d, &d->env, NULL, NULL);
    return 0;
}

/* ===================================================================== */
/* Wheel-side combination (S1-7.1)                                         */
/* ===================================================================== */
static int front_driven(const Drivetrain *d) { return d->car.rear_share <= 0.999f; }
static int rear_driven(const Drivetrain *d)  { return d->car.rear_share >= 0.001f; }

/* Combine per-wheel values the way S1-7.1 combines spins: mean per driven axle,
 * optionally times the axle's final drive, mean over driven axles. */
static double side_combine(const Drivetrain *d, const double v[4], int final_drive) {
    const int fd = front_driven(d), rd = rear_driven(d);
    double wf = (v[0] + v[1]) * 0.5, wr = (v[2] + v[3]) * 0.5;
    if (final_drive) { wf *= d->car.final_front; wr *= d->car.final_rear; }
    if (fd && rd) return (wf + wr) * 0.5;
    if (rd) return wr;
    return wf;
}

double drv_wheel_side_speed(const Drivetrain *d) {
    double w[4];
    for (int i = 0; i < 4; i++) w[i] = d->wheel[i].omega;
    return (double)d->car.ratio[d->gear] * side_combine(d, w, 1);
}

double drv_engine_torque(const Drivetrain *d) {
    double rpm = (double)(float)(d->omega_e * DRV_KR);
    return drv_engine_torque_at(&d->car, &d->tune, d->gear, d->limiter, rpm, (double)(d->throttle_mult * d->throttle));
}

/* ===================================================================== */
/* Per-frame: input shaping (S1-5) and gear selector (S1-4.5)              */
/* ===================================================================== */
static float uth(const Drivetrain *d, int g) {        /* up-shift threshold */
    double v = (double)d->car.up_rpm[g] + (double)(g == 3 ? d->jitter[2] : d->jitter[0]);
    return (float)fmin(v, (double)d->car.shift_rpm);
}
static float uth2(const Drivetrain *d, int g) {       /* Uth' */
    return (float)fmin((double)d->car.up_rpm[g] - 100.0, (double)d->car.shift_rpm);
}

static void select_gear(Drivetrain *d, int cmd, double rpm_e, float frame_dt) {
    const DrvCar *c = &d->car;
    const int n = c->gear_count;
    const int g = d->gear;
    d->report_auto = 0;

    if (cmd >= 0) {                                    /* direct command */
        if (cmd <= n + 1 && cmd != g && cmd != d->target_gear) d->target_gear = cmd;
        return;
    }
    if (cmd == DRV_CMD_AUTO) {
        d->sel_gear = -1;                              /* mark driver-selected gear "automatic" */
        if (g <= DRV_GEAR_N) { d->target_gear = 2; return; }
        if (d->mode != DRV_CLUTCH_LOCKED) return;
        if (rpm_e >= (double)uth(d, g) && g < n + 1) {
            int ok = 1;
            for (int i = 0; i < 4; i++)
                if (!d->env.wheel[i].contact || !(d->wheel[i].slip_speed < 4.0f)) ok = 0;
            if (ok) d->target_gear = g + 1;
            return;                                    /* otherwise nothing, no down-shift test */
        }
        double d1, d2;
        if (g > 2) {
            d1 = (double)c->down_rpm[g] + (double)d->jitter[1];
            d2 = ((double)c->ratio[g] / (double)c->ratio[g - 1]) * (double)uth2(d, g - 1) - 200.0;
        } else {
            d1 = d->throttle < 0.2f ? (double)c->idle_rpm + 100.0 : 0.0;
            d2 = 0.0;
        }
        if (rpm_e <= d1 && rpm_e <= d2) d->target_gear = g - 1;
        return;
    }
    if (cmd == DRV_CMD_MANUAL_ASSIST) {
        /* U-2: never invent the missing retail initial assist state. */
        if (!d->assist_state_supplied) return;
        d->manual_hold_time = (float)((double)d->manual_hold_time + frame_dt);
        if (d->mode != DRV_CLUTCH_LOCKED) return;
        int gs = d->sel_gear;
        if (gs < 2 || gs > n + 1) return;
        if (gs != d->target_gear) {
            d->manual_last_gear = g;
            d->manual_hold_time = 0;
            if (gs < d->target_gear) {
                const double ceiling = g > 2
                    ? ((double)c->ratio[g] / c->ratio[g - 1]) * 1.2 * c->max_rpm
                    : (d->throttle < 0.2f ? c->idle_rpm + 100.0 : 0.0);
                if (rpm_e > ceiling) d->sel_gear = d->target_gear = g;
                else d->target_gear = g - 1;
            } else d->target_gear = g + 1;
            return;
        }
        if (d->manual_hold_time > 10.0f ||
            (g > d->manual_last_gear && rpm_e > c->max_rpm - 300.0)) {
            d->report_auto = 1;
        } else if (rpm_e < c->idle_rpm + 100.0 && g > 2) {
            d->target_gear = g - 1;
            d->report_auto = 1;
        }
    }
}

/* S1-4.5 / U-2: caller owns the initial assist state and valid selections. */
int drv_assist_supply(Drivetrain *d, int selected, float elapsed, int remembered) {
    const int top = d->car.gear_count + 1;
    if (selected < DRV_GEAR_N || selected > top || remembered < 0 || remembered > top ||
        !(elapsed >= 0) || !isfinite(elapsed)) return -1;
    d->sel_gear = selected;
    d->manual_hold_time = elapsed;
    d->manual_last_gear = remembered;
    d->assist_state_supplied = 1;
    return 0;
}
int drv_assist_select(Drivetrain *d, int selected) {
    if (!d->assist_state_supplied || selected < 2 || selected > d->car.gear_count + 1) return -1;
    if (d->sel_gear == DRV_GEAR_N) d->manual_hold_time = 0;
    d->sel_gear = selected;
    return 0;
}

void drv_frame_begin(Drivetrain *d, const DrvInput *in, float frame_dt, double rpm_override) {
    const double rpm = (double)(float)(d->omega_e * DRV_KR);
    double t0 = clampd(in->throttle, 0.0, 1.0), b0 = clampd(in->brake, 0.0, 1.0), hb0 = clampd(in->handbrake, 0.0, 1.0);

    /* S1-5.1 pedal exclusivity */
    if (t0 > b0 && t0 > hb0) {
        if (b0 < 0.5) b0 = 0.0;
        if (hb0 < 0.5) hb0 = 0.0;
    } else if (t0 < 0.5) {
        t0 = 0.0;
    }
    /* S1-5.2 dead zone */
    double t = t0 <= 0.15 ? 0.0 : t0;
    /* S1-5.3 anti-stall floor */
    if (t > 0.0 && t < 0.5 && d->gear > DRV_GEAR_N && rpm < 3000.0) t = 0.5;
    d->throttle = (float)t; d->brake = (float)b0; d->handbrake = (float)hb0;

    select_gear(d, in->gear_cmd, rpm_override >= 0.0 ? rpm_override : rpm, frame_dt);
}

/* ===================================================================== */
/* Substep: derivatives, Euler, post-steps (S1-2.4 a-c)                    */
/* ===================================================================== */
static const double CLUTCH_RATE_K[4] = { -3.0, 0.0, 1.5, 0.0 };

static void clutch_post(Drivetrain *d) {                /* S1-4.4 */
    const DrvCar *c = &d->car;
    d->omega_w = (float)drv_wheel_side_speed(d);
    const int crawl = d->throttle < 0.3f && d->omega_w <= c->idle_rpm * DRV_KW + 1e-4f;
    switch (d->mode) {
    case DRV_CLUTCH_DISENGAGING:
        d->throttle_mult = 0.0f;
        if (d->engage <= 0.0f) { d->engage = 0.0f; d->mode = DRV_CLUTCH_OPEN; }
        break;
    case DRV_CLUTCH_OPEN:
        d->throttle_mult = 0.0f;
        d->gear = d->target_gear;
        if (d->pending_shift || !crawl) { d->mode = DRV_CLUTCH_ENGAGING; redraw_jitter(d); }
        break;
    case DRV_CLUTCH_ENGAGING:
        d->throttle_mult = 0.7f;
        if (d->engage >= 1.0f) {
            d->engage = 1.0f;
            if (fabs((double)d->omega_e - (double)d->omega_w) <= 50.0 || d->gear == DRV_GEAR_N) d->mode = DRV_CLUTCH_LOCKED;
        }
        if (crawl && d->gear != DRV_GEAR_N) d->mode = DRV_CLUTCH_DISENGAGING;
        break;
    case DRV_CLUTCH_LOCKED:
    default:
        d->throttle_mult = 1.0f;
        if (d->gear != DRV_GEAR_N && crawl) d->mode = DRV_CLUTCH_DISENGAGING;
        if (d->gear == d->target_gear) d->pending_shift = 0;
        else { d->pending_shift = 1; d->mode = DRV_CLUTCH_DISENGAGING; }
        break;
    }
    /* missed-shift timer and drag shift bonus: deferred (S1-4.4, S1-9) */
}

static void engine_post(Drivetrain *d) {                /* S1-3.5 */
    const DrvCar *c = &d->car;
    d->omega_e = (float)clampd(d->omega_e, c->idle_rpm * DRV_KW, c->max_rpm * DRV_KW);
    d->feedback = 0.0f;
    const double rpm = (double)(float)(d->omega_e * DRV_KR);
    const double e2 = c->max_rpm;
    if (d->limiter) { if (rpm < e2 - 0.0125 * e2) d->limiter = 0; }
    else if (rpm > e2 - 25.0) d->limiter = 1;
}

void drv_integrate(Drivetrain *d, float h) {
    const double dh = (double)h;
    /* (a) derivatives from the current state */
    const double te = drv_engine_torque(d);
    d->engine_torque = (float)te;
    const double we_dot = (te + (double)d->feedback) / (double)d->car.engine_inertia;
    const double c_dot = (double)d->car.clutch_rate * (double)d->tune.rate_scale * CLUTCH_RATE_K[d->mode & 3];
    double w_dot[4];
    for (int i = 0; i < 4; i++) {
        DrvWheel *w = &d->wheel[i];
        const DrvWheelEnv *e = &d->env.wheel[i];
        const double R = d->car.wheel_radius[i];
        double tb = 0.5 * ((double)w->tau - R * (double)w->force + (double)w->tau_smooth);
        w->tau_smooth = (float)tb;
        if (e->locked) { w_dot[i] = 0.0; continue; }
        double wd = (tb - 0.002 * (double)e->roll_drag * (double)w->omega
                        - (e->drag_flag ? 1.0 : 0.0) * 0.1 * (double)w->omega) / (double)DRV_WHEEL_INERTIA;
        if (w->gripping && dh > 0.001) wd -= (double)w->slip_speed / (R * dh);
        w_dot[i] = wd;
    }
    /* (b) explicit Euler on every state scalar */
    d->omega_e = (float)((double)d->omega_e + dh * we_dot);
    d->engage = (float)((double)d->engage + dh * c_dot);
    for (int i = 0; i < 4; i++) {
        DrvWheel *w = &d->wheel[i];
        w->phi = (float)((double)w->phi + dh * (double)w->omega);
        w->omega = (float)((double)w->omega + dh * w_dot[i]);
    }
    /* (c) post-steps in block order: wheels (+ suspension, later stage), engine, clutch */
    for (int i = 0; i < 4; i++) {
        DrvWheel *w = &d->wheel[i];
        w->tau = 0.0f; w->force = 0.0f; w->force_lat = 0.0f; w->slip_speed = 0.0f;
        const double turn_limit = 628.3179931640625, reset_limit = 6283.18017578125;
        if (w->phi > turn_limit) {
            w->phi = (float)((double)w->phi - turn_limit);
            if (w->phi > reset_limit) w->phi = 0;
        } else if (w->phi < -turn_limit) {
            w->phi = (float)((double)w->phi + turn_limit);
            if (w->phi < -reset_limit) w->phi = 0;
        }
        w->dir = w->omega > 1e-5f ? DRV_DIR_FORWARD : (w->omega >= -1e-5f ? DRV_DIR_STOPPED : DRV_DIR_BACKWARD);
    }
    engine_post(d);
    clutch_post(d);
}

/* ===================================================================== */
/* Coupling (S1-7)                                                         */
/* ===================================================================== */
static void dist(Drivetrain *d, double T) {             /* S1-7.2 */
    if (d->gear == DRV_GEAR_N) return;
    double nc = d->contact_count / 4.0;
    d->feedback = (float)((double)d->feedback - (nc > 0.25 ? nc : 0.25) * T);
    const double Tg = T * (double)d->car.ratio[d->gear];
    const double s = d->car.rear_share;
    const double axle_torque[2] = { (1.0 - s) * (double)d->car.final_front * Tg, s * (double)d->car.final_rear * Tg };
    static const int pair[2][2] = { { 0, 1 }, { 3, 2 } };     /* (a, b) per axle */
    for (int ax = 0; ax < 2; ax++) {
        const int a = pair[ax][0], b = pair[ax][1];
        double sa, sb;
        drv_axle_split(d->wheel[a].omega, d->wheel[b].omega, d->env.wheel[a].contact && d->env.wheel[b].contact, &sa, &sb);
        if (!d->env.wheel[a].locked) d->wheel[a].tau = (float)((double)d->wheel[a].tau + sa * axle_torque[ax]);
        if (!d->env.wheel[b].locked) d->wheel[b].tau = (float)((double)d->wheel[b].tau + sb * axle_torque[ax]);
    }
}

static double psi(double x) {                           /* S1-7.7 table */
    static const double xs[5] = { -10.0, -7.5, -3.5, -0.3, -0.05 };
    static const double ys[5] = { 1.0, 0.96, 0.925, 0.875, 0.0 };
    if (x <= xs[0]) return ys[0];
    if (x >= xs[4]) return ys[4];
    for (int i = 0; i < 4; i++)
        if (x <= xs[i + 1]) return ys[i] + (ys[i + 1] - ys[i]) * (x - xs[i]) / (xs[i + 1] - xs[i]);
    return ys[4];
}

/* S1-7.5: isolated current-state lock correction, before distribution. */
double drv_lock_correction(const Drivetrain *d) {
    const DrvCar *c = &d->car;
    const double Iw = DRV_WHEEL_INERTIA;
    const double Te = drv_engine_torque(d);
    const double ae = (Te + (double)d->feedback) / (double)c->engine_inertia;
    double beta[4];
    for (int i = 0; i < 4; i++)
        beta[i] = ((double)d->wheel[i].tau - (double)c->wheel_radius[i] * (double)d->wheel[i].force) / Iw;
    const double aw = (double)c->ratio[d->gear] * side_combine(d, beta, 1);
    const double rg = c->ratio[d->gear];
    double dlock = 1.0 / (double)c->engine_inertia;
    if (front_driven(d)) dlock += (1.0 - c->rear_share) / Iw * (rg * c->final_front) * (rg * c->final_front) * 0.5;
    if (rear_driven(d))  dlock += c->rear_share / Iw * (rg * c->final_rear) * (rg * c->final_rear) * 0.5;
    return (ae - aw) / dlock;
}

void drv_couple(Drivetrain *d, const DrvEnv *env, DrvBodyAccelFn body, void *body_ud) {
    const DrvCar *c = &d->car;
    const double Iw = DRV_WHEEL_INERTIA;
    d->env = *env;
    d->contact_count = 0;
    for (int i = 0; i < 4; i++) if (env->wheel[i].contact) d->contact_count++;
    for (int i = 0; i < 4; i++) {            /* idempotent: normally already zero after the post-step */
        d->wheel[i].tau = 0.0f; d->wheel[i].force = 0.0f; d->wheel[i].force_lat = 0.0f;
    }

    /* grip multipliers (S1-7.8) */
    double gamma[4], kv[4];
    for (int i = 0; i < 4; i++) {
        const DrvWheelEnv *e = &env->wheel[i];
        gamma[i] = (double)d->tune.grip_q * (1.0 - 0.025 * (double)d->tune.grip_loc_x);
        if (i > 1) gamma[i] *= env->rear_grip_multiplier;
        kv[i] = 1.0;
        if (d->tune.is_player && i > 1 && !e->locked && d->wheel[i].omega >= 0.0f)
            kv[i] = 0.85 + 0.15 * clampd(((double)env->body_speed - 13.4108) * 0.11185, 0.0, 1.0);
    }

    const double wside = d->omega_w; /* S1-7.3: clutch post-step snapshot. */
    const double te0 = drv_engine_torque(d);

    /* S1-7.3 initial drive */
    if (d->mode == DRV_CLUTCH_DISENGAGING || d->mode == DRV_CLUTCH_ENGAGING)
        dist(d, drv_clutch_torque(c, d->omega_e, wside, d->engage, &d->delta_prev));
    else if (d->mode == DRV_CLUTCH_LOCKED)
        dist(d, te0);

    /* S1-7.4 tyre preparation */
    for (int i = 0; i < 4; i++) {
        DrvWheel *w = &d->wheel[i];
        const DrvWheelEnv *e = &env->wheel[i];
        const int ax = i > 1;
        const double R = c->wheel_radius[i], N = e->load;
        const double si = R * (double)w->omega - (double)e->vx;
        const double vy = e->vy;
        const double mag = hypot(si, vy);
        w->slip_speed = (float)si;
        w->gripping = 1;
        if (mag > 0.0 && (e->vx != 0.0f || e->vy != 0.0f)) {
            double mu = (double)e->mu_slide_surface * gamma[i] * (double)c->mu_slide[ax] * kv[i];
            double Fs = N * mu / mag;
            if (fabs(si) > 0.5 || fabs((double)e->slip_angle) > (double)c->theta_max[ax] || e->locked) {
                double F = (double)w->w_long * si * Fs, Fy = -vy * Fs * (double)w->w_lat;
                if ((double)env->body_speed * MPS_TO_MPH < 1.0 && mu > 0.1) { F /= mu; Fy /= mu; }
                w->force = (float)F; w->force_lat = (float)Fy;
                w->gripping = 0;
            }
        }
        if (w->gripping) {
            w->force = (float)((double)w->w_long * (double)w->tau / R);
            w->force_lat = (float)((double)w->w_lat * (double)e->lateral_grip_force);
        }
    }

    /* S1-7.5 iterative pass */
    for (int pass = 0; pass < 5; pass++) {
        float A[4] = { 0, 0, 0, 0 }, F[4], Fy[4];
        for (int i = 0; i < 4; i++) { F[i] = d->wheel[i].force; Fy[i] = d->wheel[i].force_lat; }
        if (body) body(body_ud, F, Fy, A);
        for (int i = 0; i < 4; i++) {
            DrvWheel *w = &d->wheel[i];
            if (w->gripping && !env->wheel[i].locked) {
                const double R = c->wheel_radius[i];
                w->force = (float)((double)w->force + ((double)w->tau - R * (double)w->force) - (double)A[i] * Iw / R);
            }
        }
        if (d->mode == DRV_CLUTCH_LOCKED && d->gear != DRV_GEAR_N) {
            dist(d, drv_lock_correction(d));
        }
    }

    /* S1-7.6 friction limit */
    for (int i = 0; i < 4; i++) {
        DrvWheel *w = &d->wheel[i];
        const DrvWheelEnv *e = &env->wheel[i];
        const int ax = i > 1;
        const double Fmax = (double)e->mu_peak_surface * (double)c->mu_peak[ax] * gamma[i] * (double)e->load * kv[i];
        const int boosted = w->tau > 0.0f && !e->locked;
        double F = (double)w->force, Fy = (double)w->force_lat;
        double Ft = boosted ? 1.5 * F : F;
        double mag = hypot(Ft, Fy);
        if (mag > Fmax && mag > 0.001) {
            double k = Fmax / mag;
            F = Ft * k; Fy *= k;            /* the 1.5 factor is kept when scaling happened */
        }
        if (fabs((double)e->vx) > 1.0) F -= 0.15 * sin((double)e->slip_angle) * Fy;
        w->force = (float)F; w->force_lat = (float)Fy;
        d->force_long[i] = w->force; d->force_lat[i] = w->force_lat; d->load[i] = e->load;
    }

    /* S1-7.7 finishing */
    if (d->gear != DRV_GEAR_N && d->mode == DRV_CLUTCH_LOCKED && d->contact_count > 0) {
        double sl[4], ws[4];
        for (int i = 0; i < 4; i++) { sl[i] = d->wheel[i].slip_speed; ws[i] = d->wheel[i].omega; }
        const double sbar = side_combine(d, sl, 0);
        const double wold = (double)c->ratio[d->gear] * side_combine(d, ws, 1);
        const double dd = wold - (double)d->omega_e;
        const double Te = drv_engine_torque(d);
        if (d->throttle <= 0.2f || sbar <= 0.1) {
            d->feedback = (float)((double)d->feedback + clampd((double)c->clutch_k * dd, -fabs(Te), fabs(Te)));
        } else {
            d->omega_w = d->omega_e;
            const int driven[2] = { front_driven(d), rear_driven(d) };
            for (int ax = 0; ax < 2; ax++) {
                if (!driven[ax]) continue;
                const int a = ax ? 3 : 0, b = ax ? 2 : 1;
                const double fin = ax ? c->final_rear : c->final_front;
                const double dw = ((double)d->omega_e - wold) / ((double)c->ratio[d->gear] * fin);
                if ((double)env->body_speed * MPS_TO_MPH < 40.0 || (!env->wheel[a].contact && !env->wheel[b].contact)) {
                    float m = (float)(((double)d->wheel[a].omega + (double)d->wheel[b].omega) * 0.5);
                    d->wheel[a].omega = m; d->wheel[b].omega = m;
                }
                d->wheel[a].omega = (float)((double)d->wheel[a].omega + dw);
                d->wheel[b].omega = (float)((double)d->wheel[b].omega + dw);
            }
        }
    }
    if (d->gear == 2) {                   /* first gear, any clutch mode */
        const double Te = drv_engine_torque(d);
        const double phi = d->feedback;
        if (phi < Te && Te + phi < 0.0) {
            static const double kc[4] = { 0.0, -0.2, -0.4, -0.7 };
            int pc = d->tune.perf_class < 0 ? 0 : (d->tune.perf_class > 3 ? 3 : d->tune.perf_class);
            d->feedback = (float)(phi + psi(Te + phi) * kc[pc] * (Te + phi));
        }
    }
}
