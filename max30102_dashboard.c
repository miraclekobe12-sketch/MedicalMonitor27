/*
 * max30102_dashboard.c
 *
 * 
 *
 * Build:
 *   qcc -Vgcc_ntoaarch64le max30102_dashboard.c -o max30102_dashboard -lscreen -lm -pthread
 *
 * Run:
 *   ./max30102_dashboard
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>

#include <devctl.h>
#include <screen/screen.h>
#include <sys/neutrino.h>

#include "vitals.h"

/* ------------------------------------------------------------------ */
/* CONFIG                                                               */
/* ------------------------------------------------------------------ */

#define FETCH_INTERVAL_NS   20000000LL   /* 50 Hz data fetch           */
#define RENDER_INTERVAL_NS  33333333LL   /* ~30 fps render             */
#define PULSE_CODE_FETCH    2
#define PULSE_CODE_RENDER   3

#define WAVE_LEN            500          /* ~10 seconds of data @ 50Hz */
#define PANEL_PAD           16

/* ------------------------------------------------------------------ */
/* COLORS (0xFFRRGGBB)                                                  */
/* ------------------------------------------------------------------ */

#define COL_BG      0xFF0F172A
#define COL_PANEL   0xFF1E293B
#define COL_BORDER  0xFF334155
#define COL_GREEN   0xFF22C55E
#define COL_YELLOW  0xFFF59E0B
#define COL_RED     0xFFEF4444
#define COL_WAVE_BPM    0xFF38BDF8
#define COL_WAVE_SPO2   0xFFEC4899
#define COL_GRID    0xFF475569
#define COL_SUBTEXT 0xFF94A3B8
#define COL_WHITE   0xFFF1F5F9

/* ------------------------------------------------------------------ */
/* Ring buffer for waveform data                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    float values[WAVE_LEN];
    int write_idx;
    int sample_count;
} ring_buffer_t;

/* ================================================================== */
/* Shared display state                                                */
/* ================================================================== */

typedef struct {
    vital_data_t latest;
    uint64_t latest_timestamp_ms;

    ring_buffer_t bpm_buffer;
    ring_buffer_t spo2_buffer;

    pthread_mutex_t lock;
} display_state_t;

static display_state_t g_ds;
static volatile int    g_running = 1;

/* ------------------------------------------------------------------ */
/* Screen globals                                                       */
/* ------------------------------------------------------------------ */

static int      SCREEN_W = 800;
static int      SCREEN_H = 480;
static uint8_t *g_pixels = NULL;
static int      g_stride  = 0;

/* ------------------------------------------------------------------ */
/* Signal handler                                                       */
/* ------------------------------------------------------------------ */

static void stop_app(int sig) { (void)sig; g_running = 0; }

/* ------------------------------------------------------------------ */
/* Monotonic milliseconds                                               */
/* ------------------------------------------------------------------ */

static uint64_t mono_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL +
           (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* ================================================================== */
/* Ring Buffer Operations                                               */
/* ================================================================== */

static void ring_buffer_init(ring_buffer_t *rb)
{
    memset(rb, 0, sizeof(*rb));
}

static void ring_buffer_push(ring_buffer_t *rb, float value)
{
    rb->values[rb->write_idx] = value;
    rb->write_idx = (rb->write_idx + 1) % WAVE_LEN;
    if (rb->sample_count < WAVE_LEN) rb->sample_count++;
}

static void ring_buffer_reset(ring_buffer_t *rb)
{
    memset(rb, 0, sizeof(*rb));
}

/* Get value at logical index (0 = oldest, count-1 = newest) */
static float ring_buffer_get(ring_buffer_t *rb, int idx)
{
    if (idx < 0 || idx >= rb->sample_count) return 0.0f;
    int read_idx = (rb->write_idx - rb->sample_count + idx + WAVE_LEN) % WAVE_LEN;
    return rb->values[read_idx];
}

/* ================================================================== */
/* DATA-FETCH THREAD                                                    */
/* ================================================================== */

typedef struct {
    int sensor_fd;
} fetch_arg_t;

static void *fetch_thread(void *arg)
{
    fetch_arg_t *fa = (fetch_arg_t *)arg;

    int chid = ChannelCreate(0);
    int coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);

    struct sigevent   event;
    timer_t           timer_id;
    struct itimerspec tspec;

    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT,
                     PULSE_CODE_FETCH, 0);
    timer_create(CLOCK_MONOTONIC, &event, &timer_id);

    tspec.it_value.tv_sec     = 0;
    tspec.it_value.tv_nsec    = FETCH_INTERVAL_NS;
    tspec.it_interval.tv_sec  = 0;
    tspec.it_interval.tv_nsec = FETCH_INTERVAL_NS;
    timer_settime(timer_id, 0, &tspec, NULL);

    vital_data_t snap;
    int last_finger = 0;

    while (g_running) {
        struct _pulse pulse;
        if (MsgReceivePulse(chid, &pulse, sizeof(pulse), NULL) == -1) {
            if (errno == EINTR) continue;
            break;
        }
        if (pulse.code != PULSE_CODE_FETCH) continue;

        if (devctl(fa->sensor_fd, DCMD_VITAL_READ,
                   &snap, sizeof(snap), NULL) != EOK)
            continue;

        uint64_t now = mono_ms();

        pthread_mutex_lock(&g_ds.lock);

        g_ds.latest = snap;
        g_ds.latest_timestamp_ms = now;

        /* Push samples only when finger detected */
        if (snap.finger_detected) {
            if (snap.bpm > 0.0f) {
                ring_buffer_push(&g_ds.bpm_buffer, snap.bpm);
            }
            if (snap.spo2 > 0.0f) {
                ring_buffer_push(&g_ds.spo2_buffer, snap.spo2);
            }
        }

        /* Reset on finger lift */
        if (!snap.finger_detected && last_finger) {
            ring_buffer_reset(&g_ds.bpm_buffer);
            ring_buffer_reset(&g_ds.spo2_buffer);
        }

        pthread_mutex_unlock(&g_ds.lock);

        last_finger = snap.finger_detected;
    }

    timer_delete(timer_id);
    ConnectDetach(coid);
    ChannelDestroy(chid);
    return NULL;
}

/* ================================================================== */
/* SOFTWARE PIXEL RENDERER                                             */
/* ================================================================== */

static inline void put_pixel(int x, int y, uint32_t c)
{
    if ((unsigned)x >= (unsigned)SCREEN_W ||
        (unsigned)y >= (unsigned)SCREEN_H) return;
    uint8_t *p = g_pixels + y * g_stride + x * 4;
    p[0] = (c >> 16) & 0xFF;
    p[1] = (c >>  8) & 0xFF;
    p[2] =  c        & 0xFF;
    p[3] = 0xFF;
}

static void fill_rect(int x, int y, int w, int h, uint32_t c)
{
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++)
            put_pixel(xx, yy, c);
}

static void hline(int x, int y, int w, uint32_t c)
    { for (int i = 0; i < w; i++) put_pixel(x + i, y, c); }

static void vline(int x, int y, int h, uint32_t c)
    { for (int i = 0; i < h; i++) put_pixel(x, y + i, c); }

static void rect_outline(int x, int y, int w, int h, uint32_t c)
{
    hline(x, y,     w, c); hline(x, y+h-1, w, c);
    vline(x, y,     h, c); vline(x+w-1, y, h, c);
}

/* Bresenham line with thickness */
static void draw_thick_line(int x0, int y0, int x1, int y1, uint32_t col, int thickness)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (1) {
        /* Draw circle at current point for thickness */
        for (int yy = y0 - thickness/2; yy <= y0 + thickness/2; yy++) {
            for (int xx = x0 - thickness/2; xx <= x0 + thickness/2; xx++) {
                put_pixel(xx, yy, col);
            }
        }
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

/* ------------------------------------------------------------------ */
/* 5×7 Bitmap font                                                     */
/* ------------------------------------------------------------------ */

static const uint8_t FONT5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x60,0x60,0x00}, /* '.' */
    {0x62,0x64,0x08,0x13,0x23}, /* '%' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
};

static int font_index(char c)
{
    if (c == ' ') return 0;
    if (c == '.') return 1;
    if (c == '%') return 2;
    if (c == ':') return 1;
    if (c >= '0' && c <= '9') return 3  + (c - '0');
    if (c >= 'A' && c <= 'Z') return 13 + (c - 'A');
    if (c >= 'a' && c <= 'z') return 13 + (c - 'a');
    return 0;
}

static void draw_text(int x, int y, int scale, uint32_t col, const char *str)
{
    int cx = x;
    for (; *str; str++) {
        const uint8_t *glyph = FONT5x7[font_index(*str)];
        for (int ci = 0; ci < 5; ci++) {
            uint8_t bits = glyph[ci];
            for (int ri = 0; ri < 7; ri++)
                if (bits & (1 << ri))
                    fill_rect(cx + ci * scale, y + ri * scale,
                              scale, scale, col);
        }
        cx += (5 + 1) * scale;
    }
}

static int text_width(const char *str, int scale)
{
    int len = (int)strlen(str);
    return len ? (5 * len + (len - 1)) * scale : 0;
}

/* ------------------------------------------------------------------ */
/* Large 7-segment digit display                                       */
/* ------------------------------------------------------------------ */

static void draw_digit(int x, int y, int scale, uint32_t col, int num)
{
    int W = 18 * scale, H = 32 * scale, T = 4 * scale;
    static const int8_t seg[10][7] = {
        {1,1,1,1,1,1,0},{0,1,1,0,0,0,0},{1,1,0,1,1,0,1},
        {1,1,1,1,0,0,1},{0,1,1,0,0,1,1},{1,0,1,1,0,1,1},
        {1,0,1,1,1,1,1},{1,1,1,0,0,0,0},{1,1,1,1,1,1,1},
        {1,1,1,1,0,1,1},
    };
    if (num < 0 || num > 9) return;
    if (seg[num][0]) fill_rect(x+T,   y,         W-2*T, T,     col);
    if (seg[num][1]) fill_rect(x+W-T, y+T,       T,     H/2-T, col);
    if (seg[num][2]) fill_rect(x+W-T, y+H/2,     T,     H/2-T, col);
    if (seg[num][3]) fill_rect(x+T,   y+H-T,     W-2*T, T,     col);
    if (seg[num][4]) fill_rect(x,     y+H/2,     T,     H/2-T, col);
    if (seg[num][5]) fill_rect(x,     y+T,       T,     H/2-T, col);
    if (seg[num][6]) fill_rect(x+T,   y+H/2-T/2, W-2*T, T,     col);
}

static void draw_decimal_number(int x, int y, int scale, float value, uint32_t col)
{
    int int_val = (int)value;
    int frac_val = (int)((value - int_val) * 10);
    
    int W = (18 + 10) * scale;
    
    if (int_val >= 100) {
        draw_digit(x,       y, scale, col, int_val / 100);
        draw_digit(x+W,     y, scale, col, (int_val / 10) % 10);
        draw_digit(x+2*W,   y, scale, col, int_val % 10);
        draw_text(x+3*W, y + 16*scale, scale, col, ".");
        draw_digit(x+3*W+8*scale, y, scale, col, frac_val);
    } else {
        draw_digit(x,   y, scale, col, int_val / 10);
        draw_digit(x+W, y, scale, col, int_val % 10);
        draw_text(x+2*W, y + 16*scale, scale, col, ".");
        draw_digit(x+2*W+8*scale, y, scale, col, frac_val);
    }
}

static uint64_t g_start_ms = 0;

static void draw_uptime(int x, int y)
{
    uint64_t elapsed = mono_ms() - g_start_ms;
    unsigned secs  = (elapsed / 1000) % 60;
    unsigned mins  = (elapsed / 60000) % 60;
    unsigned hours = (elapsed / 3600000) % 24;

    char buf[24];
    snprintf(buf, sizeof(buf), "%02u:%02u:%02u", hours, mins, secs);
    draw_text(x, y, 1, COL_SUBTEXT, buf);
}

/* ================================================================== */
/* DRAW SMOOTH WAVEFORM GRAPH                                          */
/* ================================================================== */

static void draw_waveform(int x, int y, int w, int h, const char *title,
                          ring_buffer_t *rb, uint32_t col,
                          float val_min, float val_max)
{
    fill_rect(x, y, w, h, COL_PANEL);
    rect_outline(x, y, w, h, COL_BORDER);
    draw_text(x + 8, y + 6, 1, COL_SUBTEXT, title);

    int sample_count = rb->sample_count;
    
    if (sample_count > 2) {
        int plot_x = x + 35, plot_w = w - 45;
        int plot_y = y + 16,   plot_h = h - 28;

        /* Find min/max in current data */
        float mn = 1e9f, mx = -1e9f;
        for (int i = 0; i < sample_count; i++) {
            float v = ring_buffer_get(rb, i);
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        
        /* Use data range or specified range, whichever is larger */
        float data_rng = mx - mn;
        float spec_rng = val_max - val_min;
        float rng = (data_rng > spec_rng * 0.15f) ? data_rng : spec_rng * 0.2f;
        if (rng < 1.0f) rng = 1.0f;

        /* Center on current data */
        float mid = (mn + mx) / 2.0f;
        mn = mid - rng / 2.0f;
        mx = mid + rng / 2.0f;

        /* Draw Y-axis labels */
        for (int i = 0; i <= 4; i++) {
            int gy = plot_y + plot_h * i / 4;
            float val = mx - (rng * i / 4.0f);
            char label[12];
            snprintf(label, sizeof(label), "%.0f", val);
            draw_text(x + 2, gy - 3, 1, COL_SUBTEXT, label);
            hline(plot_x - 4, gy, 3, COL_BORDER);
        }

        /* Draw Y-axis */
        vline(plot_x - 4, plot_y, plot_h, COL_BORDER);

        /* Draw horizontal grid lines */
        for (int i = 1; i < 4; i++) {
            int gy = plot_y + plot_h * i / 4;
            for (int px = plot_x; px < plot_x + plot_w; px += 8)
                put_pixel(px, gy, COL_GRID);
        }

        /* Draw waveform */
        int prev_px = -1, prev_yy = -1;
        for (int i = 0; i < sample_count; i++) {
            float v = ring_buffer_get(rb, i);
            
            /* Map pixel X (uniform spacing) */
            int px = plot_x + (plot_w * i / sample_count);
            
            /* Map pixel Y (value to screen) */
            float norm = (v - mn) / (mx - mn);
            if (norm < 0.0f) norm = 0.0f;
            if (norm > 1.0f) norm = 1.0f;
            int yy = plot_y + plot_h - 1 - (int)(norm * (plot_h - 1));
            
            if (prev_px >= 0) {
                draw_thick_line(prev_px, prev_yy, px, yy, col, 2);
            }
            
            prev_px = px;
            prev_yy = yy;
        }

        /* Draw X-axis (time) */
        hline(plot_x - 4, plot_y + plot_h, plot_w + 4, COL_BORDER);

        /* Draw time labels */
        for (int i = 0; i <= 4; i++) {
            int tx = plot_x + (plot_w * i / 4);
            int time_sec = (sample_count * i / 4) / 50;  /* 50 Hz sampling */
            char time_label[8];
            snprintf(time_label, sizeof(time_label), "%us", time_sec);
            draw_text(tx - 8, plot_y + plot_h + 3, 1, COL_SUBTEXT, time_label);
            vline(tx, plot_y + plot_h, 3, COL_BORDER);
        }
    } else {
        const char *msg = "WAITING...";
        draw_text(x + (w - text_width(msg, 1)) / 2,
                  y + (h - 7) / 2, 1, COL_SUBTEXT, msg);
    }
}

/* ================================================================== */
/* DRAW DASHBOARD                                                      */
/* ================================================================== */

static void draw_dashboard(const vital_data_t *v)
{
    int finger = v->finger_detected;

    /* Background */
    fill_rect(0, 0, SCREEN_W, SCREEN_H, COL_BG);

    /* ---- HEADER ---- */
    int header_h = 40;
    fill_rect(0, 0, SCREEN_W, header_h, COL_PANEL);
    hline(0, header_h - 1, SCREEN_W, COL_BORDER);

    const char *title = "VITAL SIGNS MONITOR";
    draw_text((SCREEN_W - text_width(title, 2)) / 2,
              (header_h - 14) / 2, 2, COL_WHITE, title);

    uint32_t lamp_col = finger ? COL_GREEN : COL_RED;
    fill_rect(SCREEN_W - PANEL_PAD - 12, (header_h - 12) / 2, 12, 12, lamp_col);

    /* ---- VITAL SIGNS ---- */
    int vitals_y = header_h + PANEL_PAD;
    int vitals_h = 90;
    int half_w = SCREEN_W / 2;

    /* BPM Panel */
    int bpm_w = half_w - PANEL_PAD - 8;
    fill_rect(PANEL_PAD, vitals_y, bpm_w, vitals_h, COL_PANEL);
    rect_outline(PANEL_PAD, vitals_y, bpm_w, vitals_h, COL_BORDER);
    draw_text(PANEL_PAD + 8, vitals_y + 6, 1, COL_SUBTEXT, "HEART RATE");
    
    uint32_t bpm_col = (v->bpm <= 0.0f)          ? COL_SUBTEXT :
                       (v->bpm < 40 || v->bpm > 130) ? COL_RED     :
                       (v->bpm < 50 || v->bpm > 100) ? COL_YELLOW  : COL_GREEN;
    
    if (v->bpm > 0.0f) {
        draw_decimal_number(PANEL_PAD + 10, vitals_y + 28, 2, v->bpm, bpm_col);
        draw_text(half_w - 50, vitals_y + 60, 1, bpm_col, "BPM");
    } else {
        draw_text(PANEL_PAD + 20, vitals_y + 45, 2, COL_SUBTEXT, "---");
    }

    /* SpO2 Panel */
    int spo2_x = half_w + PANEL_PAD / 2;
    int spo2_w = half_w - PANEL_PAD - 8;
    fill_rect(spo2_x, vitals_y, spo2_w, vitals_h, COL_PANEL);
    rect_outline(spo2_x, vitals_y, spo2_w, vitals_h, COL_BORDER);
    draw_text(spo2_x + 8, vitals_y + 6, 1, COL_SUBTEXT, "BLOOD OXYGEN");
    
    uint32_t spo2_col = (v->spo2 <= 0.0f) ? COL_SUBTEXT :
                        (v->spo2 < 90.0f)  ? COL_RED     :
                        (v->spo2 < 95.0f)  ? COL_YELLOW  : COL_GREEN;
    
    if (v->spo2 > 0.0f) {
        draw_decimal_number(spo2_x + 10, vitals_y + 28, 2, v->spo2, spo2_col);
        draw_text(spo2_x + spo2_w - 50, vitals_y + 60, 1, spo2_col, "%");
    } else {
        draw_text(spo2_x + 20, vitals_y + 45, 2, COL_SUBTEXT, "---");
    }

    /* Uptime */
    draw_uptime(SCREEN_W - 70, vitals_y + 10);

    /* ---- WAVEFORMS ---- */
    int wave_y = vitals_y + vitals_h + PANEL_PAD;
    int wave_h = SCREEN_H - wave_y - PANEL_PAD;
    int wave_w = SCREEN_W / 2 - PANEL_PAD;

    draw_waveform(PANEL_PAD, wave_y, wave_w, wave_h,
                  "BPM WAVEFORM", &g_ds.bpm_buffer,
                  COL_WAVE_BPM, 30.0f, 150.0f);

    draw_waveform(SCREEN_W / 2 + PANEL_PAD / 2, wave_y, wave_w, wave_h,
                  "SPO2 WAVEFORM", &g_ds.spo2_buffer,
                  COL_WAVE_SPO2, 85.0f, 100.0f);
}

/* ================================================================== */
/* RENDER THREAD                                                       */
/* ================================================================== */

typedef struct {
    screen_window_t win;
    screen_buffer_t bufs[2];
} render_arg_t;

static void *render_thread(void *arg)
{
    render_arg_t *ra = (render_arg_t *)arg;
    int cur = 0;

    int chid = ChannelCreate(0);
    int coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);

    struct sigevent   event;
    timer_t           timer_id;
    struct itimerspec tspec;

    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT,
                     PULSE_CODE_RENDER, 0);
    timer_create(CLOCK_MONOTONIC, &event, &timer_id);
    tspec.it_value.tv_sec     = 0;
    tspec.it_value.tv_nsec    = RENDER_INTERVAL_NS;
    tspec.it_interval.tv_sec  = 0;
    tspec.it_interval.tv_nsec = RENDER_INTERVAL_NS;
    timer_settime(timer_id, 0, &tspec, NULL);

    while (g_running) {
        struct _pulse pulse;
        if (MsgReceivePulse(chid, &pulse, sizeof(pulse), NULL) == -1) {
            if (errno == EINTR) continue;
            break;
        }
        if (pulse.code != PULSE_CODE_RENDER) continue;

        if (screen_get_buffer_property_pv(
                ra->bufs[cur], SCREEN_PROPERTY_POINTER,
                (void **)&g_pixels) != 0) continue;
        if (!g_pixels) continue;
        if (screen_get_buffer_property_iv(
                ra->bufs[cur], SCREEN_PROPERTY_STRIDE, &g_stride) != 0)
            continue;

        vital_data_t snap;

        pthread_mutex_lock(&g_ds.lock);
        snap = g_ds.latest;
        pthread_mutex_unlock(&g_ds.lock);

        draw_dashboard(&snap);

        int rect[4] = { 0, 0, SCREEN_W, SCREEN_H };
        screen_post_window(ra->win, ra->bufs[cur], 1, rect, 0);
        cur ^= 1;
    }

    timer_delete(timer_id);
    ConnectDetach(coid);
    ChannelDestroy(chid);
    return NULL;
}

/* ================================================================== */
/* MAIN                                                                 */
/* ================================================================== */

int main(void)
{
    signal(SIGINT,  stop_app);
    signal(SIGTERM, stop_app);

    g_start_ms = mono_ms();

    memset(&g_ds, 0, sizeof(g_ds));
    pthread_mutex_init(&g_ds.lock, NULL);
    ring_buffer_init(&g_ds.bpm_buffer);
    ring_buffer_init(&g_ds.spo2_buffer);

    int sensor_fd = open(VITAL_SENSOR_PATH, O_RDONLY);
    if (sensor_fd < 0) {
        perror("open " VITAL_SENSOR_PATH);
        fprintf(stderr, "Is vital_resmgr running?\n");
        return EXIT_FAILURE;
    }

    fetch_arg_t fa = { .sensor_fd = sensor_fd };
    pthread_t fetch_tid;
    if (pthread_create(&fetch_tid, NULL, fetch_thread, &fa) != 0) {
        perror("pthread_create fetch");
        return EXIT_FAILURE;
    }

    screen_context_t ctx;
    screen_window_t  win;
    screen_buffer_t  bufs[2];

    if (screen_create_context(&ctx, SCREEN_APPLICATION_CONTEXT) != 0)
        return EXIT_FAILURE;
    if (screen_create_window(&win, ctx) != 0)
        return EXIT_FAILURE;

    {
        screen_display_t disp = NULL;
        screen_get_context_property_pv(ctx, SCREEN_PROPERTY_DISPLAY,
                                       (void **)&disp);
        if (disp) {
            int ds[2] = {0, 0};
            screen_get_display_property_iv(disp, SCREEN_PROPERTY_SIZE, ds);
            if (ds[0] > 0 && ds[1] > 0) { SCREEN_W = ds[0]; SCREEN_H = ds[1]; }
        }
    }

    int format = SCREEN_FORMAT_RGBX8888;
    int usage  = SCREEN_USAGE_WRITE | SCREEN_USAGE_NATIVE;
    screen_set_window_property_iv(win, SCREEN_PROPERTY_FORMAT, &format);
    screen_set_window_property_iv(win, SCREEN_PROPERTY_USAGE,  &usage);

    int size[2] = { SCREEN_W, SCREEN_H };
    screen_set_window_property_iv(win, SCREEN_PROPERTY_SIZE, size);
    int pos[2]  = { 0, 0 };
    screen_set_window_property_iv(win, SCREEN_PROPERTY_POSITION, pos);
    screen_set_window_property_iv(win, SCREEN_PROPERTY_SOURCE_SIZE, size);

    if (screen_create_window_buffers(win, 2) != 0) { perror("buffers"); return 1; }
    if (screen_get_window_property_pv(win, SCREEN_PROPERTY_RENDER_BUFFERS,
                                      (void **)bufs) != 0) return 1;

    render_arg_t ra = { .win = win };
    ra.bufs[0] = bufs[0]; ra.bufs[1] = bufs[1];
    pthread_t render_tid;
    pthread_create(&render_tid, NULL, render_thread, &ra);

    while (g_running) pause();

    pthread_join(render_tid, NULL);
    pthread_join(fetch_tid, NULL);

    screen_destroy_window(win);
    screen_destroy_context(ctx);
    pthread_mutex_destroy(&g_ds.lock);
    close(sensor_fd);

    return EXIT_SUCCESS;
}