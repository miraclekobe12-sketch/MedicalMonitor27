/*
 * vital_resmgr.c
 *
 *
 * Build:
 * qcc -o vital_resmgr vital_resmgr.c -lm -pthread
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <pthread.h>

#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include <time.h>
#include <sys/siginfo.h>

#include <devctl.h>
#include <hw/i2c.h>

#include "vitals.h"

/* -------------------------------------------------- */
/* CONFIG                                             */
/* -------------------------------------------------- */

#define I2C_BUS             "/dev/i2c1"
#define MAX30102_ADDR       0x57

#define SAMPLE_INTERVAL_NS  20000000LL
#define FINGER_THRESHOLD    5000

#define BPM_AVG_SIZE        8
#define SPO2_WINDOW         100

#define PULSE_CODE_SAMPLE   1

/* -------------------------------------------------- */

typedef struct { i2c_sendrecv_t hdr; uint8_t buf[8]; } rw_t;
typedef struct { i2c_send_t    hdr; uint8_t buf[8]; } w_t;

static volatile int g_running = 1;
static int g_i2c_fd = -1;

static vital_data_t g_vitals;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

/* -------------------------------------------------- */

static uint64_t mono_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL +
           (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* -------------------------------------------------- */
/* I2C                                                */
/* -------------------------------------------------- */

static int reg_write(uint8_t reg, uint8_t val)
{
    w_t m; int status;
    memset(&m,0,sizeof(m));

    m.buf[0]=reg;
    m.buf[1]=val;

    m.hdr.slave.addr=MAX30102_ADDR;
    m.hdr.slave.fmt=I2C_ADDRFMT_7BIT;
    m.hdr.len=2;
    m.hdr.stop=1;

    return devctl(g_i2c_fd,DCMD_I2C_SEND,&m,sizeof(m),&status);
}

static int reg_read(uint8_t reg,uint8_t *val)
{
    rw_t m; int status;
    memset(&m,0,sizeof(m));

    m.buf[0]=reg;

    m.hdr.slave.addr=MAX30102_ADDR;
    m.hdr.slave.fmt=I2C_ADDRFMT_7BIT;
    m.hdr.send_len=1;
    m.hdr.recv_len=1;
    m.hdr.stop=1;

    if(devctl(g_i2c_fd,DCMD_I2C_SENDRECV,&m,sizeof(m),&status)!=0)
        return -1;

    *val=m.buf[0];
    return 0;
}

static int fifo_read(uint32_t *red,uint32_t *ir)
{
    rw_t m; int status;
    memset(&m,0,sizeof(m));

    m.buf[0]=0x07;

    m.hdr.slave.addr=MAX30102_ADDR;
    m.hdr.slave.fmt=I2C_ADDRFMT_7BIT;
    m.hdr.send_len=1;
    m.hdr.recv_len=6;
    m.hdr.stop=1;

    if(devctl(g_i2c_fd,DCMD_I2C_SENDRECV,&m,sizeof(m),&status)!=0)
        return -1;

    *red = (((uint32_t)m.buf[0]<<16) |
            ((uint32_t)m.buf[1]<<8 ) |
             (uint32_t)m.buf[2]) & 0x3FFFF;

    *ir  = (((uint32_t)m.buf[3]<<16) |
            ((uint32_t)m.buf[4]<<8 ) |
             (uint32_t)m.buf[5]) & 0x3FFFF;

    return 0;
}

/* -------------------------------------------------- */

static int sensor_init(void)
{
    uint8_t v=0;
    int i;

    reg_write(0x09,0x40);
    usleep(100000);

    for(i=0;i<50;i++)
    {
        if(reg_read(0x09,&v)==0 && !(v&0x40))
            break;
        usleep(10000);
    }

    if(i==50) return -1;

    reg_write(0x08,0x5F);
    reg_write(0x09,0x03);
    reg_write(0x0A,0x27);
    reg_write(0x0C,0x24);
    reg_write(0x0D,0x24);

    return 0;
}

/* -------------------------------------------------- */
/* BPM LOGIC                                          */
/* -------------------------------------------------- */

typedef struct
{
    float dc;
    float prev;
    float prev2;

    float env;

    int armed;

    uint64_t last_peak;

    float hist[BPM_AVG_SIZE];
    int idx;
    int count;

    float bpm;

} bpm_state_t;

static void bpm_reset(bpm_state_t *b)
{
    memset(b,0,sizeof(*b));
    b->armed=1;
}

static void bpm_update(bpm_state_t *b,uint32_t ir_raw)
{
    b->dc = 0.97f*b->dc + 0.03f*(float)ir_raw;

    float x = (float)ir_raw - b->dc;

    float y = 0.7f*x + 0.3f*b->prev;

    b->env = 0.95f*b->env + 0.05f*fabsf(y);

    float thresh_hi = b->env * 0.65f;
    float thresh_lo = b->env * 0.30f;

    if(thresh_hi < 120.0f) thresh_hi = 120.0f;
    if(thresh_lo < 60.0f ) thresh_lo = 60.0f;

    int peak =
        (b->prev > b->prev2) &&
        (b->prev > y) &&
        (b->prev > thresh_hi);

    if(b->armed && peak)
    {
        uint64_t now = mono_ms();
        uint64_t dt  = now - b->last_peak;

        if(dt > 350 && dt < 1800)
        {
            float inst = 60000.0f / (float)dt;

            if(b->bpm == 0.0f || fabsf(inst - b->bpm) < 25.0f)
            {
                b->hist[b->idx++] = inst;
                b->idx %= BPM_AVG_SIZE;

                if(b->count < BPM_AVG_SIZE)
                    b->count++;

                float sum = 0.0f;
                for(int i=0;i<b->count;i++)
                    sum += b->hist[i];

                b->bpm = sum / (float)b->count;
            }
        }

        b->last_peak = now;
        b->armed = 0;
    }

    if(!b->armed && y < thresh_lo)
        b->armed = 1;

    if(b->last_peak != 0)
    {
        uint64_t now = mono_ms();
        if(now - b->last_peak > 3000)
            b->bpm = 0.0f;
    }

    b->prev2 = b->prev;
    b->prev  = y;
}

/* -------------------------------------------------- */
/* SPO2                                               */
/* -------------------------------------------------- */

typedef struct
{
    float red[SPO2_WINDOW];
    float ir[SPO2_WINDOW];
    int idx;
    int full;
    float spo2;
} spo2_state_t;

static void spo2_reset(spo2_state_t *s)
{
    memset(s,0,sizeof(*s));
}

static void spo2_update(spo2_state_t *s,uint32_t red,uint32_t ir)
{
    s->red[s->idx]=(float)red;
    s->ir [s->idx]=(float)ir;

    s->idx=(s->idx+1)%SPO2_WINDOW;
    if(s->idx==0) s->full=1;

    if(!s->full) return;

    float red_dc=0,ir_dc=0;

    for(int i=0;i<SPO2_WINDOW;i++)
    {
        red_dc+=s->red[i];
        ir_dc +=s->ir[i];
    }

    red_dc/=SPO2_WINDOW;
    ir_dc /=SPO2_WINDOW;

    float red_ac=0,ir_ac=0;

    for(int i=0;i<SPO2_WINDOW;i++)
    {
        red_ac += fabsf(s->red[i]-red_dc);
        ir_ac  += fabsf(s->ir[i]-ir_dc);
    }

    red_ac/=SPO2_WINDOW;
    ir_ac /=SPO2_WINDOW;

    if(ir_dc < 1 || ir_ac < 1) return;

    float R=(red_ac/red_dc)/(ir_ac/ir_dc);

    float val=104.0f - 17.0f*R;

    if(val>100) val=100;
    if(val<70)  val=70;

    s->spo2=val;
}

/* -------------------------------------------------- */
/* THREAD                                             */
/* -------------------------------------------------- */

static void *sampler_thread(void *arg)
{
    (void)arg;

    bpm_state_t bpm;
    spo2_state_t spo2;

    bpm_reset(&bpm);
    spo2_reset(&spo2);

    int last_finger=0;

    int chid=ChannelCreate(0);
    int coid=ConnectAttach(0,0,chid,_NTO_SIDE_CHANNEL,0);

    struct sigevent event;
    timer_t timer_id;
    struct itimerspec tspec;

    SIGEV_PULSE_INIT(&event,coid,SIGEV_PULSE_PRIO_INHERIT,
                     PULSE_CODE_SAMPLE,0);

    timer_create(CLOCK_MONOTONIC,&event,&timer_id);

    tspec.it_value.tv_sec=0;
    tspec.it_value.tv_nsec=SAMPLE_INTERVAL_NS;
    tspec.it_interval=tspec.it_value;

    timer_settime(timer_id,0,&tspec,NULL);

    while(g_running)
    {
        struct _pulse pulse;

        if(MsgReceivePulse(chid,&pulse,sizeof(pulse),NULL)==-1)
            continue;

        if(pulse.code!=PULSE_CODE_SAMPLE)
            continue;

        uint32_t red=0,ir=0;

        if(fifo_read(&red,&ir)!=0)
            continue;

        int finger = (ir >= FINGER_THRESHOLD);

        if(!finger && last_finger)
        {
            bpm_reset(&bpm);
            spo2_reset(&spo2);
        }

        if(finger)
        {
            bpm_update(&bpm,ir);
            spo2_update(&spo2,red,ir);
        }

        pthread_mutex_lock(&g_lock);

        g_vitals.bpm = finger ? bpm.bpm : 0.0f;
        g_vitals.spo2 = finger ? spo2.spo2 : 0.0f;
        g_vitals.finger_detected = finger;
        g_vitals.red_raw = red;
        g_vitals.ir_raw = ir;
        g_vitals.timestamp_ms = mono_ms();

        pthread_mutex_unlock(&g_lock);

        last_finger=finger;
    }

    timer_delete(timer_id);
    ConnectDetach(coid);
    ChannelDestroy(chid);

    return NULL;
}

/* -------------------------------------------------- */
/* RESOURCE MANAGER                                   */
/* -------------------------------------------------- */

static iofunc_attr_t g_attr;
static resmgr_attr_t g_resmgr_attr;
static dispatch_t *g_dpp;

static int io_read(resmgr_context_t *ctp,io_read_t *msg,iofunc_ocb_t *ocb)
{
    if(msg->i.nbytes < sizeof(vital_data_t))
        return EINVAL;

    vital_data_t snap;

    pthread_mutex_lock(&g_lock);
    snap=g_vitals;
    pthread_mutex_unlock(&g_lock);

    MsgReply(ctp->rcvid,sizeof(snap),&snap,sizeof(snap));

    return _RESMGR_NOREPLY;
}

static int io_devctl(resmgr_context_t *ctp,io_devctl_t *msg,
                     iofunc_ocb_t *ocb)
{
    int status=EOK;
    void *data=_DEVCTL_DATA(msg->i);

    switch(msg->i.dcmd)
    {
        case DCMD_VITAL_READ:
        {
            vital_data_t snap;

            pthread_mutex_lock(&g_lock);
            snap=g_vitals;
            pthread_mutex_unlock(&g_lock);

            memcpy(data,&snap,sizeof(snap));
            msg->o.ret_val=EOK;
            msg->o.nbytes=sizeof(snap);
            break;
        }

        default:
            status=ENOSYS;
            break;
    }

    return _RESMGR_PTR(ctp,&msg->o,sizeof(msg->o)+msg->o.nbytes);
}

/* -------------------------------------------------- */

static void sig_handler(int sig)
{
    (void)sig;
    g_running=0;
}

/* -------------------------------------------------- */

int main(void)
{
    signal(SIGINT,sig_handler);
    signal(SIGTERM,sig_handler);

    g_i2c_fd=open(I2C_BUS,O_RDWR);
    if(g_i2c_fd<0)
    {
        perror("open i2c");
        return EXIT_FAILURE;
    }

    if(sensor_init()!=0)
    {
        printf("sensor init failed\n");
        return EXIT_FAILURE;
    }

    pthread_t tid;
    pthread_create(&tid,NULL,sampler_thread,NULL);

    g_dpp=dispatch_create();

    memset(&g_resmgr_attr,0,sizeof(g_resmgr_attr));
    g_resmgr_attr.nparts_max=1;
    g_resmgr_attr.msg_max_size=2048;

    resmgr_connect_funcs_t connect_funcs;
    resmgr_io_funcs_t io_funcs;

    iofunc_func_init(_RESMGR_CONNECT_NFUNCS,&connect_funcs,
                     _RESMGR_IO_NFUNCS,&io_funcs);

    io_funcs.read=io_read;
    io_funcs.devctl=io_devctl;

    iofunc_attr_init(&g_attr,S_IFNAM|0666,NULL,NULL);

    if(resmgr_attach(g_dpp,&g_resmgr_attr,
                     VITAL_SENSOR_PATH,
                     _FTYPE_ANY,0,
                     &connect_funcs,&io_funcs,
                     &g_attr)==-1)
    {
        perror("resmgr_attach");
        return EXIT_FAILURE;
    }

    printf("vital_resmgr running\n");

    dispatch_context_t *ctp=dispatch_context_alloc(g_dpp);

    while(g_running)
    {
        ctp=dispatch_block(ctp);
        if(ctp) dispatch_handler(ctp);
    }

    pthread_join(tid,NULL);
    close(g_i2c_fd);

    return EXIT_SUCCESS;
}