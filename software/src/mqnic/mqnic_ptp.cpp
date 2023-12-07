#include <cstdint>
#include <ctime>

#include "mqnic.h"
#include "io.h"
#include "memory.h"
#include "mqnic_hw.h"

/*
 *  PID algorithm to get period converged
 *  period --> every ptp_clk, period time increased
 *  nominal period --> ptp_clk
 */

static inline void mqnic_set_phc_reg_period_fns(struct mqnic_dev *mqnic, uint32_t val) {
    iowrite32(val, mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_PERIOD_FNS);
}

static inline void mqnic_set_phc_reg_period_ns(struct mqnic_dev *mqnic, uint32_t val) {
    iowrite32(val, mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_PERIOD_NS);
}

static inline void mqnic_phc_settime(struct mqnic_dev *mqnic, const struct timespec *ts)
{
    iowrite32(0, mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_SET_FNS);
    iowrite32(ts->tv_nsec, mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_SET_NS);
    iowrite32(ts->tv_sec & 0xffffffff, mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_SET_SEC_L);
    iowrite32(ts->tv_sec >> 32, mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_SET_SEC_H);
}


static inline void mqnic_phc_gettime(struct mqnic_dev *mqnic, struct timespec *ts)
{
    ioread32(mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_GET_FNS);
    ts->tv_nsec = ioread32(mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_GET_NS);
    ts->tv_sec = ioread32(mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_GET_SEC_L);
    ts->tv_sec |= (uint64_t)(ioread32(mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_GET_SEC_H)) << 32;
    // ts->tv_nsec = ioread32(mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_CUR_NS);
    // ts->tv_sec = ioread32(mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_CUR_SEC_L);
    // ts->tv_sec |= (uint64_t)(ioread32(mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_CUR_SEC_H)) << 32;
}

static inline void mqnic_phc_adjfine(struct mqnic_dev *mqnic, uint64_t nom_per_fns, long scaled_ppm)
{
    bool neg = false;
    uint64_t adj;

    if (scaled_ppm < 0) {
        neg = true;
        scaled_ppm = -scaled_ppm;
    }

    if (nom_per_fns == 0)
        nom_per_fns = 0x4ULL << 32;

    adj = (((nom_per_fns >> 16) * scaled_ppm) + 500000)/1000000;

    if (neg)
        adj = nom_per_fns - adj;
    else
        adj = nom_per_fns + adj;

    mqnic_set_phc_reg_period_fns(mqnic, adj & 0xffffffff);
    mqnic_set_phc_reg_period_ns(mqnic, adj >> 32);
}

struct timespec mqnic_read_cpl_ts(struct mqnic_dev *mqnic, struct mqnic_ring *ring,
                const struct mqnic_cpl *cpl)
{
    uint64_t ts_s = le16_to_cpu(cpl->ts_s);
    uint32_t ts_ns = le32_to_cpu(cpl->ts_ns);

    if (!ring->ts_valid || (ring->ts_s ^ ts_s) & 0xff00) { // seconds MSBs do not match, update cached timestamp
        if (mqnic->phc_rb) {
            ring->ts_s = ioread32(mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_CUR_SEC_L);
            ring->ts_s |= (uint64_t)ioread32(mqnic->phc_rb->regs + MQNIC_RB_PHC_REG_CUR_SEC_H) << 32;
            ring->ts_valid = 1;
        }
    }
    ts_s |= ring->ts_s & 0xffffffffffffff00;
    struct timespec ret;
    ret.tv_sec = ts_s;
    ret.tv_nsec = ts_ns;
    return ret;
}

void start_sync_fpga_time(struct mqnic_dev *mqnic) {
    const int P = 256;
    const int I = 64;

    struct timespec host_ts, nic_ts;
    uint64_t nom_per_fns;

    nom_per_fns = get_reg32(mqnic->phc_rb->regs, MQNIC_RB_PHC_REG_NOM_PERIOD_FNS);
    nom_per_fns |= (uint64_t)(get_reg32(mqnic->phc_rb->regs, MQNIC_RB_PHC_REG_NOM_PERIOD_NS)) << 32;

    // get system time, and set it
    clock_gettime(CLOCK_REALTIME, &host_ts);
    mqnic_phc_settime(mqnic, &host_ts);

    // 
    int32_t cur_error=0, sum_error=0, scaled_ppm=0;
    // auto cnt=0;
    while (true) {
        usleep(100000); // 100 ms
        clock_gettime(CLOCK_REALTIME, &host_ts);
        mqnic_phc_gettime(mqnic, &nic_ts);

        cur_error = (int32_t)((int32_t)nic_ts.tv_sec - (int32_t)host_ts.tv_sec)*1e9 + (int32_t)(nic_ts.tv_nsec - host_ts.tv_nsec);
        sum_error += cur_error;

        /*
        cnt++;
        if (cnt == 50) {
            debug("nic ts %lu.%lu", nic_ts.tv_sec, nic_ts.tv_nsec);
            debug("host ts %lu.%lu", host_ts.tv_sec, host_ts.tv_nsec);
            debug("cur_error: %d", cur_error);
            cnt = 0;
        }
        */

        if (cur_error > 1000000000 || cur_error < -1000000000) {
            sum_error = 0;
            mqnic_phc_settime(mqnic, &host_ts);
        }
        else {
            scaled_ppm = -(cur_error*P) - (sum_error*I);
            mqnic_phc_adjfine(mqnic, nom_per_fns, scaled_ppm);
        }
    }
}
