#ifndef __APP_H__
#define __APP_H__

#include <cstdint>
#include <cstring>
#include <atomic>

#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/ether.h>
#include <netinet/ip.h>
#include <netinet/udp.h>

#include "mqnic/mqnic.h"
#include "common/app_thrd_info.h"
#include "common/log.h"

#include "api.h"
#include "rmtp.h"

#define WORKER_RING_SIZE 128

struct worker_ring {
    uint32_t ring_head;
    uint32_t work_head;
    uint32_t work_tail;
    uint32_t ring_tail;

    uint32_t ring_size;
    uint32_t n_empty_slots;
    uint32_t unprocessed_work_cnt;
    uint32_t unsent_work_cnt;

    uint32_t tot_tx_cnt;
    uint32_t tot_rx_cnt;
};

inline uint32_t get_proc_size (uint32_t head, uint32_t tail, uint32_t size, uint32_t thresh) {
    uint32_t ret = 0;
    if (head >= tail) {
        ret = head - tail;
    } else {
        // NOTE: it rounds up
        ret = size - tail;
    }
    ret = ret < thresh ? ret : thresh;
    return ret;
}

class app_thrd {
public:

    app_thrd (class rmtp::rmtp *rmtp_ptr,
            nullptr_t *db,
            struct mqnic_if *interface,
            uint32_t qid,
            rx_burst_t rx_fn,
            tx_burst_t tx_fn)
        : _rmtp(rmtp_ptr), _db(db), _if(interface), _qid(qid), _rx_fn(rx_fn), _tx_fn(tx_fn) {
        // init worker ring
        _worker_ring = new struct worker_ring;
        *_worker_ring = {
            .ring_head = 0,
            .work_head = 0,
            .work_tail = 0,
            .ring_tail = 0,
            .ring_size = WORKER_RING_SIZE - 1,
            .n_empty_slots = WORKER_RING_SIZE - 1,
            .unprocessed_work_cnt = 0,
            .unsent_work_cnt = 0,
            .tot_tx_cnt = 0,
            .tot_rx_cnt = 0,
        };

        // init _buf
        for (auto i=0; i<WORKER_RING_SIZE; i++) {
            _buf[i] = new char[256];
        }
    }

    ~app_thrd () {
        if (_worker_ring) {
            delete _worker_ring;
        }

        for (auto i=0; i<WORKER_RING_SIZE; i++) {
            if (_buf[i]) {
                delete [] _buf[i];
            }
        }
    }

    uint32_t do_rx();
    virtual void do_work();
    virtual uint32_t do_tx();
    void run_app();

    class rmtp::rmtp* _rmtp;
    nullptr_t *_db;
    struct mqnic_if *_if;
    uint32_t _qid;
    rx_burst_t _rx_fn;
    tx_burst_t _tx_fn;

    // auxilliary data structures
    struct worker_ring *_worker_ring;
    struct rte_mbuf *_rx_mbufs[WORKER_RING_SIZE];
    char *_buf[WORKER_RING_SIZE];
    size_t _buf_len[WORKER_RING_SIZE];
    uint8_t _dst_qids[WORKER_RING_SIZE];
    bool _is_ack[WORKER_RING_SIZE];
};

#endif
