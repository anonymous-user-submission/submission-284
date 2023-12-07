#ifndef __RMTP_H__
#define __RMTP_H__

#include <cstdint>
#include <memory>
#include <deque>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <array>

// DPDK
#include <rte_common.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_memcpy.h>
#include <rte_memory.h>
#include <rte_eal.h>
#include <rte_launch.h>
#include <rte_cycles.h>
#include <rte_prefetch.h>
#include <rte_lcore.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_string_fns.h>

#include "common/app_thrd_info.h"
#include "common/log.h"
#include "mqnic/mqnic.h"
#include "circular_buf.h"

#define CHUNK_SIZE 128
#define MAX_NUM_PKTSEG 8
#define RING_BUF_SIZE CIRCULAR_BUF_SIZE
#define MAX_NUM_SENDER 4
// #define RTO_IN_NS 10000000000 // 10s

namespace rmtp {

class rmtp;


class mcb { // message control block
public:
    struct send {
        struct rte_mbuf * tx_pkts[MAX_NUM_PKTSEG];
        uint8_t msg_len_in_pkts;
        uint8_t msg_pkt_ind;
    } _snd;

    struct receive {
        struct rte_mbuf * recv_pkts[MAX_NUM_PKTSEG]; // offset --> pkt
        uint8_t msg_len_in_pkts;
        uint8_t msg_recv_offsets;
        uint8_t msg_target_offsets;
    } _rcv;

    mcb(rmtp *r): _rmtp(r) {
        _tx_finished = false;
    }

    ~mcb() {
        free_mcb_mbufs();
    }

    void free_mcb_mbufs();

    void reset_tx(uint8_t msg_len_in_pkts) {
        _tx_finished = false;

        _snd.msg_len_in_pkts = msg_len_in_pkts;
        _snd.msg_pkt_ind = 0;
    }

    struct rte_mbuf * get_packet(uint64_t curr_cycles);

    rmtp * _rmtp;
    bool _tx_finished;

}; // class mcb


class rmtp {
public:

    rmtp (uint16_t qid,
            uint32_t msg_id_st,
            struct rte_mempool *tx_mp,
            struct mqnic_if *interface,
            rx_burst_t rx_fn,
            tx_burst_t tx_fn) {
        _qid = qid;
        _msg_id_st = msg_id_st;
        _msg_id = 0;
        _tx_mp = tx_mp;
        _if = interface;
        _rx_fn = rx_fn;
        _tx_fn = tx_fn;


        _rcv_mcbs.clear();

        _tx_mcbs = new circular_buf<mcb *>(RING_BUF_SIZE);

        for (auto i=0; i<MAX_NUM_SENDER; i++) {
            _unsent_ack_msg_ids[i].clear();
        }

        _available_tx_mcbs = new circular_buf<mcb *>(RING_BUF_SIZE);
        _available_rx_mcbs = new circular_buf<mcb *>(RING_BUF_SIZE);

        for (auto i=0; i<RING_BUF_SIZE-1; i++) {
            auto tx_mcb = new mcb(this);
            _available_tx_mcbs->enq(tx_mcb);
            auto rx_mcb = new mcb(this);
            _available_rx_mcbs->enq(rx_mcb);
        }

        _tx_head = _tx_tail = _tx_n_elements = 0;
        _tx_empty_slots = RING_BUF_SIZE - 1;

        _rx_head = _rx_tail = _rx_n_elements = 0;
        _rx_empty_slots = RING_BUF_SIZE - 1;

        _rx_to_app_buf = new circular_buf<struct rte_mbuf *>(RING_BUF_SIZE);
    }

    ~rmtp() {
        // TODO:
    }

    uint16_t _qid;
    uint32_t _msg_id;
    uint32_t _msg_id_st;
    struct rte_mempool *_tx_mp;
    struct mqnic_if *_if;
    rx_burst_t _rx_fn;
    tx_burst_t _tx_fn;

    // NOTE: server side does not need ACK
    std::unordered_map<uint32_t, mcb *> _rcv_mcbs;

    circular_buf<mcb *> *_tx_mcbs;
    std::array<std::deque<uint32_t>, MAX_NUM_SENDER> _unsent_ack_msg_ids;

    circular_buf<mcb *> *_available_tx_mcbs;
    circular_buf<mcb *> *_available_rx_mcbs;

    struct rte_mbuf *_tx_buf[RING_BUF_SIZE];
    uint32_t _tx_head, _tx_tail, _tx_empty_slots, _tx_n_elements;

    struct rte_mbuf *_rx_buf[RING_BUF_SIZE];
    uint32_t _rx_head, _rx_tail, _rx_empty_slots, _rx_n_elements;

    circular_buf<struct rte_mbuf *> *_rx_to_app_buf;

    uint32_t _tot_tx_cnt;
    uint32_t _tot_rx_cnt;

    size_t tx_batch(char *buf[], size_t buf_len[], uint8_t dst_qid[], bool ack_required[], size_t n);
    void poll_tx();
    void poll_rx();
    void do_work();
    bool process_rx_mbuf(struct rte_mbuf *mbuf);
    void run();

}; // class rmtp

} // namespace rmtp








#endif
