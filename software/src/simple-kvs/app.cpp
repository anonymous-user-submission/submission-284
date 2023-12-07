#include "app.h"

extern volatile bool force_quit;


uint32_t app_thrd::do_rx() {
    uint32_t n_rx = 0;

    if (_worker_ring->n_empty_slots < APP_RX_BATCH_SIZE) {
        return n_rx;
    }

    uint32_t rx_batch_size = 0;
    uint32_t rounded_size = WORKER_RING_SIZE - _worker_ring->ring_head;
    rx_batch_size = rounded_size < APP_RX_BATCH_SIZE ? rounded_size : APP_RX_BATCH_SIZE;

    rx_batch_size = rx_batch_size < _worker_ring->n_empty_slots ? rx_batch_size : _worker_ring->n_empty_slots;

    if (rx_batch_size > 0) {
        n_rx = app_rx_batch(_rmtp, _rx_mbufs+_worker_ring->ring_head, rx_batch_size);

        if (n_rx > 0) {
            _worker_ring->ring_head = (_worker_ring->ring_head + n_rx) % WORKER_RING_SIZE;
            _worker_ring->unprocessed_work_cnt += n_rx;
            _worker_ring->n_empty_slots -= n_rx;
        }

    }

    return n_rx;
}


void app_thrd::do_work() {

    std::string val;

    struct rte_ether_hdr *eth_h;
    struct rte_ipv4_hdr *ipv4_h;
    struct rte_udp_hdr *udp_h;
    struct rmtp_msg_hdr *rmtp_h;

    if (_worker_ring->unprocessed_work_cnt > 0) {
        uint32_t work_size = get_proc_size(_worker_ring->ring_head,
                                            _worker_ring->work_head,
                                            WORKER_RING_SIZE,
                                            APP_DO_WORK_THRESH);

        // do the work
        for (auto i=0; i<work_size; i++) {
            uint32_t ind = _worker_ring->work_head + i;
            eth_h = rte_pktmbuf_mtod(_rx_mbufs[ind], struct rte_ether_hdr *);
            ipv4_h = (struct rte_ipv4_hdr *)(eth_h + 1);
            udp_h = (struct rte_udp_hdr *)(ipv4_h + 1);
            rmtp_h = (struct rmtp_msg_hdr *)(udp_h + 1);

#ifdef DBSUPPORT
            db_get(_db, (uint8_t *)(rmtp_h+1) + 2, _rx_mbufs[ind]->pkt_len-64-2, &val);
            memcpy(_buf[ind], val.c_str(), val.size());
#endif
            _buf_len[ind] = 64;
            _dst_qids[ind] = ((rmtp_h->msg_id) >> MSG_ID_RANGE_RSHIFT) & 0xf;
            _is_ack[ind] = false;
        }
        rte_pktmbuf_free_bulk(_rx_mbufs+_worker_ring->work_head, work_size);

        _worker_ring->unprocessed_work_cnt -= work_size;
        _worker_ring->unsent_work_cnt += work_size;
        _worker_ring->work_head = (_worker_ring->work_head + work_size) % WORKER_RING_SIZE;
    }
}

uint32_t app_thrd::do_tx() {
    uint32_t n_tx = 0;
    if (_worker_ring->unsent_work_cnt < APP_TX_BATCH_SIZE) {
        return n_tx;
    }

    uint32_t tx_batch_size = get_proc_size(_worker_ring->work_head,
                                            _worker_ring->ring_tail,
                                            WORKER_RING_SIZE,
                                            APP_TX_BATCH_SIZE);

    if (tx_batch_size > 0) {
        uint32_t ind = _worker_ring->ring_tail;
        n_tx = app_tx_batch(_rmtp,
                            _buf + ind,
                            _buf_len + ind,
                            _dst_qids + ind,
                            _is_ack + ind,
                            tx_batch_size);

        if (n_tx > 0) {
            _worker_ring->ring_tail = (_worker_ring->ring_tail + n_tx) % WORKER_RING_SIZE;
            _worker_ring->unsent_work_cnt -= n_tx;
            _worker_ring->n_empty_slots += n_tx;
        }
    }

    return n_tx;
}


void app_thrd::run_app() {
    uint32_t rx_cnt = 0, tx_cnt = 0;

    rx_cnt = do_rx();
    do_work();
    tx_cnt = do_tx();

    _worker_ring->tot_rx_cnt += rx_cnt;
    _worker_ring->tot_tx_cnt += tx_cnt;
}
