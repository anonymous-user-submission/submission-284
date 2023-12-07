

#include "rmtp.h"
#include "app_thrd_info.h"

namespace rmtp {

static const uint32_t dst_qid_to_dst_ip_addr[] = {
    234881034, // "10.0.0.E"
    100663306, // "10.0.0.6"
    218103818, // "10.0.0.D"
    167772170, // "10.0.0.A"
};

// static const uint32_t dst_qid_to_dst_ip_addr[] = {
//     16843028, // "20.1.1.1"
//     67108874, // "10.0.0.4" 20.1.2.2
//     67371284, // "20.1.4.4"
//     100663306, // "10.0.0.6"
//     33554442, // "10.0.0.2"
//     168430090, // "10.10.10.10"
// };

static const uint8_t pkt_data[] = {
    0x20, 0x20, 0x20, 0x20, 0x20, 0x20, // dst MAC
    0xb8, 0xce, 0xf6, 0x08, 0x32, 0xd5, // src MAC
    0x08, 0x00,                         // ether type: IPv4
    0x45, 0x00,                         // Version, IHL, TOS
    (DEFAULT_PKTHDR_SIZE - 14) >> 8,               // ip len excluding ethernet, high byte
    (DEFAULT_PKTHDR_SIZE - 14) & 0xFF,             // ip len exlucding ethernet, low byte
    0x00, 0x00, 0x00, 0x00,             // id, flags, fragmentation
    0x40, 0x11, 0x00, 0x00,             // TTL (64), protocol (UDP), checksum
    0x0A, 0x00, 0x00, 0x01,             // src ip (10.0.0.1)
    0x0A, 0x00, 0x00, 0x02,             // dst ip (10.0.0.2)
    0x00, 0x2A, 0x05, 0x39,             // src and dst ports (42 -> 1337)
    (DEFAULT_PKTHDR_SIZE - 20 - 14) >> 8,          // udp len excluding ip & ethernet, high byte
    (DEFAULT_PKTHDR_SIZE - 20 - 14) & 0xFF,        // udp len exlucding ip & ethernet, low byte
    0x00, 0x00,                         // udp checksum, optional
    // rest of the payload is zero-filled because mempools guarantee empty bufs
};

inline void prepare_mbuf(char *buf, size_t len, uint8_t dst_qid,
                            uint8_t pkt_len,
                            uint8_t pkt_off,
                            size_t buf_len,
                            struct rte_mbuf *mbuf) {
    mbuf->data_off = RTE_PKTMBUF_HEADROOM;
    rte_mbuf_refcnt_set(mbuf, 1);

    struct rte_ether_hdr *eth_h;
    struct rte_ipv4_hdr *ipv4_h;
    struct rte_udp_hdr *udp_h;
    struct rmtp_msg_hdr *rmtp_h;
    uint8_t * payload_ptr;

    const uint32_t tot_size = sizeof(struct rte_ether_hdr) + 
                        sizeof(struct rte_ipv4_hdr) +
                        sizeof(struct rte_udp_hdr) +
                        sizeof(struct rmtp_msg_hdr) +
                        len;

    payload_ptr = rte_pktmbuf_mtod(mbuf, uint8_t *);
    eth_h = (struct rte_ether_hdr *)payload_ptr;
    ipv4_h = (struct rte_ipv4_hdr *)(eth_h + 1);
    udp_h = (struct rte_udp_hdr *)(ipv4_h + 1);
    rmtp_h = (struct rmtp_msg_hdr *)(udp_h + 1);

    // NOTE: fill with default pkt data
    memcpy(payload_ptr, pkt_data, DEFAULT_PKTHDR_SIZE);
    memcpy(payload_ptr+64, buf, len);

    ipv4_h->dst_addr = dst_qid_to_dst_ip_addr[dst_qid];
    ipv4_h->total_length = (tot_size - sizeof(struct rte_ether_hdr));
    udp_h->dgram_len = (tot_size - sizeof(rte_ether_hdr) - sizeof(rte_ipv4_hdr));

    udp_h->src_port = (0xdead); // 0
    udp_h->dst_port = (0xbeef);

    *rmtp_h = {
        .app_id = 0,
        .pkt_len = pkt_len,
        .pkt_off = pkt_off,
        .pkt_flag = RMTP_DATA_PKT,
        // .pkt_flag = RMTP_DATA_PKT | RMTP_ACK_REQUIRED,
        .tot_seg_cnt = 2,   // NOTE: no meaning for server-replied messages
        .msg_type = 1,      //
        .padding0 = dst_qid,
        .padding1 = 0,
        .padding2 = 0,
        .padding3 = 0,
        .padding4 = 0,
        .padding5 = 0,
        .padding6 = 0,
        .padding7 = 0,
        .msg_id = 0,
        .msg_acked_id = 0,
    };

    mbuf->data_len = tot_size;
    mbuf->pkt_len = tot_size;
    mbuf->nb_segs = 1;
    mbuf->next = NULL;
}

size_t rmtp::tx_batch(char *buf[], size_t buf_len[], uint8_t dst_qid[], bool ack_required[], size_t n) {

    assert(n<=MAX_BURST);
    size_t ret = 0;
    mcb *tx_mcbs[MAX_BURST];
    struct rte_mbuf *tx_mbufs[MAX_BURST];

    // 1-packet response
    if (!_available_tx_mcbs->is_empty()) {
        ret = _available_tx_mcbs->deq_batch(tx_mcbs, n);

        if (likely(ret>0 && rte_mempool_get_bulk(_tx_mp, (void **)tx_mbufs, ret)==0)) {
            for (size_t i=0; i<ret; i++) {
                prepare_mbuf(buf[i], buf_len[i], dst_qid[i]-1, 1, 0, CHUNK_SIZE, tx_mbufs[i]); // NOTE: CHUNK_SIZE is not used now
                tx_mcbs[i]->_snd.tx_pkts[0] = tx_mbufs[i];
                tx_mcbs[i]->reset_tx(1);
            }
            _tx_mcbs->enq_batch(tx_mcbs, ret);
        }
        else {
            info("here?");
            _available_tx_mcbs->enq_batch(tx_mcbs, ret);
            ret = 0;
        }
    }

    return ret;
}

void rmtp::poll_tx() {
    struct rte_ether_hdr *eth_h;
    struct rte_ipv4_hdr *ipv4_h;
    struct rte_udp_hdr *udp_h;
    struct rmtp_msg_hdr *rmtp_h;

    uint64_t curr_cycles = rte_get_timer_cycles();

    mcb * first_mcb = nullptr;

    while (!_tx_mcbs->is_empty() && _tx_empty_slots > 0) {

        auto tx_mcb = _tx_mcbs->deq();
        // if message is acked
        if (tx_mcb->_tx_finished) {
            tx_mcb->free_mcb_mbufs();
            assert(tx_mcb->_rmtp != nullptr);
            _available_tx_mcbs->enq(tx_mcb);
            continue;
        }

        if (first_mcb == tx_mcb) {
            _tx_mcbs->enq(tx_mcb);
            break;
        }

        // not in the same batch
        if (first_mcb == nullptr) {
            first_mcb = tx_mcb;
        }

        _tx_mcbs->enq(tx_mcb);
        auto tx_mbuf = tx_mcb->get_packet(curr_cycles);

        if (tx_mbuf != nullptr) {
            _tx_buf[_tx_head] = tx_mbuf;
            _tx_head = (_tx_head+1) % RING_BUF_SIZE;
            _tx_empty_slots -= 1;
            _tx_n_elements += 1;

            eth_h = rte_pktmbuf_mtod(tx_mbuf, struct rte_ether_hdr *);
            ipv4_h = (struct rte_ipv4_hdr *)(eth_h + 1);
            udp_h = (struct rte_udp_hdr *)(ipv4_h + 1);
            rmtp_h = (struct rmtp_msg_hdr *)(udp_h + 1);

            uint8_t msg_id_ind = rmtp_h->padding0;

            assert(msg_id_ind >=0 && msg_id_ind<MAX_NUM_SENDER);

            // put in ACK info if needed
            if (!_unsent_ack_msg_ids[msg_id_ind].empty()) {

                auto ack_msg_id = _unsent_ack_msg_ids[msg_id_ind].front();
                _unsent_ack_msg_ids[msg_id_ind].pop_front();
                rmtp_h->pkt_flag |= RMTP_ACK_PKT;
                rmtp_h->msg_acked_id = (ack_msg_id);
            }
        }
        else {
            // no available mbufs
            break;
        }

    }

    // send to the wire
    uint16_t n_snd = 0;
    // if (_tx_n_elements >= TX_MAX_BURST) {
    if (_tx_n_elements > 0) {
        uint32_t tx_batch_size = _tx_head >= _tx_tail?
                                    _tx_head - _tx_tail :
                                    RING_BUF_SIZE - _tx_tail;
        assert(tx_batch_size > 0);
        tx_batch_size = tx_batch_size > TX_MAX_BURST? TX_MAX_BURST : tx_batch_size;

        n_snd = _tx_fn(_if, _qid, _tx_buf+_tx_tail, tx_batch_size);

        if (n_snd > 0) {
            _tx_tail = (_tx_tail + n_snd) % RING_BUF_SIZE;
            _tx_empty_slots += n_snd;
            _tx_n_elements -= n_snd;
        }
    }

    _tot_tx_cnt += n_snd;
}

bool rmtp::process_rx_mbuf(struct rte_mbuf *mbuf) {
    struct rte_ether_hdr *eth_h;
    struct rte_ipv4_hdr *ipv4_h;
    struct rte_udp_hdr *udp_h;
    struct rmtp_msg_hdr *rmtp_h;

    eth_h = rte_pktmbuf_mtod(mbuf, struct rte_ether_hdr *);
    ipv4_h = (struct rte_ipv4_hdr *)(eth_h + 1);
    udp_h = (struct rte_udp_hdr *)(ipv4_h + 1);
    rmtp_h = (struct rmtp_msg_hdr *)(udp_h + 1);

    uint32_t msg_id = (rmtp_h->msg_id);
    uint32_t msg_acked_id = (rmtp_h->msg_acked_id);
    uint8_t pkt_len = rmtp_h->pkt_len;
    uint8_t pkt_off = rmtp_h->pkt_off;
    uint8_t pkt_flag = rmtp_h->pkt_flag;

    uint8_t msg_id_ind = ((msg_id >> MSG_ID_RANGE_RSHIFT) & 0xf) - 1;

    if (pkt_flag & RMTP_DATA_PKT) {
#ifdef QINGNIAO
        if (_rx_to_app_buf->is_full()) {
            // debug("_rx_to_app_buf full %d", _qid);
            return false;
        }
        else {
            _rx_to_app_buf->enq(mbuf);
            if (pkt_flag & RMTP_ACK_REQUIRED) {
                _unsent_ack_msg_ids[msg_id_ind].push_back(msg_id);
            }
        }
#else
        auto mcb_itr = _rcv_mcbs.find(msg_id);
        if (pkt_len == 1) {
            // deliver immediately if single-packet message
            if (_rx_to_app_buf->is_full()) {
                // debug("_rx_to_app_buf full %d", _qid);
                return false;
            }
            else {
                _rx_to_app_buf->enq(mbuf);
                if (pkt_flag & RMTP_ACK_REQUIRED) {
                    _unsent_ack_msg_ids[msg_id_ind].push_back(msg_id);
                    // debug("%x %d", msg_id, _rx_n_elements);
                }
            }
        }
        else if (mcb_itr != _rcv_mcbs.end()) {
            // 
            auto mcb_ptr = mcb_itr->second;
            mcb_ptr->_rcv.recv_pkts[pkt_off] = mbuf;

            mcb_ptr->_rcv.msg_recv_offsets |= 1<<pkt_off;

            if (mcb_ptr->_rcv.msg_recv_offsets == mcb_ptr->_rcv.msg_target_offsets) {
                // all pkt segments are received
                auto first_msg_mbuf = mcb_ptr->_rcv.recv_pkts[0];
                auto last_msg_mbuf = mcb_ptr->_rcv.recv_pkts[mcb_ptr->_rcv.msg_len_in_pkts-1];
                uint32_t add_pkt_len = 0;
                for (auto msg_seg_ind=0; msg_seg_ind<mcb_ptr->_rcv.msg_len_in_pkts-1; msg_seg_ind++) {
                    mcb_ptr->_rcv.recv_pkts[msg_seg_ind]->next = mcb_ptr->_rcv.recv_pkts[msg_seg_ind+1];
                    add_pkt_len += mcb_ptr->_rcv.recv_pkts[msg_seg_ind+1]->pkt_len;
                }
                assert(add_pkt_len == 128);

                first_msg_mbuf->pkt_len += add_pkt_len;
                first_msg_mbuf->nb_segs = mcb_ptr->_rcv.msg_len_in_pkts;
                last_msg_mbuf->next = NULL;

                if (_rx_to_app_buf->is_full()) {
                    // debug("_rx_to_app_buf full");
                    return false;
                }
                else {
                    _rx_to_app_buf->enq(first_msg_mbuf);
                    if (pkt_flag & RMTP_ACK_REQUIRED) {
                        _unsent_ack_msg_ids[msg_id_ind].push_back(msg_id);
                    }
                }

                // update tracking info
                _rcv_mcbs.erase(mcb_itr);
                _available_rx_mcbs->enq(mcb_ptr);
            }
        }
        else {
            // new message
            if (!_available_rx_mcbs->is_empty()) {
                auto rx_mcb_ptr = _available_rx_mcbs->deq();

                rx_mcb_ptr->_rcv.msg_len_in_pkts = pkt_len;
                rx_mcb_ptr->_rcv.msg_target_offsets = (1<<pkt_len) - 1;
                rx_mcb_ptr->_rcv.msg_recv_offsets = 1<<pkt_off;
                rx_mcb_ptr->_rcv.recv_pkts[pkt_off] = mbuf;

                _rcv_mcbs.emplace(msg_id, rx_mcb_ptr);
            }
        }
#endif
    }

    return true;
}

// receive from net, and dispath to mcbs
#define PREFETCH_OFFSET 3

void rmtp::poll_rx() {

    uint32_t rx_batch_size = 0;
    uint32_t rounded_size = RING_BUF_SIZE - _rx_head;
    rx_batch_size = rounded_size < RX_MAX_BURST ? rounded_size : RX_MAX_BURST;

    rx_batch_size = rx_batch_size < _rx_empty_slots ? rx_batch_size : _rx_empty_slots;

    int32_t n_rx = _rx_fn(_if, _qid, _rx_buf + _rx_head, rx_batch_size);

    // update rx pointers
    _rx_head = (_rx_head + n_rx) % RING_BUF_SIZE;
    _rx_empty_slots -= n_rx;
    _rx_n_elements += n_rx;
    _tot_rx_cnt += n_rx;
}


void rmtp::do_work() {

    if (_rx_n_elements >= RX_DO_WORK_THRESH) {

        int32_t n_work_size = _rx_head >= _rx_tail? 
                                    _rx_head-_rx_tail: 
                                    RING_BUF_SIZE-_rx_tail;
        n_work_size = n_work_size < RX_DO_WORK_THRESH? n_work_size : RX_DO_WORK_THRESH;

        if (n_work_size == 0) {
            return ;
        }

        int32_t i=0;

        auto ind = _rx_tail;
        int n_p = 0;
        bool early_terminate = false;

        for (i=0; i<PREFETCH_OFFSET && i<n_work_size; i++) {
            rte_prefetch0(rte_pktmbuf_mtod(_rx_buf[i + ind], void *));
        }

        for (i=0; i<n_work_size-PREFETCH_OFFSET; i++) {
            rte_prefetch0(rte_pktmbuf_mtod(_rx_buf[i+ ind + PREFETCH_OFFSET], void *));

            if (process_rx_mbuf(_rx_buf[i + ind])) {
                n_p += 1;
            }
            else {
                early_terminate = true;
                break;
            }
        }

        for (; i<n_work_size; i++) {
            if (early_terminate) {
                break;
            }
            if (process_rx_mbuf(_rx_buf[i + ind])) {
                n_p += 1;
            }
            else {
                break;
            }
        }
        //
        _rx_tail = (_rx_tail + n_p) % RING_BUF_SIZE;
        _rx_empty_slots += n_p;
        _rx_n_elements -= n_p;
    }
}

void rmtp::run() {
    poll_rx();
    do_work();
    poll_tx();
}

struct rte_mbuf * mcb::get_packet(uint64_t curr_cycles) {
    struct rte_mbuf *ret = nullptr;
    assert(_snd.tx_pkts[_snd.msg_pkt_ind] != nullptr);
    assert(_rmtp != nullptr);
    assert(rte_mempool_avail_count(_rmtp->_tx_mp) > 0);
    ret = rte_pktmbuf_clone(_snd.tx_pkts[_snd.msg_pkt_ind], _rmtp->_tx_mp);

    if (NULL == ret) {
        return ret;
    }

    if (_snd.msg_pkt_ind==_snd.msg_len_in_pkts-1) {
        _tx_finished = true;
    }

    _snd.msg_pkt_ind = (_snd.msg_pkt_ind+1) % _snd.msg_len_in_pkts;

    return ret;
}

void mcb::free_mcb_mbufs() {
    // only need to free tx direction here
    size_t i = 0;
    struct rte_mbuf * tx_mbufs[MAX_NUM_PKTSEG];
    for (i=0; i<_snd.msg_len_in_pkts; i++) {
        tx_mbufs[i] = _snd.tx_pkts[i];
    }
    assert(i>0);
    rte_pktmbuf_free_bulk(tx_mbufs, i);
}

} // namespace rmtp
