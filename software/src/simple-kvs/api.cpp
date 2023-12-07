
#include "api.h"

uint16_t app_tx_batch(rmtp::rmtp *rmtp_ptr,
                        char **buf,
                        size_t *buf_len,
                        uint8_t *dst_qid,
                        bool *ack_required,
                        uint16_t n_tx) {

    uint16_t ret = 0;

    ret = rmtp_ptr->tx_batch(buf, buf_len, dst_qid, ack_required, n_tx);

    return ret;
}


uint16_t app_rx_batch(rmtp::rmtp *rmtp_ptr, struct rte_mbuf **buf, uint16_t max_n_rx) {
    uint16_t ret = 0;

    ret = rmtp_ptr->_rx_to_app_buf->deq_batch(buf, max_n_rx);

    return ret;
}
