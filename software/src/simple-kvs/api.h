#ifndef __API_H__
#define __API_H__

#include "rmtp.h"

// uint16_t app_tx_batch(rmtp::rmtp *rmtp_ptr, struct rte_mbuf **tx_mbufs, uint16_t n_tx);

uint16_t app_tx_batch(rmtp::rmtp *rmtp_ptr,
        char **buf, size_t *buf_len, uint8_t *dst_qid, bool *ack_required, uint16_t n_tx);

uint16_t app_rx_batch(rmtp::rmtp *rmtp_ptr, struct rte_mbuf **buf, uint16_t max_n_rx);



#endif
