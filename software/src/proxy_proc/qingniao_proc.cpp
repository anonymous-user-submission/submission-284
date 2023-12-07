#include <cstdlib>


#include "common/helpers.h"
#include "common/app_thrd_info.h"
#include "common/log.h"

#include "qingniao_proc.h"

extern volatile bool force_quit;

int qingniao_echo_proc(void *args) {
    struct qingniao_ctxt *ctxt = (struct qingniao_ctxt *)args;

    auto interface = ctxt->eth_dev->dev->interface[0];
    auto rx_fn = ctxt->eth_dev->rx_pkts_burst;
    auto tx_fn = ctxt->eth_dev->tx_pkts_burst;
    auto mp = ctxt->mp;
    auto qid = ctxt->qid;
    auto db = ctxt->db;

    auto rmtp_ptr = new rmtp::rmtp(qid, (qid+1)*MSG_ID_ST, mp, interface, rx_fn, tx_fn);
    auto app_ins = new app_thrd(rmtp_ptr, db, interface, qid, rx_fn, tx_fn);

    struct timeval t_st, t_ed;
    gettimeofday(&t_st, NULL);

    while (!force_quit) {

        rmtp_ptr->run();
        app_ins->run_app();

        gettimeofday(&t_ed, NULL);
        if (tm_diff(t_ed, t_st) > 5000000) { // 5s
            info("APP qid[%d], tx rate pkts/s: %.3f \t rx: %.3f", qid,
                    1.0*app_ins->_worker_ring->tot_tx_cnt/tm_diff(t_ed, t_st),
                    1.0*app_ins->_worker_ring->tot_rx_cnt/tm_diff(t_ed, t_st));
            info("DR qid[%d], tx rate pkts/s: %.3f \t rx: %.3f", qid,
                    1.0*rmtp_ptr->_tot_tx_cnt/tm_diff(t_ed, t_st),
                    1.0*rmtp_ptr->_tot_rx_cnt/tm_diff(t_ed, t_st));

            app_ins->_worker_ring->tot_rx_cnt = 0;
            app_ins->_worker_ring->tot_tx_cnt = 0;
            rmtp_ptr->_tot_rx_cnt = 0;
            rmtp_ptr->_tot_tx_cnt = 0;

            t_st = t_ed;
        }
    }

    return EXIT_SUCCESS;
}


