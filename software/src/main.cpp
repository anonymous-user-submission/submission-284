#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <ctime>
#include <thread>
#include <vector>

#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/file.h>

#include <getopt.h>

// DPDK lib
#include <rte_eal.h>
#include <rte_errno.h>
#include <rte_lcore.h>
#include <rte_launch.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_mempool.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_udp.h>
#include <rte_mbuf.h>

#include "common/log.h"
#include "common/app_thrd_info.h"

#include "mqnic/pci.h"
#include "mqnic/mqnic.h"

#include "proxy_proc/qingniao_proc.h"

#include "simple-kvs/api.h"
#include "simple-kvs/app.h"
#include "simple-kvs/rmtp.h"
#include "simple-kvs/api.h"
//==============================================

static struct option long_options[] = {
    {"workload", required_argument, NULL, 0},
    {"rsr_conf", required_argument, NULL, 0},
};


struct main_args {
    int num_queues;
    int num_bknd_procs;
    char pcie_dev[32];
    char workload_spec_file[128];
    char rsr_conf_file[128];
} g_args;

//==============================================
const int kMaxNumQueues = 16;
const int kMaxNumLCores = 16;
const int kMaxNumBkndProcs = 8;

volatile bool force_quit = false;
struct mqnic_eth_dev* eth_dev = NULL;
struct shm_config* cfg = NULL;

struct rte_mempool *mps[kMaxNumLCores];

nullptr_t *db = NULL;

void signal_handler(int sig) {
    if (sig==SIGINT || sig==SIGTERM) {
        force_quit = true;
    }

    sleep(1);
    exit(-1);
}

static inline void usage(char *const proc) {
    fprintf(stderr, "Usage: %s -N <num_of_queues (should be <= 8)>"
                            "-d <pcie_addr> -b <num_of_backend_procs>\n", proc);
}

static int parse_cmd_options(int argc, char *const argv[]) {
    int opt, ret = 0, option_index;

    // init g_args
    g_args = {
        .num_queues = -1,
        .num_bknd_procs = -1,
    };

    while ((opt = getopt_long(argc, argv, "N:d:b:", long_options, &option_index)) != -1) {
        switch (opt) {
            case 0:
                if (strcmp(long_options[option_index].name, "workload")==0) {
                    snprintf(g_args.workload_spec_file, sizeof(g_args.workload_spec_file), "%s", optarg);
                }
                else if (strcmp(long_options[option_index].name, "rsr_conf")==0) {
                    snprintf(g_args.rsr_conf_file, sizeof(g_args.rsr_conf_file), "%s", optarg);
                }
                else {
                    ret = -1;
                }
                break;
            case 'N':
                g_args.num_queues = atoi(optarg);
                break;
            case 'd':
                snprintf(g_args.pcie_dev, sizeof(g_args.pcie_dev), "%s", optarg);
                break;
            case 'b':
                g_args.num_bknd_procs = atoi(optarg);
                break;
            default:
                ret = -1;
                break;
        }
    }

    // sanity check
    assert(g_args.num_queues>0 && g_args.num_queues<=kMaxNumQueues);
    assert(strcmp(g_args.pcie_dev, "") != 0);
    assert(strcmp(g_args.workload_spec_file, "") != 0);
    assert(strcmp(g_args.rsr_conf_file, "") != 0);

    return ret;
}

int main(int argc, char **argv) {

    force_quit = false;
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int ret;
    ret = rte_eal_init(argc, argv);
    if (ret==-1) {
        log_error("rte_eal_init error: %s", rte_strerror(rte_errno));
    }

    argc -= ret;
    argv += ret;

    ret = parse_cmd_options(argc, argv);
    if (ret < 0) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    // should be primary proc
    assert(rte_eal_process_type() == RTE_PROC_PRIMARY);
    assert(rte_lcore_count() > g_args.num_queues);

    // initiate NIC userspace driver
    // #queues,  rss_mask --> #queues-1
    info("creating eth device: %s, #queues: %d", g_args.pcie_dev, g_args.num_queues);
    if (g_args.num_queues == 12) {
        mqnic_eth_dev_create(&eth_dev, g_args.pcie_dev, g_args.num_queues, 15);
    }
    else {
        mqnic_eth_dev_create(&eth_dev, g_args.pcie_dev, g_args.num_queues, g_args.num_queues-1);
    }

    // get the core ids, queue ids, mempools
    unsigned int lcore_main = rte_get_main_lcore();
    unsigned int lcores[kMaxNumLCores];
    uint16_t qids[kMaxNumLCores];
    const unsigned int lcore_cnt = g_args.num_queues;

    for (auto i=0; i<kMaxNumLCores; i++) {
        qids[i] = i;

        if (i==0) {
            lcores[i] = rte_get_next_lcore(lcore_main, 1, 1);
        }
        else {
            lcores[i] = rte_get_next_lcore(lcores[i-1], 1, 1);
        }
        // 
        char mp_name[32];
        sprintf(mp_name, "tx_mp_%d", i);
        // create mempool
        struct rte_mempool* mp = rte_pktmbuf_pool_create(mp_name,
                                                            8196,
                                                            256,
                                                            0,
                                                            MAX_PKT_SIZE + RTE_PKTMBUF_HEADROOM,
                                                            rte_socket_id());
        if (NULL == mp) {
            rte_exit(EXIT_FAILURE, "can not allocate tx mempool");
        }
        mps[i] = mp;
    }
    // [DONE]

    // start PID controller to sync time
    struct mqnic_dev *dev = eth_dev->dev;
    std::thread phc_thrd;
    if (dev->phc_rb) {
        phc_thrd = std::thread(start_sync_fpga_time, dev);
    }

/*======================================================================================*/
    sleep(5); // leave some room to sync time between HW and SW
    info("Here we go");

    info("start, i am running with QingNiao");

    struct qingniao_ctxt qn_ctxts[kMaxNumLCores];
    for (auto i=0; i<lcore_cnt; i++) {
        // set ctxt
        qn_ctxts[i].eth_dev = eth_dev;
        qn_ctxts[i].qid = qids[i];
        qn_ctxts[i].mp = mps[i];
        qn_ctxts[i].db = db;
        //
        rte_eal_remote_launch(qingniao_echo_proc, (void *)&(qn_ctxts[i]), lcores[i]);
    }

    // wait for all processes to finish
    rte_eal_mp_wait_lcore();
    //
    if (phc_thrd.joinable()) {
        phc_thrd.join();
    }

    // cleanup
    ret = rte_eal_cleanup();
    if (ret==-1) {
        log_error("rte_eal_cleanup error: %s", rte_strerror(rte_errno));
    }

    return EXIT_SUCCESS;
}
