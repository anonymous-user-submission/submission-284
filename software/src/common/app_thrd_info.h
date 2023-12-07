#ifndef __APP_THRD_INFO_H__
#define __APP_THRD_INFO_H__

#include <cstdint>
#include <cstring>
#include <atomic>

#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/ether.h>
#include <netinet/ip.h>
#include <netinet/udp.h>

#include "log.h"

#define MAX_NUM_PROCS 8

#define APP_TX_BATCH_SIZE 32
#define APP_RX_BATCH_SIZE 32

#define MAX_RX_BUF_SIZE 4096
#define MEMCACHED_SERVICE_PORT 11216
#define MEMCACHED_SERVICE_ADDR "127.0.0.1"
#define SERVER_PROXY_PORT 8991
#define CLIENT_PROXY_PORT 61938
#define MSG_ID_MASK 0xfffffff
#define IP_HDR_SIZE 20

#define SERVER_PATH "rockdb_bknd"
#define CLIENT_PATH "rockdb_client"
#define MAX_PKT_SIZE 4096

#define DEFAULT_PKTHDR_SIZE 42
#define MSGHDR_DEFAULT_PADDING {'0', '1', '2', '3', '4', '5', '6', '7'}
#define RX_MAX_BURST 32
#define TX_MAX_BURST 32
#define MAX_BURST 64
#define RX_DO_WORK_THRESH 16
#define APP_DO_WORK_THRESH RX_DO_WORK_THRESH
#define MSG_ID_ST 0x1000000
#define MSG_ID_ST_MASK (MSG_ID_ST-1)
#define MSG_ID_RANGE_RSHIFT 29
#define NUM_MP_ELEMENTS 4096
#define MP_ELEMENT_SIZE 4096
#define RMTP_DATA_PKT 0x01
#define RMTP_ACK_PKT 0x02
#define RMTP_ACK_REQUIRED 0x04

// 24-B rmtp_msg_hdr
struct __attribute__((__packed__)) rmtp_msg_hdr {
    uint8_t     app_id;
    uint8_t     pkt_len;
    uint8_t     pkt_off;
    uint8_t     pkt_flag;
    uint8_t     tot_seg_cnt;
    uint8_t     msg_type;

    // reserved
    uint8_t     padding0;
    uint8_t     padding1;
    uint8_t     padding2;
    uint8_t     padding3;
    uint8_t     padding4;
    uint8_t     padding5;
    uint8_t     padding6;
    uint8_t     padding7;

    uint32_t    msg_id;
    uint32_t    msg_acked_id;
};

struct app_msg {

    struct ethhdr           *eth_hdr_;
    struct iphdr            *ip_hdr_;
    struct udphdr           *udp_hdr_;
    struct rmtp_msg_hdr     *rmtp_hdr_;

    uint8_t                 *buf_;
    uint8_t                 *payload_; // points to the RMTP payload

    uint32_t size_, payload_size_;

    // default constructor
    app_msg () : eth_hdr_(nullptr), ip_hdr_(nullptr), udp_hdr_(nullptr), rmtp_hdr_(nullptr),
        buf_(nullptr), payload_(nullptr), size_(0), payload_size_(0) {}

    app_msg (uint8_t *buf, uint32_t size) : buf_(buf), size_(size) {

        eth_hdr_    = reinterpret_cast<struct ethhdr*>(buf_);
        ip_hdr_     = reinterpret_cast<struct iphdr*>(eth_hdr_ + 1);
        udp_hdr_    = reinterpret_cast<struct udphdr*>(ip_hdr_ + 1);
        rmtp_hdr_   = reinterpret_cast<struct rmtp_msg_hdr*>(udp_hdr_ + 1);

        payload_        = reinterpret_cast<uint8_t*>(rmtp_hdr_ + 1);
        payload_size_   = size_ - sizeof(ethhdr) - sizeof(iphdr) - sizeof(udphdr) - sizeof(rmtp_msg_hdr);
    }

    struct ethhdr *eth_hdr() { return eth_hdr_; }
    struct iphdr *ip_hdr() { return ip_hdr_; }
    struct udphdr *udp_hdr() { return udp_hdr_; }
    struct rmtp_msg_hdr *rmtp_hdr() { return rmtp_hdr_; }

    uint8_t *data() { return buf_; }
    uint8_t *payload() { return payload_; }
    uint32_t size() { return size_; }
    uint32_t payload_size() { return payload_size_; }
};

#endif
