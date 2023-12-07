#ifndef __HELPERS_H__
#define __HELPERS_H__

#include <cstdlib>
#include <cstring>
#include <sys/select.h>
#include <thread>
#include <vector>
#include <cstdint>
#include <ctime>
#include <iostream>

#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/file.h>


#include "common/log.h"
#include "common/app_thrd_info.h"

// time diff in usec
inline int tm_diff(struct timeval t2, struct timeval t1) {
    return (t2.tv_sec-t1.tv_sec)*1000000 + t2.tv_usec - t1.tv_usec;
}

inline void hexdump(void* void_ptr, size_t len) {
#ifdef ENABLE_HEXDUMP
    uint8_t* ptr = (uint8_t*) void_ptr;
    char ascii[17];
    for (uint32_t i = 0; i < len; i += 16) {
        printf("%06x: ", i); 
        int j = 0; 
        for (; j < 16 && i + j < len; j++) { 
            printf("%02x", ptr[i + j]); 
            if (j % 2) { 
                printf(" "); 
            } 
            ascii[j] = isprint(ptr[i + j]) ? ptr[i + j] : '.'; 
        }
        ascii[j] = '\0'; 
        if (j < 16) { 
            for (; j < 16; j++) { 
                printf("  "); 
                if (j % 2) { 
                    printf(" "); 
                } 
            } 
        } 
        printf("  %s\n", ascii);
    }
#endif
}

#endif
