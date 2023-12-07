#ifndef IXY_LOG_H
#define IXY_LOG_H

#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <assert.h>

#ifndef NDEBUG
#define debug(fmt, ...) do {\
    fprintf(stderr, "[DEBUG] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)
#else
#define debug(fmt, ...) do {} while(0)
#undef assert
#define assert(expr) (void) (expr)
#endif

#define info(fmt, ...) do {\
    fprintf(stdout, "[INFO ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)

#define warn(fmt, ...) do {\
    fprintf(stderr, "[WARN ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)

#define log_error(fmt, ...) do {\
    fprintf(stderr, "[ERROR] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
    abort();\
} while(0)

#define check_err(expr, op) ({\
    int64_t result = (int64_t) (expr);\
    if ((int64_t) result == -1LL) {\
        fprintf(stderr, "[ERROR] %s:%d %s(): Failed to %s: %s\n", __FILE__, __LINE__, __func__, op, strerror(errno));\
        exit(errno);\
    }\
    result;\
})

#define myassert(cond, fmt, ...) do {\
    if (!(cond)) {\
        fprintf(stderr, "[ASSERTION ERROR] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
        abort();\
    }\
} while(0)

#endif //IXY_LOG_H
