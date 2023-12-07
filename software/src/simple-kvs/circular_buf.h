#ifndef __CIRCULAR_BUF_H__
#define __CIRCULAR_BUF_H__

#include <cstdint>
#include <cstdio>
#include <cstdlib>


#include <rte_common.h>
#include <rte_mbuf.h>


#define CIRCULAR_BUF_SIZE 2048

namespace rmtp {

template <typename T> 
class circular_buf {
public:

    circular_buf(size_t capacity) {
        _capacity = CIRCULAR_BUF_SIZE;
        _head = _tail = 0;
        _ele_cnt = 0;

        bufs = new T [_capacity];
    }

    ~circular_buf() {
        delete [] bufs;
    }

    size_t _capacity;
    size_t _head;
    size_t _tail;
    size_t _ele_cnt;

    T* bufs;

    bool is_empty() { return _ele_cnt==0; }
    bool is_full()  { return _ele_cnt==_capacity-1; }

    void enq (T enq_element) {
        _ele_cnt += 1;
        bufs[_head] = enq_element;

        _head = (_head + 1) % CIRCULAR_BUF_SIZE;
    }

    T deq () {
        auto ret = bufs[_tail];
        _ele_cnt -= 1;
        _tail = (_tail + 1) % CIRCULAR_BUF_SIZE;
        return ret;
    }

    size_t deq_batch(T *arr, size_t deq_n) {
        size_t ret = deq_n<=_ele_cnt? deq_n : _ele_cnt;
        for (size_t i=0; i<ret; i++) {
            arr[i] = bufs[_tail];
            _ele_cnt -= 1;
            _tail = (_tail+1) % CIRCULAR_BUF_SIZE;
        }
        return ret;
    }

    void enq_batch(T *arr, size_t enq_n) {
        for (size_t i=0; i<enq_n; i++) {
            bufs[_head] = arr[i];
            _ele_cnt += 1;
            _head = (_head + 1) % CIRCULAR_BUF_SIZE;
        }
    }
};


} // namespace rmtp



#endif
