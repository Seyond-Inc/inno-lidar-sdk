/**
 *  Copyright (C) 2025 - Innovusion Inc.
 *
 *  All Rights Reserved.
 * 
 */

#ifndef UTILS_CIRCULAR_BUFFER_H_
#define UTILS_CIRCULAR_BUFFER_H_
#include <iostream>
#include <atomic>
namespace innovusion {
/**
 * @brief CircularBuffer
 * @tparam T Type of elements in the buffer_
 * @tparam N Size of the buffer_
 */
template <typename T, size_t N>
class CircularBuffer {
 public:
  CircularBuffer() : head_(0), tail_(0), count_(0) {
  }

  bool isEmpty() const {
    return count_.load(std::memory_order_seq_cst) == 0;
  }

  bool isFull() const {
    return count_.load(std::memory_order_seq_cst) == N;
  }

  size_t size() const {
    return count_.load(std::memory_order_seq_cst);
  }

  bool enqueue(const T& value) {
    if (isFull()) {
      return false;
    }
    buffer_[tail_] = value;
    tail_ = (tail_ + 1) % N;
    count_.fetch_add(1, std::memory_order_seq_cst);
    return true;
  }

  bool dequeue(T& value) {
    if (isEmpty()) {
      return false;
    }
    value = buffer_[head_];
    head_ = (head_ + 1) % N;
    count_.fetch_sub(1, std::memory_order_seq_cst);
    return true;
  }

  bool peek(T& value) const {
    if (isEmpty()) {
      return false;
    }
    value = buffer_[head_];
    return true;
  }

 private:
  T buffer_[N];
  int32_t head_;
  int32_t tail_;
  std::atomic<int32_t> count_;
};

}  // namespace innovusion
#endif  // UTILS_CIRCULAR_BUFFER_H_
