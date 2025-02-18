/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef LOGSTORM_CONTROLLER_HPP
#define LOGSTORM_CONTROLLER_HPP

#include <mutex>  // NOLINT
#include <string>
#include <list>
#include <vector>
#include <unordered_map>
#include <cassert>
#include <utility>

namespace innovusion {

struct Node {
  static const int kMaxBufSize = 256;
  static const int kNoRateControlCounts = 10;
  static const time_t kMaxResetTimeMs = 400 * 1000;  // 400s
  static const time_t kMaxTrigIntervalMs = 70 * 1000;  // 70s
  static const int kMinResetCounter = 500;
  // use char[] instead of std::string to avoid potential memory leakage
  char key[kMaxBufSize];
  int64_t target, count, interval;
  time_t last_print_ms;
  time_t last_trigger_ms;
  explicit Node(time_t time_ms = 0) {
    key[0] = '\0';
    reset(time_ms);
  }
  void reset(time_t now_ms) {
    target = kNoRateControlCounts;
    count = 0;
    interval = 1;
    last_print_ms = now_ms;
    last_trigger_ms = now_ms;
  }
};

class LRU {
using PBI = std::pair<bool, int64_t>;

 public:
  explicit LRU(size_t capacity = kCacheListSizeDefault) {
    capacity_ = capacity;
    std::unique_lock<std::mutex> lk(mutex_);
    for (size_t i = 0; i < capacity_; i++) {
      auto node = new Node();
      assert(node);
      pool_.push_back(node);
    }
  };
  ~LRU() {
    std::unique_lock<std::mutex> lk(mutex_);
    while (!pool_.empty()) {
      auto node = pool_.front();
      pool_.pop_front();
      delete node;
    }
    while (!list_.empty()) {
      auto node = list_.front();
      list_.pop_front();
      delete node;
    }
  }
  PBI check(const std::string& key, time_t now_ms) {
    bool valid = true;
    int64_t count = 0;
    // if we hold one instance for each thread for performance opt., lock can be avoided
    std::unique_lock<std::mutex> lk(mutex_);
    if (key2node_.count(key)) {
      auto node = key2node_[key];
      if (now_ms < node->last_print_ms ||
          now_ms - node->last_print_ms >= Node::kMaxResetTimeMs ||
          now_ms - node->last_trigger_ms >= Node::kMaxTrigIntervalMs) {
        node->reset(now_ms);
      }
      node->count++;
      count = node->count;
      // xx todo(WYY):
      // if managed logs are too few,
      // log printing of the ones in list will always in reduced frequency.
      // we can add timestamp in Node to avoid such case
      // but currently it seems unnecessary
      if (node->count % node->interval == 0) {
        if (node->count >= node->target) {
          // impossible to overflow
          node->interval *= kIncreaseTimes;
          node->target *= kIncreaseTimes;
        }
        node->last_print_ms = now_ms;
      } else {
        valid = false;
      }
      node->last_trigger_ms = now_ms;
      make_latest_(node);
    } else {
      this->push(key, now_ms);
    }
    return {valid, count};
  }

  std::string get_stats(time_t now_ms) {
    // fast rerurn
    if (now_ms > last_print_ms_ &&
        now_ms - last_print_ms_ < kStatsPrintPeriod) {
      return "";
    }
    std::unique_lock<std::mutex> lk(mutex_);
    // to avoid timing reverse
    if (now_ms < last_print_ms_) {
      last_print_ms_ = now_ms;
    }
    if (now_ms - last_print_ms_ >= kStatsPrintPeriod && !list_.empty()) {
      std::string res = "[LogStormControl: ";
      for (auto it : list_) {
        res += it->key;
        res += ", count=" + std::to_string(it->count);
        res += "; ";
      }
      last_print_ms_ = now_ms;
      lk.unlock();
      // pop the last "; "
      res.pop_back();
      res.pop_back();
      res.push_back(']');
      return res;
    } else {
      return "";
    }
  }

 private:
  void push(const std::string& key, time_t now_ms) {  // called with lock
    while (key2node_.size() >= capacity_) {
      auto temp = list_.front();
      pool_.push_back(temp);
      key2node_.erase(std::string(temp->key));
      list_.pop_front();
    }
    assert(!pool_.empty());
    auto node = pool_.front();
    pool_.pop_front();
    assert(node);
    node = new (node) Node(now_ms);
    assert(node);
    node->count++;
    list_.push_back(node);
    int ret = snprintf(node->key, Node::kMaxBufSize, "%s", key.c_str());
    if (ret >= Node::kMaxBufSize) {
      node->key[Node::kMaxBufSize - 1] = '\0';
      std::string temp = std::string(node->key);
      key2node_[temp] = node;
    } else {
      key2node_[key] = node;
    }
  }

  void make_latest_(Node* node) {  // called with lock
    list_.remove(node);
    list_.push_back(node);
  }

 private:
  static const int kCacheListSizeDefault = 10;
  static const time_t kStatsPrintPeriod = 15 * 1000;  // 15s
  const int kIncreaseTimes = 10;

 private:
  std::unordered_map<std::string, Node*> key2node_;
  std::list<Node*> list_;  // cache list
  std::list<Node*> pool_;  // easy object pool
  std::mutex mutex_;
  size_t capacity_;
  time_t last_print_ms_ = 0;
};

typedef LRU LogStormController;

}  // namespace innovusion

#endif  // LOGSTORM_CONTROLLER_HPP
