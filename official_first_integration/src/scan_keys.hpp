#pragma once
#include "world.hpp"
#include <limits>

namespace first {
// Single-perception-thread scratch table. An epoch resets a scan without node
// allocation or clearing the whole backing store. Call reset with an upper
// bound on unique keys; overfilling is an error, never an endless probe.
template<class Value> class ScanKeys {
 struct Slot {Key key{};Value value{};uint32_t epoch=0;};
 std::vector<Slot> slots_;
 uint32_t epoch_=0;
 size_t count_=0;
 size_t index(Key k)const {
  uint64_t h=Hash{}(k);
  h^=h>>30;h*=0xbf58476d1ce4e5b9ULL;h^=h>>27;h*=0x94d049bb133111ebULL;h^=h>>31;
  return size_t(h)&(slots_.size()-1);
 }
 public:
 void reset(size_t expected){
  if(expected>std::numeric_limits<size_t>::max()/4)throw std::length_error("scan key capacity");
  size_t capacity=8;while(capacity<expected*2)capacity*=2;
  if(slots_.size()<capacity){slots_.clear();slots_.resize(capacity);epoch_=0;}
  if(++epoch_==0){for(auto&s:slots_)s.epoch=0;epoch_=1;}
  count_=0;
 }
 Value* find(Key k){
  if(slots_.empty())return nullptr;
  size_t i=index(k);
  while(slots_[i].epoch==epoch_){if(slots_[i].key==k)return &slots_[i].value;i=(i+1)&(slots_.size()-1);}
  return nullptr;
 }
 std::pair<Value*,bool> insert(Key k){
  if(slots_.empty())throw std::logic_error("scan key table requires reset");
  size_t i=index(k);
  while(slots_[i].epoch==epoch_){if(slots_[i].key==k)return {&slots_[i].value,false};i=(i+1)&(slots_.size()-1);}
  if(count_>=slots_.size()/2)throw std::length_error("scan key bound exceeded");
  auto&s=slots_[i];s.key=k;s.value=Value{};s.epoch=epoch_;++count_;return {&s.value,true};
 }
};
}
