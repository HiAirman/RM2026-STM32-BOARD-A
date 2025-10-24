//
// Created by HiAir on 2025/10/24.
//

#include "pid.h"

#include <math.h>

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k, float time) :
kp_(kp),
ki_(ki),
kd_(kd),
i_max_(i_max),
out_max_(out_max),
d_filter_k_(d_filter_k),
time_(time) {}

void PID::reset() {
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
    last_dout_ = 0.0f;
}

float PID::calc(float ref, float fdb) {
    ref_ = ref;
    fdb_ = fdb;
    err_ = ref_ - fdb_;
    if (err_ <= i_max_) {
        err_sum_ += err_;
    }
    pout_ = kp_ * err_;
    iout_ = ki_ * time_ * err_sum_;
    dout_ = (kd_ * (err_ - last_err_) / time_ * d_filter_k_ + last_dout_) / (1 + d_filter_k_);
    last_err_ = err_;
    last_dout_ = dout_;
    if (abs(pout_ + iout_ + dout_) > out_max_) {
        return out_max_ * (abs(pout_ + iout_ + dout_) / (pout_ + iout_ + dout_));
    }
    return pout_ + iout_ + dout_;
}