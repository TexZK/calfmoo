/*
BSD 2-Clause License

Copyright (c) 2020, Andrea Zoppi
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _CALFMOO_DSP_RESAMPLER_HPP_
#define _CALFMOO_DSP_RESAMPLER_HPP_

#include "utils-ringbuffer.hpp"

#include <cstdint>
#include <cmath>

// ============================================================================

namespace calfmoo {
namespace dsp {

#pragma pack(push)
#pragma pack()

// ============================================================================

template <typename Sample_ = float,
         typename Index_ = long,
         typename Rate_ = double,
         typename Error_ = int>
class Resampler
{
public:
    typedef Index_  Index;
    typedef Sample_ Sample;
    typedef Rate_   Rate;
    typedef Error_  Error;

protected:
    Rate in_rate_;
    Rate out_rate_;
    double ratio_;
    bool reset_;
    double last_position_;

public:
    Rate GetInputRate() const
    {
        return in_rate_;
    }

    Rate GetOutputRate() const
    {
        return out_rate_;
    }

    Rate GetOutputVsInputRatio() const
    {
        return ratio_;
    }

    void Start(Rate in_rate, Rate out_rate)
    {
        in_rate_ = in_rate;
        out_rate_ = out_rate;
        ratio_ = out_rate / in_rate;
        last_position_ = 0;
        reset_ = true;
    }

    virtual Error Stop()
    {
        return 0;
    }

    virtual Error operator()(Sample const in_samples[], Index& in_count,
                             Sample out_samples[], Index& out_count) = 0;

    template <typename InputIndex, typename OutputIndex>
    Error operator()(utils::RingBuffer<Sample, InputIndex> &inputs,
                     utils::RingBuffer<Sample, OutputIndex> &outputs)
    {
        Index in_count = 1, out_count = 1;

        while ((in_count && inputs.GetUsed()) || (out_count && outputs.GetFree())) {
            in_count = inputs.GetDequeueContiguous();
            out_count = outputs.GetEnqueueContiguous();
            (*this)(inputs.GetHeadPointer(), in_count,
                    outputs.GetTailPointer(), out_count);
            inputs.SkipHead(in_count);
            outputs.SkipTail(out_count);
        }

        return 0;
    }

public:
    Resampler() :
        in_rate_(0),
        out_rate_(0),
        ratio_(0),
        reset_(true),
        last_position_(0)
    {
        ;
    }

    Resampler(Rate in_rate, Rate out_rate)
    {
        Start(in_rate, out_rate);
    }

    virtual ~Resampler()
    {
        Stop();
    }
};

// ============================================================================

template <typename Sample_ = float,
         typename Index_ = long,
         typename Rate_ = double,
         typename Error_ = int>
class ZeroOrderHoldResampler : public Resampler<Sample_, Index_ Rate_, Error_>
{
protected:
    Sample last_value_;

protected:
    Error operator()(Sample const in_samples[], Index& in_count,
        Sample out_samples[], Index& out_count)
    {
        if (reset_) {
            last_value_ = in_samples[0];
            reset_ = false;
        }
        Sample const* in_ptr = in_samples;
        Sample* out_ptr = out_samples;
        Index in_head = 0, in_bound = in_count;
        Index out_tail = 0, out_bound = out_count;
        Rate position = last_position_, remainder;
        Rate fine_step = 1 / ratio_;
        Index int_step;

        if (in_bound > 0) {
            while (position < 1 && out_tail < out_bound) {
                *out_ptr++ = last_value_;
                ++out_tail;
                position += fine_step;
            }
        }
        remainder = position - floor(position);
        int_step = static_cast<Index>(position);
        in_head += int_step;
        in_ptr += int_step;
        position = remainder;

        while (out_tail < out_bound && in_head < in_bound) {
            *out_ptr++ = *in_ptr;
            ++out_tail;
            position += fine_step;
            remainder = position - floor(position);
            int_step = static_cast<Index>(position);
            in_ptr += int_step;
            in_head += int_step;
            position = remainder;
        }

        if (in_head > in_bound) {
            position += static_cast<Rate>(in_head - in_bound);
            in_head = in_bound;
        }

        if (in_head) {
            last_value_ = in_ptr[-1];
        }
        last_position_ = position;

        in_count = in_head;
        out_count = out_tail;

        return 0;
    }

public:
    ZeroOrderHoldResampler() :
        last_value_(0)
    {
        ;
    }

    ZeroOrderHoldResampler(Rate in_rate, Rate out_rate) :
        Resampler(in_rate, out_rate),
        last_value_(0)
    {
        ;
    }
};

// ============================================================================

template <typename Sample_ = float,
         typename Index_ = long,
         typename Rate_ = double,
         typename Error_ = int>
class LinearResampler : public Resampler<Sample_, Index_, Rate_, Error_>
{
protected:
    Sample last_value_;

public:
    static Sample Interpolate(Sample a, Sample b, Rate amount)
    {
        return static_cast<Sample>(a + amount * (b - a));
    }

protected:
    Error  operator()(Sample const in_samples[], Index& in_count,
        Sample out_samples[], Index& out_count)
    {
        if (reset_) {
            last_value_ = 0;
            reset_ = false;
        }
        Sample const* in_ptr = in_samples;
        Sample* out_ptr = out_samples;
        Index in_head = 0, in_bound = in_count;
        Index out_tail = 0, out_bound = out_count;
        Rate position = last_position_, remainder;
        Rate fine_step = 1 / ratio_;
        Index int_step;

        if (in_bound > 1) {
            while (position < 1 && out_tail < out_bound) {
                Sample sample = Interpolate(in_ptr[0], last_value_, position);
                *out_ptr++ = sample;
                ++out_tail;
                position += fine_step;
            }
        }
        remainder = position - floor(position);
        int_step = static_cast<Index>(position);
        in_head += int_step;
        in_ptr += int_step;
        position = remainder;

        while (out_tail < out_bound && in_head < in_bound) {
            Sample interpolated = Interpolate(in_ptr[0], in_ptr[-1], position);
            *out_ptr++ = interpolated;
            ++out_tail;
            position += fine_step;
            remainder = position - floor(position);
            int_step = static_cast<Index>(position);
            in_head += int_step;
            in_ptr += int_step;
            position = remainder;
        }

        if (in_head > in_bound) {
            position += in_head - in_bound;
            in_head = in_bound;
        }

        if (in_head) {
            last_value_ = in_ptr[-1];
        }
        last_position_ = position;

        in_count = in_head;
        out_count = out_tail;

        return 0;
    }

public:
    LinearResampler() :
        last_value_(0)
    {
        ;
    }

    LinearResampler(Rate in_rate, Rate out_rate) :
        Resampler(in_rate, out_rate),
        last_value_(0)
    {
        ;
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace dsp
}  // namespace calfmoo

   // ============================================================================

#endif  // _CALFMOO_DSP_RESAMPLER_HPP_
