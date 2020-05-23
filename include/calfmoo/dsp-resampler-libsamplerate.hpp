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

#ifndef _CALFMOO_DSP_RESAMPLER_LIBSAMPLERATE_HPP_
#define _CALFMOO_DSP_RESAMPLER_LIBSAMPLERATE_HPP_

#include "utils-ringbuffer.hpp"

#include <cstdint>
#include <cmath>

#include "samplerate.h"

// ============================================================================

namespace calfmoo {
namespace dsp {

#pragma pack(push)
#pragma pack()

// ============================================================================

class SecretRabbitCodeResampler
{
public:
    typedef size_t Index;
    typedef float Sample;
    typedef double Rate;
    typedef int Error;

    enum Type {
        kTypeSincBest = SRC_SINC_BEST_QUALITY,
        kTypeSincMedium = SRC_SINC_MEDIUM_QUALITY,
        kTypeSincFastest = SRC_SINC_FASTEST,
        kTypeZeroOrderHold = SRC_ZERO_ORDER_HOLD,
        kTypeLinear = SRC_LINEAR,
        kTypeCount = 5,
    };

protected:
    SRC_STATE* state_;
    SRC_DATA data_;
    Error error_;
    Type type_;
    Rate in_rate_;
    Rate out_rate_;

public:
    Error GetError() const
    {
        return error_;
    }

    Type GetType() const
    {
        return type_;
    }

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
        return data_.src_ratio;
    }

    Index GetInputLength() const
    {
        return static_cast<Index>(data_.input_frames);
    }

    Index GetInputUsed() const
    {
        return static_cast<Index>(data_.input_frames_used);
    }

    Sample const* GetInputSamples() const
    {
        return data_.data_in;
    }

    Index GetOutputLength() const
    {
        return static_cast<Index>(data_.output_frames);
    }

    Index GetOutputGenerated() const
    {
        return static_cast<Index>(data_.output_frames_gen);
    }

    Sample* GetOutputSamples() const
    {
        return data_.data_out;
    }

    bool GetStop() const
    {
        return data_.end_of_input != 0;
    }

    void RaiseStop()
    {
        data_.end_of_input = 1;
    }

    Error Start(Rate in_rate, Rate out_rate, Type type = kTypeSincFastest)
    {
        in_rate_ = in_rate;
        out_rate_ = out_rate;

        data_.data_in = NULL;
        data_.data_out = NULL;
        data_.input_frames = 0;
        data_.output_frames = 0;
        data_.input_frames_used = 0;
        data_.output_frames_gen = 0;
        data_.end_of_input = 0;
        data_.src_ratio = out_rate / in_rate;

        if (state_ && type_ == type) {
            src_reset(state_);
        }
        else {
            state_ = src_delete(state_);
            state_ = src_new(static_cast<int>(type), 1, &error_);
            type_ = type;
        }

        return error_;
    }

    Error operator()(Sample const in_samples[], Index& in_count,
        Sample out_samples[], Index& out_count)
    {
        data_.data_in = in_samples;
        data_.data_out = out_samples;
        data_.input_frames = in_count;
        data_.output_frames = out_count;

        error_ = src_process(state_, &data_);

        in_count = data_.input_frames_used;
        out_count = data_.output_frames_gen;

        return error_;
    }

    template <typename InputIndex, typename OutputIndex>
    Error operator()(utils::RingBuffer<Sample, InputIndex>& inputs,
        utils::RingBuffer<Sample, OutputIndex>& outputs)
    {
        InputIndex in_count = 1;
        OutputIndex out_count = 1;

        while ((in_count && inputs.GetUsed()) || (out_count && outputs.GetFree())) {
            in_count = inputs.GetDequeueContiguous();
            out_count = outputs.GetEnqueueContiguous();
            Error error = (*this)(inputs.GetHeadPointer(), in_count,
                outputs.GetTailPointer(), out_count);
            if (error) {
                return error;
            }
            inputs.SkipHead(in_count);
            outputs.SkipTail(out_count);
        }

        return 0;
    }

public:
    SecretRabbitCodeResampler() :
        state_(NULL),
        error_(0),
        type_(kTypeCount),
        in_rate_(0),
        out_rate_(0)
    {
        data_.data_in = NULL;
        data_.data_out = NULL;
        data_.input_frames = 0;
        data_.output_frames = 0;
        data_.input_frames_used = 0;
        data_.output_frames_gen = 0;
        data_.end_of_input = 0;
        data_.src_ratio = 0;
    }

    SecretRabbitCodeResampler(Rate in_rate, Rate out_rate, Type type = kTypeSincFastest) :
        state_(NULL),
        error_(0),
        type_(kTypeCount),
        in_rate_(0),
        out_rate_(0)
    {
        data_.data_in = NULL;
        data_.data_out = NULL;
        data_.input_frames = 0;
        data_.output_frames = 0;
        data_.input_frames_used = 0;
        data_.output_frames_gen = 0;
        data_.end_of_input = 0;
        data_.src_ratio = 0;

        Start(in_rate, out_rate, type);
    }

    ~SecretRabbitCodeResampler()
    {
        state_ = src_delete(state_);
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace dsp
}  // namespace calfmoo

   // ============================================================================

#endif  // _CALFMOO_DSP_RESAMPLER_LIBSAMPLERATE_HPP_
