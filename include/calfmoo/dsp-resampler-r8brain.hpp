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

#ifndef _CALFMOO_DSP_RESAMPLER_R8BRAIN_HPP_
#define _CALFMOO_DSP_RESAMPLER_R8BRAIN_HPP_

#include "utils-ringbuffer.hpp"

#include <cstdint>
#include <cmath>

#include "CDSPResampler.h"

// ============================================================================

namespace calfmoo {
namespace dsp {

#pragma pack(push)
#pragma pack()

// ============================================================================

class R8BrainResampler
{
public:
    typedef size_t Index;
    typedef float Sample;
    typedef double Rate;
    typedef int Error;

protected:
    Rate in_rate_;
    Rate out_rate_;
    double ratio_;
    r8b::CDSPResampler* resampler_;
    double* in_double_;
    double* out_double_;

protected:
    void Cleanup()
    {
        if (resampler_)
        {
            resampler_->clear();
            delete resampler_;
            resampler_ = NULL;
        }
        if (in_double_)
        {
            delete[] in_double_;
            in_double_ = NULL;
        }
        out_double_ = NULL;
    }

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

    Error Start(Rate in_rate, Rate out_rate, Index max_length)
    {
        Cleanup();

        in_rate_ = in_rate;
        out_rate_ = out_rate;
        ratio_ = out_rate / in_rate;

        if (sizeof(Sample) < sizeof(double)) {
            in_double_ = new double[max_length];
        }

        resampler_ = new r8b::CDSPResampler(in_rate, out_rate, static_cast<int>(max_length));

        return 0;
    }

    Error Stop()
    {
        Cleanup();
        return 0;
    }

    Error operator()(Sample const in_samples[], Index& in_count,
        Sample out_samples[], Index& out_count)
    {
        Index count = static_cast<Index>(out_count / ratio_);
        if (count > in_count) {
            count = in_count;
        }
        in_count = count;

        if (sizeof(Sample) < sizeof(double)) {
            for (Index i = 0; i < in_count; ++i) {
                in_double_[i] = static_cast<double>(in_samples[i]);
            }

            int int_count = static_cast<int>(in_count);
            count = static_cast<Index>(resampler_->process(in_double_, int_count, out_double_));
            out_count = count;

            for (Index i = 0; i < out_count; ++i) {
                out_samples[i] = static_cast<Sample>(out_double_[i]);
            }
        }
        else {
            double* in_double = reinterpret_cast<double*>(const_cast<Sample*>(in_samples));
            double* out_double = reinterpret_cast<double*>(out_samples);
            int int_count = static_cast<int>(in_count);
            count = static_cast<Index>(resampler_->process(in_double, int_count, out_double));
            out_count = count;
        }

        return 0;
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
    R8BrainResampler() :
        in_rate_(0),
        out_rate_(0),
        ratio_(0),
        resampler_(NULL),
        in_double_(NULL),
        out_double_(NULL)
    {
    }

    R8BrainResampler(Rate in_rate, Rate out_rate, Index max_length) :
        in_rate_(0),
        out_rate_(0),
        ratio_(0),
        resampler_(NULL),
        in_double_(NULL),
        out_double_(NULL)
    {
        Start(in_rate, out_rate, max_length);
    }

    ~R8BrainResampler()
    {
        Stop();
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace dsp
}  // namespace calfmoo

   // ============================================================================

#endif  // _CALFMOO_DSP_RESAMPLER_R8BRAIN_HPP_
