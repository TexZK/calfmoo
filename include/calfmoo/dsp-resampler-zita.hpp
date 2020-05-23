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

#ifndef _CALFMOO_DSP_RESAMPLER_ZITA_HPP_
#define _CALFMOO_DSP_RESAMPLER_ZITA_HPP_

#include "utils-ringbuffer.hpp"

#include <cstdint>
#include <cmath>

#include "zita-resampler/vresampler.h"

// ============================================================================

namespace calfmoo {
namespace dsp {

#pragma pack(push)
#pragma pack()

// ============================================================================

class ZitaResampler
{
public:
    typedef size_t Index;
    typedef float Sample;
    typedef unsigned Rate;
    typedef int Error;

    enum {
        kChannelCount = 1,
        kFilterQualityMin = 8,
        kFilterQualityMax = 96,
    };

protected:
    Rate in_rate_;
    Rate out_rate_;
    double ratio_;
    ::VResampler resampler_;
    Sample* inp_endex_;
    Sample* out_endex_;

public:
    Rate GetInputRate() const
    {
        return in_rate_;
    }

    Rate GetOutputRate() const
    {
        return out_rate_;
    }

    double GetOutputVsInputRatio() const
    {
        return ratio_;
    }

    Error Start(Rate in_rate, Rate out_rate, unsigned quality = 48)
    {
        in_rate_ = in_rate;
        out_rate_ = out_rate;
        ratio_ = static_cast<double>(out_rate) / static_cast<double>(in_rate);

        resampler_.setup(ratio_, kChannelCount, quality);

        return 0;
    }

    Error Stop()
    {
        return 0;
    }

    Error operator()(Sample const in_samples[], Index& in_count,
        Sample out_samples[], Index& out_count)
    {
        resampler_.inp_count = static_cast<unsigned>(in_count);
        resampler_.inp_data = in_samples;

        resampler_.out_count = static_cast<unsigned>(out_count);
        resampler_.out_data = out_samples;

        Error error = resampler_.process();

        in_count -= resampler_.inp_count;
        out_count -= resampler_.out_count;
        return error;
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

            Error error = (*this)(
                inputs.GetHeadPointer(), in_count,
                outputs.GetTailPointer(), out_count
                );
            if (error) {
                return error;
            }

            inputs.SkipHead(in_count);
            outputs.SkipTail(out_count);
        }

        return 0;
    }

public:
    ZitaResampler() :
        in_rate_(0),
        out_rate_(0),
        ratio_(0),
        inp_endex_(NULL),
        out_endex_(NULL)
    {
        resampler_.inp_count = 0;
        resampler_.out_count = 0;
        resampler_.inp_data = NULL;
        resampler_.out_data = NULL;
    }

    ZitaResampler(Rate in_rate, Rate out_rate) :
        in_rate_(0),
        out_rate_(0),
        ratio_(0),
        inp_endex_(NULL),
        out_endex_(NULL)
    {
        Start(in_rate, out_rate);
    }

    ~ZitaResampler()
    {
        ;
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace dsp
}  // namespace calfmoo

   // ============================================================================

#endif  // _CALFMOO_DSP_RESAMPLER_ZITA_HPP_
