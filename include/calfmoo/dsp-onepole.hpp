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

#ifndef _CALFMOO_DSP_ONEPOLE_HPP_
#define _CALFMOO_DSP_ONEPOLE_HPP_

#include "utils-ringbuffer.hpp"

// ============================================================================

namespace calfmoo {
namespace dsp {

#pragma pack(push)
#pragma pack()

// ============================================================================

template <typename Parameter_>
class OnePoleParameters;

template <typename Parameter_>
struct OnePoleModel;

template <typename Accumulator_>
class OnePoleState;

template <typename Accumulator_,
         typename Input_,
         typename Output_>
class OnePoleFilter;

// ============================================================================

template <typename Parameter_>
class OnePoleParameters
{
public:
    typedef Parameter_ Parameter;

protected:
    Parameter dt_;
    Parameter tau_;

public:
    const Parameter GetTimeDelta() const
    {
        return dt_;
    }

    void SetTimeDelta(const Parameter dt)
    {
        dt_ = dt;
    }

    const Parameter GetSampleRate() const
    {
        return 1 / dt_;
    }

    void SetSampleRate(const Parameter fs)
    {
        dt_ = 1 / fs;
    }

    const Parameter GetTimeConstant() const
    {
        return tau_;
    }

    void SetTimeConstant(const Parameter tau)
    {
        tau_ = tau;
    }

    const Parameter GetSettlingTime() const
    {
        return tau_ * 5;
    }

    void SetSettlingTime(const Parameter s)
    {
        tau_ = s / 5;
    }

    const Parameter GetPulsation() const
    {
        return 1 / tau_;
    }

    void SetPulsation(const Parameter w)
    {
        tau_ = 1 / w;
    }

    const Parameter GetFrequency() const
    {
        return 1 / (tau_ * static_cast<Parameter>(2 * 3.14159265358979323846));
    }

    void SetFrequency(const Parameter f)
    {
        tau_ = 1 / (f * static_cast<Parameter>(2 * 3.14159265358979323846));
    }

    OnePoleParameters &operator=(OnePoleParameters const &that)
    {
        dt_ = that.dt_;
        tau_ = that.tau_;
        return *this;
    }

public:
    OnePoleParameters()
    {
        ;
    }

    OnePoleParameters(OnePoleParameters const &that)
    {
        *this = that;
    }

    OnePoleParameters(Parameter const fs, Parameter const f)
    {
        SetSampleRate(fs);
        SetFrequency(f);
    }
};

// ----------------------------------------------------------------------------

template <typename Parameter_>
struct OnePoleModel
{
public:
    typedef Parameter_ Parameter;
    typedef OnePoleParameters<Parameter> Parameters;

public:
    Parameter alpha;

public:
    OnePoleModel &operator=(Parameters const &parameters)
    {
        Parameter const dt = parameters.GetTimeDelta();
        Parameter const tau = parameters.GetTimeConstant();
        alpha = static_cast<Parameter const>(dt / (tau + dt));
        return *this;
    }

    OnePoleModel &operator=(OnePoleModel const &that)
    {
        alpha = that.alpha;
        return *this;
    }

public:
    OnePoleModel()
    {
        ;
    }

    OnePoleModel(OnePoleModel const &that)
    {
        *this = that;
    }

    OnePoleModel(Parameters const &parameters)
    {
        Apply(parameters);
    }
};

// ----------------------------------------------------------------------------

template <typename Accumulator_>
class OnePoleState
{
public:
    typedef Accumulator_ Accumulator;

public:
    Accumulator y0;
    Accumulator y1;

public:
    void Clear(Accumulator const y = static_cast<Accumulator const>(0))
    {
        y0 = y;
        y1 = y;
    }

    OnePoleState &operator=(OnePoleState const &that)
    {
        y0 = that.y0;
        y1 = that.y1;
        return *this;
    }

public:
    OnePoleState()
    {
        ;
    }

    OnePoleState(OnePoleState const &that)
    {
        *this = that;
    }

    OnePoleState(Accumulator const y)
    {
        Clear(y);
    }
};

// ----------------------------------------------------------------------------

template <typename Accumulator_,
         typename Input_,
         typename Output_>
class OnePoleFilter
{
public:
    typedef Accumulator_ Accumulator;
    typedef Input_ Input;
    typedef Output_ Output;

    typedef OnePoleModel<Accumulator> Model;
    typedef OnePoleState<Accumulator> State;

protected:
    Model const *model_;
    State *state_;

public:
    Model const *GetModel() const
    {
        return model_;
    }

    void SetModel(Model const *model)
    {
        model_ = model;
    }

    State *GetState() const
    {
        return state_;
    }

    void SetState(State *state)
    {
        state_ = state;
    }

    void operator()(size_t count, Input const *inputs, Output *outputs)
    {
        Model const &m = *model_;
        Accumulator const alpha = m.alpha;
        State &s = *state_;
        Accumulator y0 = s.y0, y1 = s.y1;

        while (count--) {
            y1 = y0;
            y0 += (static_cast<Accumulator>(*inputs++) - y1) * alpha;
            *outputs++ = static_cast<Output>(y0);
        }

        s.y0 = y0, s.y1 = y1;
    }

    Output const operator()(Input const input)
    {
        Model const &m = *model_;
        State &s = *state_;

        s.y1 = s.y0;
        s.y0 += (input - s.y1) * m.alpha;

        return s.y0;
    }

    template <typename InputIndex, typename OutputIndex>
    size_t operator()(utils::RingBuffer<Input, InputIndex> &inputs,
                      utils::RingBuffer<Output, OutputIndex> &outputs)
    {
        size_t count = 0;
        while (inputs.GetUsed() && outputs.GetFree()) {
            size_t block = outputs.TrimEnqueueContiguous(inputs.GetDequeueContiguous());

            (*this)(block, inputs.GetHeadPointer(), outputs.GetTailPointer());

            inputs.SkipHead(block);
            outputs.SkipTail(block);
            count += block;
        }
        return count;
    }

    OnePoleFilter &operator=(OnePoleFilter const &that)
    {
        model_ = that.model_;
        state_ = that.state_;
        return *this;
    }

public:
    OnePoleFilter() :
        model_(),
        state_()
    {
        ;
    }

    OnePoleFilter(OnePoleFilter const &that)
    {
        *this = that;
    }

    OnePoleFilter(Model const *model, State *state,
                  Accumulator const y = static_cast<Accumulator const>(0)) :
        model_(model),
        state_(state)
    {
        Clear(y);
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace dsp
}  // namespace calfmoo

// ============================================================================

#endif  // _CALFMOO_DSP_ONEPOLE_HPP_
