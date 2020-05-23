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

#ifndef _CALFMOO_DSP_BIQUAD_HPP_
#define _CALFMOO_DSP_BIQUAD_HPP_

#include "utils-ringbuffer.hpp"

#include <cmath>

// ============================================================================

namespace calfmoo {
namespace dsp {

#pragma pack(push)
#pragma pack()

// ============================================================================

template <typename Parameter_>
class BiQuadParameters;

template <typename Parameter_>
struct BiQuadModel;

template <typename Accumulator_>
class BiQuadState;

template <typename Accumulator_, typename Input_, typename Output_>
class BiQuadFilter;

// ============================================================================

template <typename Parameter_>
class BiQuadParameters
{
public:
    typedef Parameter_ Parameter;

    enum Behavior {
        kBehaviorUnity,

        kBehaviorLowPass,
        kBehaviorHighPass,

        kBehaviorBandPassConstantSkirt,
        kBehaviorBandPassConstantPeak,
        kBehaviorBandPassPeakingEq,
        kBehaviorBandStop,

        kBehaviorLowShelf,
        kBehaviorHighShelf,

        kBehaviorAllPass,

        kBehaviorCount,
    };

protected:
    Behavior behavior_;
    Parameter dt_;
    Parameter w_;
    Parameter q_;
    Parameter g_;

public:
    Behavior const GetBehavior() const
    {
        return behavior_;
    }

    void SetBehavior(Behavior const behavior)
    {
        behavior_ = behavior;
    }

    Parameter const GetTimeDelta() const
    {
        return dt_;
    }

    void SetTimeDelta(Parameter const dt)
    {
        dt_ = dt;
    }

    Parameter const GetSampleRate() const
    {
        return 1 / dt_;
    }

    void SetSampleRate(Parameter const fs)
    {
        dt_ = 1 / fs;
    }

    Parameter const GetPulsation() const
    {
        return w_;
    }

    void SetPulsation(Parameter const w)
    {
        w_ = w;
    }

    Parameter const GetFrequency() const
    {
        return w_ * static_cast<Parameter const>(1 / (2 * 3.14159265358979323846));
    }

    void SetFrequency(Parameter const f)
    {
        w_ = f * static_cast<Parameter const>(2 * 3.14159265358979323846);
    }

    Parameter const GetResonance() const
    {
        return q_;
    }

    void SetResonance(Parameter const q)
    {
        q_ = q;
    }

    Parameter const GetGain() const
    {
        return g_;
    }

    void SetGain(Parameter const g)
    {
        g_ = g;
    }

    Parameter const GetGainDecibels() const
    {
        using std::log10;
        return static_cast<Parameter const>(20 * log10(g_));
    }

    void SetGainDecibels(Parameter const g)
    {
        using std::pow;
        static Parameter const kTen = 10;
        Parameter const to = g * static_cast<Parameter const>(1.0 / 20);
        g_ = static_cast<Parameter const>(pow(kTen, to));
    }

    BiQuadParameters &operator=(BiQuadParameters const &that)
    {
        behavior_ = that.behavior_;
        dt_ = that.dt_;
        w_ = that.w_;
        q_ = that.q_;
        g_ = that.g_;
    }

public:
    BiQuadParameters() :
        behavior_(),
        dt_(),
        w_(),
        q_(),
        g_()
    {
        ;
    }

    BiQuadParameters(BiQuadParameters const &that)
    {
        *this = that;
    }

    BiQuadParameters(Behavior const behavior,
                     Parameter const fs,
                     Parameter const f,
                     Parameter const q = static_cast<Parameter const>(0.707106781186547524401),
                     Parameter const g = static_cast<Parameter const>(1))
    {
        SetBehavior(behavior);
        SetSampleRate(fs);
        SetFrequency(f);
        SetResonance(q);
        SetGain(g);
    }
};

// ----------------------------------------------------------------------------

template <typename Parameter_>
struct BiQuadModel
{
public:
    typedef Parameter_ Parameter;
    typedef BiQuadParameters<Parameter> Parameters;

public:
    Parameter a1;
    Parameter a2;

    Parameter b0;
    Parameter b1;
    Parameter b2;

public:
    BiQuadModel &operator=(BiQuadModel const &that)
    {
        a1 = that.a1;
        a2 = that.a2;

        b0 = that.b0;
        b1 = that.b1;
        b2 = that.b2;

        return *this;
    }

    BiQuadModel& operator=(Parameters const& parameters);

public:
    BiQuadModel() :
        a1(),
        a2(),
        b0(),
        b1(),
        b2()
    {
        ;
    }

    BiQuadModel(BiQuadModel const &that)
    {
        *this = that;
    }

    BiQuadModel(Parameters const &parameters)
    {
        *this = parameters;
    }
};

// ----------------------------------------------------------------------------

template <typename Parameter_>
BiQuadModel<Parameter_> &BiQuadModel<Parameter_>::operator=(Parameters const &parameters)
{
    using std::sin;
    using std::cos;
    using std::sqrt;
    using std::sinh;

    Parameter const kZero = static_cast<Parameter const>(0);
    Parameter const kHalf = static_cast<Parameter const>(0.5);
    Parameter const kOne  = static_cast<Parameter const>(1);
    Parameter const kTwo  = static_cast<Parameter const>(2);

    Parameters::Behavior const behavior = parameters.GetBehavior();
    Parameter const dt = parameters.GetTimeDelta();
    Parameter const w = parameters.GetPulsation();
    Parameter const q = parameters.GetResonance();
    Parameter const g = parameters.GetGain();

    Parameter const phase = w * dt;
    Parameter const cosw = static_cast<Parameter const>(cos(phase));
    Parameter const sinw = static_cast<Parameter const>(sin(phase));
    Parameter a0, a1, a2, b0, b1, b2;

    switch (behavior) {
        case Parameters::kBehaviorLowPass: {
            // H(s) = 1 / (s^2 + s/Q + 1)

            Parameter const alpha = sinw / (q * kTwo);

            a0 = kOne         + alpha;
            a1 = cosw * -kTwo        ;
            a2 = kOne         - alpha;

            b0 = (kOne - cosw) * kHalf;
            b1 = (kOne - cosw)       ;
            b2 = (kOne - cosw) * kHalf;

            break;
        }
        case Parameters::kBehaviorHighPass: {
            // H(s) = s^2 / (s^2 + s/Q + 1)

            Parameter const alpha = sinw / (q * kTwo);

            a0 = kOne         + alpha;
            a1 = cosw * -kTwo        ;
            a2 = kOne         - alpha;

            b0 =  (kOne + cosw) * kHalf;
            b1 = -(kOne + cosw)       ;
            b2 =  (kOne + cosw) * kHalf;

            break;
        }
        case Parameters::kBehaviorBandPassConstantSkirt: {
            // H(s) = s / (s^2 + s/Q + 1)  [constant skirt gain, peak gain = Q]

            Parameter const alpha = sinw * static_cast<Parameter const>(sinh(q * w / sinw * (0.693147180559945309417 * 0.5)));

            a0 = kOne         + alpha;
            a1 = cosw * -kTwo        ;
            a2 = kOne         - alpha;

            b0 =  (q * alpha);
            b1 =  kZero      ;
            b2 = -(q * alpha);

            break;
        }
        case Parameters::kBehaviorBandPassConstantPeak: {
            // H(s) = (s/Q) / (s^2 + s/Q + 1)  [constant 0 dB peak gain]

            Parameter const alpha = sinw * static_cast<Parameter const>(sinh(q * w / sinw * (0.693147180559945309417 * 0.5)));

            a0 = kOne         + alpha;
            a1 = cosw * -kTwo        ;
            a2 = kOne         - alpha;

            b0 =  alpha;
            b1 =  kZero ;
            b2 = -alpha;

            break;
        }
        case Parameters::kBehaviorBandStop: {
            // H(s) = (s^2 + 1) / (s^2 + s/Q + 1)

            Parameter const alpha = sinw * static_cast<Parameter const>(sinh(q * w / sinw * (0.693147180559945309417 * 0.5)));

            a0 = kOne         + alpha;
            a1 = cosw * -kTwo        ;
            a2 = kOne         - alpha;

            b0 = kOne        ;
            b1 = cosw * -kTwo;
            b2 = kOne        ;

            break;
        }
        case Parameters::kBehaviorBandPassPeakingEq: {
            // H(s) = (s^2 + s*(A/Q) + 1) / (s^2 + s/(A*Q) + 1)

            Parameter const alpha = sinw * static_cast<Parameter const>(sinh(q * w / sinw * (0.693147180559945309417 * 0.5)));  // FIXME

            a0 = kOne         + alpha / g;
            a1 = cosw * -kTwo            ;
            a2 = kOne         - alpha / g;

            b0 = kOne         + alpha * g;
            b1 = cosw * -kTwo            ;
            b2 = kOne         - alpha * g;

            break;
        }
        case Parameters::kBehaviorLowShelf: {
            // H(s) = A * (s^2 + (sqrt(A)/Q)*s + A)/(A*s^2 + (sqrt(A)/Q)*s + 1)

            Parameter const am1 = g - kOne;
            Parameter const ap1 = g + kOne;
            Parameter const am1_cosw = am1 * cosw;
            Parameter const ap1_cosw = ap1 * cosw;
            Parameter const alpha = sinw * static_cast<Parameter const>(sqrt((g * g + kOne) * (kOne / (g * q) - kOne) + g * kTwo));

            a0 = (ap1 + am1_cosw + alpha)        ;
            a1 = (am1 + ap1_cosw        ) * -kTwo;
            a2 = (ap1 + am1_cosw - alpha)        ;

            b0 = (ap1 - am1_cosw + alpha) * g        ;
            b1 = (am1 - ap1_cosw        ) * g *  kTwo;
            b2 = (ap1 - am1_cosw - alpha) * g        ;

            break;
        }
        case Parameters::kBehaviorHighShelf: {
            // H(s) = A * (A*s^2 + (sqrt(A)/Q)*s + 1)/(s^2 + (sqrt(A)/Q)*s + A)

            Parameter const am1 = g - kOne;
            Parameter const ap1 = g + kOne;
            Parameter const am1_cosw = am1 * cosw;
            Parameter const ap1_cosw = ap1 * cosw;
            Parameter const alpha = sinw * static_cast<Parameter const>(sqrt((g * g + kOne) * (kOne / q - kOne) + g * kTwo));

            a0 = (ap1 - am1_cosw + alpha)        ;
            a1 = (am1 - ap1_cosw        ) *  kTwo;
            a2 = (ap1 - am1_cosw - alpha)        ;

            b0 = (ap1 + am1_cosw + alpha) * g        ;
            b1 = (am1 + ap1_cosw        ) * g * -kTwo;
            b2 = (ap1 + am1_cosw - alpha) * g        ;

            break;
        }
        case Parameters::kBehaviorAllPass: {
            // H(s) = (s^2 - s/Q + 1) / (s^2 + s/Q + 1)

            Parameter const alpha = sinw / (q * kTwo);

            a0 = kOne         + alpha;
            a1 = cosw * -kTwo        ;
            a2 = kOne         - alpha;

            b0 = kOne         - alpha;
            b1 = cosw * -kTwo        ;
            b2 = kOne         + alpha;

            break;
        }
        case Parameters::kBehaviorUnity:
        default: {
            // H(s) = 1

            a0 = kOne;
            a1 = kZero;
            a2 = kZero;

            b0 = kOne;
            b1 = kZero;
            b2 = kZero;

            break;
        }
    }

    Parameter const ra0 = kOne / a0;

    this->a1 = a1 * -ra0;
    this->a2 = a2 * -ra0;

    this->b0 = b0 * ra0;
    this->b1 = b1 * ra0;
    this->b2 = b2 * ra0;

    return *this;
}

// ----------------------------------------------------------------------------

template <typename Accumulator_>
class BiQuadState
{
public:
    typedef Accumulator_ Accum;

public:
    Accum x0;
    Accum x1;
    Accum x2;

    Accum y0;
    Accum y1;
    Accum y2;

public:
    void Clear(Accum const y = static_cast<Accum const>(0))
    {
        x0 = static_cast<Accum const>(0);
        x1 = static_cast<Accum const>(0);
        x2 = static_cast<Accum const>(0);

        y0 = y;
        y1 = y;
        y2 = y;
    }

    BiQuadState &operator=(BiQuadState const &that)
    {
        x0 = that.x0;
        x1 = that.x1;
        x2 = that.x2;

        y0 = that.y0;
        y1 = that.y1;
        y2 = that.y2;

        return *this;
    }

public:
    BiQuadState()
    {
        ;
    }

    BiQuadState(BiQuadState const &that)
    {
        *this = that;
    }

    BiQuadState(Accum const y)
    {
        Clear(y);
    }
};

// ----------------------------------------------------------------------------

template <typename Accumulator_,
         typename Input_,
         typename Output_>
class BiQuadFilter
{
public:
    typedef Accumulator_ Accumulator;
    typedef Input_ Input;
    typedef Output_ Output;

    typedef BiQuadModel<Accumulator> Model;
    typedef BiQuadState<Accumulator> State;

protected:
    Model const *model_;
    State *state_;  // TODO: pointer

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
        Accumulator const a1 = m.a1, a2 = m.a2;
        Accumulator const b0 = m.b0, b1 = m.b1, b2 = m.b2;
        State &s = *state_;
        Accumulator x0 = s.x0, x1 = s.x1, x2 = s.x2;
        Accumulator y0 = s.y0, y1 = s.y1, y2 = s.y2;

        while (count--) {
            x2 = x1;
            x1 = x0;
            x0 = static_cast<Accumulator>(*inputs++);

            y2 = y1;
            y1 = y0;
            y0 = ((y1 * a1) + (y2 * a2) +
                  (x0 * b0) + (x1 * b1) + (x2 * b2));

            *outputs++ = static_cast<Output>(y0);
        }

        s.x0 = x0, s.x1 = x1, s.x2 = x2;
        s.y0 = y0, s.y1 = y1, s.y2 = y2;
    }

    Output const operator()(Input const input)
    {
        Model const &m = *model_;
        State &s = *state_;

        s.x2 = s.x1;
        s.x1 = s.x0;
        s.x0 = static_cast<Accumulator>(input);

        s.y2 = s.y1;
        s.y1 = s.y0;
        s.y0 = ((s.y1 * m.a1) + (s.y2 * m.a2) +
                (s.x0 * m.b0) + (s.x1 * m.b1) + (s.x2 * m.b2));

        return static_cast<Output>(s.y0);
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

    BiQuadFilter &operator=(BiQuadFilter const &that)
    {
        model_ = that.model_;
        state_ = that.state_;
        return *this;
    }

public:
    BiQuadFilter()
    {
        ;
    }

    BiQuadFilter(BiQuadFilter const &that)
    {
        *this = that;
    }

    BiQuadFilter(Model const *model,
                 State *state,
                 Accumulator const y = static_cast<Accumulator const>(0)) :
        model_(model),
        state_(state)
    {
        state_.Clear(y);
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace dsp
}  // namespace calfmoo

// ============================================================================

#endif  // _CALFMOO_DSP_BIQUAD_HPP_
