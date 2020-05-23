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

#ifndef _CALFMOO_DSP_BUTTERWORTH_HPP_
#define _CALFMOO_DSP_BUTTERWORTH_HPP_

#include "dsp-onepole.hpp"
#include "dsp-biquad.hpp"
#include "utils-ringbuffer.hpp"

#include <cmath>

// ============================================================================

namespace calfmoo {
namespace dsp {

#pragma pack(push)
#pragma pack()

// ============================================================================

template <unsigned kOrder_, typename Parameter_>
class ButterworthParameters;

template <unsigned kOrder_, typename Parameter_>
struct ButterworthModel;

template <unsigned kOrder_, typename Accumulator_>
class ButterworthState;

template <unsigned kOrder_, typename Accumulator_, typename Input_, typename Output_>
class ButterworthFilter;

// ============================================================================

template <unsigned kOrder_,
         typename Parameter_>
class ButterworthParameters
{
public:
    typedef Parameter_ Parameter;

    typedef OnePoleParameters<Parameter> OddParameters;
    typedef BiQuadParameters<Parameter> EvenParameters;

    enum Behavior {
        kBehaviorUnity    = EvenParameters::kBehaviorUnity,
        kBehaviorLowPass  = EvenParameters::kBehaviorLowPass,
        kBehaviorHighPass = EvenParameters::kBehaviorHighPass,
        kBehaviorAllPass  = EvenParameters::kBehaviorAllPass,
    };

    enum {
        kOrder            = kOrder_,
        kOddFilterCount   = kOrder % 2,
        kEvenFilterCount  = kOrder / 2,
        kEvenFilterLength = kEvenFilterCount ? kEvenFilterCount : 1,
    };

protected:
    OddParameters odd_;
    EvenParameters even_[kEvenFilterLength];

public:
    OddParameters const *GetOdd() const
    {
        if (kOddFilterCount) {
            return &odd_;
        }
        else {
            return NULL;
        }
    }

    EvenParameters const *GetEven(unsigned stage) const
    {
        if (stage < kEvenFilterCount) {
            return &even_[stage];
        }
        else {
            return NULL;
        }
    }

    Behavior const GetBehavior() const
    {
        if (kEvenFilterCount) {
            return static_cast<Behavior>(even_[0].GetBehavior());
        }
        else {
            return kBehaviorUnity;
        }
    }

    void SetBehavior(Behavior const behavior)
    {
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            even_[i].SetBehavior(static_cast<EvenParameters::Behavior>(behavior));
        }
    }

    Parameter const GetTimeDelta() const
    {
        if (kOddFilterCount) {
            return odd_.GetTimeDelta();
        }
        else if (kEvenFilterCount) {
            return even_[0].GetTimeDelta();
        }
    }

    void SetTimeDelta(Parameter const dt)
    {
        if (kOddFilterCount) {
            odd_.SetTimeDelta(dt);
        }
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            even_[i].SetTimeDelta(dt);
        }
    }

    Parameter const GetSampleRate() const
    {
        if (kOddFilterCount) {
            return odd_.GetSampleRate();
        }
        else if (kEvenFilterCount) {
            return even_[0].GetSampleRate();
        }
    }

    void SetSampleRate(Parameter const fs)
    {
        if (kOddFilterCount) {
            odd_.SetSampleRate(fs);
        }
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            even_[i].SetSampleRate(fs);
        }
    }

    Parameter const GetPulsation() const
    {
        if (kOddFilterCount) {
            return odd_.GetPulsation();
        }
        else if (kEvenFilterCount) {
            return even_[0].GetPulsation();
        }
        else {
            return static_cast<Parameter>(0);
        }
    }

    void SetPulsation(Parameter const w)
    {
        if (kOddFilterCount) {
            odd_.SetPulsation(w);
        }
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            even_[i].SetPulsation(w);
        }
    }

    Parameter const GetFrequency() const
    {
        if (kOddFilterCount) {
            return odd_.GetFrequency();
        }
        else if (kEvenFilterCount) {
            return even_[0].GetFrequency();
        }
        else {
            return static_cast<Parameter>(0);
        }
    }

    void SetFrequency(Parameter const f)
    {
        Parameter const w = f * static_cast<Parameter>(2 * 3.14159265358979323846);
        if (kOddFilterCount) {
            odd_.SetPulsation(w);
        }
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            even_[i].SetPulsation(w);
        }
    }

    Parameter const GetResonance(unsigned even_stage) const
    {
        if (even_stage < kEvenFilterCount) {
            return even_[even_stage].GetResonance();
        }
        else {
            return static_cast<Parameter>(0);
        }
    }

    void UpdateResonance()
    {
        Parameter const kHalf = static_cast<Parameter>(0.5);
        Parameter const kDenom = static_cast<Parameter>(kOrder);
        Parameter const kDelta = static_cast<Parameter>(3.14159265358979323846) / kDenom;
        Parameter const kStart = kDelta / (2 - kOddFilterCount);

        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            Parameter const q = static_cast<Parameter>(kHalf / cos(kDelta * i + kStart));
            even_[i].SetResonance(q);
        }
    }

    ButterworthParameters &operator=(ButterworthParameters const &that)
    {
        if (kOddFilterCount) {
            odd_ = that.odd_;
        }
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            even_[i] = that.even_[i];
        }
    }

public:
    ButterworthParameters()
    {
        ;
    }

    ButterworthParameters(ButterworthParameters const &that)
    {
        *this = that;
    }

    ButterworthParameters(Behavior const behavior,
                          Parameter const fs,
                          Parameter const f)
    {
        SetBehavior(behavior);
        SetSampleRate(fs);
        SetFrequency(f);
        UpdateResonance();
    }
};

// ----------------------------------------------------------------------------

template <unsigned kOrder_,
         typename Parameter_>
struct ButterworthModel
{
public:
    typedef Parameter_ Parameter;

    enum {
        kOrder            = kOrder_,
        kOddFilterCount   = kOrder % 2,
        kEvenFilterCount  = kOrder / 2,
        kEvenFilterLength = kEvenFilterCount ? kEvenFilterCount : 1,
    };

    typedef ButterworthParameters<kOrder, Parameter> Parameters;

    typedef OnePoleModel<Parameter> OddModel;
    typedef BiQuadModel<Parameter> EvenModel;

public:
    OddModel odd_;
    EvenModel even_[kEvenFilterLength];

public:
    OddModel const *GetOdd() const
    {
        if (kOddFilterCount) {
            return &odd_;
        }
        else {
            return NULL;
        }
    }

    EvenModel const *GetEven(unsigned stage) const
    {
        if (stage < kEvenFilterCount) {
            return &even_[stage];
        }
        else {
            return NULL;
        }
    }

    ButterworthModel &operator=(Parameters const &parameters)
    {
        if (kOddFilterCount) {
            odd_ = *parameters.GetOdd();
        }
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            even_[i] = *parameters.GetEven(i);
        }
        return *this;
    }

    ButterworthModel &operator=(ButterworthModel const &that)
    {
        odd_ = that.odd_;
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            even_[i] = that.even_[i];
        }
        return *this;
    }

public:
    ButterworthModel()
    {
        ;
    }

    ButterworthModel(ButterworthModel const &that)
    {
        *this = that;
    }

    ButterworthModel(Parameters const &parameters)
    {
        Apply(parameters);
    }
};

// ----------------------------------------------------------------------------

template <unsigned kOrder_,
         typename Accumulator_>
class ButterworthState
{
public:
    typedef Accumulator_ Accumulator;

    enum {
        kOrder            = kOrder_,
        kOddFilterCount   = kOrder % 2,
        kEvenFilterCount  = kOrder / 2,
        kEvenFilterLength = kEvenFilterCount ? kEvenFilterCount : 1,
    };

    typedef OnePoleState<Accumulator> OddState;
    typedef BiQuadState<Accumulator> EvenState;

public:
    OddState odd_;
    EvenState even_[kEvenFilterLength];

public:
    OddState *GetOdd()
    {
        if (kOddFilterCount) {
            return &odd_;
        }
        else {
            return NULL;
        }
    }

    EvenState *GetEven(unsigned stage)
    {
        if (stage < kEvenFilterCount) {
            return &even_[stage];
        }
        else {
            return NULL;
        }
    }

    void Clear(Accumulator const y = static_cast<Accumulator>(0))
    {
        if (kOddFilterCount) {
            odd_.Clear(y);
        }
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            even_[i].Clear(y);
        }
    }

    ButterworthState &operator=(ButterworthModel<kOrder, Accumulator> const &that)
    {
        odd_ = that.odd_;
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            even_[i] = that.even_[i];
        }
        return *this;
    }

public:
    ButterworthState()
    {
        ;
    }

    ButterworthState(ButterworthState const &that)
    {
        *this = that;
    }

    ButterworthState(Accumulator const y)
    {
        Clear(y);
    }
};


// ----------------------------------------------------------------------------

template <unsigned kOrder_,
         typename Accumulator_,
         typename Input_,
         typename Output_>
class ButterworthFilter
{
public:
    typedef Accumulator_ Accumulator;
    typedef Input_ Input;
    typedef Output_ Output;

    enum {
        kOrder            = kOrder_,
        kOddFilterCount   = kOrder % 2,
        kEvenFilterCount  = kOrder / 2,
        kEvenFilterLength = kEvenFilterCount ? kEvenFilterCount : 1,
    };

    typedef ButterworthModel<kOrder, Accumulator> Model;
    typedef ButterworthState<kOrder, Accumulator> State;

    typedef OnePoleFilter<Accumulator, Input, Output> OddFilter;
    typedef BiQuadFilter<Accumulator, Input, Output> EvenFilter;

protected:
    Model const *model_;
    State *state_;
    OddFilter odd_;
    EvenFilter even_[kEvenFilterLength];

public:
    Model const *GetModel() const
    {
        return model_;
    }

    void SetModel(Model const *model)
    {
        model_ = model;

        if (model) {
            odd_.SetModel(model->GetOdd());
            for (unsigned i = 0; i < kEvenFilterCount; ++i) {
                even_[i].SetModel(model->GetEven(i));
            }
        }
    }

    State *GetState() const
    {
        return state_;
    }

    void SetState(State *state)
    {
        state_ = state;

        if (state) {
            odd_.SetState(state->GetOdd());
            for (unsigned i = 0; i < kEvenFilterCount; ++i) {
                even_[i].SetState(state->GetEven(i));
            }
        }
        else {
            odd_.SetState(NULL);
            for (unsigned i = 0; i < kEvenFilterCount; ++i) {
                even_[i].SetState(NULL);
            }
        }
    }

    void operator()(size_t count, Input const *inputs, Output *outputs)
    {
        while (count--) {
            Accumulator y = static_cast<Accumulator>(*inputs++);
            if (kOddFilterCount) {
                y = odd_(y);
            }
            for (unsigned i = 0; i < kEvenFilterCount; ++i) {
                y = even_[i](y);
            }
            *outputs++ = static_cast<Output>(y);
        }
    }

    Output const operator()(Input const input)
    {
        Accumulator y = static_cast<Accumulator>(input);
        if (kOddFilterCount) {
            y = odd_(y);
        }
        for (unsigned i = 0; i < kEvenFilterCount; ++i) {
            y = even_[i](y);
        }
        return static_cast<Output>(y);
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

    ButterworthFilter &operator=(ButterworthFilter const &that)
    {
        SetModel(that.model_);
        SetState(that.state_);
        return *this;
    }

public:
    ButterworthFilter() :
        model_(),
        state_()
    {
        (void)odd_;
        (void)even_;
    }

    ButterworthFilter(ButterworthFilter const &that)
    {
        *this = that;
    }

    ButterworthFilter(Model const *model,
                      State *state,
                      Accumulator const y = static_cast<Accumulator>(0))
    {
        SetModel(model);
        SetState(state);
        if (state) {
            state.Clear(y);
        }
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace dsp
}  // namespace calfmoo

   // ============================================================================

#endif  // _CALFMOO_DSP_BUTTERWORTH_HPP_
