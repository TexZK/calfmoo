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

#ifndef _CALFMOO_YM7128B_HPP_
#define _CALFMOO_YM7128B_HPP_

#include <cstdint>

#include "YM7128B_emu.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4309)
#endif

// ============================================================================

namespace calfmoo {
namespace ym7128b {

#pragma pack(push)
#pragma pack()

// ============================================================================

typedef YM7128B_Address Address;
typedef YM7128B_Register Register;
typedef YM7128B_Tap Tap;

enum class Reg {
    GL1     = YM7128B_Reg_GL1,
    GL2     = YM7128B_Reg_GL2,
    GL3     = YM7128B_Reg_GL3,
    GL4     = YM7128B_Reg_GL4,
    GL5     = YM7128B_Reg_GL5,
    GL6     = YM7128B_Reg_GL6,
    GL7     = YM7128B_Reg_GL7,
    GL8     = YM7128B_Reg_GL8,

    GR1     = YM7128B_Reg_GR1,
    GR2     = YM7128B_Reg_GR2,
    GR3     = YM7128B_Reg_GR3,
    GR4     = YM7128B_Reg_GR4,
    GR5     = YM7128B_Reg_GR5,
    GR6     = YM7128B_Reg_GR6,
    GR7     = YM7128B_Reg_GR7,
    GR8     = YM7128B_Reg_GR8,

    VM      = YM7128B_Reg_VM,
    VC      = YM7128B_Reg_VC,
    VL      = YM7128B_Reg_VL,
    VR      = YM7128B_Reg_VR,

    C0      = YM7128B_Reg_C0,
    C1      = YM7128B_Reg_C1,

    T0      = YM7128B_Reg_T0,
    T1      = YM7128B_Reg_T1,
    T2      = YM7128B_Reg_T2,
    T3      = YM7128B_Reg_T3,
    T4      = YM7128B_Reg_T4,
    T5      = YM7128B_Reg_T5,
    T6      = YM7128B_Reg_T6,
    T7      = YM7128B_Reg_T7,
    T8      = YM7128B_Reg_T8,

    Count   = YM7128B_Reg_Count,
};

enum class InputChannel {
    Mono    = YM7128B_InputChannel_Mono,
    Count   = YM7128B_InputChannel_Count,
};

enum class OutputChannel {
    Left    = YM7128B_OutputChannel_Left,
    Right   = YM7128B_OutputChannel_Right,
    Count   = YM7128B_OutputChannel_Count,
};

enum class ChipInfo {
    ClockRate       = YM7128B_Clock_Rate,
    WriteRate       = YM7128B_Write_Rate,
    InputRate       = YM7128B_Input_Rate,
    Oversampling    = YM7128B_Oversampling,
    OutputRate      = YM7128B_Output_Rate,
    AddressMax      = YM7128B_Reg_Count,
    BufferLength    = YM7128B_Buffer_Length,

    TapCount        = YM7128B_Tap_Count,
    TapValueBits    = YM7128B_Tap_Value_Bits,
    TapValueCount   = YM7128B_Tap_Value_Count,
    TapValueMask    = YM7128B_Tap_Value_Mask,

    GainLaneCount   = YM7128B_Gain_Lane_Count,
    GainDataBits    = YM7128B_Gain_Data_Bits,
    GainDataCount   = YM7128B_Gain_Data_Count,
    GainDataMask    = YM7128B_Gain_Data_Mask,
    GainDataSign    = YM7128B_Gain_Data_Sign,

};

// ============================================================================

typedef YM7128B_Fixed       Fixed;
typedef YM7128B_Accumulator Accumulator;
typedef YM7128B_Float       Gain;

enum class FixedInfo {
    Bits        = YM7128B_Fixed_Bits,
    Mask        = YM7128B_Fixed_Mask,
    Decimals    = YM7128B_Fixed_Decimals,
    Rounding    = YM7128B_Fixed_Rounding,
    Max         = YM7128B_Fixed_Max,
    Min         = YM7128B_Fixed_Min,
};

enum class SignalInfo {
    Bits        = YM7128B_Signal_Bits,
    ClearBits   = YM7128B_Signal_Clear_Bits,
    ClearMask   = YM7128B_Signal_Clear_Mask,
    Mask        = YM7128B_Signal_Mask,
    Decimals    = YM7128B_Signal_Decimals,
};

enum class GainInfo {
    Max         = YM7128B_Gain_Max,
    Min         = YM7128B_Gain_Min,
};

enum class CoeffInfo {
    Count       = YM7128B_Coeff_Count,
    ValueBits   = YM7128B_Coeff_Value_Bits,
    ValueCount  = YM7128B_Coeff_Value_Count,
    ValueMask   = YM7128B_Coeff_Value_Mask,

    Bits        = YM7128B_Coeff_Bits,
    ClearBits   = YM7128B_Coeff_Clear_Bits,
    ClearMask   = YM7128B_Coeff_Clear_Mask,
    Mask        = YM7128B_Coeff_Mask,
    Decimals    = YM7128B_Coeff_Decimals,
};

// ============================================================================

template <typename Input_, typename Output_, typename Rate_, typename Index_>
class ChipBase
{
public:
    typedef Input_  Input;
    typedef Output_ Output;
    typedef Rate_   Rate;
    typedef Index_  Index;

public:
    virtual void Start(Rate sample_rate, Index buffer_length) = 0;
    virtual void Stop() = 0;

    virtual Register Read(Address address) const = 0;
    virtual void Write(Address address, Register data) = 0;

    virtual void operator()(size_t count, Input const* inputs, Output** outputs) = 0;

    virtual ~ChipBase() = 0 {}
};

// ============================================================================

template <typename Input_, typename Output_, typename Rate_, typename Index_>
class ChipFixed : public ChipBase<Input_, Output_, Rate_, Index_>
{
private:
    YM7128B_ChipFixed impl_;

public:
    ChipFixed()
    {
        YM7128B_ChipFixed_Ctor(&impl_);
        YM7128B_ChipFixed_Reset(&impl_);
    }

    ~ChipFixed()
    {
        YM7128B_ChipFixed_Dtor(&impl_);
    }

    void Start(Rate sample_rate, Index block_size)
    {
        (void)sample_rate;
        (void)block_size;
        YM7128B_ChipFixed_Start(&impl_);
    }

    void Stop()
    {
        YM7128B_ChipFixed_Stop(&impl_);
    }

    void operator()(size_t count, Input const* inputs, Output** outputs)
    {
        YM7128B_ChipFixed_Process_Data data;

        for (size_t i = 0; i < count; ++i) {
            for (unsigned c = 0; c < YM7128B_InputChannel_Count; ++c) {
                Input const k = static_cast<Input>(YM7128B_Fixed_Max);
                YM7128B_Fixed input = static_cast<YM7128B_Fixed>(inputs[i] * k);
                data.inputs[c] = input;
            }

            YM7128B_ChipFixed_Process(&impl_, &data);

            for (unsigned c = 0; c < YM7128B_OutputChannel_Count; ++c) {
                for (unsigned n = 0; n < YM7128B_Oversampling; ++n) {
                    Output const k = static_cast<Output>(1) / static_cast<Output>(YM7128B_Fixed_Max);
                    Output const v = static_cast<Output>(data.outputs[c][n]);
                    outputs[c][i * YM7128B_Oversampling + n] = v * k;
                }
            }
        }
    }

    Register Read(Address address) const
    {
        return YM7128B_ChipFixed_Read(&impl_, address);
    }

    void Write(Address address, Register data)
    {
        YM7128B_ChipFixed_Write(&impl_, address, data);
    }
};

// ============================================================================

template <typename Input_, typename Output_, typename Rate_, typename Index_>
class ChipFloat : public ChipBase<Input_, Output_, Rate_, Index_>
{
private:
    YM7128B_ChipFloat impl_;

public:
    ChipFloat()
    {
        YM7128B_ChipFloat_Ctor(&impl_);
        YM7128B_ChipFloat_Reset(&impl_);
    }

    ~ChipFloat()
    {
        YM7128B_ChipFloat_Dtor(&impl_);
    }

    void Start(Rate sample_rate, Index block_size)
    {
        (void)sample_rate;
        (void)block_size;
        YM7128B_ChipFloat_Start(&impl_);
    }

    void Stop()
    {
        YM7128B_ChipFloat_Stop(&impl_);
    }

    void operator()(size_t count, Input const* inputs, Output** outputs)
    {
        YM7128B_ChipFloat_Process_Data data;

        for (size_t i = 0; i < count; ++i) {
            for (unsigned c = 0; c < YM7128B_InputChannel_Count; ++c) {
                YM7128B_Float input = static_cast<YM7128B_Float>(inputs[i]);
                data.inputs[c] = input;
            }

            YM7128B_ChipFloat_Process(&impl_, &data);

            for (unsigned c = 0; c < YM7128B_OutputChannel_Count; ++c) {
                for (unsigned n = 0; n < YM7128B_Oversampling; ++n) {
                    Output const v = static_cast<Output>(data.outputs[c][n]);
                    outputs[c][i * YM7128B_Oversampling + n] = v;
                }
            }
        }
    }

    Register Read(Address address) const
    {
        return YM7128B_ChipFloat_Read(&impl_, address);
    }

    void Write(Address address, Register data)
    {
        YM7128B_ChipFloat_Write(&impl_, address, data);
    }
};

// ============================================================================

template <typename Input_, typename Output_, typename Rate_, typename Index_>
class ChipIdeal : public ChipBase<Input_, Output_, Rate_, Index_>
{
private:
    YM7128B_ChipIdeal impl_;

public:
    ChipIdeal()
    {
        YM7128B_ChipIdeal_Ctor(&impl_);
        YM7128B_ChipIdeal_Reset(&impl_);
    }

    ~ChipIdeal()
    {
        YM7128B_ChipIdeal_Dtor(&impl_);
    }

    void Start(Rate sample_rate, Index block_size)
    {
        (void)block_size;
        YM7128B_TapIdeal tap_rate = static_cast<YM7128B_TapIdeal>(sample_rate);
        YM7128B_ChipIdeal_SetSampleRate(&impl_, tap_rate);
        YM7128B_ChipIdeal_Start(&impl_);
    }

    void Stop()
    {
        YM7128B_ChipIdeal_Stop(&impl_);
    }

    void operator()(size_t count, Input const* inputs, Output** outputs)
    {
        YM7128B_ChipIdeal_Process_Data data;

        for (size_t i = 0; i < count; ++i) {
            for (unsigned c = 0; c < YM7128B_InputChannel_Count; ++c) {
                YM7128B_Float input = static_cast<YM7128B_Float>(inputs[i]);
                data.inputs[c] = input;
            }

            YM7128B_ChipIdeal_Process(&impl_, &data);

            for (unsigned c = 0; c < YM7128B_OutputChannel_Count; ++c) {
                Output const v = static_cast<Output>(data.outputs[c]);
                outputs[c][i] = v;
            }
        }
    }

    Register Read(Address address) const
    {
        return YM7128B_ChipIdeal_Read(&impl_, address);
    }

    void Write(Address address, Register data)
    {
        YM7128B_ChipIdeal_Write(&impl_, address, data);
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace ym7128b
}  // namespace calfmoo

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif  // !_CALFMOO_YM7128B_HPP_
