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

#ifndef _CALFMOO_UTILS_RATESLOTTER_HPP_
#define _CALFMOO_UTILS_RATESLOTTER_HPP_

// ============================================================================

namespace calfmoo {
namespace utils {

#pragma pack(push)
#pragma pack()

// ============================================================================

template <typename Rate_ = unsigned>
class RateSlotter
{
public:
    typedef Rate_ Rate;

protected:
    Rate async_rate_;
    Rate call_rate_;
    Rate remainder_;

public:
    Rate GetAsyncRate() const
    {
        return async_rate_;
    }

    Rate GetCallRate() const
    {
        return call_rate_;
    }

    Rate GetRemainder() const
    {
        return remainder_;
    }

    void SetRemainder(Rate remainder)
    {
        remainder_ = remainder;
    }

    void Reset(Rate async_rate, Rate call_rate, Rate remainder = 0)
    {
        async_rate_ = async_rate;
        call_rate_ = call_rate;
        remainder_ = remainder;
    }

    void Clear(Rate remainder = 0)
    {
        remainder_ = remainder;
    }

    Rate operator()()
    {
        Rate error = remainder_ + async_rate_;
        Rate count = error / call_rate_;
        remainder_ = error % call_rate_;
        return count;
    }

public:
    RateSlotter() :
        async_rate_(0),
        call_rate_(0),
        remainder_(0)
    {
        ;
    }

    RateSlotter(Rate async_rate, Rate call_rate, Rate remainder = 0)
    {
        Reset(async_rate, call_rate, remainder);
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace utils
}  // namespace calfmoo

#endif  // !_CALFMOO_UTILS_RATESLOTTER_HPP_
