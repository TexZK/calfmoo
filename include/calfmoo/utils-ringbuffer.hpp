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

#ifndef _CALFMOO_UTILS_RINGBUFFER_HPP_
#define _CALFMOO_UTILS_RINGBUFFER_HPP_

#include <cstddef>
#include <cstring>
#include <algorithm>

// ============================================================================

namespace calfmoo {
namespace utils {

#pragma pack(push)
#pragma pack()

// ============================================================================

template <typename Item_, typename Index_ = size_t>
class RingBuffer
{
public:
    typedef Item_ Item;
    typedef Index_ Index;

protected:
    Item* buffer_;
    Index length_;
    Index free_;
    Index head_;
    Index tail_;

public:
    Item* GetBuffer() const
    {
        return &buffer_[0];
    }

    Index GetLength() const
    {
        return length_;
    }

    Index GetFree() const
    {
        return free_;
    }

    Index GetUsed() const
    {
        return length_ - free_;
    }

    Index GetHeadIndex() const
    {
        return head_;
    }

    Index GetHeadMargin() const
    {
        return length_ - head_;
    }

    Item* GetHeadPointer() const
    {
        return &buffer_[head_];
    }

    Index GetTailIndex() const
    {
        return tail_;
    }

    Index GetTailMargin() const
    {
        return length_ - tail_;
    }

    Item* GetTailPointer() const
    {
        return &buffer_[tail_];
    }

    void Initialize(Index length, Item buffer[])
    {
        buffer_ = buffer;
        length_ = length;
        free_ = length;
        head_ = 0;
        tail_ = 0;
    }

    Index TrimEnqueue(Index length) const
    {
        return std::min(length, free_);
    }

    Index TrimEnqueueContiguous(Index length) const
    {
        return std::min(TrimEnqueue(length), GetTailMargin());
    }

    Index GetEnqueueContiguous() const
    {
        return std::min(free_, GetTailMargin());
    }

    Index SkipTail(Index length)
    {
        length = TrimEnqueue(length);
        free_ -= length;
        tail_ = (tail_ + length) % length_;
        return length;
    }

    Index Enqueue(Index length, Item const buffer[])
    {
        length = TrimEnqueue(length);
        free_ -= length;
        Index margin = GetTailMargin();
        if (length <= margin) {
            std::copy(&buffer[0], &buffer[length], &buffer_[tail_]);
        }
        else {
            std::copy(&buffer[0], &buffer[margin], &buffer_[tail_]);
            std::copy(&buffer[margin], &buffer[length], &buffer_[0]);
        }
        tail_ = (tail_ + length) % length_;
        return length;
    }

    void Enqueue(Item item)
    {
        Enqueue(1, &item);
    }

    Index Fill(Index length, Item const value)
    {
        Index total = 0;
        while (length && free_) {
            Index count = TrimEnqueueContiguous(length);
            free_ -= count;
            std::fill(&buffer_[tail_], &buffer_[tail_ + count], value);
            tail_ = (tail_ + count) % length_;
            length -= count;
            total += count;
        }
        return total;
    }

    Index TrimDeqeueue(Index length) const
    {
        return std::min(length, GetUsed());
    }

    Index TrimDequeueContiguous(Index length) const
    {
        return std::min(TrimDeqeueue(length), GetHeadMargin());
    }

    Index GetDequeueContiguous() const
    {
        return std::min(GetUsed(), GetHeadMargin());
    }

    Index SkipHead(Index length)
    {
        length = TrimDeqeueue(length);
        head_ = (head_ + length) % length_;
        free_ += length;
        return length;
    }

    Index Dequeue(Index length, Item buffer[])
    {
        length = TrimDeqeueue(length);
        Index margin = GetHeadMargin();
        if (length <= margin) {
            std::copy(&buffer_[head_], &buffer_[head_ + length], &buffer[0]);
        }
        else {
            std::copy(&buffer_[head_], &buffer_[length_], &buffer[0]);
            std::copy(&buffer_[0], &buffer_[length - margin], &buffer[margin]);
        }
        head_ = (head_ + length) % length_;
        free_ += length;
        return length;
    }

    Item Dequeue()
    {
        Item item;
        Dequeue(1, &item);
        return item;
    }

    bool IsFragmented() const
    {
        return head_ > tail_;
    }

    void Compact()
    {
        if (head_) {
            if (IsFragmented()) {
                Index margin = GetHeadMargin();
                if (margin <= free_) {
                    std::copy(&buffer_[0], &buffer_[tail_], &buffer_[margin]);
                    std::copy(&buffer_[head_], &buffer_[length_], &buffer_[0]);
                }
                else {
                    std::rotate(&buffer_[0], &buffer_[length_ - head_], &buffer_[length_]);
                }
            }
            else {
                std::copy(&buffer_[head_], &buffer_[tail_], &buffer_[0]);
            }
            head_ = 0;
            tail_ = GetUsed() % length_;
        }
    }

    void Clear()
    {
        free_ = length_;
        head_ = 0;
        tail_ = 0;
    }

    Item& operator[](int index) const
    {
        return buffer_[index];
    }

public:
    RingBuffer() :
        buffer_(NULL),
        length_(0),
        free_(0),
        head_(0),
        tail_(0)
    {
        ;
    }

    RingBuffer(Index length, Item buffer[], Index tail = 0, Index head = 0) :
        buffer_(buffer),
        length_(length)
    {
        free_ = length - std::min(length, tail);
        head_ = head % length;
        tail_ = tail % length;
    }

public:
    static Index Move(RingBuffer& src, RingBuffer& dst, Index length)
    {
        Index total = 0;
        while (dst.GetFree() && src.GetUsed()) {
            Index count = src.TrimDequeueContiguous(dst.TrimEnqueueContiguous(length));
            Item* src_head = src.GetHeadPointer();
            Item* dst_tail = dst.GetTailPointer();

            std::copy(&src_head[0], &src_head[count], &dst_tail[0]);

            src.SkipHead(count);
            dst.SkipTail(count);
            total += count;
            length -= count;
        }
        return total;
    }

    static Index Move(RingBuffer& src, RingBuffer& dst)
    {
        return Move(src, dst, dst.GetLength());
    }
};

// ============================================================================

#pragma pack(pop)

}  // namespace utils
}  // namespace calfmoo

// ============================================================================

#endif  // _CALFMOO_UTILS_RINGBUFFER_HPP_
