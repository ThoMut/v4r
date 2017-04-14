/******************************************************************************
 * Copyright (c) 2015 Thomas Faeulhammer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/

#pragma once

namespace v4r
{
        enum FeatureType
        {
            SIFT_GPU = 0x01, // 00000001
            SIFT_OPENCV = 0x02, // 00000010
            SHOT  = 0x04, // 00000100
            OURCVFH  = 0x08,  // 00001000
            FPFH = 0x10,  // 00010000
            ESF = 0x20,  // 00100000
            SHOT_COLOR = 0x40,  // 01000000
#if PCL_VERSION >= 100702
            ALEXNET = 0x80,  // 10000000
            ROPS = 0x200,  // 10000000
#else
            ALEXNET = 0x80,
#endif
            SIMPLE_SHAPE = 0x400,
            GLOBAL_COLOR = 0x800
        };
}
