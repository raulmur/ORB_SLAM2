/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PANGOLIN_FIX_SIZE_BUFFER_QUEUE_H
#define PANGOLIN_FIX_SIZE_BUFFER_QUEUE_H

#include <list>

#include <pangolin/compat/thread.h>
#include <pangolin/compat/mutex.h>
#include <pangolin/compat/condition_variable.h>
#include <pangolin/compat/locks.h>

namespace pangolin
{

template<typename BufPType>
class FixSizeBuffersQueue
{

public:
    FixSizeBuffersQueue() {}

    ~FixSizeBuffersQueue() {
//        // Deallocate everything.
//        boostd::lock_guard<boostd::mutex> vlock(vMtx);
//        while(validBuffers.size() > 0){
//            delete[] validBuffers.front();
//            validBuffers.pop_front();
//        }
//        boostd::lock_guard<boostd::mutex> elock(eMtx);
//        while(emptyBuffers.size() > 0){
//            delete[] emptyBuffers.front();
//            emptyBuffers.pop_front();
//        }
    }

//    void init(unsigned int num, unsigned int sizeBytes) {
//        maxNumBuffers = num;
//        bufferSizeBytes = sizeBytes;
//        // lock queue
//        boostd::lock_guard<boostd::mutex> vlock(vMtx);
//        boostd::lock_guard<boostd::mutex> elock(eMtx);

//        // Put back any valid buffer to the available buffers queue.
//        while(validBuffers.size() > 0){
//            emptyBuffers.push_back(validBuffers.front());
//            validBuffers.pop_front();
//        }
//        // Allocate buffers
//        while(emptyBuffers.size() < maxNumBuffers) {
//            emptyBuffers.push_back(new unsigned char[bufferSizeBytes]);
//        }
//    }

    BufPType getNewest() {
        boostd::lock_guard<boostd::mutex> vlock(vMtx);
        boostd::lock_guard<boostd::mutex> elock(eMtx);
        if(validBuffers.size() == 0) {
            // Empty queue.
            return 0;
        } else {
            // Requeue all but newest buffers.
            while(validBuffers.size() > 1) {
                emptyBuffers.push_back(validBuffers.front());
                validBuffers.pop_front();
            }
            // Return newest buffer.
            BufPType bp = validBuffers.front();
            validBuffers.pop_front();
            return bp;
        }
    }

    BufPType getNext() {
        boostd::lock_guard<boostd::mutex> vlock(vMtx);
        if(validBuffers.size() == 0) {
            // Empty queue.
            return 0;
        } else {
            // Return oldest buffer.
            BufPType bp = validBuffers.front();
            validBuffers.pop_front();
            return bp;
        }
    }

    BufPType getFreeBuffer() {
        boostd::lock_guard<boostd::mutex> vlock(vMtx);
        boostd::lock_guard<boostd::mutex> elock(eMtx);
        if(emptyBuffers.size() > 0) {
            // Simply get a free buffer from the free buffers list.
            BufPType bp = emptyBuffers.front();
            emptyBuffers.pop_front();
            return bp;
        } else {
            if(validBuffers.size() == 0) {
                // Queue not yet initialized.
                throw std::runtime_error("Queue not yet initialised.");
            } else {
                std::cerr << "Out of free buffers." << std::endl;
                // No free buffers return oldest among the valid buffers.
                BufPType bp = validBuffers.front();
                validBuffers.pop_front();
                return bp;
            }
        }
    }

    void addValidBuffer(BufPType bp) {
        // Add buffer to valid buffers queue.
        boostd::lock_guard<boostd::mutex> vlock(vMtx);
        validBuffers.push_back(bp);
    }

    void returnOrAddUsedBuffer(BufPType bp) {
        // Add buffer back to empty buffers queue.
        boostd::lock_guard<boostd::mutex> elock(eMtx);
        emptyBuffers.push_back(bp);
    }

    const size_t AvailableFrames() const {
        boostd::lock_guard<boostd::mutex> vlock(vMtx);
        return validBuffers.size();
    }

    const size_t EmptyBuffers() const {
        boostd::lock_guard<boostd::mutex> elock(eMtx);
        return emptyBuffers.size();
    }

    bool DropNFrames(size_t n) {
        boostd::lock_guard<boostd::mutex> vlock(vMtx);
        if(validBuffers.size() < n) {
            return false;
        } else {
            boostd::lock_guard<boostd::mutex> elock(eMtx);
            // Requeue all but newest buffers.
            for(unsigned int i=0; i<n; ++i) {
                emptyBuffers.push_back(validBuffers.front());
                validBuffers.pop_front();
            }
            return true;
        }
    }

//    unsigned int BufferSizeBytes(){
//        return bufferSizeBytes;
//    }

private:
    std::list<BufPType> validBuffers;
    std::list<BufPType> emptyBuffers;
    mutable boostd::mutex vMtx;
    mutable boostd::mutex eMtx;
//    unsigned int maxNumBuffers;
//    unsigned int bufferSizeBytes;
};

}

#endif // PANGOLIN_FIX_SIZE_BUFFER_QUEUE_H
