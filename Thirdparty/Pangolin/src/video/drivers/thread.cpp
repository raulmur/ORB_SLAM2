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

#include <pangolin/video/drivers/thread.h>

#ifdef DEBUGTHREAD
  #include <pangolin/utils/timer.h>
  #define TSTART() pangolin::basetime start,last,now; start = pangolin::TimeNow(); last = start;
  #define TGRABANDPRINT(...)  now = pangolin::TimeNow(); fprintf(stderr,"THREAD: "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, " %fms.\n",1000*pangolin::TimeDiff_s(last, now)); last = now;
  #define DBGPRINT(...) fprintf(stderr,"THREAD: "); fprintf(stderr, __VA_ARGS__); fprintf(stderr,"\n");
#else
  #define TSTART()
  #define TGRABANDPRINT(...)
  #define DBGPRINT(...)
#endif

namespace pangolin
{

const uint64_t grab_fail_thread_sleep_us = 1000;
const uint64_t capture_timout_ms = 5000;

ThreadVideo::ThreadVideo(VideoInterface* src, size_t num_buffers)
    : quit_grab_thread(true)
{
    if(!src) {
        throw VideoException("ThreadVideo: VideoInterface in must not be null");
    }
    videoin.push_back(src);

//    // queue init allocates buffers.
//    queue.init(num_buffers, (unsigned int)videoin[0]->SizeBytes());

    for(size_t i=0; i < num_buffers; ++i)
    {
        queue.returnOrAddUsedBuffer( GrabResult(videoin[0]->SizeBytes()) );
    }

    Start();
}

ThreadVideo::~ThreadVideo()
{
    Stop();
    delete videoin[0];

    // Invalidate all buffers
    queue.DropNFrames(queue.AvailableFrames());

    // Remove and delete buffers
    while( queue.EmptyBuffers() ) {
        delete[] queue.getFreeBuffer().buffer;
    }
}

//! Implement VideoInput::Start()
void ThreadVideo::Start()
{
    // Only start thread if not already running.
    if(quit_grab_thread) {
        videoin[0]->Start();
        quit_grab_thread = false;
        grab_thread = boostd::thread(boostd::ref(*this));
    }
}

//! Implement VideoInput::Stop()
void ThreadVideo::Stop()
{
    quit_grab_thread = true;
    if(grab_thread.joinable()) {
        grab_thread.join();
    }
    videoin[0]->Stop();
}

//! Implement VideoInput::SizeBytes()
size_t ThreadVideo::SizeBytes() const
{
    return videoin[0]->SizeBytes();
}

//! Implement VideoInput::Streams()
const std::vector<StreamInfo>& ThreadVideo::Streams() const
{
    return videoin[0]->Streams();
}

const json::value& ThreadVideo::DeviceProperties() const
{
    device_properties = GetVideoDeviceProperties(videoin[0]);
    return device_properties;
}

const json::value& ThreadVideo::FrameProperties() const
{
    return frame_properties;
}

uint32_t ThreadVideo::AvailableFrames() const
{
    return (uint32_t)queue.AvailableFrames();
}

bool ThreadVideo::DropNFrames(uint32_t n)
{
    return queue.DropNFrames(n);
}

//! Implement VideoInput::GrabNext()
bool ThreadVideo::GrabNext( unsigned char* image, bool wait )
{
    TSTART()
    if(queue.AvailableFrames() == 0 && !wait) {
        // No frames available, no wait, simply return false.
        DBGPRINT("GrabNext no available frames no wait.");
        return false;
    }else{
        if(queue.AvailableFrames() == 0 && wait) {
            // Must return a frame so block on notification from grab thread.
            std::unique_lock<boostd::mutex> lk(cvMtx);
            DBGPRINT("GrabNext no available frames wait for notification.");
            if(cv.wait_for(lk, boostd::chrono::milliseconds(capture_timout_ms)) == boostd::cv_status::timeout)
                throw std::runtime_error("ThreadVideo: GrabNext blocking read for frames reached timeout.");
        }

        // At least one valid frame in queue, return it.
        GrabResult grab = queue.getNext();
        if(grab.return_status) {
            DBGPRINT("GrabNext at least one frame available.");
            std::memcpy(image, grab.buffer, videoin[0]->SizeBytes());
            frame_properties = grab.frame_properties;
        }else{
            DBGPRINT("GrabNext returned false")
        }
        queue.returnOrAddUsedBuffer(grab);

        TGRABANDPRINT("GrabNext took")
        return grab.return_status;
    }
}

//! Implement VideoInput::GrabNewest()
bool ThreadVideo::GrabNewest( unsigned char* image, bool wait )
{
    TSTART()
    if(queue.AvailableFrames() == 0 && !wait) {
        // No frames available, no wait, simply return false.
        DBGPRINT("GrabNext no available frames no wait.");
        return false;
    }else{
        if(queue.AvailableFrames() == 0 && wait) {
            // Must return a frame so block on notification from grab thread.
            std::unique_lock<boostd::mutex> lk(cvMtx);
            DBGPRINT("GrabNewest no available frames wait for notification.");
            if(cv.wait_for(lk, boostd::chrono::milliseconds(capture_timout_ms)) == boostd::cv_status::timeout)
                throw std::runtime_error("ThreadVideo: GrabNext blocking read for frames reached timeout.");
        }

        // At least one valid frame in queue, return it.
        DBGPRINT("GrabNewest at least one frame available.");
        GrabResult grab = queue.getNewest();
        if(grab.return_status) {
            std::memcpy(image, grab.buffer, videoin[0]->SizeBytes());
            frame_properties = grab.frame_properties;
        }
        queue.returnOrAddUsedBuffer(grab);
        TGRABANDPRINT("GrabNewest memcpy of available frame took")

        return grab.return_status;
    }
}

void ThreadVideo::operator()()
{
    DBGPRINT("Grab thread Started.")
    // Spinning thread attempting to read from videoin[0] as fast as possible
    // relying on the videoin[0] blocking grab.
    while(!quit_grab_thread) {
        // Get a buffer from the queue;

        if(queue.EmptyBuffers() > 0) {
            GrabResult grab = queue.getFreeBuffer();

            // Blocking grab (i.e. GrabNext with wait = true).
            grab.return_status = videoin[0]->GrabNext(grab.buffer, true);

            if(grab.return_status){
                grab.frame_properties = GetVideoFrameProperties(videoin[0]);
            }else{
                std::this_thread::sleep_for(std::chrono::microseconds(grab_fail_thread_sleep_us) );
            }
            queue.addValidBuffer(grab);

            DBGPRINT("Grab thread got frame. valid:%d free:%d",queue.AvailableFrames(),queue.EmptyBuffers())
            // Let listening threads know we got a frame in case they are waiting.
            cv.notify_all();
        }else{
            std::this_thread::sleep_for(std::chrono::microseconds(grab_fail_thread_sleep_us) );
        }
        boostd::this_thread::yield();
    }
    DBGPRINT("Grab thread Stopped.")

    return;
}

std::vector<VideoInterface*>& ThreadVideo::InputStreams()
{
    return videoin;
}

}

#undef TSTART
#undef TGRABANDPRINT
#undef DBGPRINT
