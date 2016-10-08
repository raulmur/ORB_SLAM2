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

#include <pangolin/video/drivers/join.h>

#ifdef DEBUGJOIN
  #include <pangolin/utils/timer.h>
  #define TSTART() pangolin::basetime start,last,now; start = pangolin::TimeNow(); last = start;
  #define TGRABANDPRINT(...)  now = pangolin::TimeNow(); fprintf(stderr,"JOIN: "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, " %fms.\n",1000*pangolin::TimeDiff_s(last, now)); last = now;
  #define DBGPRINT(...) fprintf(stderr,"JOIN: "); fprintf(stderr, __VA_ARGS__); fprintf(stderr,"\n");
#else
  #define TSTART()
  #define TGRABANDPRINT(...)
  #define DBGPRINT(...)
#endif

namespace pangolin
{
VideoJoiner::VideoJoiner(const std::vector<VideoInterface*>& src)
    : src(src), size_bytes(0), sync_tolerance_us(0)
{
    // Add individual streams
    for(size_t s=0; s< src.size(); ++s)
    {
        VideoInterface& vid = *src[s];
        for(size_t i=0; i < vid.Streams().size(); ++i)
        {
            const StreamInfo si = vid.Streams()[i];
            const VideoPixelFormat fmt = si.PixFormat();
            const Image<unsigned char> img_offset = si.StreamImage((unsigned char*)size_bytes);
            streams.push_back(StreamInfo(fmt, img_offset));
        }
        size_bytes += src[s]->SizeBytes();
    }
}

VideoJoiner::~VideoJoiner()
{
    for(size_t s=0; s< src.size(); ++s) {
        src[s]->Stop();
        delete src[s];
    }
}

size_t VideoJoiner::SizeBytes() const
{
    return size_bytes;
}

const std::vector<StreamInfo>& VideoJoiner::Streams() const
{
    return streams;
}

void VideoJoiner::Start()
{
    for(size_t s=0; s< src.size(); ++s) {
        src[s]->Start();
    }
}

void VideoJoiner::Stop()
{
    for(size_t s=0; s< src.size(); ++s) {
        src[s]->Stop();
    }
}

bool VideoJoiner::Sync(int64_t tolerance_us, int64_t expected_delta_us)
{
    for(size_t s=0; s< src.size(); ++s)
    {
       VideoPropertiesInterface* vpi = dynamic_cast<VideoPropertiesInterface*>(src[s]);
       if(!vpi) {
         return false;
       }
    }
    sync_tolerance_us = tolerance_us;
    expected_timestamp_delta_us = expected_delta_us;
    return true;
}

bool VideoJoiner::GrabNext(unsigned char* image, bool wait)
{
    int64_t rt = 0;
    size_t offset = 0;
    std::vector<size_t> offsets;
    std::vector<int64_t> reception_times;
    int64_t newest = std::numeric_limits<int64_t>::min();
    int64_t oldest = std::numeric_limits<int64_t>::max();
    int grabbed_all = (int)src.size();

    TSTART()
    DBGPRINT("Entering GrabNext:")
    for(size_t s=0; s<src.size(); ++s) {
        VideoInterface& vid = *src[s];
        if(vid.GrabNext(image+offset,wait)) {
            --grabbed_all;
        }
        offsets.push_back(offset);
        offset += vid.SizeBytes();
        if(sync_tolerance_us > 0) {
           VideoPropertiesInterface* vidpi = dynamic_cast<VideoPropertiesInterface*>(src[s]);
           if(vidpi->FrameProperties().contains(PANGO_HOST_RECEPTION_TIME_US)) {
               rt = vidpi->FrameProperties()[PANGO_HOST_RECEPTION_TIME_US].get<int64_t>();
               reception_times.push_back(rt);
               if(newest < rt) newest = rt;
               if(oldest > rt) oldest = rt;
           } else {
               pango_print_error("Stream %lu in join does not support sync_tolerance_us option.\n", (unsigned long)s);
               throw std::runtime_error("Grab next stream does not support sync_tolerance_us option.\n");
           }
        }
        TGRABANDPRINT("Stream %ld grab took ",s)
    }

    if(grabbed_all != 0){
        // Source is waiting on data or end of stream.
        return false;
    }

    if(sync_tolerance_us > 0) {
        if(std::abs(newest - oldest - expected_timestamp_delta_us) > sync_tolerance_us){
            pango_print_warn("Join timestamps not within %lu us trying to sync\n", (unsigned long)sync_tolerance_us);

            for(size_t n=0; n<10; ++n){
                for(size_t s=0; s<src.size(); ++s) {
                    if(reception_times[s] < (newest - sync_tolerance_us)) {
                        VideoInterface& vid = *src[s];
                        if(vid.GrabNewest(image+offsets[s],false)) {
                            VideoPropertiesInterface* vidpi = dynamic_cast<VideoPropertiesInterface*>(src[s]);
                            if(vidpi->FrameProperties().contains(PANGO_HOST_RECEPTION_TIME_US)) {
                                rt = vidpi->FrameProperties()[PANGO_HOST_RECEPTION_TIME_US].get<int64_t>();
                            }
                            if(newest < rt) newest = rt;
                            if(oldest > rt) oldest = rt;
                            reception_times[s] = rt;
                        }
                    }
                }
            }
        }

        if(std::abs(newest - oldest - expected_timestamp_delta_us) > sync_tolerance_us ) {
            TGRABANDPRINT("NOT IN SYNC newest:%ld oldest:%ld delta:%ld syncing took ", newest, oldest, (newest - oldest));
            return false;
        } else {
            TGRABANDPRINT("    IN SYNC newest:%ld oldest:%ld delta:%ld syncing took ", newest, oldest, (newest - oldest));
            return true;
        }
    } else {
        return true;
    }
}

bool AllInterfacesAreBufferAware(std::vector<VideoInterface*>& src){
  for(size_t s=0; s<src.size(); ++s) {
      if(!dynamic_cast<BufferAwareVideoInterface*>(src[s])) return false;
  }
  return true;
}

bool VideoJoiner::GrabNewest( unsigned char* image, bool wait )
{
  TSTART()
  DBGPRINT("Entering GrabNewest:");
  if(AllInterfacesAreBufferAware(src)) {
     DBGPRINT("All interfaces are BufferAwareVideoInterface.")
     unsigned int minN = std::numeric_limits<unsigned int>::max();
     //Find smallest number of frames it is safe to drop.
     for(size_t s=0; s<src.size(); ++s) {
         auto bai = dynamic_cast<BufferAwareVideoInterface*>(src[s]);
         unsigned int n = bai->AvailableFrames();
         minN = std::min(n, minN);
         DBGPRINT("Interface %ld has %u frames available.",s ,n)
     }
     TGRABANDPRINT("Quering avalable frames took ")
     DBGPRINT("Safe number of buffers to drop: %d.",((minN > 1) ? (minN-1) : 0));

     //Safely drop minN-1 frames on each interface.
     if(minN > 1) {
         for(size_t s=0; s<src.size(); ++s) {
             auto bai = dynamic_cast<BufferAwareVideoInterface*>(src[s]);
             if(!bai->DropNFrames(minN - 1)) {
                 pango_print_error("Stream %lu did not drop %u frames altough available.\n", (unsigned long)s, (minN-1));
                 return false;
             }
         }
         TGRABANDPRINT("Dropping %u frames on each interface took ",(minN -1));
     }
     return GrabNext(image, wait);
  } else {
      DBGPRINT("NOT all interfaces are BufferAwareVideoInterface.")
      // Simply calling GrabNewest on the child streams might cause loss of sync,
      // instead we perform as many GrabNext as possible on the first stream and
      // then pull the same number of frames from every other stream.
      size_t offset = 0;
      std::vector<size_t> offsets;
      std::vector<int64_t> reception_times;
      int64_t newest = std::numeric_limits<int64_t>::min();
      int64_t oldest = std::numeric_limits<int64_t>::max();
      bool grabbed_any = false;
      int first_stream_backlog = 0;
      int64_t rt = 0;
      bool got_frame = false;

      do {
          got_frame = src[0]->GrabNext(image+offset,false);
          if(got_frame) {
              if(sync_tolerance_us > 0) {
                  VideoPropertiesInterface* vidpi = dynamic_cast<VideoPropertiesInterface*>(src[0]);
                  if(vidpi->FrameProperties().contains(PANGO_HOST_RECEPTION_TIME_US)) {
                      rt = vidpi->FrameProperties()[PANGO_HOST_RECEPTION_TIME_US].get<int64_t>();
                  } else {
                      pango_print_error("Stream %u in join does not support startup_sync_us option.\n", 0);
                      throw std::runtime_error("Grab newest stream in join does not support startup_sync_us option.\n");
                  }
              }
              first_stream_backlog++;
              grabbed_any = true;
          }
      } while(got_frame);
      offsets.push_back(offset);
      offset += src[0]->SizeBytes();
      if(sync_tolerance_us > 0) {
          reception_times.push_back(rt);
          if(newest < rt) newest = rt;
          if(oldest > rt) oldest = rt;
      }
      TGRABANDPRINT("Stream 0 grab took ");

      for(size_t s=1; s<src.size(); ++s) {
          for (int i=0; i<first_stream_backlog; i++){
              grabbed_any |= src[s]->GrabNext(image+offset,true);
              if(sync_tolerance_us > 0) {
                  VideoPropertiesInterface* vidpi = dynamic_cast<VideoPropertiesInterface*>(src[s]);
                  if(vidpi->FrameProperties().contains(PANGO_HOST_RECEPTION_TIME_US)) {
                      rt = vidpi->FrameProperties()[PANGO_HOST_RECEPTION_TIME_US].get<int64_t>();
                  } else {
                      pango_print_error("Stream %lu in join does not support startup_sync_us option.\n", (unsigned long)s);
                  }
              }
          }
          offsets.push_back(offset);
          offset += src[s]->SizeBytes();
          if(sync_tolerance_us > 0) {
              reception_times.push_back(rt);
              if(newest < rt) newest = rt;
              if(oldest > rt) oldest = rt;
          }
      }
      TGRABANDPRINT("Stream >=1 grab took ");

      if(sync_tolerance_us > 0) {
          if(std::abs(newest - oldest - expected_timestamp_delta_us) > sync_tolerance_us){
              pango_print_warn("Join timestamps not within %lu us trying to sync\n", (unsigned long)sync_tolerance_us);

              for(size_t n=0; n<10; ++n){
                  for(size_t s=0; s<src.size(); ++s) {
                      if(reception_times[s] < (newest - sync_tolerance_us)) {
                          VideoInterface& vid = *src[s];
                          if(vid.GrabNewest(image+offsets[s],false)) {
                              VideoPropertiesInterface* vidpi = dynamic_cast<VideoPropertiesInterface*>(src[s]);
                              if(vidpi->FrameProperties().contains(PANGO_HOST_RECEPTION_TIME_US)) {
                                  rt = vidpi->FrameProperties()[PANGO_HOST_RECEPTION_TIME_US].get<int64_t>();
                              }
                              if(newest < rt) newest = rt;
                              if(oldest > rt) oldest = rt;
                              reception_times[s] = rt;
                          }
                      }
                  }
              }
          }

          if(std::abs(newest - oldest - expected_timestamp_delta_us) > sync_tolerance_us ) {
              TGRABANDPRINT("NOT IN SYNC newest:%ld oldest:%ld delta:%ld syncing took ", newest, oldest, (newest - oldest));
              return false;
          } else {
              TGRABANDPRINT("    IN SYNC newest:%ld oldest:%ld delta:%ld syncing took ", newest, oldest, (newest - oldest));
              return true;
          }
      } else {
          return true;
      }
  }

}

std::vector<VideoInterface*>& VideoJoiner::InputStreams()
{
    return src;
}

}

#undef TSTART
#undef TGRABANDPRINT
#undef DBGPRINT
