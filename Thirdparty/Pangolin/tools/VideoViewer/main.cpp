#include <pangolin/pangolin.h>
#include <pangolin/video/video_record_repeat.h>
#include <pangolin/gl/gltexturecache.h>
#include <pangolin/gl/glpixformat.h>
#include <pangolin/handler/handler_image.h>
#include <pangolin/utils/file_utils.h>
#include <pangolin/utils/timer.h>

#ifdef DEBUGVIDEOVIEWER
#  include <pangolin/compat/thread.h>
#endif // DEBUGVIDEOVIEWER

template<typename To, typename From>
void ConvertPixels(pangolin::Image<To>& to, const pangolin::Image<From>& from)
{
    for(size_t y=0; y < to.h; ++y) {
        for(size_t x=0; x < to.w; ++x) {
            to.RowPtr((int)y)[x] = static_cast<To>( from.RowPtr((int)y)[x] );
        }
    }
}

void VideoViewer(const std::string& input_uri, const std::string& output_uri)
{
    pangolin::Var<int>  record_timelapse_frame_skip("viewer.record_timelapse_frame_skip", 1 );
    pangolin::Var<int>  end_frame("viewer.end_frame", std::numeric_limits<int>::max() );
    pangolin::Var<bool> video_wait("video.wait", true);
    pangolin::Var<bool> video_newest("video.newest", false);

    // Open Video by URI
    pangolin::VideoRecordRepeat video(input_uri, output_uri);
    const size_t num_streams = video.Streams().size();

    if(num_streams == 0) {
        pango_print_error("No video streams from device.\n");
        return;
    }

    // Output details of video stream
    for(size_t s = 0; s < num_streams; ++s) {
        const pangolin::StreamInfo& si = video.Streams()[s];
        std::cout << "Stream " << s << ": " << si.Width() << " x " << si.Height()
                  << " " << si.PixFormat().format << " (pitch: " << si.Pitch() << " bytes)" << std::endl;
    }

    // Check if video supports VideoPlaybackInterface
    pangolin::VideoPlaybackInterface* video_playback = pangolin::FindFirstMatchingVideoInterface<pangolin::VideoPlaybackInterface>(video);
    const int total_frames = video_playback ? video_playback->GetTotalFrames() : std::numeric_limits<int>::max();
    const int slider_size = (total_frames < std::numeric_limits<int>::max() ? 20 : 0);

    if( video_playback ) {
        if(total_frames < std::numeric_limits<int>::max() ) {
            std::cout << "Video length: " << total_frames << " frames" << std::endl;
        }
        end_frame = 0;
    }

    std::vector<unsigned char> buffer;
    buffer.resize(video.SizeBytes()+1);

    // Create OpenGL window - guess sensible dimensions
    pangolin::CreateWindowAndBind( "VideoViewer",
        (int)(video.Width() * num_streams),
        (int)(video.Height() + slider_size)
    );

    // Assume packed OpenGL data unless otherwise specified
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Setup resizable views for video streams
    std::vector<pangolin::GlPixFormat> glfmt;
    std::vector<std::pair<float,float> > gloffsetscale;
    std::vector<size_t> strides;
    std::vector<pangolin::ImageViewHandler> handlers;
    handlers.reserve(num_streams);

    size_t scratch_buffer_bytes = 0;

    pangolin::View& container = pangolin::Display("streams");
    container.SetLayout(pangolin::LayoutEqual)
             .SetBounds(pangolin::Attach::Pix(slider_size), 1.0, 0.0, 1.0);
    for(unsigned int d=0; d < num_streams; ++d) {
        const pangolin::StreamInfo& si = video.Streams()[d];
        pangolin::View& view = pangolin::CreateDisplay().SetAspect(si.Aspect());
        container.AddDisplay(view);
        glfmt.push_back(pangolin::GlPixFormat(si.PixFormat()));
        gloffsetscale.push_back(std::pair<float,float>(0.0f, 1.0f) );
        if( si.PixFormat().bpp % 8 ) {
            pango_print_warn("Stream %i: Unable to display formats that are not a multiple of 8 bits.", d);
        }
        if( (8*si.Pitch()) % si.PixFormat().bpp ) {
            pango_print_warn("Stream %i: Unable to display formats whose pitch is not a whole number of pixels.", d);
        }
        if(glfmt.back().gltype == GL_DOUBLE) {
            scratch_buffer_bytes = std::max(scratch_buffer_bytes, sizeof(float)*si.Width() * si.Height());
        }
        strides.push_back( (8*si.Pitch()) / si.PixFormat().bpp );
        handlers.push_back( pangolin::ImageViewHandler(si.Width(), si.Height()) );
        view.SetHandler(&handlers.back());
    }

    // current frame in memory buffer and displaying.
    pangolin::Var<int> frame("ui.frame", -1, 0, total_frames-1 );
    pangolin::Slider frame_slider("frame", frame.Ref() );
    if(video_playback && total_frames < std::numeric_limits<int>::max())
    {
        frame_slider.SetBounds(0.0, pangolin::Attach::Pix(slider_size), 0.0, 1.0);
        pangolin::DisplayBase().AddDisplay(frame_slider);
    }

    std::vector<unsigned char> scratch_buffer;
    scratch_buffer.resize(scratch_buffer_bytes);

    std::vector<pangolin::Image<unsigned char> > images;

#ifdef CALLEE_HAS_CPP11
    const int FRAME_SKIP = 30;
    const char show_hide_keys[]  = {'1','2','3','4','5','6','7','8','9'};
    const char screenshot_keys[] = {'!','"','#','$','%','^','&','*','('};

    // Show/hide streams
    for(size_t v=0; v < container.NumChildren() && v < 9; v++) {
        pangolin::RegisterKeyPressCallback(show_hide_keys[v], [v,&container](){
            container[v].ToggleShow();
        } );
        pangolin::RegisterKeyPressCallback(screenshot_keys[v], [v,&images,&video](){
            if(v < images.size() && images[v].ptr) {
                try{
                    pangolin::SaveImage(
                        images[v], video.Streams()[v].PixFormat(),
                        pangolin::MakeUniqueFilename("capture.png")
                    );
                }catch(std::exception e){
                    pango_print_error("Unable to save frame: %s\n", e.what());
                }
            }
        } );
    }

    pangolin::RegisterKeyPressCallback('r', [&](){
        if(!video.IsRecording()) {
            video.SetTimelapse( static_cast<size_t>(record_timelapse_frame_skip) );
            video.Record();
            pango_print_info("Started Recording.\n");
        }else{
            video.Stop();
            pango_print_info("Finished recording.\n");
        }
        fflush(stdout);
    });
    pangolin::RegisterKeyPressCallback('p', [&](){
        video.Play();
        end_frame = std::numeric_limits<int>::max();
        pango_print_info("Playing from file log.\n");
        fflush(stdout);
    });
    pangolin::RegisterKeyPressCallback('s', [&](){
        video.Source();
        end_frame = std::numeric_limits<int>::max();
        pango_print_info("Playing from source input.\n");
        fflush(stdout);
    });
    pangolin::RegisterKeyPressCallback(' ', [&](){
        end_frame = (frame < end_frame) ? frame : std::numeric_limits<int>::max();
    });
    pangolin::RegisterKeyPressCallback('w', [&](){
        video_wait = !video_wait;
        if(video_wait) {
            pango_print_info("Gui wait's for video frame.\n");
        }else{
            pango_print_info("Gui doesn't wait for video frame.\n");
        }
    });
    pangolin::RegisterKeyPressCallback('d', [&](){
        video_newest = !video_newest;
        if(video_newest) {
            pango_print_info("Discarding old frames.\n");
        }else{
            pango_print_info("Not discarding old frames.\n");
        }
    });
    pangolin::RegisterKeyPressCallback('<', [&](){
        if(video_playback) {
            frame = video_playback->Seek(frame - FRAME_SKIP) -1;
            end_frame = frame + 1;
        }else{
            pango_print_warn("Unable to skip backward.");
        }
    });
    pangolin::RegisterKeyPressCallback('>', [&](){
        if(video_playback) {
            frame = video_playback->Seek(frame + FRAME_SKIP) -1;
            end_frame = frame + 1;
        }else{
            end_frame = frame + FRAME_SKIP;
        }
    });
    pangolin::RegisterKeyPressCallback(',', [&](){
        if(video_playback) {
            frame = video_playback->Seek(frame - 1) -1;
            end_frame = frame+1;
        }else{
            pango_print_warn("Unable to skip backward.");
        }
    });
    pangolin::RegisterKeyPressCallback('.', [&](){
        // Pause at next frame
        end_frame = frame+1;
    });
    pangolin::RegisterKeyPressCallback('0', [&](){
        video.RecordOneFrame();
    });
    pangolin::RegisterKeyPressCallback('a', [&](){
        // Adapt scale
        for(unsigned int i=0; i<images.size(); ++i) {
            if(container[i].HasFocus()) {
                pangolin::Image<unsigned char>& img = images[i];
                pangolin::ImageViewHandler& ivh = handlers[i];

                const bool have_selection = std::isfinite(ivh.GetSelection().Area()) && std::abs(ivh.GetSelection().Area()) >= 4;
                pangolin::XYRangef froi = have_selection ? ivh.GetSelection() : ivh.GetViewToRender();
                gloffsetscale[i] = pangolin::GetOffsetScale(img, froi.Cast<int>(), glfmt[i]);
            }
        }
    });
    pangolin::RegisterKeyPressCallback('g', [&](){
        std::pair<float,float> os_default(0.0f, 1.0f);

        // Get the scale and offset from the container that has focus.
        for(unsigned int i=0; i<images.size(); ++i) {
            if(container[i].HasFocus()) {
                pangolin::Image<unsigned char>& img = images[i];
                pangolin::ImageViewHandler& ivh = handlers[i];

                const bool have_selection = std::isfinite(ivh.GetSelection().Area()) && std::abs(ivh.GetSelection().Area()) >= 4;
                pangolin::XYRangef froi = have_selection ? ivh.GetSelection() : ivh.GetViewToRender();
                os_default = pangolin::GetOffsetScale(img, froi.Cast<int>(), glfmt[i]);
                break;
            }
        }

        // Adapt scale for all images equally
        // TODO : we're assuming the type of all the containers images' are the same.
        for(unsigned int i=0; i<images.size(); ++i) {
            gloffsetscale[i] = os_default;
        }

    });
#endif // CALLEE_HAS_CPP11

#ifdef DEBUGVIDEOVIEWER
    unsigned int delayms = 0;
    pangolin::RegisterKeyPressCallback('z', [&](){
      // Adapt delay
      delayms += 1;
      std::cout << "                  Fake delay " << delayms << "ms" << std::endl;
    });

    pangolin::RegisterKeyPressCallback('x', [&](){
      // Adapt delay
      delayms = (delayms > 1) ? delayms-1 : 0;
    });

    pangolin::basetime start,now;
#endif // DEBUGVIDEOVIEWER

    // Stream and display video
    while(!pangolin::ShouldQuit())
    {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        glColor3f(1.0f, 1.0f, 1.0f);

        if(frame.GuiChanged()) {
            if(video_playback) {
                frame = video_playback->Seek(frame) -1;
            }
            end_frame = frame + 1;
        }

#ifdef DEBUGVIDEOVIEWER
        boostd::this_thread::sleep_for(boostd::chrono::milliseconds(delayms));
        std::cout << "-------------------------------------------------------" << std::endl;
        now = pangolin::TimeNow();
        std::cout << "      FPS: " << 1.0/pangolin::TimeDiff_s(start, now) << " artificial delay: " << delayms <<"ms"<< std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;
        start = now;
#endif
        if ( frame < end_frame ) {
            if( video.Grab(&buffer[0], images, video_wait, video_newest) ) {
                frame = frame +1;
            }
        }
#ifdef DEBUGVIDEOVIEWER
        const pangolin::basetime end = pangolin::TimeNow();
        std::cout << "Total grab time: " << 1000*pangolin::TimeDiff_s(start, end) << "ms" << std::endl;
#endif

        glLineWidth(1.5f);
        glDisable(GL_DEPTH_TEST);

        for(unsigned int i=0; i<images.size(); ++i)
        {
            if(container[i].IsShown()) {
                container[i].Activate();
                pangolin::Image<unsigned char>& image = images[i];

                // Get texture of correct dimension / format
                const pangolin::GlPixFormat& fmt = glfmt[i];
                pangolin::GlTexture& tex = pangolin::TextureCache::I().GlTex((GLsizei)image.w, (GLsizei)image.h, fmt.scalable_internal_format, fmt.glformat, GL_FLOAT);

                // Upload image data to texture
                tex.Bind();
                if(fmt.gltype == GL_DOUBLE) {
                    // Convert to float first, using scrath_buffer for storage
                    pangolin::Image<float> fimage(image.w, image.h, image.w*sizeof(float), (float*)scratch_buffer.data());
                    ConvertPixels<float,double>( fimage, image.Reinterpret<double>() );
                    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
                    tex.Upload(fimage.ptr,0,0, (GLsizei)fimage.w, (GLsizei)fimage.h, fmt.glformat, GL_FLOAT);
                }else{
                    glPixelStorei(GL_UNPACK_ROW_LENGTH, (GLint)strides[i]);
                    tex.Upload(image.ptr,0,0, (GLsizei)image.w, (GLsizei)image.h, fmt.glformat, fmt.gltype);
                }

                // Render
                handlers[i].UpdateView();
                handlers[i].glSetViewOrtho();
                const std::pair<float,float> os = gloffsetscale[i];
                pangolin::GlSlUtilities::OffsetAndScale(os.first, os.second);
                handlers[i].glRenderTexture(tex);
                pangolin::GlSlUtilities::UseNone();
                handlers[i].glRenderOverlay();
            }
        }
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

        // leave in pixel orthographic for slider to render.
        pangolin::DisplayBase().ActivatePixelOrthographic();
        if(video.IsRecording()) {
            pangolin::glRecordGraphic(pangolin::DisplayBase().v.w-14.0f, pangolin::DisplayBase().v.h-14.0f, 7.0f);
        }
        pangolin::FinishFrame();
    }
}


int main( int argc, char* argv[] )
{
    const std::string dflt_output_uri = "pango://video.pango";

    if( argc > 1 ) {
        const std::string input_uri = std::string(argv[1]);
        const std::string output_uri = (argc > 2) ? std::string(argv[2]) : dflt_output_uri;
        try{
            VideoViewer(input_uri, output_uri);
        } catch (pangolin::VideoException e) {
            std::cout << e.what() << std::endl;
        }
    }else{
        const std::string input_uris[] = {
            "dc1394:[fps=30,dma=10,size=640x480,iso=400]//0",
            "convert:[fmt=RGB24]//v4l:///dev/video0",
            "convert:[fmt=RGB24]//v4l:///dev/video1",
            "openni:[img1=rgb]//",
            "test:[size=160x120,n=1,fmt=RGB24]//"
            ""
        };

        std::cout << "Usage  : VideoViewer [video-uri]" << std::endl << std::endl;
        std::cout << "Where video-uri describes a stream or file resource, e.g." << std::endl;
        std::cout << "\tfile:[realtime=1]///home/user/video/movie.pvn" << std::endl;
        std::cout << "\tfile:///home/user/video/movie.avi" << std::endl;
        std::cout << "\tfiles:///home/user/seqiemce/foo%03d.jpeg" << std::endl;
        std::cout << "\tdc1394:[fmt=RGB24,size=640x480,fps=30,iso=400,dma=10]//0" << std::endl;
        std::cout << "\tdc1394:[fmt=FORMAT7_1,size=640x480,pos=2+2,iso=400,dma=10]//0" << std::endl;
        std::cout << "\tv4l:///dev/video0" << std::endl;
        std::cout << "\tconvert:[fmt=RGB24]//v4l:///dev/video0" << std::endl;
        std::cout << "\tmjpeg://http://127.0.0.1/?action=stream" << std::endl;
        std::cout << "\topenni:[img1=rgb]//" << std::endl;
        std::cout << std::endl;

        // Try to open some video device
        for(int i=0; !input_uris[i].empty(); ++i )
        {
            try{
                pango_print_info("Trying: %s\n", input_uris[i].c_str());
                VideoViewer(input_uris[i], dflt_output_uri);
                return 0;
            }catch(pangolin::VideoException) { }
        }
    }

    return 0;
}
