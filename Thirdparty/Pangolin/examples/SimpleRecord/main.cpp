#include <pangolin/pangolin.h>

void RecordSample(const std::string input_uri, const std::string record_uri)
{
    // Setup Video Source
    pangolin::VideoInput video(input_uri);
    const pangolin::VideoPixelFormat vid_fmt = video.PixFormat();
    const unsigned w = video.Width();
    const unsigned h = video.Height();

    pangolin::VideoOutput recorder( record_uri );
    recorder.SetStreams(video.Streams());

    // Create OpenGL window
    pangolin::CreateWindowAndBind("Main",w,h);

    // Create viewport for video with fixed aspect
    pangolin::View vVideo((float)w/h);

    // OpenGl Texture for video frame
    pangolin::GlTexture texVideo(w,h,GL_RGBA);

    // Allocate image buffer. The +1 is to give ffmpeg some alignment slack
    // swscale seems to have a bug which goes over the array by 1...
    unsigned char* img = new unsigned char[video.SizeBytes() + 1];
    
    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        if( video.GrabNext(img,true) )
        {
            // Upload to GPU as texture for display
            texVideo.Upload(img, vid_fmt.channels==1 ? GL_LUMINANCE:GL_RGB, GL_UNSIGNED_BYTE);

            // Record video frame
            recorder.WriteStreams(img);
        }

        // Activate video viewport and render texture
        vVideo.Activate();
        texVideo.RenderToViewportFlipY();

        // Swap back buffer with front and process window events via GLUT
        pangolin::FinishFrame();
    }

    delete[] img;
}

int main( int argc, char* argv[] )
{
    std::string record_uri = "ffmpeg:[fps=30,bps=8388608]//video.avi";

    std::string input_uris[] = {
        "dc1394:[fps=30,dma=10,size=640x480,iso=400]//0",
        "convert:[fmt=RGB24]//v4l:///dev/video0",
        "convert:[fmt=RGB24]//v4l:///dev/video1",
        ""
    };

    if( argc >= 2 ) {
        const std::string uri = std::string(argv[1]);
        if( argc == 3 ) {
            record_uri = std::string(argv[2]);
        }
        RecordSample(uri, record_uri);
    }else{
        std::cout << "Usage  : SimpleRecord [video-uri] [output-uri]" << std::endl << std::endl;
        std::cout << "Where video-uri describes a stream or file resource, e.g." << std::endl;
        std::cout << "\tfile:[realtime=1]///home/user/video/movie.pvn" << std::endl;
        std::cout << "\tfile:///home/user/video/movie.avi" << std::endl;
        std::cout << "\tfiles:///home/user/seqiemce/foo%03d.jpeg" << std::endl;
        std::cout << "\tdc1394:[fmt=RGB24,size=640x480,fps=30,iso=400,dma=10]//0" << std::endl;
        std::cout << "\tdc1394:[fmt=FORMAT7_1,size=640x480,pos=2+2,iso=400,dma=10]//0" << std::endl;
        std::cout << "\tv4l:///dev/video0" << std::endl;
        std::cout << "\tconvert:[fmt=RGB24]//v4l:///dev/video0" << std::endl;
        std::cout << "\tmjpeg://http://127.0.0.1/?action=stream" << std::endl;
        std::cout << std::endl;

        // Try to open some video device
        for(int i=0; !input_uris[i].empty(); ++i )
        {
            try{
                std::cout << "Trying: " << input_uris[i] << std::endl;
                RecordSample(input_uris[i], record_uri);
                return 0;
            }catch(pangolin::VideoException) {}
        }
    }

    return 0;

}
