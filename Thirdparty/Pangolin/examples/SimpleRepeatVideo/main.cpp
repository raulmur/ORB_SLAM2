/**
 * @author  Steven Lovegrove
 * Copyright (C) 2010  Steven Lovegrove
 *                     Imperial College London
 **/

#include <pangolin/pangolin.h>
#include <pangolin/video/video_record_repeat.h>
#include <pangolin/var/input_record_repeat.h>

void RecordSample(const std::string input_uri, const std::string output_uri, const std::string ui_file)
{
    // Setup Video Source
    pangolin::VideoRecordRepeat video(input_uri, output_uri);
    const pangolin::VideoPixelFormat vid_fmt = video.PixFormat();
    const unsigned w = video.Width();
    const unsigned h = video.Height();

    pangolin::InputRecordRepeat input("ui.");
    input.LoadBuffer(ui_file);

    // Create OpenGL window
    const int panel_width = 200;
    pangolin::CreateWindowAndBind("Main",w + panel_width,h);

    // Create viewport for video with fixed aspect
    pangolin::CreatePanel("ui.")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(panel_width));

    pangolin::View& vVideo = pangolin::Display("Video")
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(panel_width), 1.0)
        .SetAspect((float)w/h);

    // OpenGl Texture for video frame
    pangolin::GlTexture texVideo(w,h,GL_RGBA);

    unsigned char* img = new unsigned char[video.SizeBytes()];

    pangolin::Var<bool> record("ui.Record",false,false);
    pangolin::Var<bool> play("ui.Play",false,false);
    pangolin::Var<bool> source("ui.Source",false,false);
    pangolin::Var<bool> realtime("ui.realtime",true,true);

    pangolin::Var<float> hue("ui.Hue",0,0,360);
    pangolin::Var<bool> colour("ui.Colour Video",false,true);

    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Load next video frame
        if( video.GrabNext(img,true) ) {
            texVideo.Upload(img, vid_fmt.channels==1 ? GL_LUMINANCE:GL_RGB, GL_UNSIGNED_BYTE);

            // Associate input with this video frame
            input.SetIndex(video.FrameId());            
        }

        // Activate video viewport and render texture
        vVideo.Activate();

        if( colour ) {
            glColorHSV(hue,0.5,1.0);
        }else{
            glColor3f(1,1,1);
        }
        texVideo.RenderToViewportFlipY();

        if(pangolin::Pushed(record)) {
            video.Record();
            input.Record();
        }

        if(pangolin::Pushed(play)) {
            video.Play(realtime);
            input.PlayBuffer(0,input.Size()-1);
            input.SaveBuffer(ui_file);
        }

        if(pangolin::Pushed(source)) {
            video.Source();
            input.Stop();
            input.SaveBuffer(ui_file);
        }

        pangolin::FinishFrame();
    }

    delete[] img;
}

int main( int argc, char* argv[] )
{
    std::string uris[] = {
        "dc1394:[fps=30,dma=10,size=640x480,iso=400]//0",
        "convert:[fmt=RGB24]//v4l:///dev/video0",
        "convert:[fmt=RGB24]//v4l:///dev/video1",
        ""
    };

    std::string filename = "video.pango";

    if( argc >= 2 ) {
        const std::string uri = std::string(argv[1]);
        if( argc == 3 ) {
            filename = std::string(argv[2]);
        }
        RecordSample(uri, filename, filename + ".ui");
    }else{
        std::cout << "Usage  : SimpleRepeatVideo [video-uri] [buffer-filename]" << std::endl << std::endl;
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
        for(int i=0; !uris[i].empty(); ++i )
        {
            try{
                std::cout << "Trying: " << uris[i] << std::endl;
                RecordSample(uris[i], filename, filename + ".ui");
                return 0;
            }catch(pangolin::VideoException) {}
        }
    }

    return 0;
}
