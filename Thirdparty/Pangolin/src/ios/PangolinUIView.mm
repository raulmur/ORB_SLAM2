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

#import <QuartzCore/QuartzCore.h>

#import <pangolin/ios/PangolinUIView.h>
#import <pangolin/ios/PangolinAppDelegate.h>

#include <pangolin/pangolin.h>
#include <pangolin/platform.h>
#include <pangolin/gl/glinclude.h>
#include <pangolin/display/display.h>
#include <pangolin/display/display_internal.h>

namespace pangolin
{
    extern __thread PangolinGl* context;
}


@implementation PangolinUIView

+ (Class)layerClass {
    return [CAEAGLLayer class];
}

- (void)setupLayer {
    _eaglLayer = (CAEAGLLayer*) self.layer;
    _eaglLayer.opaque = YES;
}

- (void)setupContext {
    EAGLRenderingAPI api = kEAGLRenderingAPIOpenGLES2;
    _context = [[EAGLContext alloc] initWithAPI:api];
    if (!_context) {
        NSLog(@"Failed to initialize OpenGLES 2.0 context");
        exit(1);
    }
    
    if (![EAGLContext setCurrentContext:_context]) {
        NSLog(@"Failed to set current OpenGL context");
        exit(1);
    }
}

- (void)setupRenderBuffer {
    glGenRenderbuffers(1, &_colorRenderBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, _colorRenderBuffer);
    [_context renderbufferStorage:GL_RENDERBUFFER fromDrawable:_eaglLayer];
}

- (void)setupDepthBuffer {
    glGenRenderbuffers(1, &_depthRenderBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, _depthRenderBuffer);
    const int w = self.frame.size.width * self.contentScaleFactor;
    const int h = self.frame.size.height * self.contentScaleFactor;
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, w, h);
}

- (void)setupFrameBuffer {
    GLuint framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, _colorRenderBuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, _depthRenderBuffer);
}

- (void)render:(CADisplayLink*)displayLink {    
    pangolin::glEngine().prog_fixed.Bind();

    if(pangolin::context->user_app) {
        pangolin::context->user_app->Render();
    }
    
    pangolin::RenderViews();
    pangolin::PostRender();
    
    [_context presentRenderbuffer:GL_RENDERBUFFER];
}

- (void)setupDisplayLink {
    CADisplayLink* displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(render:)];
    [displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
}

- (void)setup
{
    self.contentScaleFactor = 2;
    
    [self setupLayer];
    [self setupContext];
    [self setupDepthBuffer];
    [self setupRenderBuffer];
    [self setupFrameBuffer];
    [self setupDisplayLink];
    
    pangolin::PangolinCommonInit();
    
    const int w = self.frame.size.width * self.contentScaleFactor;
    const int h = self.frame.size.height * self.contentScaleFactor;
    pangolin::process::Resize(w,h);
    
    if(pangolin::context->user_app) {
        pangolin::context->user_app->Init();
    }
}

- (id)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    
    if (self) {
        [self setup];
    }
    return self;
}

- (void)awakeFromNib
{
    [self setup];
}

- (void)dealloc
{
    _context = nil;
}

@end


namespace pangolin
{
    int LaunchUserApp(UserApp& app)
    {
        // Create new context
        BindToContext("UserApp");
        
        // Reference user application
        context->user_app = &app;

        // Start IOS Window containing GL Context from which we'll receive events
        // These events will be communicated to pangolin::process::... and the UserApp app.
        int argc = 1;
        char* argv[] = { (char*)"dummy" };
        @autoreleasepool {
            return UIApplicationMain(argc, argv, nil, NSStringFromClass([PangolinAppDelegate class]));
        }
    }
        
    // Implement platform agnostic version
    void CreateWindowAndBind(std::string window_title, int w, int h )
    {
        throw std::runtime_error("pangolin::CreateWindowAndBind(...) Not supported on this platform");
    }
    
    // Implement platform agnostic version
    void FinishFrame()
    {
        throw std::runtime_error("pangolin::FinishFrame() Not supported on this platform");
    }
    
}

