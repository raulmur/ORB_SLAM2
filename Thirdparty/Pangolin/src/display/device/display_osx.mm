/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
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

#include <pangolin/platform.h>
#include <pangolin/gl/glinclude.h>
#include <pangolin/display/display.h>
#include <pangolin/display/display_internal.h>
#include <pangolin/display/device/OsxWindow.h>
#include <pangolin/display/device/PangolinNSGLView.h>
#include <pangolin/display/device/PangolinNSApplication.h>

// Hack to fix window focus issue
// http://www.miscdebris.net/blog/2010/03/30/solution-for-my-mac-os-x-gui-program-doesnt-get-focus-if-its-outside-an-application-bundle/
extern "C" { void CPSEnableForegroundOperation(ProcessSerialNumber* psn); }
inline void FixOsxFocus()
{
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    ProcessSerialNumber psn;
    GetCurrentProcess( &psn );
    CPSEnableForegroundOperation( &psn );
    SetFrontProcess( &psn );
#pragma clang diagnostic pop
}

namespace pangolin
{
extern __thread PangolinGl* context;
}

namespace pangolin
{

WindowInterface& CreateWindowAndBind(std::string window_title, int w, int h, const Params& params )
{
    bool highres = params.Get<bool>(PARAM_HIGHRES, true);

    OsxWindow* win = new OsxWindow(window_title, w, h, highres);

    // Add to context map
    AddNewContext(window_title, boostd::shared_ptr<PangolinGl>(win) );

    return *context;
}

OsxWindow::OsxWindow(
    const std::string& title, int width, int height, bool USE_RETINA
) {
    context = this;

    PangolinGl::is_double_buffered = true;
    PangolinGl::windowed_size[0] = width;
    PangolinGl::windowed_size[1] = height;

//    // These are important I think!
//    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
//    [pool release];

    ///////////////////////////////////////////////////////////////////////
    // Make sure Application is initialised correctly.
    // This can be run repeatedly.

    NSApp = [PangolinNSApplication sharedApplication];
    PangolinAppDelegate *delegate = [[PangolinAppDelegate alloc] init];

    [[PangolinNSApplication sharedApplication] setDelegate:delegate];
    [[PangolinNSApplication sharedApplication] setPresentationOptions:NSFullScreenWindowMask];

    [NSApp run_pre];
    [NSApp run_step];

    ///////////////////////////////////////////////////////////////////////
    // Create Window

    NSRect viewRect = NSMakeRect( 0.0, 0.0, width, height );

    _window = [[NSWindow alloc] initWithContentRect:viewRect styleMask:NSTitledWindowMask|NSMiniaturizableWindowMask|NSResizableWindowMask|NSClosableWindowMask backing:NSBackingStoreBuffered defer:YES];
    [_window setTitle:[NSString stringWithUTF8String:title.c_str()]];
    [_window setOpaque:YES];
    [_window makeKeyAndOrderFront:NSApp];
    [_window setCollectionBehavior: NSWindowCollectionBehaviorFullScreenPrimary];

    PangolinWindowDelegate *windelegate = [[PangolinWindowDelegate alloc] init];
    [_window setDelegate:windelegate];

    ///////////////////////////////////////////////////////////////////////
    // Setup Menu

//    NSMenu *mainMenuBar;
//    NSMenu *appMenu;
//    NSMenuItem *menuItem;

//    mainMenuBar = [[NSMenu alloc] init];

//    appMenu = [[NSMenu alloc] initWithTitle:@"Pangolin Application"];
//    menuItem = [appMenu addItemWithTitle:@"Quit Pangolin Application" action:@selector(terminate:) keyEquivalent:@"q"];
//    [menuItem setKeyEquivalentModifierMask:NSCommandKeyMask];

//    menuItem = [[NSMenuItem alloc] init];
//    [menuItem setSubmenu:appMenu];

//    [mainMenuBar addItem:menuItem];

//    //[NSApp performSelector:@selector(setAppleMenu:) withObject:appMenu];
//    [appMenu release];
//    [NSApp setMainMenu:mainMenuBar];

    ///////////////////////////////////////////////////////////////////////
    // Create OpenGL View for Window

    NSOpenGLPixelFormatAttribute attrs[] =
    {
        NSOpenGLPFADoubleBuffer,
        NSOpenGLPFADepthSize, 32,
        NSOpenGLPFAOpenGLProfile, NSOpenGLProfileVersionLegacy,
        0
    };

    NSOpenGLPixelFormat *format = [[NSOpenGLPixelFormat alloc] initWithAttributes:attrs];
    view = [[PangolinNSGLView alloc] initWithFrame:_window.frame pixelFormat:format];
    [format release];
#if MAC_OS_X_VERSION_MAX_ALLOWED >= 1070
    if( USE_RETINA && floor(NSAppKitVersionNumber) > NSAppKitVersionNumber10_6)
        [view setWantsBestResolutionOpenGLSurface:YES];
#endif /*MAC_OS_X_VERSION_MAX_ALLOWED*/

    [_window setContentView:view];

    [NSApp run_step];

    glewInit();

    FixOsxFocus();
}

OsxWindow::~OsxWindow()
{
    // Not sure how to deallocate...
}

void OsxWindow::StartFullScreen()
{
    if(!is_fullscreen) {
        [_window toggleFullScreen:nil];
        is_fullscreen = true;
    }
}

void OsxWindow::StopFullScreen()
{
    if(is_fullscreen) {
        [_window toggleFullScreen:nil];
        is_fullscreen = false;
    }
}

void OsxWindow::ToggleFullscreen()
{
    [_window toggleFullScreen:nil];
    PangolinGl::is_fullscreen = !PangolinGl::is_fullscreen;
}

void OsxWindow::Move(int x, int y)
{

}

void OsxWindow::Resize(unsigned int w, unsigned int h)
{

}

void OsxWindow::MakeCurrent()
{
    [[view openGLContext] makeCurrentContext];
}

void OsxWindow::SwapBuffers()
{
    [[view openGLContext] flushBuffer];
//    [[view openGLContext] update];
//    [view setNeedsDisplay:YES];
}

void OsxWindow::ProcessEvents()
{
    [NSApp run_step];
}

}
