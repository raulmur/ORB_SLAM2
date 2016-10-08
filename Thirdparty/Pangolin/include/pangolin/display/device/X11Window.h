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

#ifndef PANGOLIN_X11WINDOW_H
#define PANGOLIN_X11WINDOW_H

#include <pangolin/platform.h>
#include <pangolin/display/display_internal.h>

#include <stdexcept>
#include <string>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

namespace pangolin
{

struct X11Display
{
    X11Display(const char* name = 0) {
        XInitThreads();
        display = XOpenDisplay(name);
        if (!display) {
            throw std::runtime_error("Pangolin X11: Failed to open X display");
        }
    }

    ~X11Display() {
        XCloseDisplay(display);
    }

    // Owns the display
    ::Display* display;
};

struct X11GlContext : public GlContextInterface
{
    X11GlContext(boostd::shared_ptr<X11Display> &d, ::GLXFBConfig chosenFbc, boostd::shared_ptr<X11GlContext> shared_context = boostd::shared_ptr<X11GlContext>() );
    ~X11GlContext();

    boostd::shared_ptr<X11Display> display;

    boostd::shared_ptr<X11GlContext> shared_context;

    // Owns the OpenGl Context
    ::GLXContext glcontext;
};

struct X11Window : public PangolinGl
{
    X11Window(
        const std::string& title, int width, int height,
        boostd::shared_ptr<X11Display>& display, ::GLXFBConfig chosenFbc
    );

    ~X11Window();

    void ToggleFullscreen() PANGOLIN_OVERRIDE;

    void Move(int x, int y) PANGOLIN_OVERRIDE;

    void Resize(unsigned int w, unsigned int h) PANGOLIN_OVERRIDE;

    void MakeCurrent(GLXContext ctx);

    void MakeCurrent() PANGOLIN_OVERRIDE;

    void SwapBuffers() PANGOLIN_OVERRIDE;

    void ProcessEvents() PANGOLIN_OVERRIDE;

    // References the X11 display and context.
    boostd::shared_ptr<X11Display> display;
    boostd::shared_ptr<X11GlContext> glcontext;

    // Owns the X11 Window and Colourmap
    ::Window win;
    ::Colormap cmap;

    Atom delete_message;
};

}

#endif // PANGOLIN_X11WINDOW_H
