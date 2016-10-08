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

#include <pangolin/display/display.h>
#include <pangolin/display/device/PangolinNSApplication.h>

////////////////////////////////////////////////////////////////////
// PangolinNSApplication
////////////////////////////////////////////////////////////////////

@implementation PangolinNSApplication

- (void)run_pre
{
    [[NSNotificationCenter defaultCenter]
        postNotificationName:NSApplicationWillFinishLaunchingNotification
        object:NSApp];
    [[NSNotificationCenter defaultCenter]
        postNotificationName:NSApplicationDidFinishLaunchingNotification
        object:NSApp];
}

- (void)run_step
{
    NSEvent *event;
    do{
        event = [self
                nextEventMatchingMask:NSAnyEventMask
                untilDate:nil
//                untilDate: [NSDate distantFuture]
                inMode:NSDefaultRunLoopMode
                dequeue:YES];
        [self sendEvent:event];
        [self updateWindows];
    }while(event != nil);
}

@end

////////////////////////////////////////////////////////////////////
// PangolinWindowDelegate
////////////////////////////////////////////////////////////////////

@implementation PangolinWindowDelegate

- (BOOL)windowShouldClose:(id)sender {
    pangolin::Quit();
    return YES;
}

@end

////////////////////////////////////////////////////////////////////
// PangolinAppDelegate
////////////////////////////////////////////////////////////////////

@implementation PangolinAppDelegate

- (void)dealloc
{
    [super dealloc];
}

@end
