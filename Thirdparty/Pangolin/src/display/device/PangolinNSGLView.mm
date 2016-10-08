#include <pangolin/gl/glinclude.h>
#include <pangolin/display/device/PangolinNSGLView.h>
#include <pangolin/display/display.h>
#include <pangolin/display/display_internal.h>
#include <pangolin/handler/handler_enums.h>

namespace pangolin
{
extern __thread PangolinGl* context;
}

////////////////////////////////////////////////////////////////////
// Input maps
////////////////////////////////////////////////////////////////////

inline
int mapMouseButton(int osx_button )
{
    const int map[] = {0, 2, 1, 3, 4, 5, 6, 7, 8, 9, 10};
    return map[osx_button];
}

inline
int mapKeymap(int osx_key)
{
    if(osx_key == NSUpArrowFunctionKey)
        return pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_UP;
    else if(osx_key == NSDownArrowFunctionKey)
        return pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_DOWN;
    else if(osx_key == NSLeftArrowFunctionKey)
        return pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_LEFT;
    else if(osx_key == NSRightArrowFunctionKey)
        return pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT;
    else if(osx_key == NSPageUpFunctionKey)
        return pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_PAGE_UP;
    else if(osx_key == NSPageDownFunctionKey)
        return pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_PAGE_DOWN;
    else if(osx_key == NSHomeFunctionKey)
        return pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_HOME;
    else if(osx_key == NSEndFunctionKey)
        return pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_END;
    else if(osx_key == NSInsertFunctionKey)
        return pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_INSERT;
    else if(osx_key == NSDeleteCharacter )
        return NSBackspaceCharacter;
    else if(osx_key == NSDeleteFunctionKey)
        return NSDeleteCharacter;
    else {
        return osx_key;
    }
}

////////////////////////////////////////////////////////////////////
// PangolinNSGLView
////////////////////////////////////////////////////////////////////

@implementation PangolinNSGLView

-(id)initWithFrame:(NSRect)frameRect pixelFormat:(NSOpenGLPixelFormat *)format
{
    self = [super initWithFrame:frameRect pixelFormat:format];
    context = pangolin::context;
    return(self);
}

- (void)prepareOpenGL
{
    [super prepareOpenGL];
}

-(void)reshape
{
#if MAC_OS_X_VERSION_MAX_ALLOWED >= MAC_OS_X_VERSION_10_7
    if ( [self wantsBestResolutionOpenGLSurface] && [ _window respondsToSelector:@selector(backingScaleFactor) ] )
        backing_scale = [_window backingScaleFactor];
    else
#endif
        backing_scale = 1.0;

    pangolin::process::Resize(self.bounds.size.width * backing_scale, self.bounds.size.height * backing_scale);

    [[self openGLContext] update];
}

-(BOOL)acceptsFirstResponder
{
    return(YES);
}

-(BOOL)becomeFirstResponder
{
    return(YES);
}

-(BOOL)resignFirstResponder
{
    return(YES);
}

-(BOOL)isFlipped
{
    return(YES);
}

-(NSPoint)_Location:(NSEvent *)theEvent
{
    NSPoint location = [self convertPoint: [theEvent locationInWindow] fromView: nil];
    location.x *= backing_scale;
    location.y *= backing_scale;
    return location;
}

////////////////////////////////////////////////////////////////////
// Keyboard
////////////////////////////////////////////////////////////////////

-(void)keyDown:(NSEvent *)theEvent
{
    const NSPoint location = [self _Location: theEvent];
    NSString *str = [theEvent characters];
    int len = (int)[str length];
    for(int i = 0; i < len; i++)
    {
        const int osx_key = [str characterAtIndex:i];
        pangolin::process::Keyboard(mapKeymap(osx_key), location.x, location.y);
    }
}

-(void)keyUp:(NSEvent *)theEvent
{
    const NSPoint location = [self _Location: theEvent];
    NSString *str = [theEvent characters];
    int len = (int)[str length];
    for(int i = 0; i < len; i++)
    {
        const int osx_key = [str characterAtIndex:i];
        pangolin::process::KeyboardUp(mapKeymap(osx_key), location.x, location.y);
    }
}

- (void)flagsChanged:(NSEvent *)event
{
    unsigned int flags = [event modifierFlags] & NSDeviceIndependentModifierFlagsMask;

    if(flags&NSShiftKeyMask) {
        context->mouse_state |=  pangolin::KeyModifierShift;
    }else{
        context->mouse_state &= ~pangolin::KeyModifierShift;
    }

    if(flags&NSControlKeyMask) {
        context->mouse_state |=  pangolin::KeyModifierCtrl;
    }else{
        context->mouse_state &= ~pangolin::KeyModifierCtrl;
    }

    if(flags&NSAlternateKeyMask) {
        context->mouse_state |=  pangolin::KeyModifierAlt;
    }else{
        context->mouse_state &= ~pangolin::KeyModifierAlt;
    }

    if(flags&NSCommandKeyMask) {
        context->mouse_state |=  pangolin::KeyModifierCmd;
    }else{
        context->mouse_state &= ~pangolin::KeyModifierCmd;
    }

    if(flags&NSFunctionKeyMask) {
        context->mouse_state |=  pangolin::KeyModifierFnc;
    }else{
        context->mouse_state &= ~pangolin::KeyModifierFnc;
    }
}

////////////////////////////////////////////////////////////////////
// Mouse Input
////////////////////////////////////////////////////////////////////

-(void)mouseDownCommon:(NSEvent *)theEvent
{
    const int button = (int)[theEvent buttonNumber];
    const NSPoint location = [self _Location: theEvent];
    pangolin::process::Mouse(mapMouseButton(button), 0, location.x, location.y);
}

-(void)mouseUpCommon:(NSEvent *)theEvent
{
    const int button = (int)[theEvent buttonNumber];
    const NSPoint location = [self _Location: theEvent];
    pangolin::process::Mouse(mapMouseButton(button), 1, location.x, location.y);
}

- (void)mouseDraggedCommon: (NSEvent *)theEvent
{
    const NSPoint location = [self _Location: theEvent];
    pangolin::process::MouseMotion(location.x, location.y);
//    pangolin::process::SubpixMotion(location.x, location.y, 1.0, 0.0, 0.0, 0.0);
}

-(void)mouseDown:(NSEvent *)theEvent
{
    [self mouseDownCommon:theEvent];
}

-(void)mouseUp:(NSEvent *)theEvent
{
    [self mouseUpCommon:theEvent];
}

- (void)mouseDragged: (NSEvent *)theEvent
{
    [self mouseDraggedCommon:theEvent];
}

-(void)rightMouseDown:(NSEvent *)theEvent
{
    [self mouseDownCommon:theEvent];
}

-(void)rightMouseUp:(NSEvent *)theEvent
{
    [self mouseUpCommon:theEvent];
}

- (void)rightMouseDragged: (NSEvent *)theEvent
{
    [self mouseDraggedCommon:theEvent];
}

-(void)otherMouseDown:(NSEvent *)theEvent
{
    [self mouseDownCommon:theEvent];
}

-(void)otherMouseUp:(NSEvent *)theEvent
{
    [self mouseUpCommon:theEvent];
}

- (void)otherMouseDragged: (NSEvent *)theEvent
{
    [self mouseDraggedCommon:theEvent];
}

- (void)mouseMoved: (NSEvent *)theEvent
{
    const NSPoint location = [self _Location: theEvent];
//    pangolin::process::PassiveMouseMotion(location.x, location.y);
    pangolin::process::SubpixMotion(location.x, location.y, 0.0, 0.0, 0.0, 0.0);
}

- (void)scrollWheel:(NSEvent *)theEvent
{
    const NSPoint location = [self _Location: theEvent];

    float dx, dy;
    if([theEvent respondsToSelector:@selector(scrollingDeltaX)]) {
       dx = theEvent.scrollingDeltaX; dy = theEvent.scrollingDeltaY;
    } else {
       dx = theEvent.deltaX; dy = theEvent.deltaY;
    }

    if(dx != 0.0f || dy != 0.0f) {
        pangolin::process::SpecialInput(
            pangolin::InputSpecialScroll,
            location.x, context->base.v.h - location.y,
            dx, dy,
            0, 0
        );
    }
}

- (void)magnifyWithEvent: (NSEvent *)theEvent
{
    const NSPoint location = [self _Location: theEvent];
    const float dm = theEvent.magnification;
    if(dm != 0.0f) {
        pangolin::process::SpecialInput(
                pangolin::InputSpecialZoom,
                location.x, context->base.v.h - location.y,
                dm, 0.0f, 0.0f, 0.0f
        );
    }
}

- (void)rotateWithEvent: (NSEvent *)theEvent
{
    const NSPoint location = [self _Location: theEvent];
    const float dr = theEvent.rotation;
    if(dr != 0.0f) {
        pangolin::process::SpecialInput(
                pangolin::InputSpecialRotate,
                location.x, context->base.v.h - location.y,
                dr, 0.0f, 0.0f, 0.0f
        );
    }
}

- (void)mouseEntered: (NSEvent *)theEvent
{
}

- (void)mouseExited: (NSEvent *)theEvent
{
}

-(void)dealloc
{
    [super dealloc];
}

@end
