//
//  GlTestViewController.h
//  gltest
//
//  Created by Steven Lovegrove on 30/01/2014.
//  Copyright (c) 2014 Steven Lovegrove. All rights reserved.
//

#import <UIKit/UIKit.h>

#include <OpenGLES/ES2/gl.h>
#include <OpenGLES/ES2/glext.h>

@interface PangolinUIView : UIView {
    CAEAGLLayer* _eaglLayer;
    EAGLContext* _context;

    GLuint _colorRenderBuffer;
    GLuint _depthRenderBuffer;
}

@end