/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Steven Lovegrove
 *
 * Based largely on android_native_app_glue.c from Android NDK
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

#include <pangolin/gl/glinclude.h>
#include <pangolin/display/display.h>
#include <pangolin/display/display_internal.h>
#include <pangolin/display/device/display_android.h>

#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <dlfcn.h>

#include <android/configuration.h>
#include <android/looper.h>
#include <android/native_activity.h>
#include <android/sensor.h>
#include <android/log.h>
#include <android/window.h>

#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <stdexcept>

#include <jni.h>

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/resource.h>

namespace pangolin
{

extern __thread PangolinGl* context;

/**
 * Shared state for our app.
 */
struct engine {
    android_app* app;
    ANativeActivity* activity;
    
    int has_focus;
    EGLDisplay display;
    EGLSurface surface;
    EGLContext context;
    int32_t width;
    int32_t height;
//    struct saved_state state;
};

/**
 * Initialize an EGL context for the current display.
 */
static int engine_init_display(struct engine* engine) {
    // initialize OpenGL ES and EGL

    /*
     * Here specify the attributes of the desired configuration.
     * Below, we select an EGLConfig with at least 8 bits per color
     * component compatible with on-screen windows
     */
    const EGLint attribs[] = {
            EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_BLUE_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_RED_SIZE, 8,
            EGL_NONE
    };
    EGLint w, h, format;
    EGLint numConfigs;
    EGLConfig config;
    EGLSurface surface;
    EGLContext context;

    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

    eglInitialize(display, 0, 0);

    /* Here, the application chooses the configuration it desires. In this
     * sample, we have a very simplified selection process, where we pick
     * the first EGLConfig that matches our criteria */
    eglChooseConfig(display, attribs, &config, 1, &numConfigs);

    /* EGL_NATIVE_VISUAL_ID is an attribute of the EGLConfig that is
     * guaranteed to be accepted by ANativeWindow_setBuffersGeometry().
     * As soon as we picked a EGLConfig, we can safely reconfigure the
     * ANativeWindow buffers to match, using EGL_NATIVE_VISUAL_ID. */
    eglGetConfigAttrib(display, config, EGL_NATIVE_VISUAL_ID, &format);

    ANativeWindow_setBuffersGeometry(engine->app->window, 0, 0, format);
//    ANativeActivity_setWindowFlags(engine->app->activity, AWINDOW_FLAG_FULLSCREEN, 0 );

    EGLint const attrib_list[] = {
#ifdef HAVE_GLES_2
        EGL_CONTEXT_CLIENT_VERSION, 2,
#endif
        EGL_NONE
    };

    surface = eglCreateWindowSurface(display, config, engine->app->window, NULL);
    context = eglCreateContext(display, config, NULL, attrib_list);

    if (eglMakeCurrent(display, surface, surface, context) == EGL_FALSE) {
        LOGW("Unable to eglMakeCurrent");
        return -1;
    }

    eglQuerySurface(display, surface, EGL_WIDTH, &w);
    eglQuerySurface(display, surface, EGL_HEIGHT, &h);

    engine->display = display;
    engine->context = context;
    engine->surface = surface;
    engine->width = w;
    engine->height = h;
    
    pangolin::process::Resize(engine->width,engine->height);

    // Initialize GL state.
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glDisable(GL_DEPTH_TEST);

    return 0;
}

/**
 * Just the current frame in the display.
 */
static void engine_draw_frame(struct engine* engine) {
    if (engine->display != NULL) {
    }
}

/**
 * Tear down the EGL context currently associated with the display.
 */
static void engine_term_display(struct engine* engine) {
    if (engine->display != EGL_NO_DISPLAY) {
        eglMakeCurrent(engine->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (engine->context != EGL_NO_CONTEXT) {
            eglDestroyContext(engine->display, engine->context);
        }
        if (engine->surface != EGL_NO_SURFACE) {
            eglDestroySurface(engine->display, engine->surface);
        }
        eglTerminate(engine->display);
    }
    engine->has_focus = 0;
    engine->display = EGL_NO_DISPLAY;
    engine->context = EGL_NO_CONTEXT;
    engine->surface = EGL_NO_SURFACE;
}

void UnpressAll(float last_x, float last_y)
{
    if(context->mouse_state & pangolin::MouseButtonLeft) {
        pangolin::process::Mouse(0, 1, last_x, last_y);
    }
    if(context->mouse_state & pangolin::MouseButtonMiddle) {
        pangolin::process::Mouse(1, 1, last_x, last_y);
    }
    if(context->mouse_state & pangolin::MouseButtonRight) {
        pangolin::process::Mouse(2, 1, last_x, last_y);
    }
    if(context->mouse_state & pangolin::MouseWheelUp) {
        pangolin::process::Mouse(3, 1, last_x, last_y);
    }
    if(context->mouse_state & pangolin::MouseWheelDown) {
        pangolin::process::Mouse(4, 1, last_x, last_y);
    }
    context->mouse_state = 0;
}

int PangolinKeyFromAndroidKeycode(int32_t keycode, bool shift)
{
    if( AKEYCODE_0 <= keycode && keycode <= AKEYCODE_9) {
        return '0' + (keycode - AKEYCODE_0);
    }
    
    if( AKEYCODE_A <= keycode && keycode <= AKEYCODE_Z) {
        return (shift ? 'A' : 'a') + (keycode - AKEYCODE_A);
    }
    
    if(shift) {
        switch (keycode) {
        case AKEYCODE_GRAVE:     return '~';
        default:
            LOGI("Unknown keycode (with shift): %d", keycode);
            return '?';
        }
    }else{
        switch (keycode) {
        case AKEYCODE_COMMA:     return ',';
        case AKEYCODE_PERIOD:    return '.';
        case AKEYCODE_SPACE:     return ' ';
        case AKEYCODE_ENTER:     return '\r';
        case AKEYCODE_TAB:       return '\t';
        case AKEYCODE_DEL:       return '\b';
        case AKEYCODE_SLASH:     return '/';
        case AKEYCODE_BACKSLASH: return '\\';
        case AKEYCODE_SEMICOLON: return ';';
        case AKEYCODE_APOSTROPHE:return '\'';
        case AKEYCODE_MINUS:     return '-';
        case AKEYCODE_EQUALS:    return '=';
        case AKEYCODE_PLUS:      return '+';
        case AKEYCODE_AT:        return '@';
        case AKEYCODE_GRAVE:     return '`';
        default:
            LOGI("Unknown keycode: %d", keycode);
            return '?';
        }
    }
}

/**
 * Process the next input event.
 */
static int32_t engine_handle_input(struct android_app* app, AInputEvent* event) {
    struct engine* engine = (struct engine*)app->userData;

    static float last_x = 0;
    static float last_y = 0;

    const int32_t input_type = AInputEvent_getType(event);
    
    if (input_type == AINPUT_EVENT_TYPE_MOTION) {
        engine->has_focus = 1;        
        

        const float x = AMotionEvent_getX(event, 0);
        const float y = AMotionEvent_getY(event, 0);
        const int32_t actionAndPtr = AMotionEvent_getAction(event);
        const int32_t action = AMOTION_EVENT_ACTION_MASK & actionAndPtr;
//        const int32_t ptrindex = (AMOTION_EVENT_ACTION_POINTER_INDEX_MASK & actionAndPtr) >> AMOTION_EVENT_ACTION_POINTER_INDEX_SHIFT;
                
        const size_t num_ptrs = AMotionEvent_getPointerCount(event);

        switch(action)
        {
        case AMOTION_EVENT_ACTION_UP:
        case AMOTION_EVENT_ACTION_POINTER_UP:
            UnpressAll(last_x, last_y);
            break;
        case AMOTION_EVENT_ACTION_DOWN:
        case AMOTION_EVENT_ACTION_POINTER_DOWN:
            UnpressAll(last_x, last_y);
            if(num_ptrs <=2) {
                const int button = (num_ptrs==1) ? 0 : 2;
                pangolin::process::Mouse(button, 0, x, y);
            }
            break;
        case AMOTION_EVENT_ACTION_MOVE:
            if(num_ptrs == 3) {
                const double dx = x - last_x;
                const double dy = y - last_y;
                pangolin::process::Scroll(dx,dy);
            }else{
                pangolin::process::MouseMotion(x,y);
            }
            break;
        default:
            break;
        }

        last_x = x;
        last_y = y;

        return 1;
    }else if(AINPUT_EVENT_TYPE_KEY) {
        static bool shift = false;
        
        const int32_t action = AKeyEvent_getAction(event);
        const int32_t keycode  = AKeyEvent_getKeyCode(event);
        
        if(keycode == AKEYCODE_SHIFT_LEFT) {
            shift = (action == AKEY_EVENT_ACTION_DOWN);
            return 1;
        }
        
        unsigned char key = PangolinKeyFromAndroidKeycode(keycode, shift);
        
        if(action == AKEY_EVENT_ACTION_DOWN) {
            pangolin::process::Keyboard(key, last_x, last_y);
        }else{
            pangolin::process::KeyboardUp(key, last_x, last_y);
        }
    }
    return 1;
}

/**
 * Process the next main command.
 */
static void engine_handle_cmd(struct android_app* app, int32_t cmd) {
    struct engine* engine = (struct engine*)app->userData;
    switch (cmd) {
        case APP_CMD_SAVE_STATE:
//            // The system has asked us to save our current state.  Do so.
//            engine->app->savedState = malloc(sizeof(struct saved_state));
//            *((struct saved_state*)engine->app->savedState) = engine->state;
//            engine->app->savedStateSize = sizeof(struct saved_state);
            break;
        case APP_CMD_INIT_WINDOW:
            // The window is being shown, get it ready.
            if (engine->app->window != NULL) {
                engine_init_display(engine);
                engine_draw_frame(engine);
            }
            break;
        case APP_CMD_TERM_WINDOW:
            // The window is being hidden or closed, clean it up.
            engine_term_display(engine);
            break;
        case APP_CMD_GAINED_FOCUS:
            engine->has_focus = 1;            
            break;
        case APP_CMD_LOST_FOCUS:
            engine->has_focus = 0;
            break;
    }
}

}

// Define library entry point.
extern "C" {

JNIEnv* GetEnvAttachThread(JavaVM* vm)
{
    JNIEnv* env;
    switch (vm->GetEnv((void**)&env, JNI_VERSION_1_6)) {
    case JNI_OK:
        break;
    case JNI_EDETACHED:
        if (vm->AttachCurrentThread(&env, NULL)!=0) {
            LOGE("Could not attach current thread");
            throw std::runtime_error("Could not attach current thread");
        }
        break;
    case JNI_EVERSION:
        LOGE("Invalid Java version");
        throw std::runtime_error("Invalid Java version");
    }
    return env;
}

// https://groups.google.com/d/msg/android-ndk/Tk3g00wLKhk/TJQucoaE_asJ
void displayKeyboard(android_app* app, bool pShow) {
    jint lFlags = 0;
    JNIEnv* env = GetEnvAttachThread(app->activity->vm);
    if(env) {
        // Retrieves NativeActivity. 
        jobject lNativeActivity = app->activity->clazz; 
        jclass ClassNativeActivity = env->GetObjectClass(lNativeActivity); 
    
        // Retrieves Context.INPUT_METHOD_SERVICE. 
        jclass ClassContext = env->FindClass("android/content/Context"); 
        jfieldID FieldINPUT_METHOD_SERVICE = env->GetStaticFieldID(ClassContext, "INPUT_METHOD_SERVICE", "Ljava/lang/String;"); 
        jobject INPUT_METHOD_SERVICE = env->GetStaticObjectField(ClassContext, FieldINPUT_METHOD_SERVICE); 
    //    jniCheck(INPUT_METHOD_SERVICE); 
    
        // lInputMethodManager = getSystemService(Context.INPUT_METHOD_SERVICE). 
        jclass ClassInputMethodManager = env->FindClass( "android/view/inputmethod/InputMethodManager"); 
        jmethodID MethodGetSystemService = env->GetMethodID( ClassNativeActivity, "getSystemService",  "(Ljava/lang/String;)Ljava/lang/Object;"); 
        jobject lInputMethodManager = env->CallObjectMethod( lNativeActivity, MethodGetSystemService, INPUT_METHOD_SERVICE); 
    
        // lDecorView = getWindow().getDecorView(). 
        jmethodID MethodGetWindow = env->GetMethodID( ClassNativeActivity, "getWindow",  "()Landroid/view/Window;"); 
        jobject lWindow = env->CallObjectMethod(lNativeActivity,  MethodGetWindow); 
        jclass ClassWindow = env->FindClass(  "android/view/Window"); 
        jmethodID MethodGetDecorView = env->GetMethodID( ClassWindow, "getDecorView", "()Landroid/view/View;"); 
        jobject lDecorView = env->CallObjectMethod(lWindow,  MethodGetDecorView); 
    
        if (pShow) { 
            // Runs lInputMethodManager.showSoftInput(...). 
            jmethodID MethodShowSoftInput = env->GetMethodID( ClassInputMethodManager, "showSoftInput", "(Landroid/view/View;I)Z"); 
            /*jboolean lResult = */env->CallBooleanMethod( lInputMethodManager, MethodShowSoftInput, lDecorView, lFlags); 
        } else { 
            // Runs lWindow.getViewToken() 
            jclass ClassView = env->FindClass( "android/view/View"); 
            jmethodID MethodGetWindowToken = env->GetMethodID( ClassView, "getWindowToken", "()Landroid/os/IBinder;"); 
            jobject lBinder = env->CallObjectMethod(lDecorView, MethodGetWindowToken); 
    
            // lInputMethodManager.hideSoftInput(...). 
            jmethodID MethodHideSoftInput = env->GetMethodID( ClassInputMethodManager, "hideSoftInputFromWindow", "(Landroid/os/IBinder;I)Z"); 
            /*jboolean lRes = */env->CallBooleanMethod( lInputMethodManager, MethodHideSoftInput, lBinder, lFlags); 
        } 
    
        // Finished with the JVM. 
        app->activity->vm->DetachCurrentThread();
    }
}

pangolin::engine g_engine;

static void free_saved_state(struct android_app* android_app) {
    pthread_mutex_lock(&android_app->mutex);
    if (android_app->savedState != NULL) {
        free(android_app->savedState);
        android_app->savedState = NULL;
        android_app->savedStateSize = 0;
    }
    pthread_mutex_unlock(&android_app->mutex);
}

int8_t android_app_read_cmd(struct android_app* android_app) {
    int8_t cmd;
    if (read(android_app->msgread, &cmd, sizeof(cmd)) == sizeof(cmd)) {
        switch (cmd) {
            case APP_CMD_SAVE_STATE:
                free_saved_state(android_app);
                break;
        }
        return cmd;
    } else {
        LOGE("No data on command pipe!");
    }
    return -1;
}

static void print_cur_config(struct android_app* android_app) {
    char lang[2], country[2];
    AConfiguration_getLanguage(android_app->config, lang);
    AConfiguration_getCountry(android_app->config, country);

    LOGV("Config: mcc=%d mnc=%d lang=%c%c cnt=%c%c orien=%d touch=%d dens=%d "
            "keys=%d nav=%d keysHid=%d navHid=%d sdk=%d size=%d long=%d "
            "modetype=%d modenight=%d",
            AConfiguration_getMcc(android_app->config),
            AConfiguration_getMnc(android_app->config),
            lang[0], lang[1], country[0], country[1],
            AConfiguration_getOrientation(android_app->config),
            AConfiguration_getTouchscreen(android_app->config),
            AConfiguration_getDensity(android_app->config),
            AConfiguration_getKeyboard(android_app->config),
            AConfiguration_getNavigation(android_app->config),
            AConfiguration_getKeysHidden(android_app->config),
            AConfiguration_getNavHidden(android_app->config),
            AConfiguration_getSdkVersion(android_app->config),
            AConfiguration_getScreenSize(android_app->config),
            AConfiguration_getScreenLong(android_app->config),
            AConfiguration_getUiModeType(android_app->config),
            AConfiguration_getUiModeNight(android_app->config));
}

void android_app_pre_exec_cmd(struct android_app* android_app, int8_t cmd) {
    switch (cmd) {
        case APP_CMD_INPUT_CHANGED:
            LOGV("APP_CMD_INPUT_CHANGED\n");
            pthread_mutex_lock(&android_app->mutex);
            if (android_app->inputQueue != NULL) {
                AInputQueue_detachLooper(android_app->inputQueue);
            }
            android_app->inputQueue = android_app->pendingInputQueue;
            if (android_app->inputQueue != NULL) {
                LOGV("Attaching input queue to looper");
                AInputQueue_attachLooper(android_app->inputQueue,
                        android_app->looper, LOOPER_ID_INPUT, NULL,
                        &android_app->inputPollSource);
            }
            pthread_cond_broadcast(&android_app->cond);
            pthread_mutex_unlock(&android_app->mutex);
            break;

        case APP_CMD_INIT_WINDOW:
            LOGV("APP_CMD_INIT_WINDOW\n");
            pthread_mutex_lock(&android_app->mutex);
            android_app->window = android_app->pendingWindow;
            pthread_cond_broadcast(&android_app->cond);
            pthread_mutex_unlock(&android_app->mutex);
            break;

        case APP_CMD_TERM_WINDOW:
            LOGV("APP_CMD_TERM_WINDOW\n");
            pthread_cond_broadcast(&android_app->cond);
            break;

        case APP_CMD_RESUME:
        case APP_CMD_START:
        case APP_CMD_PAUSE:
        case APP_CMD_STOP:
            LOGV("activityState=%d\n", cmd);
            pthread_mutex_lock(&android_app->mutex);
            android_app->activityState = cmd;
            pthread_cond_broadcast(&android_app->cond);
            pthread_mutex_unlock(&android_app->mutex);
            break;

        case APP_CMD_CONFIG_CHANGED:
            LOGV("APP_CMD_CONFIG_CHANGED\n");
            AConfiguration_fromAssetManager(android_app->config,
                    android_app->activity->assetManager);
            print_cur_config(android_app);
            break;

        case APP_CMD_DESTROY:
            LOGV("APP_CMD_DESTROY\n");
            android_app->destroyRequested = 1;
            break;
    }
}

void android_app_post_exec_cmd(struct android_app* android_app, int8_t cmd) {
    switch (cmd) {
        case APP_CMD_TERM_WINDOW:
            LOGV("APP_CMD_TERM_WINDOW\n");
            pthread_mutex_lock(&android_app->mutex);
            android_app->window = NULL;
            pthread_cond_broadcast(&android_app->cond);
            pthread_mutex_unlock(&android_app->mutex);
            break;

        case APP_CMD_SAVE_STATE:
            LOGV("APP_CMD_SAVE_STATE\n");
            pthread_mutex_lock(&android_app->mutex);
            android_app->stateSaved = 1;
            pthread_cond_broadcast(&android_app->cond);
            pthread_mutex_unlock(&android_app->mutex);
            break;

        case APP_CMD_RESUME:
            free_saved_state(android_app);
            break;
    }
}

static void android_app_destroy(struct android_app* android_app) {
    LOGV("+android_app_destroy!");
    free_saved_state(android_app);
    pthread_mutex_lock(&android_app->mutex);
    if (android_app->inputQueue != NULL) {
        AInputQueue_detachLooper(android_app->inputQueue);
    }
    AConfiguration_delete(android_app->config);
    android_app->destroyed = 1;
    pthread_cond_broadcast(&android_app->cond);
    pthread_mutex_unlock(&android_app->mutex);
    // Can't touch android_app object after this.
    LOGV("-android_app_destroy!");
}

static void process_input(struct android_app* app, struct android_poll_source* source) {
    AInputEvent* event = NULL;
    if (AInputQueue_getEvent(app->inputQueue, &event) >= 0) {
        
        // HACK: Override back buttom to show / hide keyboard.
        int type = AInputEvent_getType(event);
        if(type == AINPUT_EVENT_TYPE_KEY) {
            if(AKeyEvent_getAction(event) == AKEY_EVENT_ACTION_DOWN) {
                static bool keyboard_shown = false;
                if( AKeyEvent_getKeyCode(event) == AKEYCODE_BACK ) {
                    displayKeyboard(app,!keyboard_shown);
                    keyboard_shown = !keyboard_shown;
                    AInputQueue_finishEvent(app->inputQueue, event, 1);
                    return;
                }
            }
        }    
        
        if (AInputQueue_preDispatchEvent(app->inputQueue, event)) {
            return;
        }
        int32_t handled = 0;
        if (app->onInputEvent != NULL) handled = app->onInputEvent(app, event);
        AInputQueue_finishEvent(app->inputQueue, event, handled);
    } else {
        LOGE("Failure reading next input event: %s\n", strerror(errno));
    }
}

static void process_cmd(struct android_app* app, struct android_poll_source* source) {
    int8_t cmd = android_app_read_cmd(app);
    android_app_pre_exec_cmd(app, cmd);
    if (app->onAppCmd != NULL) app->onAppCmd(app, cmd);
    android_app_post_exec_cmd(app, cmd);
}

static void* android_app_entry(void* param) {
    LOGV("+android_app_entry");
    struct android_app* android_app = (struct android_app*)param;

    android_app->config = AConfiguration_new();
    AConfiguration_fromAssetManager(android_app->config, android_app->activity->assetManager);

    print_cur_config(android_app);

    android_app->cmdPollSource.id = LOOPER_ID_MAIN;
    android_app->cmdPollSource.app = android_app;
    android_app->cmdPollSource.process = process_cmd;
    android_app->inputPollSource.id = LOOPER_ID_INPUT;
    android_app->inputPollSource.app = android_app;
    android_app->inputPollSource.process = process_input;

    ALooper* looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    ALooper_addFd(looper, android_app->msgread, LOOPER_ID_MAIN, ALOOPER_EVENT_INPUT, NULL,
            &android_app->cmdPollSource);
    android_app->looper = looper;

    pthread_mutex_lock(&android_app->mutex);
    android_app->running = 1;
    pthread_cond_broadcast(&android_app->cond);
    pthread_mutex_unlock(&android_app->mutex);
    
    std::string sargv;
    
    // Load command line from ARGV parameter
    JNIEnv *env = GetEnvAttachThread(android_app->activity->vm);
    if(env) {
        jobject me = android_app->activity->clazz;
        
        jclass acl = env->GetObjectClass(me); //class pointer of NativeActivity
        jmethodID giid = env->GetMethodID(acl, "getIntent", "()Landroid/content/Intent;");
        jobject intent = env->CallObjectMethod(me, giid); //Got our intent
        
        jclass icl = env->GetObjectClass(intent); //class pointer of Intent
        jmethodID gseid = env->GetMethodID(icl, "getStringExtra", "(Ljava/lang/String;)Ljava/lang/String;");
        
        jstring jsARGV = (jstring)env->CallObjectMethod(intent, gseid, env->NewStringUTF("ARGV"));
        
        
        if(jsARGV) {
            const char *chARGV = env->GetStringUTFChars(jsARGV, 0);
            if(chARGV) {
                sargv = std::string(chARGV);
                LOGI("ARGV: pango %s", chARGV);
            }
            env->ReleaseStringUTFChars(jsARGV, chARGV);    
        }
        
        android_app->activity->vm->DetachCurrentThread();
    }

    // Set up argv/argc to pass to users main
    std::vector<std::string> vargv;
    vargv.push_back("pango");
    
    // Parse parameters from ARGV android intent parameter
    std::istringstream iss(sargv);
    std::copy(std::istream_iterator<std::string>(iss),
             std::istream_iterator<std::string>(),
             std::back_inserter<std::vector<std::string> >(vargv));    

    char* argv[vargv.size()+1];
    for(size_t ac = 0; ac < vargv.size(); ++ac) {
        argv[ac] = new char[vargv[ac].size()];
        strcpy( argv[ac], vargv[ac].c_str() );
    }
    argv[vargv.size()] = NULL;

    // Find main symbol
    void (*main)(int, char**);
    *(void **) (&main) = dlsym( dlopen(android_app->application_so, RTLD_NOW), "main");
    if (!main) {
        LOGE( "undefined symbol main, crap" );
        exit(1);
    }
    // Call users standard main entry point.
    (*main)(vargv.size(), argv);
    
    // Clean up parameters
    for(size_t ac = 0; ac < vargv.size(); ++ac) {
        delete[] argv[ac];
    }    
    
    android_app_destroy(android_app);
    
    LOGV("-android_app_entry");
    
    return NULL;
}

static void android_app_write_cmd(struct android_app* android_app, int8_t cmd) {
    if (write(android_app->msgwrite, &cmd, sizeof(cmd)) != sizeof(cmd)) {
        LOGE("Failure writing android_app cmd: %s\n", strerror(errno));
    }
}

static void android_app_set_input(struct android_app* android_app, AInputQueue* inputQueue) {
    pthread_mutex_lock(&android_app->mutex);
    android_app->pendingInputQueue = inputQueue;
    android_app_write_cmd(android_app, APP_CMD_INPUT_CHANGED);
    while (android_app->inputQueue != android_app->pendingInputQueue) {
        pthread_cond_wait(&android_app->cond, &android_app->mutex);
    }
    pthread_mutex_unlock(&android_app->mutex);
}

static void android_app_set_window(struct android_app* android_app, ANativeWindow* window) {
    pthread_mutex_lock(&android_app->mutex);
    if (android_app->pendingWindow != NULL) {
        android_app_write_cmd(android_app, APP_CMD_TERM_WINDOW);
    }
    android_app->pendingWindow = window;
    if (window != NULL) {
        android_app_write_cmd(android_app, APP_CMD_INIT_WINDOW);
    }
    while (android_app->window != android_app->pendingWindow) {
        pthread_cond_wait(&android_app->cond, &android_app->mutex);
    }
    pthread_mutex_unlock(&android_app->mutex);
}

static void android_app_set_activity_state(struct android_app* android_app, int8_t cmd) {
    pthread_mutex_lock(&android_app->mutex);
    android_app_write_cmd(android_app, cmd);
    while (android_app->activityState != cmd) {
        pthread_cond_wait(&android_app->cond, &android_app->mutex);
    }
    pthread_mutex_unlock(&android_app->mutex);
}

static void android_app_free(struct android_app* android_app) {
    pthread_mutex_lock(&android_app->mutex);
    android_app_write_cmd(android_app, APP_CMD_DESTROY);
    while (!android_app->destroyed) {
        pthread_cond_wait(&android_app->cond, &android_app->mutex);
    }
    pthread_mutex_unlock(&android_app->mutex);

    close(android_app->msgread);
    close(android_app->msgwrite);
    pthread_cond_destroy(&android_app->cond);
    pthread_mutex_destroy(&android_app->mutex);
    free(android_app);
}

static void onDestroy(ANativeActivity* activity) {
    LOGV("Destroy: %p\n", activity);
    android_app_free((struct android_app*)activity->instance);
}

static void onStart(ANativeActivity* activity) {
    LOGV("Start: %p\n", activity);
    android_app_set_activity_state((struct android_app*)activity->instance, APP_CMD_START);
}

static void onResume(ANativeActivity* activity) {
    LOGV("Resume: %p\n", activity);
    android_app_set_activity_state((struct android_app*)activity->instance, APP_CMD_RESUME);
}

static void* onSaveInstanceState(ANativeActivity* activity, size_t* outLen) {
    struct android_app* android_app = (struct android_app*)activity->instance;
    void* savedState = NULL;

    LOGV("SaveInstanceState: %p\n", activity);
    pthread_mutex_lock(&android_app->mutex);
    android_app->stateSaved = 0;
    android_app_write_cmd(android_app, APP_CMD_SAVE_STATE);
    while (!android_app->stateSaved) {
        pthread_cond_wait(&android_app->cond, &android_app->mutex);
    }

    if (android_app->savedState != NULL) {
        savedState = android_app->savedState;
        *outLen = android_app->savedStateSize;
        android_app->savedState = NULL;
        android_app->savedStateSize = 0;
    }

    pthread_mutex_unlock(&android_app->mutex);

    return savedState;
}

static void onPause(ANativeActivity* activity) {
    LOGV("Pause: %p\n", activity);
    android_app_set_activity_state((struct android_app*)activity->instance, APP_CMD_PAUSE);
}

static void onStop(ANativeActivity* activity) {
    LOGV("Stop: %p\n", activity);
    android_app_set_activity_state((struct android_app*)activity->instance, APP_CMD_STOP);
}

static void onConfigurationChanged(ANativeActivity* activity) {
    struct android_app* android_app = (struct android_app*)activity->instance;
    LOGV("ConfigurationChanged: %p\n", activity);
    android_app_write_cmd(android_app, APP_CMD_CONFIG_CHANGED);
}

static void onLowMemory(ANativeActivity* activity) {
    struct android_app* android_app = (struct android_app*)activity->instance;
    LOGV("LowMemory: %p\n", activity);
    android_app_write_cmd(android_app, APP_CMD_LOW_MEMORY);
}

static void onWindowFocusChanged(ANativeActivity* activity, int focused) {
    LOGV("WindowFocusChanged: %p -- %d\n", activity, focused);
    android_app_write_cmd((struct android_app*)activity->instance,
            focused ? APP_CMD_GAINED_FOCUS : APP_CMD_LOST_FOCUS);
}

static void onNativeWindowCreated(ANativeActivity* activity, ANativeWindow* window) {
    LOGV("NativeWindowCreated: %p -- %p\n", activity, window);
    android_app_set_window((struct android_app*)activity->instance, window);
}

static void onNativeWindowDestroyed(ANativeActivity* activity, ANativeWindow* window) {
    LOGV("NativeWindowDestroyed: %p -- %p\n", activity, window);
    android_app_set_window((struct android_app*)activity->instance, NULL);
}

static void onInputQueueCreated(ANativeActivity* activity, AInputQueue* queue) {
    LOGV("InputQueueCreated: %p -- %p\n", activity, queue);
    android_app_set_input((struct android_app*)activity->instance, queue);
}

static void onInputQueueDestroyed(ANativeActivity* activity, AInputQueue* queue) {
    LOGV("InputQueueDestroyed: %p -- %p\n", activity, queue);
    android_app_set_input((struct android_app*)activity->instance, NULL);
}

static void onContentRectChanged(ANativeActivity* activity, const ARect* rect) {
    LOGV("onContentRectChanged: %p -- (%d, %d), (%d, %d)\n", activity, rect->left, rect->top, rect->right, rect->bottom);
}

void DeferredNativeActivity_onCreate(
        ANativeActivity* activity,
        void* savedState,
        size_t savedStateSize,
        const char* load_target
    )
{
    activity->callbacks->onDestroy = onDestroy;
    activity->callbacks->onStart = onStart;
    activity->callbacks->onResume = onResume;
    activity->callbacks->onSaveInstanceState = onSaveInstanceState;
    activity->callbacks->onPause = onPause;
    activity->callbacks->onStop = onStop;
    activity->callbacks->onConfigurationChanged = onConfigurationChanged;
    activity->callbacks->onLowMemory = onLowMemory;
    activity->callbacks->onWindowFocusChanged = onWindowFocusChanged;
    activity->callbacks->onNativeWindowCreated = onNativeWindowCreated;
    activity->callbacks->onNativeWindowDestroyed = onNativeWindowDestroyed;
    activity->callbacks->onInputQueueCreated = onInputQueueCreated;
    activity->callbacks->onInputQueueDestroyed = onInputQueueDestroyed;
    activity->callbacks->onContentRectChanged = onContentRectChanged;

    // Create threaded android_app
    android_app* app = (struct android_app*)malloc(sizeof(struct android_app));
    memset(app, 0, sizeof(struct android_app));
    app->activity = activity;
    app->application_so = load_target;

    pthread_mutex_init(&app->mutex, NULL);
    pthread_cond_init(&app->cond, NULL);

    if (savedState != NULL) {
        app->savedState = malloc(savedStateSize);
        app->savedStateSize = savedStateSize;
        memcpy(app->savedState, savedState, savedStateSize);
    }

    int msgpipe[2];
    if (pipe(msgpipe)) {
        LOGE("could not create pipe: %s", strerror(errno));
        exit(1);
    }
    app->msgread = msgpipe[0];
    app->msgwrite = msgpipe[1];

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_create(&app->thread, &attr, android_app_entry, app);

    // Wait for thread to start.
    pthread_mutex_lock(&app->mutex);
    while (!app->running) {
        pthread_cond_wait(&app->cond, &app->mutex);
    }
    pthread_mutex_unlock(&app->mutex);

    activity->instance = app;
    
    // Save global variables for use later
    memset(&g_engine, 0, sizeof(pangolin::engine));
    app->userData = &g_engine;
    app->onAppCmd = pangolin::engine_handle_cmd;
    app->onInputEvent = pangolin::engine_handle_input;
    g_engine.app = app;
    g_engine.activity = activity;

//    // Load existing state if it exists
//    if (app->savedState != NULL) {
//        // We are starting with a previous saved state; restore from it.
//        g_engine.state = *(struct pangolin::saved_state*)app->savedState;
//    }
}
}

namespace pangolin
{

void CreateAndroidWindowAndBind(std::string name)
{
    LOGI("*****************************************************************");
    LOGV("+CreateAndroidWindowAndBind");
    // Bind and Wait for GL Context
    pangolin::BindToContext(name);
    ProcessAndroidEvents();    
    LOGV("-CreateAndroidWindowAndBind");    
}

void ProcessAndroidEvents()
{
    do {
        // Read all pending events.
        int ident;
        int events;
        struct android_poll_source* source;
        
        // If not animating, we will block forever waiting for events.
        // If animating, we loop until all events are read, then continue
        // to draw the next frame of animation.
        while ((ident=ALooper_pollAll(g_engine.has_focus ? 0 : -1, NULL, &events,
                (void**)&source)) >= 0) {
    
            // Process this event.
            if (source != NULL) {
                source->process(g_engine.app, source);
            }
        
            // Check if we are exiting.
            if (g_engine.app->destroyRequested != 0) {
                engine_term_display(&g_engine);
                context->quit = true;
                return;
            }
        }
    } while (g_engine.display == NULL);
}

void FinishAndroidFrame()
{
    ProcessAndroidEvents();
    RenderViews();
    PostRender();
    eglSwapBuffers(g_engine.display, g_engine.surface);    
}

// Implement platform agnostic version
void CreateWindowAndBind(std::string window_title, int /*w*/, int /*h*/, const Params& /*params*/ )
{
    CreateAndroidWindowAndBind(window_title);

#ifdef HAVE_GLES_2
    // Bind default compatibility shader
    pangolin::glEngine().prog_fixed.Bind();
#endif
}

// Implement platform agnostic version
void FinishFrame()
{
    FinishAndroidFrame();
}

void SetFullscreen(bool /*fullscreen*/)
{
    // Do nothing
}

void PangolinPlatformInit(PangolinGl& /*context*/)
{
}

void PangolinPlatformDeinit(PangolinGl& /*context*/)
{
}


}
