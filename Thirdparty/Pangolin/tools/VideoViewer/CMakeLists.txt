# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(VideoViewer main.cpp)
target_link_libraries(VideoViewer ${Pangolin_LIBRARIES})


# Make file association
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux" AND ${CMAKE_MAJOR_VERSION} VERSION_GREATER "2.9.9" )
    file( GENERATE
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/pango.desktop"
        CONTENT
"[Desktop Entry]
Version=1.0
Type=Application
Categories=AudioVideo;Player;Video;
Name=PangoVideoViewer
GenericName=Video Viewer
Comment=View images and video
Exec=$<TARGET_FILE:VideoViewer> %U
TryExec=$<TARGET_FILE:VideoViewer>
Icon=application-x-pango
Terminal=false
StartupNotify=false
MimeType=application/x-pango;image/x-portable-graymap;image/x-portable-pixmap;image/x-tga;image/tiff;image/jpeg;image/png;"
    )

    add_custom_target(
        install-mime-info
        COMMAND mkdir -p $ENV{HOME}/.local/share/mime/packages/
        COMMAND mkdir -p $ENV{HOME}/.local/share/applications/
        COMMAND mkdir -p $ENV{HOME}/.local/share/icons/hicolor/scalable/mimetypes/
        COMMAND cp "${CMAKE_CURRENT_SOURCE_DIR}/application-x-pango.xml" $ENV{HOME}/.local/share/mime/packages/
        COMMAND cp "${CMAKE_CURRENT_SOURCE_DIR}/application-x-pango.svg" $ENV{HOME}/.local/share/icons/hicolor/scalable/mimetypes/
        COMMAND cp "${CMAKE_CURRENT_BINARY_DIR}/pango.desktop"           $ENV{HOME}/.local/share/applications/
        COMMAND gtk-update-icon-cache   $ENV{HOME}/.local/share/icons/hicolor -f -t
        COMMAND update-mime-database    $ENV{HOME}/.local/share/mime
        COMMAND update-desktop-database $ENV{HOME}/.local/share/applications
        DEPENDS VideoViewer
        VERBATIM
    )
endif()
