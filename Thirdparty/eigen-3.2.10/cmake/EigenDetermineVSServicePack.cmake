include(CMakeDetermineVSServicePack)

# The code is almost identical to the CMake version. The only difference is that we remove
# _DetermineVSServicePack_FastCheckVersionWithCompiler which lead to errors on some systems.
function(EigenDetermineVSServicePack _pack)
    if(NOT DETERMINED_VS_SERVICE_PACK OR NOT ${_pack})

        if(NOT DETERMINED_VS_SERVICE_PACK)
            _DetermineVSServicePack_CheckVersionWithTryCompile(DETERMINED_VS_SERVICE_PACK _cl_version)
            if(NOT DETERMINED_VS_SERVICE_PACK)
                _DetermineVSServicePack_CheckVersionWithTryRun(DETERMINED_VS_SERVICE_PACK _cl_version)
            endif()
        endif()

        if(DETERMINED_VS_SERVICE_PACK)

            if(_cl_version)
                # Call helper function to determine VS version
                _DetermineVSServicePackFromCompiler(_sp "${_cl_version}")
                if(_sp)
                    set(${_pack} ${_sp} CACHE INTERNAL
                        "The Visual Studio Release with Service Pack")
                endif()
            endif()
        endif()
    endif()
endfunction()
