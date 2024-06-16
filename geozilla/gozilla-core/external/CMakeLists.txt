function(set_folder_property DIR FOLDER_NAME)
    get_property(TARGETS DIRECTORY ${DIR} PROPERTY BUILDSYSTEM_TARGETS)
    foreach(TARGET_NAME IN LISTS TARGETS)
        set_target_properties(${TARGET_NAME} PROPERTIES FOLDER ${FOLDER_NAME})
    endforeach()

    get_property(SUBDIRS DIRECTORY ${DIR} PROPERTY SUBDIRECTORIES)
    foreach(SUBDIR IN LISTS SUBDIRS)
        set_folder_property(${SUBDIR} ${FOLDER_NAME})
    endforeach()
endfunction()

function(add_external DIR)
    add_subdirectory(${DIR})
    set_folder_property(${DIR} "External")
endfunction()

add_external(cesium-native)
