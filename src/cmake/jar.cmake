function(add_executable_jar _TARGET_NAME _ENTRY_POINT)
    set(_JAVA_ENTRY_POINT_OPTION e)
    set(_JAVA_ENTRY_POINT_VALUE ${_ENTRY_POINT})

    add_jar2(${_TARGET_NAME} ${ARGN})

    # Just to avoid unwanted vars reuse in such case:
    # add_executable_jar(hello.jar, com.hello.Main,
    #                    com/hello/Main.java,
    #                    com/hello/Hello.java)
    # add_jar(hello.jar,
    #         com/hello/Hello.java)
    unset(_JAVA_ENTRY_POINT_OPTION)
    unset(_JAVA_ENTRY_POINT_VALUE)
endfunction(add_executable_jar)

function(add_jar2 _TARGET_NAME)
    set(_JAVA_SOURCE_FILES ${ARGN})

    if (LIBRARY_OUTPUT_PATH)
        set(CMAKE_JAVA_LIBRARY_OUTPUT_PATH ${LIBRARY_OUTPUT_PATH})
    else (LIBRARY_OUTPUT_PATH)
        set(CMAKE_JAVA_LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
    endif (LIBRARY_OUTPUT_PATH)

    set(CMAKE_JAVA_INCLUDE_PATH
        ${CMAKE_JAVA_INCLUDE_PATH}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_JAVA_OBJECT_OUTPUT_PATH}
        ${CMAKE_JAVA_LIBRARY_OUTPUT_PATH}
    )

    if (WIN32 AND NOT CYGWIN)
        set(CMAKE_JAVA_INCLUDE_FLAG_SEP ";")
    else (WIN32 AND NOT CYGWIN)
        set(CMAKE_JAVA_INCLUDE_FLAG_SEP ":")
    endif(WIN32 AND NOT CYGWIN)

    foreach (JAVA_INCLUDE_DIR ${CMAKE_JAVA_INCLUDE_PATH})
       set(CMAKE_JAVA_INCLUDE_PATH_FINAL "${CMAKE_JAVA_INCLUDE_PATH_FINAL}${CMAKE_JAVA_INCLUDE_FLAG_SEP}${JAVA_INCLUDE_DIR}")
    endforeach(JAVA_INCLUDE_DIR)

    set(CMAKE_JAVA_CLASS_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${_TARGET_NAME}.dir")

    set(_JAVA_TARGET_OUTPUT_NAME "${_TARGET_NAME}.jar")
    if (CMAKE_JAVA_TARGET_OUTPUT_NAME AND CMAKE_JAVA_TARGET_VERSION)
        set(_JAVA_TARGET_OUTPUT_NAME "${CMAKE_JAVA_TARGET_OUTPUT_NAME}-${CMAKE_JAVA_TARGET_VERSION}.jar")
        set(_JAVA_TARGET_OUTPUT_LINK "${CMAKE_JAVA_TARGET_OUTPUT_NAME}.jar")
    elseif (CMAKE_JAVA_TARGET_VERSION)
        set(_JAVA_TARGET_OUTPUT_NAME "${_TARGET_NAME}-${CMAKE_JAVA_TARGET_VERSION}.jar")
        set(_JAVA_TARGET_OUTPUT_LINK "${_TARGET_NAME}.jar")
    elseif (CMAKE_JAVA_TARGET_OUTPUT_NAME)
        set(_JAVA_TARGET_OUTPUT_NAME "${CMAKE_JAVA_TARGET_OUTPUT_NAME}.jar")
    endif (CMAKE_JAVA_TARGET_OUTPUT_NAME AND CMAKE_JAVA_TARGET_VERSION)
    # reset
    set(CMAKE_JAVA_TARGET_OUTPUT_NAME)

    set(_JAVA_CLASS_FILES)
    set(_JAVA_COMPILE_FILES)
    set(_JAVA_DEPENDS)
    set(_JAVA_RESOURCE_FILES)
    foreach(_JAVA_SOURCE_FILE ${_JAVA_SOURCE_FILES})
        get_filename_component(_JAVA_EXT ${_JAVA_SOURCE_FILE} EXT)
        get_filename_component(_JAVA_FILE ${_JAVA_SOURCE_FILE} NAME_WE)
        get_filename_component(_JAVA_PATH ${_JAVA_SOURCE_FILE} PATH)
        get_filename_component(_JAVA_FULL ${_JAVA_SOURCE_FILE} ABSOLUTE)

        file(RELATIVE_PATH _JAVA_REL_BINARY_PATH ${CMAKE_CURRENT_BINARY_DIR} ${_JAVA_FULL})
        file(RELATIVE_PATH _JAVA_REL_SOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${_JAVA_FULL})
        string(LENGTH ${_JAVA_REL_BINARY_PATH} _BIN_LEN)
        string(LENGTH ${_JAVA_REL_SOURCE_PATH} _SRC_LEN)
        if (${_BIN_LEN} LESS ${_SRC_LEN})
            set(_JAVA_REL_PATH ${_JAVA_REL_BINARY_PATH})
        else (${_BIN_LEN} LESS ${_SRC_LEN})
            set(_JAVA_REL_PATH ${_JAVA_REL_SOURCE_PATH})
        endif (${_BIN_LEN} LESS ${_SRC_LEN})
        get_filename_component(_JAVA_REL_PATH ${_JAVA_REL_PATH} PATH)

        if (_JAVA_EXT MATCHES ".java")
            list(APPEND _JAVA_COMPILE_FILES ${_JAVA_SOURCE_FILE})
            set(_JAVA_CLASS_FILE "${CMAKE_JAVA_CLASS_OUTPUT_PATH}/${_JAVA_REL_PATH}/${_JAVA_FILE}.class")
            set(_JAVA_CLASS_FILES ${_JAVA_CLASS_FILES} ${_JAVA_CLASS_FILE})

        elseif (_JAVA_EXT MATCHES ".jar"
                OR _JAVA_EXT MATCHES ".war"
                OR _JAVA_EXT MATCHES ".ear"
                OR _JAVA_EXT MATCHES ".sar")
            list(APPEND CMAKE_JAVA_INCLUDE_PATH ${_JAVA_SOURCE_FILE})

        elseif (_JAVA_EXT STREQUAL "")
            list(APPEND CMAKE_JAVA_INCLUDE_PATH ${JAVA_JAR_TARGET_${_JAVA_SOURCE_FILE}} ${JAVA_JAR_TARGET_${_JAVA_SOURCE_FILE}_CLASSPATH})
            list(APPEND _JAVA_DEPENDS ${JAVA_JAR_TARGET_${_JAVA_SOURCE_FILE}})

        else (_JAVA_EXT MATCHES ".java")
            __java_copy_file(${CMAKE_CURRENT_SOURCE_DIR}/${_JAVA_SOURCE_FILE}
                             ${CMAKE_JAVA_CLASS_OUTPUT_PATH}/${_JAVA_SOURCE_FILE}
                             "Copying ${_JAVA_SOURCE_FILE} to the build directory")
            list(APPEND _JAVA_RESOURCE_FILES ${_JAVA_SOURCE_FILE})
        endif (_JAVA_EXT MATCHES ".java")
    endforeach(_JAVA_SOURCE_FILE)

    # create an empty java_class_filelist
    if (NOT EXISTS ${CMAKE_JAVA_CLASS_OUTPUT_PATH}/java_class_filelist)
        file(WRITE ${CMAKE_JAVA_CLASS_OUTPUT_PATH}/java_class_filelist "")
    endif()

    if (_JAVA_COMPILE_FILES)
        # Compile the java files and create a list of class files
        add_custom_command(
            # NOTE: this command generates an artificial dependency file
            OUTPUT ${CMAKE_JAVA_CLASS_OUTPUT_PATH}/java_compiled_${_TARGET_NAME}
            COMMAND ${Java_JAVAC_EXECUTABLE}
                ${CMAKE_JAVA_COMPILE_FLAGS}
                -classpath "${CMAKE_JAVA_INCLUDE_PATH_FINAL}"
                -d ${CMAKE_JAVA_CLASS_OUTPUT_PATH}
                ${_JAVA_COMPILE_FILES}
            COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_JAVA_CLASS_OUTPUT_PATH}/java_compiled_${_TARGET_NAME}
            DEPENDS ${_JAVA_COMPILE_FILES}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "Building Java objects for ${_TARGET_NAME}.jar"
        )
        add_custom_command(
            OUTPUT ${CMAKE_JAVA_CLASS_OUTPUT_PATH}/java_class_filelist
            COMMAND ${CMAKE_COMMAND}
                -DCMAKE_JAVA_CLASS_OUTPUT_PATH=${CMAKE_JAVA_CLASS_OUTPUT_PATH}
                -DCMAKE_JAR_CLASSES_PREFIX="${CMAKE_JAR_CLASSES_PREFIX}"
                -P ${_JAVA_CLASS_FILELIST_SCRIPT}
            DEPENDS ${CMAKE_JAVA_CLASS_OUTPUT_PATH}/java_compiled_${_TARGET_NAME}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        )
    endif (_JAVA_COMPILE_FILES)

    # create the jar file
    if (CMAKE_JNI_TARGET)
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_NAME}
            COMMAND ${Java_JAR_EXECUTABLE}
                -cf${_JAVA_ENTRY_POINT_OPTION} ${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_NAME}
                ${_JAVA_ENTRY_POINT_VALUE}
                ${_JAVA_RESOURCE_FILES} @java_class_filelist
            COMMAND ${CMAKE_COMMAND}
                -D_JAVA_TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR}
                -D_JAVA_TARGET_OUTPUT_NAME=${_JAVA_TARGET_OUTPUT_NAME}
                -D_JAVA_TARGET_OUTPUT_LINK=${_JAVA_TARGET_OUTPUT_LINK}
                -P ${_JAVA_SYMLINK_SCRIPT}
            COMMAND ${CMAKE_COMMAND}
                -D_JAVA_TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR}
                -D_JAVA_TARGET_OUTPUT_NAME=${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_NAME}
                -D_JAVA_TARGET_OUTPUT_LINK=${_JAVA_TARGET_OUTPUT_LINK}
                -P ${_JAVA_SYMLINK_SCRIPT}
            DEPENDS ${_JAVA_RESOURCE_FILES} ${_JAVA_DEPENDS} ${CMAKE_JAVA_CLASS_OUTPUT_PATH}/java_class_filelist
            WORKING_DIRECTORY ${CMAKE_JAVA_CLASS_OUTPUT_PATH}
            COMMENT "Creating Java archive ${_JAVA_TARGET_OUTPUT_NAME}"
        )
    else ()
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_NAME}
            COMMAND ${Java_JAR_EXECUTABLE}
                -cf${_JAVA_ENTRY_POINT_OPTION} ${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_NAME}
                ${_JAVA_ENTRY_POINT_VALUE}
                ${_JAVA_RESOURCE_FILES} @java_class_filelist
            COMMAND ${CMAKE_COMMAND}
                -D_JAVA_TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR}
                -D_JAVA_TARGET_OUTPUT_NAME=${_JAVA_TARGET_OUTPUT_NAME}
                -D_JAVA_TARGET_OUTPUT_LINK=${_JAVA_TARGET_OUTPUT_LINK}
                -P ${_JAVA_SYMLINK_SCRIPT}
            WORKING_DIRECTORY ${CMAKE_JAVA_CLASS_OUTPUT_PATH}
            DEPENDS ${_JAVA_RESOURCE_FILES} ${_JAVA_DEPENDS} ${CMAKE_JAVA_CLASS_OUTPUT_PATH}/java_class_filelist
            COMMENT "Creating Java archive ${_JAVA_TARGET_OUTPUT_NAME}"
        )
    endif (CMAKE_JNI_TARGET)

    # Add the target and make sure we have the latest resource files.
    add_custom_target(${_TARGET_NAME} ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_NAME})

    set_property(
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            INSTALL_FILES
                ${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_NAME}
    )

    if (_JAVA_TARGET_OUTPUT_LINK)
        set_property(
            TARGET
                ${_TARGET_NAME}
            PROPERTY
                INSTALL_FILES
                    ${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_NAME}
                    ${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_LINK}
        )

        if (CMAKE_JNI_TARGET)
            set_property(
                TARGET
                    ${_TARGET_NAME}
                PROPERTY
                    JNI_SYMLINK
                        ${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_LINK}
            )
        endif (CMAKE_JNI_TARGET)
    endif (_JAVA_TARGET_OUTPUT_LINK)

    set_property(
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            JAR_FILE
                ${CMAKE_CURRENT_BINARY_DIR}/${_JAVA_TARGET_OUTPUT_NAME}
    )

    set_property(
        TARGET
            ${_TARGET_NAME}
        PROPERTY
            CLASSDIR
                ${CMAKE_JAVA_CLASS_OUTPUT_PATH}
    )

endfunction(add_jar2)