import qbs

CppApplication {
    type: "application" // To suppress bundle generation on Mac
    Depends {
        name: "Qt"
        submodules: ["quick","gui","widgets","gui"]
    }
    //Setting up source path for openCV
    property string glewSourcePath: {
        if (qbs.hostOS.contains("windows"))
            return "C:/glew-1.13.0"
        if (qbs.hostOS.contains("linux"))
            return "~/glew-1.13.0"
    }

    //Setting up source path for openCV
    property string opencvSourcePath: {
        if (qbs.hostOS.contains("windows"))
            return "C:/opencv"
        if (qbs.hostOS.contains("linux"))
            return "~/opencv"
    }
    files: [
        "OpenGL/src/*.cpp",
        "OpenGL/include/*.h",
        "STLTool/src/*.cpp",
        "STLTool/include/*.h",
        "OpenGL/Shaders/*.vert",
        "OpenGL/Shaders/*.frag",
        glewSourcePath + "/src/glew.c"
    ]

    cpp.includePaths: [
        ".",
        "OpenGL/include",
        "STLTool/include"
    ]

    cpp.libraryPaths:[
        opencvSourcePath + "/build/x64/vc12/lib/"
    ]

    cpp.systemIncludePaths: [
        glewSourcePath + "/include/",
        opencvSourcePath + "/build/include/"
    ]

    cpp.staticLibraries: [
        "opengl32.lib",
        opencvSourcePath + "/build/x64/vc12/lib/opencv_ts300.lib",
        opencvSourcePath + "/build/x64/vc12/lib/opencv_world300.lib"
    ]


    Group {     // Properties for the produced executable
        fileTagsFilter: product.type
        qbs.install: true
    }

    cpp.defines: [
        "GLEW_STATIC",
        "GLEW_NO_GLU"
    ]
}

