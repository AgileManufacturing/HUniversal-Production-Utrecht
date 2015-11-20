import qbs

CppApplication {
    type: "application" // To suppress bundle generation on Mac
    files: [
        "*.cpp", "*.h","OpenGL/*.cpp","OpenGL/*.h"
    ]

    Depends {
        name: "Qt"
        submodules: ["quick","gui","widgets","gui"]
    }

    cpp.includePaths: [
        ".",
    ]

    cpp.systemIncludePaths: [
        "C:/glew-1.13.0/include/",
        "C:/opencv/build/include/"
    ]

    cpp.staticLibraries: [
        "opengl32.lib",
        "C:/opencv/build/x64/vc12/lib/opencv_ts300d.lib",
        "C:/opencv/build/x64/vc12/lib/opencv_world300d.lib"
    ]

    cpp.libraryPaths:[
        "C:/opencv/build/x64/vc12/lib/"
    ]

    Group {
        name: "GLEW"

        property string glewSourcePath: {
            if (qbs.hostOS.contains("windows"))
                return "C:/glew-1.13.0"
            if (qbs.hostOS.contains("linux"))
                return "~/glew-1.13.0"
        }

        files: [
            glewSourcePath + "/src/glew.c",
        ]
    }
//    Group {
//        name: "OpenCV"
//        property string openCVPath: {
//            if(qbs.hostOS.contains("windows"))
//                return "C:/opencv"
//            if(qbs.hostOS.contains("linux"))
//                return "~/opencv"
//        }
//        cpp.staticLibraries: [
//            openCVPath + "/build/x64/vc12/lib/opencv_ts300d.lib",
//            openCVPath + "/build/x64/vc12/lib/opencv_world300d.lib"
//        ]
//    }

    Group {     // Properties for the produced executable
        fileTagsFilter: product.type
        qbs.install: true
    }

    cpp.defines: [
        "GLEW_STATIC",
        "GLEW_NO_GLU"
    ]
}

