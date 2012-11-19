# mongo-cxx-driver #
if(${SCONS_FOUND})
	rexos_add_scons_library(mongo-cxx-driver)
else()
	rexos_cannot_build(mongo-cxx-driver, "libraries/mongo-cxx-driver")
endif()