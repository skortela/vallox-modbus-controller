set (mqttVallox_src
    mqttVallox.cpp
    vallox.cpp
    debug.cpp
    )

# Make build flags compiler specific
if (CMAKE_COMPILER_IS_GNUCC)
	if (CMAKE_BUILD_TYPE STREQUAL "Debug")
		set (CMAKE_C_FLAGS "-D_REETRANT -ggdb -fPIC -Wall -Wextra -pedantic -O0 -Wno-missing-field-initializers")
	elseif( CMAKE_BUILD_TYPE STREQUAL "Release" )
		set (CMAKE_C_FLAGS "-D_REETRANT -DNDEBUG -fPIC -Wall -Wextra -pedantic -O3 -Wno-missing-field-initializers")
	endif ()
endif (CMAKE_COMPILER_IS_GNUCC)

LINK_DIRECTORIES(/usr/local/lib )

add_executable (mqttVallox ${mqttVallox_src})

TARGET_LINK_LIBRARIES(mqttVallox paho-mqtt3c)

# The rule to install daemon binary
install (TARGETS mqttVallox
	RUNTIME
	DESTINATION bin)
