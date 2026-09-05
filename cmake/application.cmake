add_library(roller_domain STATIC ${CMAKE_SOURCE_DIR}/Application/src/controller.cpp)
target_include_directories(roller_domain PUBLIC ${CMAKE_SOURCE_DIR}/Application/include)
target_compile_features(roller_domain PUBLIC cxx_std_23)
target_compile_options(roller_domain PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror)
set(CO_ROOT ${CMAKE_SOURCE_DIR}/ThirdParty/CANopenNode)
add_library(canopen_stack STATIC
    ${CO_ROOT}/CANopen.c
    ${CO_ROOT}/301/CO_ODinterface.c
    ${CO_ROOT}/301/CO_NMT_Heartbeat.c
    ${CO_ROOT}/301/CO_HBconsumer.c
    ${CO_ROOT}/301/CO_Emergency.c
    ${CO_ROOT}/301/CO_SDOserver.c
    ${CO_ROOT}/301/CO_SYNC.c
    ${CO_ROOT}/301/CO_PDO.c
    ${CMAKE_SOURCE_DIR}/Protocol/CANopen/OD.c
    ${CMAKE_SOURCE_DIR}/Protocol/CANopen/co_driver.cpp)
target_include_directories(canopen_stack PUBLIC ${CMAKE_SOURCE_DIR}/Protocol/CANopen)
target_include_directories(canopen_stack SYSTEM PUBLIC ${CO_ROOT})
add_library(roller_protocol STATIC ${CMAKE_SOURCE_DIR}/Protocol/CANopen/node.cpp)
target_link_libraries(roller_protocol PUBLIC roller_domain canopen_stack)
target_compile_options(roller_protocol PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror)
