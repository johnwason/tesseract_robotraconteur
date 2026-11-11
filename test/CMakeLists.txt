find_package(GTest REQUIRED)

# The tests run the installed Tesseract support robot and task composer configuration
get_filename_component(TESSERACT_RR_TEST_TESSERACT_SHARE "${tesseract_DIR}/../../../share" ABSOLUTE)
get_filename_component(TESSERACT_RR_TEST_PLANNING_SHARE "${tesseract_planning_DIR}/../../../share" ABSOLUTE)
set(TESSERACT_RR_TEST_URDF_FILE "${TESSERACT_RR_TEST_TESSERACT_SHARE}/tesseract/support/urdf/abb_irb2400.urdf")
set(TESSERACT_RR_TEST_SRDF_FILE "${TESSERACT_RR_TEST_TESSERACT_SHARE}/tesseract/support/urdf/abb_irb2400.srdf")
set(TESSERACT_RR_TEST_TASK_PLUGIN_CONFIG_FILE
    "${TESSERACT_RR_TEST_PLANNING_SHARE}/tesseract_planning/task_composer/config/task_composer_plugins.yaml")

foreach(f ${TESSERACT_RR_TEST_URDF_FILE} ${TESSERACT_RR_TEST_SRDF_FILE} ${TESSERACT_RR_TEST_TASK_PLUGIN_CONFIG_FILE})
    if(NOT EXISTS "${f}")
        message(FATAL_ERROR "Test data file not found: ${f}")
    endif()
endforeach()

add_executable(${PROJECT_NAME}_service_test tesseract_robotraconteur_service_test.cpp service_process.cpp)
target_link_libraries(${PROJECT_NAME}_service_test PRIVATE ${PROJECT_NAME}_thunk GTest::gtest Eigen3::Eigen)
target_compile_definitions(
    ${PROJECT_NAME}_service_test
    PRIVATE TESSERACT_RR_TEST_SERVICE_EXECUTABLE="$<TARGET_FILE:${PROJECT_NAME}_service>"
            TESSERACT_RR_TEST_URDF_FILE="${TESSERACT_RR_TEST_URDF_FILE}"
            TESSERACT_RR_TEST_SRDF_FILE="${TESSERACT_RR_TEST_SRDF_FILE}"
            TESSERACT_RR_TEST_TASK_PLUGIN_CONFIG_FILE="${TESSERACT_RR_TEST_TASK_PLUGIN_CONFIG_FILE}"
            TESSERACT_RR_TEST_RESOURCE_PATH="${TESSERACT_RR_TEST_TESSERACT_SHARE}")
target_cxx_version(${PROJECT_NAME}_service_test PRIVATE VERSION ${TESSERACT_CXX_VERSION})
add_dependencies(${PROJECT_NAME}_service_test ${PROJECT_NAME}_service)

# One test process starts one service instance shared by all test cases
add_test(NAME ${PROJECT_NAME}_service_test COMMAND ${PROJECT_NAME}_service_test)
set_tests_properties(${PROJECT_NAME}_service_test PROPERTIES TIMEOUT 600)
