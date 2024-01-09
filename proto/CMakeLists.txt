project(cart_pole_proto)

find_package(Protobuf REQUIRED)
set(PROTO_GENERATE_PATH ${PROJECT_SOURCE_DIR}/proto_gen)
set(Protobuf_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}
)

file(
  GLOB_RECURSE PROTO_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ./*.proto)
foreach(PROTO_FILE ${PROTO_FILES})
  get_filename_component(RELATIVE_DIR ${PROTO_FILE} DIRECTORY)

  execute_process(COMMAND mkdir -p ${PROTO_GENERATE_PATH}/${RELATIVE_DIR})
  execute_process(COMMAND touch
                          ${PROTO_GENERATE_PATH}/${RELATIVE_DIR}/__init__.py)

  message(
    "protoc --python_out ${PROTO_GENERATE_PATH} -I ${CMAKE_CURRENT_SOURCE_DIR} ${PROTO_FILE}"
  )
  execute_process(
    COMMAND
      protoc --python_out ${PROTO_GENERATE_PATH} -I
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${PROTO_FILE})
endforeach()

set(PROTOBUF_GENERATE_CPP_APPEND_PATH FALSE)
list(APPEND PROTO_INCLUDE_PATH ${CMAKE_SOURCE_DIR})
list(APPEND PROTO_INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
# Use PROTOBUF_IMPORT_DIRS to set both import and export dir for protobuf.
set(PROTOBUF_IMPORT_DIRS ${PROTO_INCLUDE_PATH} PROTOC_OUT_DIR
                         ${PROTO_GENERATE_PATH})

protobuf_generate_cpp(ProtoSources ProtoHeaders ${PROTO_FILES})

execute_process(COMMAND touch ${PROTO_GENERATE_PATH}/__init__.py)

file(GLOB_RECURSE PROTO_SOURCES ${PROTO_GENERATE_PATH}/*.cc
     ${PROTO_GENERATE_PATH}/*.py)

list(APPEND PROTO_SOURCES ${ProtoSources})

add_library(${PROJECT_NAME} STATIC 
            ${PROTO_SOURCES})

# Specify different include directories for build time and install time.
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${PROTO_GENERATE_PATH}>
         $<BUILD_INTERFACE:${PROTOBUF_INCLUDE_DIRS}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
                    ${PROTOBUF_LIBRARIES})

install(DIRECTORY ${PROTO_GENERATE_PATH} DESTINATION ${PROJECT_NAME})
