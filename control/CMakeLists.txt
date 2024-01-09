project(cart_pole_control)

add_executable(nmpc nmpc.cpp)
target_link_libraries(nmpc SHARED
                    ${IPOPT_LIBRARIES}
                    ${CPPAD_LIBRARIES})