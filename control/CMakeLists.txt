project(cart_pole_control)

add_executable(nmpc 
            nmpc.cpp
            nmpc_problem.cpp
            fg_eval.cpp)
target_link_libraries(nmpc PUBLIC
                    ${IPOPT_LIBRARIES}
                    ${CPPAD_LIBRARIES})