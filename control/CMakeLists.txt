project(cart_pole_control)

add_library(nmpc SHARED
            nmpc.cpp
            nmpc_problem.cpp
            fg_eval.cpp)
target_link_libraries(nmpc PUBLIC
                    ${IPOPT_LIBRARIES}
                    ${CPPAD_LIBRARIES}
                    cart_pole_proto
                    cart_pole_utils)