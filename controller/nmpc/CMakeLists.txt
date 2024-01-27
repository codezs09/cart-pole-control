project(cart_pole_nmpc)

add_library(nmpc SHARED
            nmpc.cpp
            regular/nmpc_problem_regular.cpp
            regular/fg_eval_regular.cpp
            loose/nmpc_problem_loose.cpp
            loose/fg_eval_loose.cpp
            )
target_link_libraries(nmpc PUBLIC
                    ${IPOPT_LIBRARIES}
                    ${CPPAD_LIBRARIES}
                    cart_pole_proto
                    cart_pole_utils)