# Install script for directory: /home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/build")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres" TYPE FILE FILES
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/autodiff_cost_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/autodiff_first_order_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/autodiff_local_parameterization.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/c_api.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/ceres.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/conditioned_cost_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/context.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/cost_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/cost_function_to_functor.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/covariance.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/crs_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/cubic_interpolation.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/dynamic_autodiff_cost_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/dynamic_cost_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/dynamic_cost_function_to_functor.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/dynamic_numeric_diff_cost_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/evaluation_callback.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/first_order_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/gradient_checker.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/gradient_problem.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/gradient_problem_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/iteration_callback.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/jet.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/local_parameterization.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/loss_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/normal_prior.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/numeric_diff_cost_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/numeric_diff_first_order_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/numeric_diff_options.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/ordered_groups.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/problem.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/rotation.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/sized_cost_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/tiny_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/tiny_solver_autodiff_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/tiny_solver_cost_function_adapter.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/types.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/version.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres/internal" TYPE FILE FILES
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/array_selector.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/autodiff.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/disable_warnings.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/eigen.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/fixed_array.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/householder_vector.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/integer_sequence_algorithm.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/line_parameterization.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/memory.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/numeric_diff.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/parameter_dims.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/port.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/reenable_warnings.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/include/ceres/internal/variadic_evaluate.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres" TYPE FILE FILES
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/accelerate_sparse.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/array_utils.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/blas.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/block_evaluate_preparer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/block_jacobi_preconditioner.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/block_jacobian_writer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/block_random_access_dense_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/block_random_access_diagonal_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/block_random_access_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/block_random_access_sparse_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/block_sparse_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/block_structure.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/bundle_adjustment_test_util.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/callbacks.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/canonical_views_clustering.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/casts.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/cgnr_linear_operator.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/cgnr_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/compressed_col_sparse_matrix_utils.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/compressed_row_jacobian_writer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/compressed_row_sparse_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/concurrent_queue.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/conjugate_gradients_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/context_impl.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/coordinate_descent_minimizer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/corrector.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/covariance_impl.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/cxsparse.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/dense_jacobian_writer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/dense_normal_cholesky_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/dense_qr_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/dense_sparse_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/detect_structure.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/dogleg_strategy.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/dynamic_compressed_row_finalizer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/dynamic_compressed_row_jacobian_writer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/dynamic_compressed_row_sparse_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/dynamic_sparse_normal_cholesky_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/eigensparse.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/evaluator.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/evaluator_test_utils.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/execution_summary.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/file.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/float_cxsparse.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/float_suitesparse.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/function_sample.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/gradient_checking_cost_function.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/gradient_problem_evaluator.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/graph.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/graph_algorithms.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/implicit_schur_complement.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/inner_product_computer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/invert_psd_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/is_close.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/iterative_refiner.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/iterative_schur_complement_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/lapack.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/levenberg_marquardt_strategy.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/line_search.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/line_search_direction.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/line_search_minimizer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/line_search_preprocessor.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/linear_least_squares_problems.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/linear_operator.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/linear_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/low_rank_inverse_hessian.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/map_util.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/minimizer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/numeric_diff_test_utils.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/pair_hash.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/parallel_for.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/parallel_utils.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/parameter_block.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/parameter_block_ordering.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/partitioned_matrix_view.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/partitioned_matrix_view_impl.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/polynomial.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/preconditioner.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/preprocessor.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/problem_impl.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/program.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/program_evaluator.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/random.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/reorder_program.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/residual_block.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/residual_block_utils.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/schur_complement_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/schur_eliminator.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/schur_eliminator_impl.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/schur_jacobi_preconditioner.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/schur_templates.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/scoped_thread_token.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/scratch_evaluate_preparer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/single_linkage_clustering.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/small_blas.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/small_blas_generic.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/solver_utils.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/sparse_cholesky.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/sparse_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/sparse_normal_cholesky_solver.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/split.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/stl_util.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/stringprintf.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/subset_preconditioner.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/suitesparse.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/test_util.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/thread_pool.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/thread_token_provider.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/tiny_solver_test_util.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/triplet_sparse_matrix.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/trust_region_minimizer.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/trust_region_preprocessor.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/trust_region_step_evaluator.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/trust_region_strategy.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/visibility.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/visibility_based_preconditioner.h"
    "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/internal/ceres/wall_time.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/ceres/internal" TYPE FILE FILES "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/build/config/ceres/internal/config.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake"
         "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/build/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres/CeresTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/build/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/build/CMakeFiles/Export/lib/cmake/Ceres/CeresTargets-release.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE RENAME "CeresConfig.cmake" FILES "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/build/CeresConfig-install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/build/CeresConfigVersion.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Ceres" TYPE FILE FILES "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/cmake/FindGlog.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/build/internal/ceres/cmake_install.cmake")
  include("/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/build/examples/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/lenovo/catkin_ws_debug2/src/RTK-Visual-Inertial-Navigation/ceres-solver-modified/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
