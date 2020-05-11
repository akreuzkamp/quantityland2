add_executable(tst_quantityland2 tst_quantityland2.cpp)
target_link_libraries(tst_quantityland2 PUBLIC quantityland2)
add_test(NAME tst_quantityland2 COMMAND tst_quantityland2)

find_package(GSL REQUIRED)

# add_executable(tst_gslintegration tst_gslintegration.cpp)
# target_link_libraries(tst_gslintegration PUBLIC quantityland2 GSL::gsl fftw3)
# add_test(NAME tst_gslintegration COMMAND tst_gslintegration)
