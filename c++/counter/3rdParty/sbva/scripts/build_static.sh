rm -rf CMake*
rm -rf src
rm -rf cmake*
rm -rf Make*
rm -rf sbva*
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_SHARED_LIBS=OFF ..
make -j$(nproc)
